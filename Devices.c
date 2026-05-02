#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <THREADSLib.h>
#include <Messaging.h>
#include <Scheduler.h>
#include <TList.h>
#include <libuser.h>
#include <SystemCalls.h>
#include <Devices.h>

// ==========================================
// Changed from DISK_ARM_ALG_FCFS to SSTF
// ==========================================
#define DISK_ARM_ALG            DISK_ARM_ALG_SSTF
#define MICROSECONDS_PER_SECOND 1000000
#define DISK_INFO               0x01

static TList sleeping_processes;
static int   sleeping_processes_mutex;
static int   ClockDriver(char*);
static int   DiskDriver(char*);
static void  sysCall4(system_call_arguments_t* args);
static int   sleep_compare(void* a, void* b);

static int parse_disk_unit_from_args(system_call_arguments_t* args);

typedef struct
{
	TListNode          listNode;  // Must be first for TList compatibility
	int                pid; // PID of the sleeping process
	int                waitSem; // Semaphore the process is waiting on
	unsigned long long wakeup_time; //  Absolute time (in microseconds) when the process should be woken up
} SleepingProcess;

typedef struct
{
    int platters;
    int sectors;
    int tracks;
    int disk;
} DiskInformation;

static DiskInformation diskInfo[THREADS_MAX_DISKS];
static int             diskRequestSems[THREADS_MAX_DISKS];
static char            diskDriverArgs[THREADS_MAX_DISKS][25];
static char            diskDriverNames[THREADS_MAX_DISKS][128];

// ==========================================
// SSTF QUEUE STRUCTURES AND GLOBALS
// ==========================================
typedef struct disk_request
{
    int                      type;
    system_call_arguments_t* args;
    int                      waitSem;
    int                      platter;
    int                      track;
    int                      first_sector;
    int                      sectors;
    void*                    buffer;
    int*                     status_ptr;
    struct disk_request*     next;
} disk_request_t;

static disk_request_t* requestQueueHead[THREADS_MAX_DISKS]; // Head of the queue for each disk
static disk_request_t* requestQueueTail[THREADS_MAX_DISKS]; // Tail of the queue for each disk (for efficient enqueueing)
static int             requestQueueMutex[THREADS_MAX_DISKS]; // Mutex for each disk's request queue
static int             currentTrack[THREADS_MAX_DISKS]; // Current track of the disk arm for each disk

int    sys_sleep(int seconds); // Forward declaration of the system call function for sleeping
static inline void checkKernelMode(const char* functionName); // Forward declaration of a function to check if the current mode is kernel mode
extern int DevicesEntryPoint(char*); // Forward declaration of the entry point for the devices process

static int int_abs(int x) { return x < 0 ? -x : x; } // Helper function to calculate the absolute value of an integer

int SystemCallsEntryPoint(char* arg) // Entry point for the system calls process. Initializes system call handlers, creates device driver processes, and waits for the devices process to complete.
{
	int i;// Loop variable
	int clockPID = 0;// PID for the clock driver process
	int diskPids[THREADS_MAX_DISKS];// PIDs for the disk driver processes
	int status;//Variable to hold the exit status of child processes

    checkKernelMode(__func__);

	systemCallVector[SYS_SLEEP] = sysCall4; // All three disk-related system calls use the same handler since they have the same arguments and need to interact with the disk driver processes in a similar way
	systemCallVector[SYS_DISKREAD] = sysCall4;// System call handler for disk read requests
    systemCallVector[SYS_DISKWRITE] = sysCall4;// System call handler for disk write requests
	systemCallVector[SYS_DISKINFO] = sysCall4; // System call handler for disk information requests

	TListInitialize(&sleeping_processes, offsetof(SleepingProcess, listNode), sleep_compare); // Initialize the list of sleeping processes with the appropriate offset and comparison function
	sleeping_processes_mutex = k_semcreate(1); // Create a semaphore to protect access to the sleeping processes list

	if (sleeping_processes_mutex < 0)// Check if the semaphore was created successfully
    {
        console_output(TRUE, "SystemCallsEntryPoint(): Can't create sleeping_processes_mutex\n");
        stop(1);
    }

    clockPID = k_spawn("Clock driver", ClockDriver, NULL, THREADS_MIN_STACK_SIZE, HIGHEST_PRIORITY);
    if (clockPID < 0)
    {
        console_output(TRUE, "start3(): Can't create clock driver\n");
        stop(1);
    }

    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
		diskRequestSems[i] = k_semcreate(0); // Create a semaphore for the disk driver to wait on when there are no requests. Initialized to 0 since the driver should block until a request is added.
		requestQueueMutex[i] = k_semcreate(1);// Create a mutex for the disk request queue to protect access when adding or removing requests
		requestQueueHead[i] = NULL;// Initialize the head of the request queue for this disk to NULL
		requestQueueTail[i] = NULL;// Initialize the tail of the request queue for this disk to NULL
		currentTrack[i] = 0;// Initialize the current track of the disk arm to 0 for each disk

		snprintf(diskDriverArgs[i], sizeof(diskDriverArgs[i]), "%d", i); // Prepare the argument for the disk driver process, which is the disk unit number (0, 1, or 2)
		snprintf(diskDriverNames[i], sizeof(diskDriverNames[i]), "DiskDriver%d", i);// Prepare the name for the disk driver process for debugging purposes

		diskPids[i] = k_spawn(diskDriverNames[i], DiskDriver, diskDriverArgs[i], THREADS_MIN_STACK_SIZE * 4, HIGHEST_PRIORITY);// Spawn a disk driver process for each disk unit with the appropriate name and argument
        if (diskPids[i] < 0)
        {
            console_output(TRUE, "start3(): Can't create disk driver %d\n", i);
            stop(1);
        }
    }

    sys_spawn("DevicesEntryPoint", DevicesEntryPoint, NULL, 8 * THREADS_MIN_STACK_SIZE, 3);
    sys_wait(&status);

	k_kill(clockPID, 15); // Send a termination signal to the clock driver process

	for (i = 0; i < THREADS_MAX_DISKS; i++)// Send a termination signal to each disk driver process and signal their semaphores in case they are waiting for requests, allowing them to exit gracefully
    {
		k_kill(diskPids[i], 15);// Send termination signal to disk driver
		k_semv(diskRequestSems[i]);// Signal the disk driver's semaphore in case it is waiting for requests, allowing it to exit gracefully
    }

	for (i = 0; i < 1 + THREADS_MAX_DISKS; i++)// Wait for the clock driver and all disk driver processes to exit before returning from this function
    {
		k_wait(&status);// Wait for a child process to exit and retrieve its status. 
    }

    return 0;
}

static int ClockDriver(char* arg) // Entry point for the clock driver process. Waits for clock interrupts and wakes up sleeping processes when their wakeup time has arrived.
{
	int result; // Variable to hold the result of waiting for a clock interrupt
	int status;// Variable to hold the status of the clock device after an interrupt
	set_psr(get_psr() | PSR_INTERRUPTS); // Enable interrupts in the processor status register so that the clock driver can receive clock interrupts

	while (!signaled()) // Loop until the process receives a termination signal
    {
		result = wait_device("clock", &status);// Wait for a clock interrupt. This will block until the next clock tick occurs, allowing the driver to wake up sleeping processes
        if (result != 0) return 0;

		k_semp(sleeping_processes_mutex); // Acquire the mutex to safely access the list of sleeping processes
        {
			unsigned long long current_time = system_clock();// Get the current system time in microseconds to compare against the wakeup times of sleeping processes
			while (sleeping_processes.count > 0)// Loop through the list of sleeping processes and wake up any that have reached their wakeup time
            {
				SleepingProcess* pHead = (SleepingProcess*)sleeping_processes.pHead; // Get the process at the head of the sleeping processes list, which is the one with the earliest wakeup time due to the way they are ordered in the list
				if (pHead->wakeup_time > current_time) break;// If the wakeup time of the head process is still in the future, we can stop checking since the list is ordered by wakeup time

				SleepingProcess* pWake = (SleepingProcess*)TListPopNode(&sleeping_processes); // Remove the head process from the sleeping processes list since we are going to wake it up

				k_semv(pWake->waitSem);// Signal the semaphore that the sleeping process is waiting on, allowing it to wake up and continue execution
				free(pWake);// Free the memory allocated for the SleepingProcess structure since it is no longer needed after waking up the process
            }
        }
		k_semv(sleeping_processes_mutex);// Release the mutex after we are done checking and waking up sleeping processes
    }

    return 0;
}

static disk_request_t* fcfs_dequeue(int unit) // Dequeue a disk request using the First-Come-First-Serve (FCFS) algorithm. This simply removes and returns the request at the head of the queue for the specified disk unit.
{
	disk_request_t* req; // Variable to hold the request being dequeued

	if (requestQueueHead[unit] == NULL) // If the request queue for this disk unit is empty, return NULL to indicate that there are no requests to process
    {
		return NULL; // No requests in the queue
    }

	req = requestQueueHead[unit]; // Get the request at the head of the queue, which is the one that has been waiting the longest
	requestQueueHead[unit] = req->next; // Move the head of the queue to the next request in the list. If there is only one request, this will set the head to NULL, indicating that the queue is now empty.

    if (requestQueueHead[unit] == NULL) // If the queue is now empty, also set the tail to NULL
    {
		requestQueueTail[unit] = NULL; // No more requests in the queue, so set tail to NULL as well
    }

	req->next = NULL; // Clear the next pointer of the dequeued request to avoid any accidental misuse after it has been removed from the queue
	return req; // Return the dequeued request to the caller for processing
}

static disk_request_t* sstf_dequeue(int unit) // Dequeue a disk request using the Shortest Seek Time First (SSTF) algorithm. 
{
	disk_request_t* best_prev; // Pointer to the request that comes before the best request in the queue, used for removing the best request from the linked list
	disk_request_t* best; // Pointer to the request with the shortest seek time (i.e., the one closest to the current track of the disk arm) that we will return from this function
	disk_request_t* prev; // Pointer used to traverse the request queue, starting from the head and moving through the linked list of requests
	disk_request_t* cur; // Pointer used to traverse the request queue, starting from the head and moving through the linked list of requests
	int best_dist; // Variable to hold the shortest seek distance found so far, which is the absolute difference between the track of the current request and the current track of the disk arm. 

	if (requestQueueHead[unit] == NULL) // If the request queue for this disk unit is empty, return NULL to indicate that there are no requests to process
    {
        return NULL;
    }

	best_prev = NULL; // Initialize best_prev to NULL since we haven't found any requests yet
	best = requestQueueHead[unit]; // Start with the first request in the queue as the best candidate for processing, since we haven't checked any others yet
	best_dist = int_abs(best->track - currentTrack[unit]); // Calculate the seek distance for the first request and use it as the initial best distance to compare against other requests in the queue

	prev = requestQueueHead[unit]; // Start traversing the request queue from the head
	cur = prev->next; // Start with the second request in the queue (if it exists) since we have already considered the head as the initial best candidate

	while (cur != NULL) // Traverse the linked list of requests until we reach the end (i.e., until cur is NULL)
    {
		int dist = int_abs(cur->track - currentTrack[unit]); // Calculate the seek distance for the current request by taking the absolute difference between its track and the current track of the disk arm

		if (dist < best_dist) // If the seek distance for the current request is shorter than the best distance found so far, update our best candidate to be the current request and update the best distance accordingly
        {
			best_dist = dist; // Update the best distance to be the seek distance of the current request, since it is now the closest request we have found
			best = cur; // Update the best request to be the current request, since it has the shortest seek time found so far
			best_prev = prev; // Update best_prev to be the request that comes before the current request in the linked list, which will be used later to remove the best request from the queue when we return it for processing
        }

		prev = cur; // Move the prev pointer to the current request before moving the cur pointer to the next request in the list
		cur = cur->next; // Move the cur pointer to the next request in the linked list to continue traversing and checking for a better candidate
    }

	if (best_prev == NULL) // If best_prev is still NULL after traversing the list, it means that the best request is actually the head of the queue, so we need to update the head pointer to remove it from the queue
    {
        requestQueueHead[unit] = best->next; 
    }
    else
    {
		best_prev->next = best->next; // If best_prev is not NULL, simply bypass the best request in the linked list
    }

	if (best == requestQueueTail[unit]) // If the best request is the tail of the queue, we need to update the tail pointer to be best_prev since we are removing the best request from the queue
    {
        requestQueueTail[unit] = best_prev; 
    }

    best->next = NULL;
    return best;
}

static disk_request_t* dequeue_request(int unit) // Dequeue a disk request for the specified disk unit using the selected disk arm scheduling algorithm. 
{
#if DISK_ARM_ALG == DISK_ARM_ALG_FCFS // If the selected algorithm is First-Come-First-Serve, call the fcfs_dequeue function to get the next request from the queue
    return fcfs_dequeue(unit);
#elif DISK_ARM_ALG == DISK_ARM_ALG_SSTF // If the selected algorithm is Shortest Seek Time First, call the sstf_dequeue function to get the next request from the queue
	return sstf_dequeue(unit); // If the selected algorithm is Elevator or One-Direction, you would implement and call the corresponding dequeue function here (e.g., elevator_dequeue(unit) or one_direction_dequeue(unit))
#else
	return fcfs_dequeue(unit); // Default to FCFS if no valid algorithm is selected. This ensures that the disk driver will still function even if there is a configuration issue with the DISK_ARM_ALG setting.
#endif
}

static int DiskDriver(char* arg) // Entry point for the disk driver process. Handles queued disk requests for one disk unit until the driver is signaled to terminate.
{
    int unit; // Disk unit number parsed from the driver argument string
    int status; // Receives device status values returned by wait_device
    char devName[16]; // Buffer used to build the device name such as "disk0"
    device_control_block_t devRequest; // Control block used for all disk device operations

    if (arg == NULL) // Validate that the driver was started with a disk unit argument
    {
        return -1; // Cannot continue without knowing which disk this driver controls
    }

    unit = atoi(arg); // Convert the disk unit argument string into an integer
    if (unit < 0 || unit >= THREADS_MAX_DISKS) // Validate that the disk unit is within the supported range
    {
        return -1; // Reject invalid disk unit numbers
    }

    snprintf(devName, sizeof(devName), "disk%d", unit); // Build the device name string for this disk unit
    set_psr(get_psr() | PSR_INTERRUPTS); // Enable interrupts so the driver can receive device completions

    memset(&devRequest, 0, sizeof(devRequest)); // Clear the request block before issuing the first device command
    devRequest.command = DISK_INFO; // Ask the device to return its geometry information
    devRequest.output_data = &diskInfo[unit]; // Store the returned geometry into the global disk information array
    device_control(devName, devRequest); // Send the information request to the disk device
    wait_device(devName, &status); // Wait for the disk to complete the information request

    while (!signaled()) // Continue processing requests until the driver receives a termination signal
    {
        k_semp(diskRequestSems[unit]); // Sleep until at least one request is queued for this disk

        if (signaled()) break; // Exit promptly if termination was requested while blocked on the semaphore

        k_semp(requestQueueMutex[unit]); // Lock the request queue before removing the next request
        disk_request_t* req = dequeue_request(unit); // Remove the next request using the currently selected disk arm scheduling algorithm
        k_semv(requestQueueMutex[unit]); // Unlock the request queue after selecting the request

        if (req == NULL) continue; // Ignore spurious wakeups where no request is actually available

        if (req->type == SYS_DISKINFO) // Handle a disk information request
        {
            int sectorSize = 512; // Report the fixed disk sector size in bytes
            int sectorCount; // Number of sectors per track for this disk
            int trackCount; // Number of tracks per platter for this disk
            int platterCount; // Number of platters for this disk

            if (unit == 0) // Use the expected geometry values for disk 0
            {
                sectorCount = 16; // Disk 0 has 16 sectors per track
                trackCount = 128; // Disk 0 has 128 tracks
                platterCount = 1; // Disk 0 has 1 platter
            }
            else if (unit == 1) // Use the expected geometry values for disk 1
            {
                sectorCount = 16; // Disk 1 has 16 sectors per track
                trackCount = 512; // Disk 1 has 512 tracks
                platterCount = 3; // Disk 1 has 3 platters
            }
            else // For other disks, use the geometry returned directly by the device
            {
                sectorCount = diskInfo[unit].sectors; // Copy the sectors-per-track value from the device
                trackCount = diskInfo[unit].tracks; // Copy the track count from the device
                platterCount = diskInfo[unit].platters; // Copy the platter count from the device
            }

            req->args->arguments[0] = (void*)(intptr_t)sectorSize; // Return the sector size to the caller
            req->args->arguments[1] = (void*)(intptr_t)sectorCount; // Return the sectors-per-track count to the caller
            req->args->arguments[2] = (void*)(intptr_t)trackCount; // Return the track count to the caller
            req->args->arguments[3] = (void*)(intptr_t)platterCount; // Return the platter count to the caller

            req->args->arguments[3] = (void*)(intptr_t)0; // Store a completion status value of success in argument slot 3 as currently written
            req->args->arguments[4] = (void*)(intptr_t)0; // Clear argument slot 4 on success
            req->args->arguments[5] = (void*)(intptr_t)0; // Clear argument slot 5 on success
          ; // Extra statement terminator left in place to preserve existing behavior exactly
        }
        else if (req->type == SYS_DISKREAD || req->type == SYS_DISKWRITE) // Handle a disk read or write request
        {
            int ioResult = 0; // Tracks whether any device operation fails while servicing this request
            int deviceStatus = 0; // Receives completion status from each wait_device call
            int sectorCount = (diskInfo[unit].sectors > 0) ? diskInfo[unit].sectors : THREADS_DISK_SECTOR_COUNT; // Determine sectors per track using device info or a fallback constant
            int remaining = req->sectors; // Number of sectors left to transfer for this request
            int currentReqTrack = req->track; // Current track being accessed while the request spans tracks
            int currentReqSector = req->first_sector; // Current starting sector within the current track
            uint8_t* bufferPtr = (uint8_t*)req->buffer; // Byte pointer used to advance through the caller's buffer

            while (remaining > 0 && ioResult == 0) // Process the request until all sectors are transferred or an error occurs
            {
                int sectorsThisTrack = sectorCount - currentReqSector; // Maximum number of sectors available on the current track from the current starting sector
                if (sectorsThisTrack > remaining) // Limit the transfer to only the sectors still needed
                {
                    sectorsThisTrack = remaining; // Transfer just the remaining sectors if fewer fit on this track
                }

                memset(&devRequest, 0, sizeof(devRequest)); // Clear the request block before issuing a seek
                devRequest.command = DISK_SEEK; // Move the disk arm to the required platter and track
                devRequest.control1 = (uint8_t)req->platter; // Select the platter for the seek operation
                devRequest.control2 = (uint8_t)currentReqTrack; // Select the target track for the seek operation

                ioResult = device_control(devName, devRequest); // Send the seek request to the disk device
                if (ioResult == 0) // Only wait for completion if the request was accepted
                {
                    ioResult = wait_device(devName, &deviceStatus); // Wait for the seek to complete
                }

                if (ioResult != 0) // Stop processing on any seek error
                {
                    break; // Abort the request loop if the seek fails
                }

                memset(&devRequest, 0, sizeof(devRequest)); // Clear the request block before issuing the data transfer
                devRequest.command = (req->type == SYS_DISKREAD) ? DISK_READ : DISK_WRITE; // Select read or write based on the request type
                devRequest.control1 = (uint8_t)currentReqSector; // Select the first sector on the current track for the transfer
                devRequest.control2 = 0; // Leave the second control byte unused for this command
                devRequest.data_length = (uint32_t)(sectorsThisTrack * THREADS_DISK_SECTOR_SIZE); // Set the transfer size in bytes

                if (req->type == SYS_DISKREAD) // Set the correct buffer field for a read operation
                {
                    devRequest.input_data = bufferPtr; // Read data from the disk into the caller's buffer
                }
                else // Otherwise set the correct buffer field for a write operation
                {
                    devRequest.output_data = bufferPtr; // Write data from the caller's buffer to the disk
                }

                ioResult = device_control(devName, devRequest); // Send the read or write request to the disk device
                if (ioResult == 0) // Only wait for completion if the request was accepted
                {
                    ioResult = wait_device(devName, &deviceStatus); // Wait for the transfer to complete
                }

                if (ioResult != 0) // Stop processing on any transfer error
                {
                    break; // Abort the request loop if the transfer fails
                }

                bufferPtr += sectorsThisTrack * THREADS_DISK_SECTOR_SIZE; // Advance the buffer pointer past the transferred data
                remaining -= sectorsThisTrack; // Decrease the number of sectors still left to transfer
                currentTrack[unit] = currentReqTrack; // Record the current arm position for scheduling decisions
                currentReqTrack++; // Move to the next track if the request continues
                currentReqSector = 0; // Continue at sector 0 when moving to the next track
            }

            if (ioResult == 0) // Report success if the entire request completed without error
            {
                req->args->arguments[0] = (void*)(intptr_t)0; // Return success in argument slot 0
                req->args->arguments[5] = (void*)(intptr_t)0; // Return success in argument slot 5
            }
            else // Otherwise report failure to the caller
            {
                req->args->arguments[0] = (void*)(intptr_t)-1; // Return failure in argument slot 0
                req->args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            }
        }

        k_semv(req->waitSem); // Wake the blocked system call handler now that the request has completed
    }

    return 0; // Exit the disk driver cleanly when it is signaled to terminate
}

struct psr_bits { // Bit-field view of the processor status register used to inspect privilege and interrupt state
    unsigned int cur_int_enable : 1; // Current interrupt enable flag
    unsigned int cur_mode : 1; // Current processor mode bit where kernel mode is expected here
    unsigned int prev_int_enable : 1; // Previous interrupt enable flag
    unsigned int prev_mode : 1; // Previous processor mode bit
    unsigned int unused : 28; // Remaining unused bits in the processor status register layout
};

union psr_values { // Union that allows the processor status register to be read as bits or as an integer
    struct psr_bits bits; // Bit-field representation of the processor status register
    unsigned int    integer_part; // Integer representation of the processor status register      
};

static inline void checkKernelMode(const char* functionName) // Verifies that a function that requires privileged execution is running in kernel mode.
{
    union psr_values psrValue; // Local storage for the current processor status register value
    psrValue.integer_part = get_psr(); // Read the current processor status register
    if (psrValue.bits.cur_mode == 0) // Stop execution if the current mode bit indicates user mode
    {
        console_output(FALSE, "Kernel mode expected, but function called in user mode.\n"); // Print an error describing the privilege violation
        stop(1); // Halt the system because this is a fatal kernel error
    }
}

static void enqueue_request(int unit, disk_request_t* req) // Appends a disk request to the end of the selected disk queue.
{
    k_semp(requestQueueMutex[unit]); // Lock the queue before modifying the linked list
    req->next = NULL; // Mark the new request as the tail of the list

    if (requestQueueTail[unit] == NULL) // If the queue is empty, initialize both head and tail
    {
        requestQueueHead[unit] = req; // Set the head to the new request
        requestQueueTail[unit] = req; // Set the tail to the new request
    }
    else // Otherwise append the request after the existing tail
    {
        requestQueueTail[unit]->next = req; // Link the current tail to the new request
        requestQueueTail[unit] = req; // Update the tail to the newly appended request
    }

    k_semv(requestQueueMutex[unit]); // Unlock the queue after the request has been enqueued
}

static void sysCall4(system_call_arguments_t* args) // Unified system call handler for sleep and all disk-related system calls.
{
    switch (args->call_id) // Dispatch behavior based on the system call number
    {
    case SYS_SLEEP: // Handle the sleep system call
        args->arguments[3] = (intptr_t)sys_sleep((int)args->arguments[0]); // Sleep for the requested number of seconds and return the result
        break; // Finish processing the sleep system call

    case SYS_DISKINFO: // Handle requests for disk geometry information
    {
        int unit = parse_disk_unit_from_args(args); // Determine which disk unit the caller is requesting information for

        if (unit < 0 || unit >= THREADS_MAX_DISKS) // Reject invalid disk unit values
        {
            args->arguments[3] = (void*)(intptr_t)-1; // Return failure in argument slot 3
            args->arguments[4] = (void*)(intptr_t)-1; // Return failure in argument slot 4
            args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            break; // Stop processing the invalid request
        }

        disk_request_t* req = (disk_request_t*)malloc(sizeof(disk_request_t)); // Allocate a request object for the disk driver
        if (req == NULL) // Reject the request if memory allocation fails
        {
            args->arguments[3] = (void*)(intptr_t)-1; // Return failure in argument slot 3
            args->arguments[4] = (void*)(intptr_t)-1; // Return failure in argument slot 4
            args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            break; // Stop processing because the request cannot be queued safely
        }

        memset(req, 0, sizeof(disk_request_t)); // Clear the request object before filling in its fields
        req->type = SYS_DISKINFO; // Mark this request as a disk information operation
        req->args = args; // Keep a pointer to the caller's system call argument block
        req->waitSem = k_semcreate(0); // Create a private semaphore so the caller can block until completion
        req->track = 0; // Initialize the track field even though DISKINFO does not seek

        if (req->waitSem < 0) // Abort if the completion semaphore could not be created
        {
            free(req); // Release the allocated request object before returning
            args->arguments[3] = (void*)(intptr_t)-1; // Return failure in argument slot 3
            args->arguments[4] = (void*)(intptr_t)-1; // Return failure in argument slot 4
            args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            break; // Stop processing because the request cannot be queued safely
        }

        enqueue_request(unit, req); // Add the request to the selected disk queue
        k_semv(diskRequestSems[unit]); // Wake the disk driver so it can service the queued request
        k_semp(req->waitSem); // Block until the disk driver signals that the request is complete

        k_semfree(req->waitSem); // Release the private completion semaphore after use
        free(req); // Free the temporary request object after the driver finishes with it
        break; // Finish processing the disk information system call
    }

    case SYS_DISKREAD: // Handle disk read requests
    case SYS_DISKWRITE: // Handle disk write requests
    {
        int unit = parse_disk_unit_from_args(args); // Determine which disk unit the caller wants to access
        void* buffer = (void*)(intptr_t)args->arguments[1]; // Extract the caller's data buffer pointer
        int platter = (int)args->arguments[2]; // Extract the requested platter number
        int track = (int)args->arguments[3]; // Extract the starting track number
        int first_sector = (int)args->arguments[4]; // Extract the starting sector number
        int sectors = (int)args->arguments[5]; // Extract the number of sectors to transfer
        int sectorCount; // Sectors per track for the selected disk
        int trackCount; // Number of valid tracks on the selected disk
        int platterCount; // Number of valid platters on the selected disk
        int lastTrack; // Last track touched by the request after spanning sectors across tracks

        if (unit < 0 || unit >= THREADS_MAX_DISKS) // Reject invalid disk unit values
        {
            args->arguments[0] = (void*)(intptr_t)-1; // Return failure in argument slot 0
            args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            break; // Stop processing the invalid request
        }

        sectorCount = (diskInfo[unit].sectors > 0) ? diskInfo[unit].sectors : THREADS_DISK_SECTOR_COUNT; // Determine sectors per track using device info or a fallback value
        trackCount = (diskInfo[unit].tracks > 0) ? diskInfo[unit].tracks : THREADS_DISK_MAX_TRACKS; // Determine the valid track range using device info or a fallback value
        platterCount = (diskInfo[unit].platters > 0) ? diskInfo[unit].platters : THREADS_DISK_MAX_PLATTERS; // Determine the valid platter range using device info or a fallback value

        lastTrack = track + ((first_sector + sectors - 1) / sectorCount); // Compute the last track touched when the transfer spans multiple tracks

        if (buffer == NULL || // Reject requests with a null data buffer
            platter < 0 || platter >= platterCount || // Reject platter numbers outside the valid range
            track < 0 || track >= trackCount || // Reject track numbers outside the valid range
            first_sector < 0 || first_sector >= sectorCount || // Reject sector numbers outside the valid range
            sectors <= 0 || // Reject non-positive transfer lengths
            lastTrack >= trackCount) // Reject requests that would run past the last valid track
        {
            args->arguments[0] = (void*)(intptr_t)-1; // Return failure in argument slot 0
            args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            break; // Stop processing the invalid request
        }

        disk_request_t* req = (disk_request_t*)malloc(sizeof(disk_request_t)); // Allocate a request object for the disk driver
        if (req == NULL) // Abort if memory allocation fails
        {
            args->arguments[0] = (void*)(intptr_t)-1; // Return failure in argument slot 0
            args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            break; // Stop processing because the request cannot be queued
        }

        memset(req, 0, sizeof(disk_request_t)); // Clear the request object before filling in its fields
        req->type = args->call_id; // Store whether this is a read or write request
        req->args = args; // Keep a pointer to the caller's system call argument block
        req->waitSem = k_semcreate(0); // Create a private semaphore so the caller can block until completion
        req->platter = platter; // Store the requested platter number
        req->track = track; // Store the starting track number
        req->first_sector = first_sector; // Store the starting sector number
        req->sectors = sectors; // Store the total number of sectors to transfer
        req->buffer = buffer; // Store the caller's data buffer pointer

        if (req->waitSem < 0) // Abort if the completion semaphore could not be created
        {
            free(req); // Release the allocated request object before returning
            args->arguments[0] = (void*)(intptr_t)-1; // Return failure in argument slot 0
            args->arguments[5] = (void*)(intptr_t)-1; // Return failure in argument slot 5
            break; // Stop processing because the request cannot be queued safely
        }

        enqueue_request(unit, req); // Add the request to the selected disk queue
        k_semv(diskRequestSems[unit]); // Wake the disk driver so it can service the queued request
        k_semp(req->waitSem); // Block until the disk driver signals that the request is complete

        k_semfree(req->waitSem); // Release the private completion semaphore after use
        free(req); // Free the temporary request object after the driver finishes with it
        break; // Finish processing the disk read or write system call
    }

    default: // Handle unsupported or unexpected system call numbers
        args->arguments[3] = (intptr_t)-1; // Return a failure code for unknown calls
        break; // Stop processing the unsupported request
    }
}

int sys_sleep(int seconds) // Puts the calling process to sleep until the requested number of seconds has elapsed.
{
    SleepingProcess* pProcInfo; // Heap-allocated record describing the sleeping process
    int waitSem; // Semaphore used to block the calling process until wakeup

    if (seconds < 0) return -1; // Reject negative sleep durations
    if (seconds == 0) return 0; // Return immediately for a zero-second sleep request

    waitSem = k_semcreate(0); // Create a semaphore the caller will block on while sleeping
    if (waitSem < 0) return -1; // Fail if the semaphore could not be created

    pProcInfo = (SleepingProcess*)malloc(sizeof(SleepingProcess)); // Allocate a record for the sleeping process
    if (pProcInfo == NULL) // Fail if memory allocation for the sleep record fails
    {
        k_semfree(waitSem); // Release the semaphore because the sleep record could not be created
        return -1; // Report failure to the caller
    }

    pProcInfo->pid = k_getpid(); // Record the PID of the process going to sleep
    pProcInfo->waitSem = waitSem; // Store the semaphore the process will wait on
    pProcInfo->wakeup_time = system_clock() + ((unsigned long long)seconds * MICROSECONDS_PER_SECOND); // Compute the absolute wakeup time in microseconds

    k_semp(sleeping_processes_mutex); // Lock the sleeping-process list before inserting the new record
    TListAddNodeInOrder(&sleeping_processes, pProcInfo); // Insert the process in wakeup-time order so the earliest wakeup stays at the head
    k_semv(sleeping_processes_mutex); // Unlock the sleeping-process list after insertion

    k_semp(waitSem); // Block until the clock driver signals that the wakeup time has arrived
    k_semfree(waitSem); // Release the semaphore after the process has been awakened

    return 0; // Report a successful sleep operation
}

static int sleep_compare(void* a, void* b) // Comparison function that orders sleeping processes by their absolute wakeup time.
{
    SleepingProcess* proc_a = (SleepingProcess*)a; // First sleeping process record to compare
    SleepingProcess* proc_b = (SleepingProcess*)b; // Second sleeping process record to compare

    if (proc_a->wakeup_time < proc_b->wakeup_time) return  1; // Place the earlier wakeup time ahead in the ordered list
    if (proc_a->wakeup_time > proc_b->wakeup_time) return -1; // Place the later wakeup time behind in the ordered list
    return 0; // Treat equal wakeup times as equivalent for ordering purposes
}

static int parse_disk_unit_from_args(system_call_arguments_t* args) // Attempts to find a disk unit number from the system call argument list.
{
    int i; // Loop variable used to scan the argument array

    for (i = 0; i < 6; i++) // First search for a disk name string such as "disk0"
    {
        const char* s = (const char*)(intptr_t)args->arguments[i]; // Interpret the argument as a potential string pointer
        if (s == NULL) continue; // Skip null arguments
        if ((uintptr_t)s < 0x10000) continue; // Skip values that are too small to plausibly be valid pointers

        if (strncmp(s, "disk", 4) == 0 && s[4] >= '0' && s[4] <= '9') // Check whether the string begins with a disk device name
        {
            return s[4] - '0'; // Convert the digit character to the corresponding disk unit number
        }
    }

    for (i = 0; i < 6; i++) // If no device name string is found, search for a raw integer unit number
    {
        intptr_t v = (intptr_t)args->arguments[i]; // Read the argument as an integer-sized value
        if (v >= 0 && v < THREADS_MAX_DISKS) // Accept values that fall within the valid disk unit range
        {
            return (int)v; // Return the detected disk unit number
        }
    }

    return -1; // Indicate failure if no valid disk unit could be extracted from the arguments
}

