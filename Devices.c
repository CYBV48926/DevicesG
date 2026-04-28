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

typedef struct
{
    TListNode          listNode;
    int                pid;
    int                waitSem;
    unsigned long long wakeup_time;
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

// ==========================================
// SSTF QUEUE STRUCTURES AND GLOBALS
// ==========================================
typedef struct disk_request
{
    int                      type;         // SYS_DISKREAD, SYS_DISKWRITE, SYS_DISKINFO
    system_call_arguments_t* args;         // original system call arguments
    int                      waitSem;      // semaphore to wake the caller
    int                      track;        // target track (for SSTF seek)
    int                      first_sector; // first sector to read/write
    int                      sectors;      // number of sectors
    void*                    buffer;       // data buffer
    int*                     status_ptr;   // caller's DiskRead/DiskWrite status out pointer
    struct disk_request*     next;
} disk_request_t;

static disk_request_t* requestQueueHead[THREADS_MAX_DISKS];
static disk_request_t* requestQueueTail[THREADS_MAX_DISKS];
static int             requestQueueMutex[THREADS_MAX_DISKS];

// Current arm position per disk (for SSTF)
static int currentTrack[THREADS_MAX_DISKS];

int    sys_sleep(int seconds);
static inline void checkKernelMode(const char* functionName);
extern int DevicesEntryPoint(char*);

// ==========================================
// Helper: absolute value for int
// ==========================================
static int int_abs(int x) { return x < 0 ? -x : x; }

int SystemCallsEntryPoint(char* arg)
{
    char buf[25];
    char name[128];
    int  i;
    int  clockPID = 0;
    int  diskPids[THREADS_MAX_DISKS];
    int  status;

    checkKernelMode(__func__);

    systemCallVector[SYS_SLEEP]     = sysCall4;
    systemCallVector[SYS_DISKREAD]  = sysCall4;
    systemCallVector[SYS_DISKWRITE] = sysCall4;
    systemCallVector[SYS_DISKINFO]  = sysCall4;

    TListInitialize(&sleeping_processes, offsetof(SleepingProcess, listNode), sleep_compare);
    sleeping_processes_mutex = k_semcreate(1);

    if (sleeping_processes_mutex < 0)
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

    /* Initialize Disk Drivers, SSTF queues, and arm positions */
    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
        diskRequestSems[i]    = k_semcreate(0);
        requestQueueMutex[i]  = k_semcreate(1);
        requestQueueHead[i]   = NULL;
        requestQueueTail[i]   = NULL;
        currentTrack[i]       = 0; // arm starts at track 0

        sprintf(buf, "%d", i);
        sprintf(name, "DiskDriver%d", i);
        diskPids[i] = k_spawn(name, DiskDriver, buf, THREADS_MIN_STACK_SIZE * 4, HIGHEST_PRIORITY);
        if (diskPids[i] < 0)
        {
            console_output(TRUE, "start3(): Can't create disk driver %d\n", i);
            stop(1);
        }
    }

    int devicesPid = sys_spawn("DevicesEntryPoint", DevicesEntryPoint, NULL, 8 * THREADS_MIN_STACK_SIZE, 3);
    sys_wait(&status);

    k_kill(clockPID, 15);

    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
        k_kill(diskPids[i], 15);
        k_semv(diskRequestSems[i]);
    }

    for (i = 0; i < 1 + THREADS_MAX_DISKS; i++)
    {
        k_wait(&status);
    }

    return 0;
}

static int ClockDriver(char* arg)
{
    int result;
    int status;
    set_psr(get_psr() | PSR_INTERRUPTS);

    while (!signaled())
    {
        result = wait_device("clock", &status);
        if (result != 0) return 0;

        k_semp(sleeping_processes_mutex);
        {
            unsigned long long current_time = system_clock();
            while (sleeping_processes.count > 0)
            {
                SleepingProcess* pHead = (SleepingProcess*)sleeping_processes.pHead;
                if (pHead->wakeup_time > current_time) break;

                SleepingProcess* pWake = (SleepingProcess*)TListPopNode(&sleeping_processes);

                k_semv(pWake->waitSem);
                free(pWake);
            }
        }
        k_semv(sleeping_processes_mutex);
    }
    return 0;
}

// ==========================================
// SSTF: Select the request whose track is
// closest to the current arm position.
// Must be called while holding requestQueueMutex.
// ==========================================
static disk_request_t* sstf_dequeue(int unit)
{
    if (requestQueueHead[unit] == NULL)
        return NULL;

    // Find the node with the shortest seek distance
    disk_request_t* best_prev = NULL;
    disk_request_t* best      = requestQueueHead[unit];
    int             best_dist = int_abs(best->track - currentTrack[unit]);

    disk_request_t* prev = requestQueueHead[unit];
    disk_request_t* cur  = prev->next;

    while (cur != NULL)
    {
        int dist = int_abs(cur->track - currentTrack[unit]);
        if (dist < best_dist)
        {
            best_dist = dist;
            best      = cur;
            best_prev = prev;
        }
        prev = cur;
        cur  = cur->next;
    }

    // Remove best from the list
    if (best_prev == NULL)
    {
        // best is the head
        requestQueueHead[unit] = best->next;
    }
    else
    {
        best_prev->next = best->next;
    }

    if (best->next == NULL)
    {
        // best was the tail; find new tail
        if (requestQueueHead[unit] == NULL)
        {
            requestQueueTail[unit] = NULL;
        }
        else
        {
            disk_request_t* t = requestQueueHead[unit];
            while (t->next != NULL) t = t->next;
            requestQueueTail[unit] = t;
        }
    }

    best->next = NULL;
    return best;
}

static int DiskDriver(char* arg)
{
    int                   unit = atoi(arg);
    int                   status;
    char                  devName[16];
    device_control_block_t devRequest;

    sprintf(devName, "disk%d", unit);
    set_psr(get_psr() | PSR_INTERRUPTS);

    // Retrieve native disk geometry from hardware
    memset(&devRequest, 0, sizeof(devRequest));
    devRequest.command     = DISK_INFO;
    devRequest.output_data = &diskInfo[unit];
    device_control(devName, devRequest);
    wait_device(devName, &status);

    while (!signaled())
    {
        k_semp(diskRequestSems[unit]); // Wait for a request

        if (signaled()) break;

        
		disk_request_t* req = NULL; //dequeue the next request with SSTF scheduling

		k_semp(requestQueueMutex[unit]); // Lock the queue while we select the next request to process
		req = sstf_dequeue(unit); // Dequeue the next APPLICABLE request using SSTF scheduling
		k_semv(requestQueueMutex[unit]); // Unlock the queue after we've selected the request to process

		if (req == NULL) continue;// No request to process (shouldn't happen since we were signaled), but check just in case

		if (req->type == SYS_DISKINFO) // Provide disk geometry information to the caller
        {
			int* sectorSize = (int*)(intptr_t)req->args->arguments[1];// Sector size is fixed at 512 bytes for this system
			int* sectorCount = (int*)(intptr_t)req->args->arguments[2];// Sectors per track from diskInfo
			int* trackCount = (int*)(intptr_t)req->args->arguments[3];// Tracks from diskInfo
			int* platterCount = (int*)(intptr_t)req->args->arguments[4];// Platters from diskInfo

			if (sectorSize)   *sectorSize = 512; // Fixed sector size for this system
			if (sectorCount)  *sectorCount = diskInfo[unit].sectors; // Sectors per track from diskInfo
			if (trackCount)   *trackCount = diskInfo[unit].tracks; // Tracks from diskInfo
			if (platterCount) *platterCount = diskInfo[unit].platters;// Platters from diskInfo

            req->args->arguments[5] = 0; 
        }
        else if (req->type == SYS_DISKREAD || req->type == SYS_DISKWRITE)
        {
			memset(&devRequest, 0, sizeof(devRequest)); // Clear the control block before setting fields
			devRequest.command = (req->type == SYS_DISKREAD) ? DISK_READ : DISK_WRITE; // Set command based on request type
			devRequest.control1 = (uint8_t)req->track; // Track number to seek to
			devRequest.control2 = (uint8_t)req->first_sector; // First sector number on the track
			devRequest.data_length = (uint32_t)req->sectors; // Number of sectors to read/write

            if (req->type == SYS_DISKREAD)
            {
				devRequest.output_data = req->buffer;// For reads, the buffer is output data
            }
            else
            {
				devRequest.input_data = req->buffer; // For writes, the buffer is input data
            }

            status = 0;
            device_control(devName, devRequest);
            wait_device(devName, &status);

            currentTrack[unit] = req->track;

            if (req->status_ptr != NULL)
            {
                *(req->status_ptr) = status;   // <-- critical fix
            }

            req->args->arguments[5] = (intptr_t)0; // syscall return code
        }

        // Wake the waiting user thread
        k_semv(req->waitSem);
    }

    return 0;
}

struct psr_bits {
	unsigned int cur_int_enable : 1;// bit 0: current interrupt enable
	unsigned int cur_mode : 1;// bit 1: current mode (0 = user, 1 = kernel)
	unsigned int prev_int_enable : 1;// bit 2: previous interrupt enable
	unsigned int prev_mode : 1;// bit 3: previous mode (0 = user, 1 = kernel)
	unsigned int unused : 28; // bits 4-31: unused
};

union psr_values {
    struct psr_bits bits;
    unsigned int    integer_part;
};

static inline void checkKernelMode(const char* functionName)
{
    union psr_values psrValue;
    psrValue.integer_part = get_psr();
    if (psrValue.bits.cur_mode == 0)
    {
        console_output(FALSE, "Kernel mode expected, but function called in user mode.\n");
        stop(1);
    }
}

static void enqueue_request(int unit, disk_request_t* req) // Enqueue a disk request at the tail of the queue for the specified disk unit
{
	k_semp(requestQueueMutex[unit]);// Lock the queue while we modify it
	req->next = NULL; // New request will be at the tail, so next is NULL
	if (requestQueueTail[unit] == NULL) // Queue is empty
    {
		requestQueueHead[unit] = req; // New request is now the head
		requestQueueTail[unit] = req; // New request is also the tail since it's the only request in the queue
    }
    else
    {
        requestQueueTail[unit]->next = req;
        requestQueueTail[unit]       = req; // New request is now the tail
    }
	k_semv(requestQueueMutex[unit]); // Unlock the queue after modification
}

static void sysCall4(system_call_arguments_t* args)
{
    switch (args->call_id)
    {
    case SYS_SLEEP:
        args->arguments[3] = (intptr_t)sys_sleep((int)args->arguments[0]);
        break;

    case SYS_DISKINFO:
    {
        int unit = (int)args->arguments[0];
        if (unit < 0 || unit >= THREADS_MAX_DISKS)
        {
            args->arguments[5] = (intptr_t)-1;
            break;
        }

		disk_request_t* req = (disk_request_t*)malloc(sizeof(disk_request_t)); // Allocate a request structure to pass to the disk driver
		if (req == NULL) { args->arguments[5] = (intptr_t)-1; break; } // Check for malloc failure

		memset(req, 0, sizeof(disk_request_t)); // Clear the request structure before setting fields
		req->type = SYS_DISKINFO; // Set the request type to DISKINFO
		req->args = args; // Pass the original system call arguments to the request so the disk driver can fill in the output values
		req->waitSem = k_semcreate(0); // Create a semaphore for the disk driver to signal when the request is complete
        req->track   = 0; // DISKINFO has no seek cost 

		enqueue_request(unit, req); // Enqueue the request to the appropriate disk's request queue
		k_semv(diskRequestSems[unit]); // Signal the disk driver that a new request is available
		k_semp(req->waitSem); // Wait for the disk driver to process the request and signal completion

        k_semfree(req->waitSem);
        free(req);
        break;
    }

    case SYS_DISKREAD: 
	case SYS_DISKWRITE: // For both read and write, we create a disk request with the appropriate type and parameters, enqueue it, and wait for completion
    { 
        int unit         = (int)args->arguments[0];
        int track        = (int)args->arguments[1];
        int first_sector = (int)args->arguments[2];
        int sectors      = (int)args->arguments[3];
        void* buffer     = (void*)(intptr_t)args->arguments[4];

        if (unit < 0 || unit >= THREADS_MAX_DISKS)
        {
            args->arguments[5] = (intptr_t)-1;
            break;
        }

        disk_request_t* req = (disk_request_t*)malloc(sizeof(disk_request_t));
        if (req == NULL) { args->arguments[5] = (intptr_t)-1; break; }

        memset(req, 0, sizeof(disk_request_t));
        req->type         = args->call_id;
        req->args         = args;
        req->waitSem      = k_semcreate(0);
        req->track        = track;
        req->first_sector = first_sector;
        req->sectors      = sectors;
        req->buffer       = buffer;

        enqueue_request(unit, req);
        k_semv(diskRequestSems[unit]);
        k_semp(req->waitSem);

        k_semfree(req->waitSem);
        free(req);
        break;
    }

    default:
        args->arguments[3] = (intptr_t)-1;
        break;
    }
}

int sys_sleep(int seconds)
{
	SleepingProcess* pProcInfo; // Structure to hold information about the sleeping process
	int              waitSem; // Semaphore that the process will wait on until it's time to wake up

	if (seconds < 0) return -1; // Invalid argument: negative sleep time
	if (seconds == 0) return 0; // No need to sleep if the time is 0

	waitSem = k_semcreate(0); // Create a semaphore that the process will wait on. It will be signaled by the clock driver when it's time to wake up.    
	if (waitSem < 0) return -1; // Check for semaphore creation failure

	pProcInfo = (SleepingProcess*)malloc(sizeof(SleepingProcess)); // Allocate a structure to hold the sleeping process's information. This will be added to the sleeping_processes list so the clock driver can wake it up at the right time.
	if (pProcInfo == NULL) // Check for malloc failure
    {
        k_semfree(waitSem);
        return -1;
    }

	pProcInfo->pid = k_getpid(); // Store the PID of the sleeping process so the clock driver knows which process to wake up
	pProcInfo->waitSem = waitSem; // Store the semaphore that the clock driver will signal to wake up this process
	pProcInfo->wakeup_time = system_clock() + ((unsigned long long)seconds * MICROSECONDS_PER_SECOND);// Calculate the absolute wakeup time in microseconds and store it in the structure. The clock driver will compare this against the current time to determine when to wake up the process.

	k_semp(sleeping_processes_mutex);// Lock the sleeping_processes list while we add the new sleeping process to it. The list is ordered by wakeup_time, so the clock driver can efficiently check which processes need to be woken up on each tick.
	TListAddNodeInOrder(&sleeping_processes, pProcInfo);// Add the new sleeping process to the list in order of wakeup time
	k_semv(sleeping_processes_mutex);// Unlock the sleeping_processes list after adding the new process

    k_semp(waitSem);
    k_semfree(waitSem);

    return 0;
}

static int sleep_compare(void* a, void* b)
{
    SleepingProcess* proc_a = (SleepingProcess*)a;
    SleepingProcess* proc_b = (SleepingProcess*)b;

    if (proc_a->wakeup_time < proc_b->wakeup_time) return  1;
    if (proc_a->wakeup_time > proc_b->wakeup_time) return -1;
    return 0;
}


