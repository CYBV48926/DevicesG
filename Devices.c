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

#define DISK_ARM_ALG   DISK_ARM_ALG_FCFS
#define MICROSECONDS_PER_SECOND 1000000
#define DISK_INFO 0x01

static TList sleeping_processes; // List of SleepingProcess, sorted by wakeup_time
static int sleeping_processes_mutex; // Semaphore to protect sleeping_processes list
static int ClockDriver(char*); // Entry point for the clock driver thread
static int DiskDriver(char*); // Entry point for the disk driver threads
static void sysCall4(system_call_arguments_t* args); // Common handler for system calls with 4 arguments (sleep, disk read/write/info)
static int sleep_compare(void* a, void* b); // Comparison function for sorting sleeping processes by wakeup_time

typedef struct
{
	TListNode listNode; // Must be first for TList compatibility
	int pid; //// PID of the sleeping process
	int waitSem; // Semaphore that the sleeping process is waiting on; signaled by clock driver when it's time to wake up
	unsigned long long wakeup_time; // Absolute time (in microseconds) when the process should be woken up; used for sorting in the sleep queue
} SleepingProcess; // Structure representing a sleeping process, stored in the sleeping_processes list

typedef struct
{
	int platters; // Number of platters in the disk
	int sectors;// Number of sectors per track
	int tracks; // Number of tracks per platter
	int disk; // Disk identifier (e.g., disk number)
} DiskInformation;

static DiskInformation diskInfo[THREADS_MAX_DISKS]; // Array to store disk information for each disk, populated by disk drivers at startup and used to respond to disk info system calls
static int diskRequestSems[THREADS_MAX_DISKS]; // Array of semaphores for each disk driver to wait on for incoming requests

int sys_sleep(int seconds); // System call handler for sleep; puts the calling process to sleep for the specified number of seconds
static inline void checkKernelMode(const char* functionName); // Helper function to check if the current execution context is in kernel mode; if not, it prints an error and stops the system              
extern int DevicesEntryPoint(char*); // Entry point for the first user-level process that will run the device tests

int SystemCallsEntryPoint(char* arg)
{
    char    buf[25];
    char    name[128];
    int     i;
    int     clockPID = 0;
    int     diskPids[THREADS_MAX_DISKS];
    int     status;

    checkKernelMode(__func__);

    /* Assign system call handlers */
    systemCallVector[SYS_SLEEP] = sysCall4;
    systemCallVector[SYS_DISKREAD] = sysCall4;
    systemCallVector[SYS_DISKWRITE] = sysCall4;
    systemCallVector[SYS_DISKINFO] = sysCall4;

    /* Initialize sleep queue + mutex */
	TListInitialize(&sleeping_processes, offsetof(SleepingProcess, listNode), sleep_compare); // Initialize the sleeping_processes list with the offset of the listNode within the SleepingProcess structure
	sleeping_processes_mutex = k_semcreate(1); // Binary semaphore to protect access to the sleeping_processes list

	if (sleeping_processes_mutex < 0) // Check if semaphore creation was successful
    {
        console_output(TRUE, "SystemCallsEntryPoint(): Can't create sleeping_processes_mutex\n");
        stop(1);
    }

    /* Create and start the clock driver */
    clockPID = k_spawn("Clock driver", ClockDriver, NULL, THREADS_MIN_STACK_SIZE, HIGHEST_PRIORITY);
    if (clockPID < 0)
    {
        console_output(TRUE, "start3(): Can't create clock driver\n");
        stop(1);
    }

    /* Create the disk drivers */
    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
        // Safe wakeup semaphore for each driver
        diskRequestSems[i] = k_semcreate(0);

        sprintf(buf, "%d", i);
        sprintf(name, "DiskDriver%d", i);
        diskPids[i] = k_spawn(name, DiskDriver, buf, THREADS_MIN_STACK_SIZE * 4, HIGHEST_PRIORITY);
        if (diskPids[i] < 0)
        {
            console_output(TRUE, "start3(): Can't create disk driver %d\n", i);
            stop(1);
        }
    }

    /* Create first user-level process and wait for it to finish */
    int devicesPid = sys_spawn("DevicesEntryPoint", DevicesEntryPoint, NULL, 8 * THREADS_MIN_STACK_SIZE, 3);

    // Use sys_wait to wait for the user space tests (DevicesTest01) to complete and terminate
    sys_wait(&status);

    // 1. Signal the active clock driver
    k_kill(clockPID, 15);

    // 2. Kill and forcefully wake up the idle disk drivers
    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
        k_kill(diskPids[i], 15);
        k_semv(diskRequestSems[i]); // Kick driver out of idle loop
    }

    // 3. Gracefully wait for all kernel drivers to exit and clean themselves up
    for (i = 0; i < 1 + THREADS_MAX_DISKS; i++)
    {
        k_wait(&status);
    }

    return 0; // Natural clean termination
}

static int ClockDriver(char* arg)
{
    int result;
    int status;

    set_psr(get_psr() | PSR_INTERRUPTS);

    while (!signaled())
    {
        result = wait_device("clock", &status);
        if (result != 0)
        {
            return 0;
        }

        k_semp(sleeping_processes_mutex);

        {
            unsigned long long current_time = system_clock();

            while (sleeping_processes.count > 0)
            {
                SleepingProcess* pHead = (SleepingProcess*)sleeping_processes.pHead;

                /* List is sorted, so if head is not ready, rest are also not ready */
                if (pHead->wakeup_time > current_time)
                {
                    break;
                }

                SleepingProcess* pWake = (SleepingProcess*)TListPopNode(&sleeping_processes);

                /* Wake up the process */
                k_semv(pWake->waitSem);
                free(pWake);
            }
        }

        k_semv(sleeping_processes_mutex);
    }

    return 0;
}

static int DiskDriver(char* arg)
{
    int unit = atoi(arg);
    int status;
    char devName[16];
    device_control_block_t devRequest;

    sprintf(devName, "disk%d", unit);
    set_psr(get_psr() | PSR_INTERRUPTS); 

    /* Read the disk info asynchronously during startup */
    memset(&devRequest, 0, sizeof(devRequest));
    devRequest.command = DISK_INFO;

    // THREADS populates platters, tracks, and sizes natively into the provided structure space
    devRequest.output_data = &diskInfo[unit];
    device_control(devName, devRequest);
    wait_device(devName, &status);

    while (!signaled())
    {
        /* Idle explicitly awaiting a request; harmlessly broken by OS shutdown k_kill -> k_semv */
        k_semp(diskRequestSems[unit]);

        if (signaled())
        {
            break;
        }
    }

    return 0;
}

struct psr_bits {
	unsigned int cur_int_enable : 1; // 1 if interrupts are enabled, 0 if disabled
	unsigned int cur_mode : 1; // 1 if in kernel mode, 0 if in user mode
	unsigned int prev_int_enable : 1; // Previous interrupt enable state before the last mode switch; used to restore interrupt state when switching back
	unsigned int prev_mode : 1; // Previous mode before the last mode switch; used to restore mode when switching back
	unsigned int unused : 28; // Unused bits in the PSR; should be set to 0
};

union psr_values {
	struct psr_bits bits; // Bitfield representation of the PSR for easy access to individual flags
	unsigned int integer_part; // Integer representation of the PSR for direct manipulation and comparison; the entire PSR can be read or written as a single integer value
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

static void sysCall4(system_call_arguments_t* args)
{
    switch (args->call_id)
    {
    case SYS_SLEEP:
		args->arguments[3] = (intptr_t)sys_sleep((int)args->arguments[0]); // arg0 = seconds, return value in arg3
        break;
    case SYS_DISKINFO:
    {
		int unit = (int)args->arguments[0]; // arg0 = disk unit number
		int* platters = (int*)args->arguments[1]; // arg1 = pointer to store number of platters
		int* sectors = (int*)args->arguments[2]; // arg2 = pointer to store number of sectors per track
		int* tracks = (int*)args->arguments[3]; // arg3 = pointer to store number of tracks per platter
		int* disk = (int*)args->arguments[4]; // arg4 = pointer to store disk identifier (not used in this implementation, set to 0)

		if (unit < 0 || unit >= THREADS_MAX_DISKS) // Validate disk unit number
        {
			args->arguments[5] = (intptr_t)-1; // arg5 = status code; set to -1 for invalid unit number
            break;
        }

		if (platters) *platters = diskInfo[unit].platters; // Write number of platters to caller's provided pointer
		if (sectors) *sectors = THREADS_DISK_SECTOR_COUNT; // Write number of sectors per track to caller's provided pointer; using constant since it's the same for all disks in this implementation
		if (tracks) *tracks = diskInfo[unit].tracks; // Write number of tracks per platter to caller's provided pointer
		if (disk) *disk = 0; // Write disk identifier to caller's provided pointer; not used in this implementation, so set to 0

        args->arguments[5] = (intptr_t)0;
        break;
    }
    case SYS_DISKREAD:
    case SYS_DISKWRITE:
        args->arguments[5] = (intptr_t)-1;
        break;
    default:
        args->arguments[3] = (intptr_t)-1;
        break;
    }
}

int sys_sleep(int seconds)
{
    SleepingProcess* pProcInfo;
    int waitSem;

    if (seconds < 0) return -1;
    if (seconds == 0) return 0;

    waitSem = k_semcreate(0);
    if (waitSem < 0) return -1;

    pProcInfo = (SleepingProcess*)malloc(sizeof(SleepingProcess));
    if (pProcInfo == NULL)
    {
        k_semfree(waitSem);
        return -1;
    }

    pProcInfo->pid = k_getpid();
    pProcInfo->waitSem = waitSem;
    pProcInfo->wakeup_time = system_clock() + ((unsigned long long)seconds * MICROSECONDS_PER_SECOND);

    k_semp(sleeping_processes_mutex);
    TListAddNodeInOrder(&sleeping_processes, pProcInfo);
    k_semv(sleeping_processes_mutex);

    // Block calling process
    k_semp(waitSem);

    // Clear out
    k_semfree(waitSem);

    return 0;
}

static int sleep_compare(void* a, void* b)
{
	SleepingProcess* proc_a = (SleepingProcess*)a; // Cast void pointers to SleepingProcess pointers for comparison
	SleepingProcess* proc_b = (SleepingProcess*)b; // Compare wakeup times to determine order in the sleeping_processes list; earlier wakeup time should come before later wakeup time

   
	if (proc_a->wakeup_time < proc_b->wakeup_time) return 1; // Return 1 if proc_a should come before proc_b (earlier wakeup time)
	if (proc_a->wakeup_time > proc_b->wakeup_time) return -1; // Return -1 if proc_a should come after proc_b (later wakeup time)
    return 0;
}
