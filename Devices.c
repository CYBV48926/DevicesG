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

/* Set the disk arm scheduling algorithm.
 * See Devices.h for available constants (DISK_ARM_ALG_FCFS, DISK_ARM_ALG_SSTF, etc.).
 * You must implement FCFS and SSTF. Change this value to test each algorithm.
 * Submissions will be assessed with DISK_ARM_ALG_FCFS and DISK_ARM_ALG_SSTF. */
#define DISK_ARM_ALG   DISK_ARM_ALG_FCFS
#define MICROSECONDS_PER_SECOND 1000000 
#define DISK_INFO 0x01

static TList sleeping_processes;
static int sleeping_processes_mutex;    
static int ClockDriver(char*);
static int DiskDriver(char*);
static void sysCall4(system_call_arguments_t* args);
static int sleep_compare(void* a, void* b);

typedef struct devices_proc
{
    struct devices_proc* pNext;
    struct devices_proc* pPrev;
    int pid;
    int waitSem;                    // Semaphore for blocking/unblocking
    long long wakeup_time;
    void* buffer;
    int track;
    int sector;
    int platter;
    int sectorCount;
    int operation;
    int status;
} DevicesProcess;

typedef struct
{
    TListNode listNode;
    int pid;
    int waitSem;                    // Add semaphore to sleeping process
    long long wakeup_time;
} SleepingProcess;

typedef struct
{
    int tracks;
    int platters;
    char deviceName[THREADS_MAX_DEVICE_NAME];
} DiskInformation;

static DevicesProcess devicesProcs[MAXPROC];
static DiskInformation diskInfo[THREADS_MAX_DISKS];
int  sys_sleep(int seconds);
static inline void checkKernelMode(const char* functionName);
extern int DevicesEntryPoint(char*);

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

    /* Initialize sleep queue + mutex */
    TListInitialize(&sleeping_processes, offsetof(SleepingProcess, listNode), sleep_compare);
    sleeping_processes_mutex = k_semcreate(1);
    if (sleeping_processes_mutex < 0)
    {
        console_output(TRUE, "SystemCallsEntryPoint(): Can't create sleeping_processes_mutex\n");
        stop(1);
    }

    /* Initialize the process table */
    for (i = 0; i < MAXPROC; ++i)
    {
        devicesProcs[i].pid = -1;
        devicesProcs[i].waitSem = -1;
        devicesProcs[i].pNext = NULL;
        devicesProcs[i].pPrev = NULL;
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
    
    // Use sys_wait to wait for DevicesEntryPoint to terminate
    sys_wait(&status);

    // Now get rid of the various drivers by notifying them to terminate

    // 1. Signal the clock driver
    k_kill(clockPID, 15); // Sending a SIGTERM (typically 15) to break the !signaled() loop

    // 2. Terminate the disk drivers 
    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
        k_kill(diskPids[i], 15); 
    }

    // Use k_wait to wait for all the drivers to exit cleanly
    k_wait(&clockPID);
    
    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
        k_wait(&diskPids[i]);
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
        if (result != 0)
        {
            return 0;
        }

        k_semp(sleeping_processes_mutex);

        {
            long long current_time = system_clock();

            while (sleeping_processes.count > 0)
            {
                SleepingProcess* pHead = (SleepingProcess*)sleeping_processes.pHead;
                
                /* List is sorted, so if head is not ready, rest are also not ready */
                if (pHead->wakeup_time > current_time)
                {
                    break;
                }

                SleepingProcess* pWake = (SleepingProcess*)TListPopNode(&sleeping_processes);
                
                /* Wake up the process by signaling its semaphore */
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
    int currentTrack = 0;
    int status;
    char devName[16];
    device_control_block_t devRequest;

    /* Construct the correct device name based on the unit number (e.g. "disk0") */
    sprintf(devName, "disk%d", unit);

    set_psr(get_psr() | PSR_INTERRUPTS);

    /* Operating loop */
    while (!signaled())
    {
        /* Block on the actual device interrupt instead of a dummy semaphore! 
           This prevents check_deadlock from falsely halting the simulator */
        int result = wait_device(devName, &status);
        
        /* result != 0 means the device wait was aborted by a signal (k_kill) */
        if (result != 0)
        {
            break; 
        }
    }
    
    return 0;
}


struct psr_bits {
    unsigned int cur_int_enable : 1;
    unsigned int cur_mode : 1;
    unsigned int prev_int_enable : 1;
    unsigned int prev_mode : 1;
    unsigned int unused : 28;
};

union psr_values {
    struct psr_bits bits;
    unsigned int integer_part;
};

/*****************************************************************************
   Name - checkKernelMode
   Purpose - Checks the PSR for kernel mode and stops if in user mode
   Parameters -
   Returns -
   Side Effects - Will stop if not in kernel mode
****************************************************************************/
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
        args->arguments[3] = (intptr_t)sys_sleep((int)args->arguments[0]);
        break;
    case SYS_DISKREAD:
        args->arguments[3] = (intptr_t)-1;
        break;
    case SYS_DISKWRITE:
        args->arguments[3] = (intptr_t)-1;
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

    if (seconds < 0)
    {
        return -1;
    }

    if (seconds == 0)
    {
        return 0;
    }

    /* Create a semaphore for this process to block on */
    waitSem = k_semcreate(0);
    if (waitSem < 0)
    {
        return -1;
    }

    pProcInfo = (SleepingProcess*)malloc(sizeof(SleepingProcess));
    if (pProcInfo == NULL)
    {
        k_semfree(waitSem);
        return -1;
    }

    pProcInfo->pid = k_getpid();
    pProcInfo->waitSem = waitSem;
    pProcInfo->wakeup_time = system_clock() + ((long long)seconds * MICROSECONDS_PER_SECOND);

    k_semp(sleeping_processes_mutex);
    TListAddNodeInOrder(&sleeping_processes, pProcInfo);
    k_semv(sleeping_processes_mutex);

    /* Block on the semaphore - ClockDriver will signal it */
    k_semp(waitSem);
    
    /* Clean up the semaphore after waking up */
    k_semfree(waitSem);

    return 0;
}

static int sleep_compare(void* a, void* b)
{
    SleepingProcess* proc_a = (SleepingProcess*)a;
    SleepingProcess* proc_b = (SleepingProcess*)b;

    if (proc_a->wakeup_time < proc_b->wakeup_time) return -1;
    if (proc_a->wakeup_time > proc_b->wakeup_time) return 1;
    return 0;
}