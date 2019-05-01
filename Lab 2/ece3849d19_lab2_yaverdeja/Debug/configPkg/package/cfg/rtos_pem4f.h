/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-B06
 */

#include <xdc/std.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>
extern const ti_sysbios_family_arm_m3_Hwi_Handle adc_hwi;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle buttonTask;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle clock0;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semButtons;

#include <ti/sysbios/knl/Mailbox.h>
extern const ti_sysbios_knl_Mailbox_Handle mailbox0;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle userInputTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle displayTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle waveformTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle processingTask;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semWaveform;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semDisplay;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semProcessing;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle sem_cs;

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

