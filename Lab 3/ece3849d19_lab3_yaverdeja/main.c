/* main.c
 * ECE 3849 Lab 3
 * Created on: April 30, 2019
 * Authors: Yil Verdeja (Box 349)
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"

#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include <string.h>
#include <stdlib.h>
#include "sysctl_pll.h"         //Does this necessarily go here?
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"      //Ditto
#include "driverlib/udma.h"
#include "inc/tm4c1294ncpdt.h"  //Ditto
#include "math.h"
#include "inc/hw_memmap.h"

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", "1 V"};
const char * const gTriggerSlopeStr[] = {"Rising", "Falling"};
tContext sContext;

// CPU LOAD count
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    ButtonInit();

    count_unloaded = cpu_load_count(); // get initial cpu load count

    /* Start BIOS */
    BIOS_start();
    return (0);
}

void displayTask_func(UArg arg1, UArg arg2) { // pri 4
    char tscale_str[50];   // time string buffer for time scale
    char vscale_str[50];   // time string buffer for voltage scale
    char tslope_str[50];   // time string buffer for trigger edge
    char frequency_str[50];   // time string buffer for frequency edge
    while(true){
        Semaphore_pend(semDisplay, BIOS_WAIT_FOREVER); // from user input

        Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load
        Semaphore_post(sem_cs); // protect critical section

        tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1,
                                     GrContextDpyHeightGet(&sContext)-1}; // full-screen rectangle

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(&sContext, ClrWhite); // yellow text

        // draw grid in blue
        GrContextForegroundSet(&sContext, ClrBlue); // yellow text
        int i;
        for (i = 1; i < 128; i+=21){
            GrLineDraw(&sContext, i, 0, i, 128);
            GrLineDraw(&sContext, 0, i, 128, i);
        }

        // draw center grid lines in dark blue
        GrContextForegroundSet(&sContext, ClrDarkBlue); // blue
        if (spectrumMode){
            GrLineDraw(&sContext, 0, 22, 128, 22);
        } else {
            GrLineDraw(&sContext, 64, 0, 64, 128);
            GrLineDraw(&sContext, 0, 64, 128, 64);
        }

        // draw waveform
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        int x;
        int y_old;
        Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
        for (x = 0; x < LCD_HORIZONTAL_MAX - 1; x++) {
            if (x!=0)
                GrLineDraw(&sContext, x-1, y_old, x, processedWaveform[x]);
            y_old = processedWaveform[x];
        }
        Semaphore_post(sem_cs);

        // Time Scale, Voltage Scale, Trigger Slope and frequency
        GrContextForegroundSet(&sContext, ClrWhite); // yellow text
        if (spectrumMode){
            snprintf(tscale_str, sizeof(tscale_str), "20kHz"); // convert time scale to string
            snprintf(vscale_str, sizeof(vscale_str), "20dB"); // convert vscale to string
        } else {
            snprintf(tscale_str, sizeof(tscale_str), "20us"); // convert time scale to string
            snprintf(vscale_str, sizeof(vscale_str), gVoltageScaleStr[stateVperDiv]); // convert vscale to string
            snprintf(tslope_str, sizeof(tslope_str), gTriggerSlopeStr[risingSlope]); // convert slope to string
            snprintf(frequency_str, sizeof(frequency_str), "f = %6.3f Hz", avg_frequency); // convert frequency to string
            GrStringDraw(&sContext, tslope_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 + 20, /*y*/ 5, /*opaque*/ false);
            GrStringDraw(&sContext, frequency_str, /*length*/ -1, /*x*/ 7, /*y*/ 120, /*opaque*/ false);
        }

        GrStringDraw(&sContext, tscale_str, /*length*/ -1, /*x*/ 7, /*y*/ 5, /*opaque*/ false);
        GrStringDraw(&sContext, vscale_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 - 20, /*y*/ 5, /*opaque*/ false);

        GrFlush(&sContext); // flush the frame buffer to the LCD
        Semaphore_pend(semDisplay, BIOS_WAIT_FOREVER); // block again
    }
}
