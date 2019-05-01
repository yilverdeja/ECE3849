/**
 * main.c
 *
 * ECE 3849 Lab 1
 * Created on: April 1, 2019
 * Authors: Yil Verdeja (Box 349)
 *
 * Main program. Uses the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include <string.h>
#include <stdlib.h>
#include "sysctl_pll.h"         //Does this necessarily go here?
#include "driverlib/adc.h"      //Ditto
#include "inc/tm4c1294ncpdt.h"  //Ditto
#include "inc/hw_memmap.h"
#include "driverlib/timer.h"

uint32_t gSystemClock; // [Hz] system clock frequency
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", "1 V"};
const char * const gTimeScaleStr[] = {"100 ms", "50 ms", "20 ms", "10 ms", "5 ms", "2 ms", "1 ms", "500 us", "200 us", "100 us", "50 us", "20 us"};
const char * const gTriggerSlopeStr[] = {"Rising", "Falling"};

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;

int main(void)
{
    IntMasterDisable();

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    char tscale_str[50];   // time string buffer for time scale
    char vscale_str[50];   // time string buffer for voltage scale
    char tslope_str[50];   // time string buffer for trigger edge
    char cpu_str[50];   // time string buffer for cpu load
    char bpresses[10]; // holds fifo button presses
    uint32_t stateVperDiv = 3; // 4 states
    uint32_t y; // holds the converted ADC samples
    float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1}; // array of voltage scale per division

    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    ButtonInit();

    count_unloaded = cpu_load_count(); // get initial cpu load count

    IntMasterEnable();

    while (true) {
        trigger_value = zero_crossing_point(); // Dynamically finds the ADC_OFFSET, ~2055

        // CPU Load calculations
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load

        int i; // for loop iterations

        // determines fScale
        float fScale = (VIN_RANGE/(1 << ADC_BITS))*(PIXELS_PER_DIV/fVoltsPerDiv[stateVperDiv]);

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(&sContext, ClrWhite); // yellow text
		
        triggerSearch(); // searches for trigger
		
        if (fifo_get(bpresses)) {
            // read bpresses and change state based on bpresses buttons and whether it is being pressed currently
            for (i = 0; i < 10; i++){
                if (bpresses[i]==('u') && gButtons == 4){ // increment state
                    stateVperDiv = (++stateVperDiv) & 3;
                } else if (bpresses[i]==('t') && gButtons == 2){ //trigger
                    risingSlope = !risingSlope;
                } else if (bpresses[i]==('h') && gButtons == 1){ //time scale
                    stateTperDiv = (++stateTperDiv) % 12;
                }

            }
        }

        // draw grid in blue
        GrContextForegroundSet(&sContext, ClrBlue); // yellow text

        for (i = 1; i < 128; i+=21){
            GrLineDraw(&sContext, i, 0, i, 128);
            GrLineDraw(&sContext, 0, i, 128, i);
        }

        // draw center grid lines in dark blue
        GrContextForegroundSet(&sContext, ClrDarkBlue); // yellow text
        GrLineDraw(&sContext, 64, 0, 64, 128);
        GrLineDraw(&sContext, 0, 64, 128, 64);

        // draw waveform
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        int x;
        int y_old;
        for (x = 0; x < LCD_HORIZONTAL_MAX - 1; x++) {
            y = ((int)(LCD_VERTICAL_MAX/2) - (int)(fScale*(int)(trigger_samples[x] - trigger_value)));
            if (x!=0)
                GrLineDraw(&sContext, x-1, y_old, x, y);
            y_old = y;
        }

        // Time Scale, Voltage Scale, Trigger Slope and CPU Load
        GrContextForegroundSet(&sContext, ClrWhite); // white text
        snprintf(tscale_str, sizeof(tscale_str), gTimeScaleStr[stateTperDiv]); // convert time to string
        GrStringDraw(&sContext, tscale_str, /*length*/ -1, /*x*/ 7, /*y*/ 5, /*opaque*/ false);

        snprintf(vscale_str, sizeof(vscale_str), gVoltageScaleStr[stateVperDiv]);
        GrStringDraw(&sContext, vscale_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 - 20, /*y*/ 5, /*opaque*/ false);

        snprintf(tslope_str, sizeof(tslope_str), gTriggerSlopeStr[risingSlope]);
        GrStringDraw(&sContext, tslope_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 + 20, /*y*/ 5, /*opaque*/ false);

        snprintf(cpu_str, sizeof(cpu_str), "CPU Load: %.3f%c", cpu_load*100, 37);
        GrStringDraw(&sContext, cpu_str, /*length*/ -1, /*x*/ 5, /*y*/ LCD_VERTICAL_MAX-10, /*opaque*/ false);

        GrFlush(&sContext); // flush the frame buffer to the LCD

    }
}


