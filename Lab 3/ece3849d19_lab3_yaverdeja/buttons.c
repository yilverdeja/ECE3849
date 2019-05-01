/*
 * buttons.c
 *
 * ECE 3849 Lab 3
 * Created on: April 30, 2019
 * Authors: Yil Verdeja (Box 349)
 *
 * ECE 3849 Lab 3 button, ADC, PWM, tasks, etc. handling
 */
#include <stdint.h>
#include <stdbool.h>

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
#include <ti/sysbios/gates/GateMutexPri.h>


#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "buttons.h"
#include "driverlib/udma.h"
#include "inc/tm4c1294ncpdt.h"
#include "Crystalfontz128x128_ST7735.h"
#include "math.h"

// ANALOG COMPARATOR
#include "driverlib/comp.h"
#include "driverlib/pin_map.h"

// PWM INIT
#include "driverlib/pwm.h"
//#include "inc/tm4c1294ncpdt.h"

// KISS FFT
#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

//ADC ISR
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;    // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // circular buffer
volatile uint16_t trigger_samples[ADC_TRIGGER_SIZE];
volatile uint16_t fft_samples[NFFT];
volatile int16_t processedWaveform[ADC_TRIGGER_SIZE];
volatile uint32_t gADCErrors;                       // number of missed ADC deadlines
volatile uint32_t trigger_value;

float fScale;

// DMA Setup
#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64]; // uDMA control table (global)

// public globals
volatile uint32_t gButtons = 0; // debounced button state, one per bit in the lowest bits
                                // button is pressed if its bit is 1, not pressed if 0
uint32_t gJoystick[2] = {0};    // joystick coordinates
uint32_t gADCSamplingRate;      // [Hz] actual ADC sampling rate
volatile bool risingSlope = true; // determines whether the slope is rising or falling
volatile bool spectrumMode = false; // determines the mode of the oscilloscope - True: Sine, False: Spectrum

// imported globals
extern uint32_t gSystemClock;   // [Hz] system clock frequency

// Volts per div states and constants
volatile uint32_t stateVperDiv = 3; // 4 states
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1}; // array of voltage scale per division

volatile bool gDMAPrimary = true; // is DMA occurring in the primary channel?

// CPU LOAD globals imported
extern uint32_t count_unloaded;
extern uint32_t count_loaded;
extern float cpu_load;

// Time Capture ISR
uint32_t prevCount = 0;
uint32_t timerPeriod = 0;
uint32_t multiPeriodInterval = 0;
uint32_t accumulatedPeriods = 0;
float avg_frequency;

// PWM INIT
#define PWM_PERIOD 258 // PWM period = 2^8 + 2 system clock cycles
uint32_t gPhase = 0; // phase accumulator
uint32_t gPhaseIncrement = 166215234; // phase increment for 18 kHz
#define PWM_WAVEFORM_INDEX_BITS 10
#define PWM_WAVEFORM_TABLE_SIZE (1 << PWM_WAVEFORM_INDEX_BITS)
uint8_t gPWMWaveformTable[PWM_WAVEFORM_TABLE_SIZE] = {0};

// initialize all button and joystick handling hardware
void ButtonInit(void)
{
    // initialize timer 3 in one-shot mode for polled timing
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, (gSystemClock)/100); // 1 sec interval

    // GPIO PJ0 and PJ1 = EK-TM4C1294XL buttons 1 and 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // analog input AIN13, at GPIO PD2 = BoosterPack Joystick HOR(X)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    // analog input AIN17, at GPIO PK1 = BoosterPack Joystick VER(Y)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    // GPIO PD4 = BoosterPack Joystick SEL
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4);

    // GPIO PH1 = BoosterPack Button 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);

    // GPIO PK6 = BoosterPack Button 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);


    // initialize ADC peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC Clock
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor); // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor); // only ADC0 has PLL clock divisor control
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // GPIO setup for analog input AIN3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    // initialize ADC0 sampling sequence
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);                             // Joystick HOR(X)
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);  // Joystick VER(Y)
    ADCSequenceEnable(ADC0_BASE, 0);

    // initialize ADC1 sampling sequence
    ADCSequenceDisable(ADC1_BASE, 0);      // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);    // specify the "Always" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END |ADC_CTL_CH3);// in the 0th step, sample channel 3 (AIN3)

    // ANALOG COMPARATOR CONFIGURE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);
    ComparatorRefSet(COMP_BASE, COMP_REF_1_65V);
    ComparatorConfigure(COMP_BASE, 1,
                        COMP_TRIG_NONE | COMP_INT_BOTH | // can also use COMP_INT_RISE
                        COMP_ASRCP_REF | COMP_OUTPUT_NORMAL);

    // configure GPIO for comparator input C1- at BoosterPack Connector #1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // GPIO PC_4
    GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_4);
    // configure GPIO for comparator output C1o at BoosterPack Connector #1 pin 15
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // GPIO PD_1
    GPIOPinTypeComparatorOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD1_C1O);

    // configure GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    // Count positive edges
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff); // use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff); // use maximum prescale value
    // CaptureA event interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Initialize DMA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);
    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                           UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                           (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                           UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                           (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

    ADCSequenceDMAEnable(ADC1_BASE, 0); // enable DMA for ADC1 sequence 0
    //ADCIntEnable(ADC1_BASE, 0);            // enable sequence 0 interrupt in the ADC1 peripheral
    ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt
    ADCSequenceEnable(ADC1_BASE, 0);       // enable the sequence.  it is now sampling

    // PWM
    // use M0PWM1, at GPIO PF1, which is BoosterPack Connector #1 pin 40
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1); // PF1 = M0PWM1
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA,
    GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_PERIOD/2); // initial 50% duty cycle
    PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true); // invert PWM output
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // enable PWM output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0); // enable PWM generator
    // enable PWM interrupt in the PWM peripheral
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);

    fillWaveformTable(); // Fills the waveform table using a for loop

}

// update the debounced button state gButtons
void ButtonDebounce(uint32_t buttons)
{
    int32_t i, mask;
    static int32_t state[BUTTON_COUNT]; // button state: 0 = released
                                        // BUTTON_PRESSED_STATE = pressed
                                        // in between = previous state
    for (i = 0; i < BUTTON_COUNT; i++) {
        mask = 1 << i;
        if (buttons & mask) {
            state[i] += BUTTON_STATE_INCREMENT;
            if (state[i] >= BUTTON_PRESSED_STATE) {
                state[i] = BUTTON_PRESSED_STATE;
                gButtons |= mask; // update debounced button state
            }
        }
        else {
            state[i] -= BUTTON_STATE_DECREMENT;
            if (state[i] <= 0) {
                state[i] = 0;
                gButtons &= ~mask;
            }
        }
    }
}

// sample joystick and convert to button presses
void ButtonReadJoystick(void)
{
    ADCProcessorTrigger(ADC0_BASE, 0);          // trigger the ADC sample sequence for Joystick X and Y
    while(!ADCIntStatus(ADC0_BASE, 0, false));  // wait until the sample sequence has completed
    ADCSequenceDataGet(ADC0_BASE, 0, gJoystick);// retrieve joystick data
    ADCIntClear(ADC0_BASE, 0);                  // clear ADC sequence interrupt flag

    // process joystick movements as button presses using hysteresis
    if (gJoystick[0] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 5; // joystick right in position 5
    if (gJoystick[0] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 5);

    if (gJoystick[0] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 6; // joystick left in position 6
    if (gJoystick[0] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 6);

    if (gJoystick[1] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 7; // joystick up in position 7
    if (gJoystick[1] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 7);

    if (gJoystick[1] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 8; // joystick down in position 8
    if (gJoystick[1] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 8);
}

// autorepeat button presses if a button is held long enough
uint32_t ButtonAutoRepeat(void)
{
    static int count[BUTTON_AND_JOYSTICK_COUNT] = {0}; // autorepeat counts
    int i;
    uint32_t mask;
    uint32_t presses = 0;
    for (i = 0; i < BUTTON_AND_JOYSTICK_COUNT; i++) {
        mask = 1 << i;
        if (gButtons & mask)
            count[i]++;     // increment count if button is held
        else
            count[i] = 0;   // reset count if button is let go
        if (count[i] >= BUTTON_AUTOREPEAT_INITIAL &&
                (count[i] - BUTTON_AUTOREPEAT_INITIAL) % BUTTON_AUTOREPEAT_NEXT == 0)
            presses |= mask;    // register a button press due to auto-repeat
    }
    return presses;
}

//Lab 1 ADC ISR Recommended Structure
void ADC_ISR(void){
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag
    // Check the primary DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)
        gDMAPrimary = false; // DMA is currently occurring in the alternate buffer
    }
    // Check the alternate DMA channel for end of transfer, and restart if needed.
    // Also set the gDMAPrimary global.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
        gDMAPrimary = true; // DMA is currently occurring in the alternate buffer
    }
    // The DMA channel may be disabled if the CPU is paused by the debugger.
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10); // re-enable the DMA channel
    }
}

int32_t getADCBufferIndex(void)
{
    int32_t index;
    IArg key;
    key = GateHwi_enter(gateHwi0);
    if (gDMAPrimary) { // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
    }
    else { // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
    }
    GateHwi_leave(gateHwi0, key);
    return index;
}

// Searches the trigger
void triggerSearch(void){
    int32_t trigger_index;
    int32_t i;

    trigger_index = getADCBufferIndex();

    // A local buffer retrieves 128 samples of the gADCBuffer from the trigger_index previously found
    for (i = 0; i < ADC_TRIGGER_SIZE; i++){
        trigger_samples[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger_index - (ADC_TRIGGER_SIZE - 1) + i)];
    }

}

// Returns the zero-crossing point of the ADC waveform by finding the max and min points and averaging them.
uint32_t zero_crossing_point(void){
    int max = 0;
    int min = 10000;

    int i;
    for (i = 0; i < ADC_BUFFER_SIZE; i++){
        if (gADCBuffer[i] > max){
            max = gADCBuffer[i];
        }

        if (gADCBuffer[i] < min){
            min = gADCBuffer[i];
        }
    }

    return (max+min)/2;
}

// Returns the cpu load count
uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

// Button Task to handle button presses in Mailbox Queue
void buttonTask_func(UArg arg1, UArg arg2) { // pri 2
    while(true){

        Semaphore_pend(semButtons, BIOS_WAIT_FOREVER); // from clock
        // read hardware button state
        uint32_t gpio_buttons =
                ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0) | // EK-TM4C1294XL buttons in positions 0 and 1
                (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1 | // BoosterPack button 1
                (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3 | // BoosterPack button 2
                ~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4); // BoosterPack buttons joystick select button

        uint32_t old_buttons = gButtons;    // save previous button state
        ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();      // autorepeat presses if a button is held long enough
        char button_char;

        if (presses & 2) {
            // trigger slope change
            button_char = 't';
            Mailbox_post(mailbox0, &button_char, TIMEOUT);
        }

        if (presses & 4) {
            // increment
            button_char = 'u';
            Mailbox_post(mailbox0, &button_char, TIMEOUT);
        }

        if (presses & 8) {
            // spectrum mode
            button_char = 's';
            Mailbox_post(mailbox0, &button_char, TIMEOUT);
        }
    }
}

// Waveform Task to handle waveform mode and creating waveform to print
void waveformTask_func(UArg arg1, UArg arg2) { // pri 1
    IntMasterEnable();
    while(true){
        Semaphore_pend(semWaveform, BIOS_WAIT_FOREVER); // from processing
        trigger_value = zero_crossing_point(); // Dynamically finds the ADC_OFFSET
        if (spectrumMode){
            int i;
            int buffer_ind = gADCBufferIndex;

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            for (i = 0; i < NFFT; i++){
                fft_samples[i] = gADCBuffer[ADC_BUFFER_WRAP(buffer_ind - NFFT + i)];
            }
            Semaphore_post(sem_cs);

        } else {
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            triggerSearch(); // searches for trigger
            Semaphore_post(sem_cs);
        }
        Semaphore_post(semProcessing); // to processing

    }
}

// Processing Task to correct spectrum wave and sine wave to LCD
void processingTask_func(UArg arg1, UArg arg2) { // pri 5
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    static float w[NFFT]; // window function
    for (i = 0; i < NFFT; i++) {
     // Blackman window
        w[i] = (0.42f - 0.5f * cosf(2*PI*i/(NFFT-1)) + 0.08f * cosf(4*PI*i/(NFFT-1)));
    }

    while(true){
        Semaphore_pend(semProcessing, BIOS_WAIT_FOREVER); // from waveform
        if (spectrumMode){

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            for (i = 0; i < NFFT; i++) { // generate an input waveform
             in[i].r = ((float)fft_samples[i] - trigger_value) * w[i]; // real part of waveform
             in[i].i = 0; // imaginary part of waveform
            }
            Semaphore_post(sem_cs);

            kiss_fft(cfg, in, out); // compute FFT

            // convert first 128 bins of out[] to dB for display
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            for (i = 0; i < ADC_TRIGGER_SIZE - 1; i++) {
                processedWaveform[i] = (int)roundf(128 - 10*log10f(out[i].r*out[i].r +out[i].i*out[i].i));
            }
            Semaphore_post(sem_cs);

        } else {
            fScale = (VIN_RANGE/(1 << ADC_BITS))*(PIXELS_PER_DIV/fVoltsPerDiv[stateVperDiv]); // determines fScale
            int i;

            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            for (i = 0; i < ADC_TRIGGER_SIZE - 1; i++) {
                processedWaveform[i] = ((int)(ADC_TRIGGER_SIZE/2)
                        - (int)roundf(fScale*(int)(trigger_samples[i] - trigger_value)));
            }
            Semaphore_post(sem_cs);
        }
        Semaphore_post(semWaveform); // to waveform
        Semaphore_post(semDisplay); // to display
    }
}

// signal button task periodically using a semaphore
void clock_func(UArg arg1){
    Semaphore_post(semButtons); // to buttons
}


// User input task to read button presses and take actions accordingly
void userInputTask_func(UArg arg1, UArg arg2) { // pri 3
    char bpresses[10]; // holds fifo button presses
    while(true){
        if (Mailbox_pend(mailbox0, &bpresses, TIMEOUT)) {
            // read bpresses and change state based on bpresses buttons and whether it is being pressed currently
            int i;
            Semaphore_pend(sem_cs, BIOS_WAIT_FOREVER); // protect critical section
            for (i = 0; i < 10; i++){
                if (bpresses[i]==('u') && gButtons == 4){ // increment state
                    stateVperDiv = (++stateVperDiv) & 3;
                } else if (bpresses[i]==('t') && gButtons == 2){ // trigger
                    risingSlope = !risingSlope;
                } else if (bpresses[i]==('s') && gButtons == 8){ // spectrum mode
                    spectrumMode = !spectrumMode;
                }

            }
            Semaphore_post(sem_cs);
        }

        Semaphore_post(semDisplay); // to display

    }
}

// Timer Capture ISR is an interrupt that calculates the period of the ISR
void timercapture_ISR(UArg arg0){
    // Clear the timer0A Capture interrupt flag
    TIMER0_ICR_R = TIMER_ICR_CAECINT;

    // Use timervalueget() to read full 24 bit captured time count
    uint32_t currCount = TimerValueGet(TIMER0_BASE, TIMER_A);

    timerPeriod = (currCount - prevCount) & 0xFFFFFF;

    prevCount = currCount;

    multiPeriodInterval += timerPeriod;
    accumulatedPeriods++;

}

// Clock to post to frequency task
void clockfreq_func(UArg arg1){
    Semaphore_post(semFrequency); // to buttons
}

// Determines the average frequency as the ratio of the number of accumulated periods
// to the accumulated interval, converted from
// clock cycles to seconds
void frequencyTask_func(UArg arg1, UArg arg2) {
    IArg key;
    uint32_t accu_int, accu_count;

    while (true) {
        Semaphore_pend(semFrequency, BIOS_WAIT_FOREVER); // from clock

        // Protect global shared data
        key = GateHwi_enter(gateHwi0);
        // Retrieve global data and reset
        accu_int = multiPeriodInterval;
        accu_count = accumulatedPeriods;
        // Reset globals back to 0
        multiPeriodInterval = 0;
        accumulatedPeriods = 0;
        GateHwi_leave(gateHwi0, key);

        // determine average frequency
        float avg_period = (float)accu_int/accu_count;
        avg_frequency = 1/(avg_period) * (float)gSystemClock;

    }
}

// PWM ISR with 0 latency
void PWM_ISR(void) {
    PWM0_0_ISC_R = PWM_0_ISC_INTCNTZERO; // clear PWM interrupt flag
    gPhase += gPhaseIncrement;
    // write directly to the Compare B register that determines the duty cycle
    PWM0_0_CMPB_R = 1 + gPWMWaveformTable[gPhase >> (32 - PWM_WAVEFORM_INDEX_BITS)];
}

// Fills the waveform table
void fillWaveformTable(void) {
    int i;
    float a= 255.0/2.0;
    float b = 2*PI/PWM_WAVEFORM_TABLE_SIZE;
    for (i = 0; i < PWM_WAVEFORM_TABLE_SIZE; i++) {
        gPWMWaveformTable[i] = (uint8_t)(a*sinf(b*i) + a);
    }
}



