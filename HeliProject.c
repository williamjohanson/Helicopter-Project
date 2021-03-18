//*****************************************************************************
// Helicopter Program
//
// Author: Ben Tait, Flynn Hill, William Johanson
// Original code based on ADCdemo1.c from P.J. Bones	UCECE
// Last modified:	30/05/2019
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "buttons4.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "motorControl.h"
#include "PIDController.h"
#include "heliYaw.h"
#include "serialCom.h"
#include "driverlib/pwm.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 50 //Size of buffer to be used to average height sensor
#define SAMPLE_RATE_HZ 200
#define ANGLE_CONVERSION 360/448 //Degrees per encoder pulse

//************************************************************************
// Global variables
//*****************************************************************************
//Circular buffer for altitude ADC
static circBuf_t    g_inBuffer;		    // Buffer of size BUF_SIZE integers (sample values)

//Heli rig height parameters
static uint16_t     currentHeight;
static uint16_t     currentHeightADC;      // Mean ADC height value calculated
static uint16_t     targetHeight;
static uint16_t     targetHeightADC;
static uint16_t     landedHeight;          // The ADC value for helicopter landed state

//Heli rig yaw parameters
static int16_t      currentYaw;
static int16_t      targetYaw = 0;

//Flight mode for landing or flying sortie programs
static uint8_t      flightMode = 0; //Set flight mode state to landed at start

//Flag for completed heli landing, triggered if landing proceedure and at landed height and reference yaw
static uint8_t      landed = true;

//4Hz clock tick
static uint8_t      slowTick = false;

//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
void
SysTickIntHandler(void)
/* ISR triggered on each clock pulse. Used to trigger ADC conversion of analogue output of altitude sensor, and updating slow tick
 * signal at 4Hz for control of serial output and PID controller
 */
{
    static uint8_t tickCount = 0;

    const uint8_t ticksPerSlow = SYSTICK_RATE_HZ / SLOWTICK_RATE_HZ;

    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3); 

    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true; //Ticks every 4Hz
    }
}

//*****************************************************************************
// The handler for the ADC conversion complete interrupt, and peripheral button and switch events.
// Writes to the circular buffer.
//*****************************************************************************
void
ADCIntHandler(void)
/* ISR for ADC conversion of altitude sensor output voltage, and storage of value into circular buffer at an updated
 * pointer. Uses circBuffer module
 */
{
	uint32_t ulValue;
	// Get the single sample from ADC0.  ADC_BASE is defined in
	// inc/hw_memmap.h
	ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
	//
	// Place it in the circular buffer (advancing write index)
	writeCircBuf (&g_inBuffer, ulValue);
	//
	// Clean up, clearing the interrupt
	ADCIntClear(ADC0_BASE, 3);
}

void
SWIntHandler(void)
/* Interrupt ISR when SW1 activated. If SW1 changing from 0 to 1 (landed to flying) set flight mode to 1 and trigger landed
 * state flag to false. Change flight mode to 0 when low signal detected
 */
{
    if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_7) == GPIO_PIN_7) {
        flightMode = 1;
        landed = false;
        targetHeight = 10;

    } else {
        flightMode = 0;
    }
    //Clear SW1 interrupt to allow operation of main program
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_7);
}
//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
resetPeripherals(void)
{
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset (PWM_TAIL_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_TAIL_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO
    SysCtlPeripheralReset (LEFT_BUT_PERIPH);      // LEFT button GPIO
    SysCtlPeripheralReset (RIGHT_BUT_PERIPH);     // RIGHT button GPIO
    SysCtlPeripheralReset (SYSCTL_PERIPH_ADC0);   // Reset ADC
}

void
initClock (void)
/* Initiliase system clock pulse for timing of background processes, and triggering ADC interrupt using SysTickInterrupt
 *
 */
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

void
initADC (void)
/* Initialise ADC read from AIN9 for detecting helicopter altitude from sensor. Analogue values from AIN9 is converted to
 * digital value with resolution 1.24 bits per mV resolution (2^12 bits for voltage range of 3,300 mV)
 */
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH9) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}


void calcHeightADC (void)
/* Take the contents of the altitude sensor circular buffer and calculate the average ADC value
 * for average heli height
 */
{
    uint32_t sum = 0;
    uint16_t i;
    //Sum current values of circular buffer
    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf (&g_inBuffer);
    //Calculate mean digital valyue for height from circular buffer
    currentHeightADC = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
}

void
initHeli(void)
/*Take sample of current height at initialisation and update landed ADC value
 *
 */
{
    initYaw();
    initMainPWM ();
    initTailPWM ();
    enablePWMOutput();
    calcHeightADC();
    landedHeight = currentHeightADC;
}

void
initSW1(void)
/* Initialise SW slider switch for changing flight mode between landed and flying state
 *
 */
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet (GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    GPIOIntRegister(GPIO_PORTA_BASE, SWIntHandler);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_7); //Set SW1 as interrupt
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_INT_PIN_7, GPIO_BOTH_EDGES);
}
//*****************************************************************************
//
// Function to display the mean ADC value, percentage of height, or blank screen.
//
//*****************************************************************************
void
updateDisplay(uint16_t PWMMain, uint16_t PWMTail)
/* Display heli rig values on OLED display. Show Target height, target yaw, and
 * duty cycles of main and tail rotors
 */
{
	char string[17];  // 16 characters across the display
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    // Update line on display.

	usnprintf (string, sizeof(string), "Height = %4d%c", targetHeight, '%');
    OLEDStringDraw (string, 0, 0);
    usnprintf (string, sizeof(string), "Yaw = %4d", targetYaw);
    OLEDStringDraw (string, 0, 1);
    //Display current yaw
    usnprintf (string, sizeof(string), "Main PWM = %4d%c", PWMMain, '%');
    OLEDStringDraw (string, 0, 2);
    //Display target yaw
    usnprintf (string, sizeof(string), "Tail PWM = %4d%c", PWMTail, '%');
    OLEDStringDraw (string, 0, 3);
}

void
updateSerial(uint16_t PWMMain, uint16_t PWMTail)
/* Send information to uart serial terminal
 * Send target and current heights in %, current and target yaw in degrees, flight mode,
 * and duty cycles of main and tail rotors
 */
{
    char string[MAX_STR_LEN] = "";
    usprintf (string, "Current Height = %d%c\n", currentHeight, '%');
    UARTSend (string);
    usprintf (string, "Target Height = %d%c\n", targetHeight, '%');
    UARTSend (string);
    usprintf (string, "Current Yaw = %d\n", currentYaw);
    UARTSend (string);
    usprintf (string, "Target Yaw = %d\n", targetYaw);
    UARTSend (string);
    usprintf (string, "Main Duty = %d%c\n", PWMMain, '%');
    UARTSend (string);
    usprintf (string, "Tail Duty = %d%c\n", PWMTail, '%');
    UARTSend (string);
    usprintf (string, "Flight Mode = %d\n", flightMode);
    UARTSend (string);
}

//
void
pollButtons(void)
/* Update button state and check if the up, down, left or right buttons have been pressed
 * Increase and decrease taret height and target yaw following successful button pressing
 */
{
   updateButtons();
   if (checkButton(UP) == PUSHED) {
       if (targetHeight < 100) {
           targetHeight += 10;
       }
   }
   if (checkButton(DOWN) == PUSHED) {
       if (targetHeight > 0) {
           targetHeight -= 10;
       }
   }
   if (checkButton(LEFT) == PUSHED) {
       targetYaw -= 15;
   }
   if (checkButton(RIGHT) == PUSHED) {
       targetYaw += 15;
   }
}

void
getHeliPos(void)
/* Calculate the current heli height and yaw positions from encoder count and ADC of altitude sensor
 *
 */
{
    int16_t heightDiff;
    //Calculate the current heli height from sensor analog output in digital value
    calcHeightADC();

    //Calculate height difference between target and landed heights in ADC
    heightDiff = landedHeight - currentHeightADC;

    //Convert current heli digital height to % of total height (1V difference from sensor output)
    currentHeight = (heightDiff*100)/RANGE_ADC;

    //Set boundary condition to prevent height difference less than 0
    if (heightDiff <= 0) {
        currentHeight = 0;
    }

    //Convert current yaw count in encoder pulses to degrees
    currentYaw = getCurrentYaw() * ANGLE_CONVERSION;

}

void
runHeli(void)
/* Main helicopter run function, sending updated info to serial output and OLED display, and update PWM duty cycles of Main and Tail
 * rotors every 4 Hz
 */
{
    uint16_t PWMTail;
    uint16_t PWMMain;

    if (slowTick) {
        PWMMain = PIDMainControl(currentHeightADC, targetHeightADC, landedHeight);
        PWMTail = PIDTailControl(targetYaw, currentYaw);
        updateSerial(PWMMain, PWMTail);
        updateDisplay(PWMMain, PWMTail);
        slowTick = false;
    }
}

void
initPeripherals(void)
/* Initialise all peripherals required for helicopter
 */
{
    initClock ();
    initADC ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    OLEDInitialise ();
    initButtons ();
    initSW1();
    initialiseUSB_UART();
    initHeli();
}

void
toggleSW1Interrupt(void)
{
    if (landed) { //Toggle SW1 interrupt if detected reference pulse via ISR
        GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_7); //Reenable SW1 as interrupt
    } else {
        GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_7); //Reenable SW1 as interrupt
    }
}

void
heliLanding(void)
/* Main landing procedure for heli landing on flight mode 0. Toggle off SW1 interrupt to prevent changing
 * flight mode until fully landed. Set target height to 0%, and target yaw to reference at 0. Toggle landed
 * flag once fully landed to allow flight mode switching
 */
{
   toggleSW1Interrupt(); //Prevent SW1 activity until heli had fully landed and motors are off

   //Set landed height conditions
   targetHeight = 0;
   targetYaw = 0;

   //Check heli state if below 3% height and reference yaw orientation with in +/- 5 encoder counts
   if (currentHeight <= 3 && (currentYaw < 5 && currentYaw > -5)) {
       //Turn off main and tail motors
       setPWMMain(0);
       setPWMTail(0);
       //Set landed flag to true to reenable SW1 interrupt
       landed = true;
   } else {
       //Update OLED and serial output at 4Hz, and update PWM of motors at 4Hz
       runHeli();
   }
}

void
flySortie(void)
/* Program for running sortie of project once in flying state (flightMode = 1). Find home position if the reference ISR has
 * not occured, otherwise poll buttons regularly and run main helirun function
 */
{
    if (!isHomed()) { //If home signal not detected via ISR
        setPWMTail(15); //Slowly rotate heli until reference pulse detected
        setPWMMain(30);
    } else {
        pollButtons();
        runHeli();
    }
}

/***********************************************
 * Start main program
***********************************************/
int main(void)
{
    //Reset and initialise all required peripherals and initial values
    resetPeripherals();
    initPeripherals();

    // Enable interrupts to the processor.
    IntMasterEnable();

    //Start kernel
    while(1)
    {
        getHeliPos();
        targetHeightADC = landedHeight - (targetHeight*RANGE_ADC)/100;
        switch (flightMode)
        {
        case 0:
            heliLanding(); //If flightmode is 0, begin landing
            break;
        case 1:
            flySortie(); //If flightmode is 1, run sortie
            break;
        }
	}
}
