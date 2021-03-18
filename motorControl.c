/**********************************************************
 *
 * motorControl.c
 *
 * Module for updating PWM duty cycle for main and tail motors of helicopter
 * rig
 *
 * Modified from pwmGen.c
 *
 * Created by P.J. Bones   UCECE
 * Modified by: Ben Tait
 * Last modified: 30/05/2019
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "utils/ustdlib.h"
#include "stdlib.h"
#include "motorControl.h"


/*******************************************
 *      Local prototypes
 *******************************************/
void initMainPWM (void);
void initTailPWM (void);
void setPWMMain (uint32_t u32Duty);
void setPWMTail (uint32_t u32Duty);
/************************************************************/
/* InitialiseMainPWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/
void
initMainPWM (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);


    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    setPWMMain(PWM_DUTY_START);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}

/************************************************************/
/* InitialiseTailPWM
 * M1PWM5 (PF1, J3-10) is used for the main rotor motor
 *********************************************************/
void
initTailPWM (void)
{
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setPWMTail(PWM_DUTY_START);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setPWMMain (uint32_t u32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t u32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, u32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, u32Period * u32Duty / 100);
}

/********************************************************
 * Function to set the freq, duty cycle of M1PWM5
 ********************************************************/
void
setPWMTail (uint32_t u32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t u32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, u32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, u32Period * u32Duty / 100);
}

void
enablePWMOutput(void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}
