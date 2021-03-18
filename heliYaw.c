// *******************************************************
//
// heliYaw.c
//
// Supporting module for detecting and calculating yaw of the helicopter rig
// using quadrature encoder and reference GPIO pins and associated ISRs
//
//
// Created by: Ben Tait
// Last modified:  30.5.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "utils/ustdlib.h"
#include "stdlib.h"
#include "heliYaw.h"

static uint32_t     currentYawState;
static int16_t      currentYawCount;        //Incremental/Decremental yaw
static uint32_t     lastYawState;         //State of last state from quadrature encoder
static uint8_t      homed = false;   //Home signal of encoder

void
yawIntHandler(void)
/* ISR triggered on a rising or falling edge of the quadrature encoder signals A or B. Retaining memory of
 * previous state the current yaw count value is incremented or decremented based on direction of rotation determined from
 * value of changing state
 */
{
    //Number of slots is 112. Quadrature encoding: 448 max
    currentYawState = GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1);
    switch(currentYawState)
    {
    case 0:
        switch(lastYawState)
        {
        case 1:
            currentYawCount++;
            break;
        case 2:
            currentYawCount--;
            break;
        }
        break;
    case 1:
        switch(lastYawState)
        {
        case 3:
            currentYawCount++;
            break;
        case 0:
            currentYawCount--;
            break;
        }
        break;
    case 2:
        switch(lastYawState)
        {
        case 0:
            currentYawCount++;
            break;
        case 3:
            currentYawCount--;
            break;
        }
        break;
    case 3:
        switch(lastYawState)
        {
        case 2:
            currentYawCount++;
            break;
        case 1:
            currentYawCount--;
            break;
        }
        break;
    }
    lastYawState = currentYawState;
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_PIN_1);
}

int8_t
isHomed(void)
/* Simple return function to main program, triggered once reference home signal has been detected by the corresponding ISR
 */
{
    return homed;
}

void
homeIntHandler(void)
/* ISR once reference home signal detected, set currentYawCount to 0 to set the reference position to 0, and trigger homed flag
 * to true
 */
{
    currentYawCount = 0;
    homed = true;
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_INT_PIN_4); //Disable encoder home signal as interrupt
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
}

int16_t
getCurrentYaw(void)
/* Simple function which returns the updated currentYawCount once called un main function
 *
 */
{
    return currentYawCount;
}

void
initYaw(void)
/* Set all required GPIO pins for quadrature encoder and reference encoder position, and set corresponding interrupts to
 * the required ISRs
 */
{
    //EncurrentYawStatele GPIO PB0 and PB1 Peripherals
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet (GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPadConfigSet (GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Initialise interrupt for detecting yaw from quadrature encoder pulses
    GPIOIntRegister(GPIO_PORTB_BASE, yawIntHandler);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1); //Set SW1 as interrupt
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_BOTH_EDGES);

    GPIOIntRegister(GPIO_PORTC_BASE, homeIntHandler);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4); //Set encoder home signal as interrupt
}
