/**********************************************************
 *
 * PIDController.c
 *
 * PID controller module for controlling helicopter rig main and tail rotor duty
 * cycles. Main rotor uses a PID controller, tail rotor uses a PI controller.
 *
 * Created by: Ben Tait
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
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "utils/ustdlib.h"
#include "stdlib.h"
#include "motorControl.h"
#include "PIDController.h"

//Global error values for use in PID controllers
static int32_t      errorI = 0;
static int32_t      errorI_t = 0;
static double       pastError;
static double       pastError_t;
static int32_t      errorD;
static int32_t      errorD_t;

uint16_t
PIDMainControl(uint16_t currentHeight, uint16_t targetHeight, uint16_t landedHeight)
/* Main rotor PID controller. Take current heli height parameters and calculate the required error values.
 * Save a running sum of the integral error for use in the I controller. Because the I component of the controller must
 * create a continuous positive PWM signal to hold a constant altitude, error I is bounded to values above 0.
 * Limit PWM output to between 2% and 98% before being applied to the set PWM function.
 * Calculated duty is returned for displaying on the OLED and serial output
 */
{
    uint16_t PWMMain;
    double error; //Error signal between current height and target height

    error = currentHeight - targetHeight; //Positive if going upwards
    errorI += error * DELTA_T; //integrate error every 0.25 seconds (change magic number eventually)


    //Boundary condition to prevent negative contributing integral error
    if (errorI <= 0) {
        errorI = 0;
    }
    errorD = (pastError - error)/DELTA_T;

    PWMMain = errorI*KI + error*KP - errorD*KD;

    if (PWMMain >= 98) {
        PWMMain = 98;
    }
    if (PWMMain <= 2) {
        PWMMain = 2;
    }

    setPWMMain(PWMMain);
    pastError = error;
    return PWMMain;
}

uint16_t
PIDTailControl(int16_t targetYaw, int16_t currentYaw)
/* Tail rotor PID controller. Take current yaw parameters and calculate the required error values.
 * Save a running sum of the integral error for use in the I controller. Because the I component of the controller must
 * create a continuous positive PWM signal to counter torque produced by the main rotor output, error I is bounded to values above 0.
 * Limit PWM output to between 2% and 98% before being applied to the set PWM function.
 * Calculated duty is returned for displaying on the OLED and serial output
 */
{
    uint16_t PWMTail;
    double error; //Error signal between current height and target height
    error = targetYaw - currentYaw;     //Error in yaw in degrees
    errorI_t = errorI_t + error * DELTA_T; //integrate error every 0.1 seconds (change magic number eventually)

    //Boundary condition to prevent negative contributing integral error
    if (errorI_t <= 0) {
        errorI_t = 0;
    }

    PWMTail = errorI_t*KI_t + error*KP_t;

    if (PWMTail >= 98) {
        PWMTail = 98;
    }
    if (PWMTail <= 2) {
        PWMTail = 2;
    }
    setPWMTail(PWMTail);
    pastError_t = error;
    return PWMTail;
}

