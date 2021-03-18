/**********************************************************
 *
 * PIDController.h
 *
 * PID controller module for controlling helicopter rig main and tail rotor duty
 * cycles. Main rotor uses a PID controller, tail rotor uses a PI controller.
 *
 * Created by: Ben Tait
 * Last modified: 30/05/2019
 **********************************************************/
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

//Main Controller Gains
#define KP 0.01
#define KI 0.01
#define KD 0.01

//Tail Controller Gains
#define KP_t 0.12
#define KI_t 0.03

#define DELTA_T 0.25

//
#define RANGE_ADC (SENSOR_VOLTAGE_RANGE*HEIGHT_V_TO_DIGITAL)    //Digital rep of 1V. V per V/digital
#define SENSOR_VOLTAGE_RANGE 1000                               //Change in sensor voltage for 0% to 100% (1000mV)
#define HEIGHT_V_TO_DIGITAL 4096/3300                           //Analog voltage to digital altitude sensor output

uint16_t PIDMainControl (uint16_t currentHeightADC, uint16_t targetHeightADC, uint16_t landedHeight);

uint16_t PIDTailControl(int16_t targetYaw, int16_t currentYawCount);

#endif /*PIDCONTROLLER_H*/
