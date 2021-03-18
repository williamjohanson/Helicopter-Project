// *******************************************************
//
// heliYaw.h
//
// Supporting module for detecting and calculating yaw of the helicopter rig
// using quadrature encoder and reference GPIO pins and associated ISRs
//
//
// Created by: Ben Tait
// Last modified:  30.5.2019
//
// *******************************************************

#ifndef HELIYAW_H
#define HELIYAW_H

#include <stdint.h>
#include <stdbool.h>

#define ENCODER_PORT SYSCTL_PERIPH_GPIOB
#define REF_PORT SYSCTL_PERIPH_GPIOC
#define ENCODER_A_PIN GPIO_PIN_0
#define ENCODER_B_PIN GPIO_PIN_1
#define ENCODER_A_INT GPIO_INT_PIN_0
#define ENCODER_B_INT GPIO_INT_PIN_1
#define REF_PIN GPIO_PIN_4
#define REF_INT GPIO_INT_PIN_4

void yawIntHandler(void);

int8_t isHomed(void);

void homeIntHandler(void);

int16_t getCurrentYaw(void);

void initYaw(void);

#endif /*HELIYAW_H*/
