/**********************************************************
 *
 * motorControl.h
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

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdint.h>
#include <stdbool.h>


#define SYSTICK_RATE_HZ    100
// PWM configuration
#define PWM_RATE_HZ         200
#define PWM_DIVIDER_CODE    SYSCTL_PWMDIV_4
#define PWM_DIVIDER         4
#define PWM_DUTY_START      0

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M1PWM5 (gen 2)
//  ---Tail Rotor PWM: PF1, J3-10
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

void
initMainPWM (void);

void
initTailPWM (void);

void
setPWMMain (uint32_t u32Duty);

void
setPWMTail (uint32_t u32Duty);

void
enablePWMOutput(void);

#endif /*MOTORCONTROL_H*/
