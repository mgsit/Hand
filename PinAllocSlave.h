/* 
 * File:   PinAllocSlave.h
 * Author: mgsit
 *
 * Created on April 27, 2014, 6:03 PM
 */

#ifndef PINALLOCSLAVE_H
#define	PINALLOCSLAVE_H

#include "AD.h"
#include "pwm.h"

// AD sensor pins
#define FSR_1_PIN AN1 // 41
#define FSR_2_PIN AN0 // 42
#define FSR_3_PIN AN10 // A3
#define THERMOPILE_PIN AN11 // A9

// PWM Motor Pins
// right=mtr1, left=mtr2, middle=mtr3, wrist=mtr4 (silkscreen names)
#define RIGHT_FINGER_MOTOR_PIN      PWM_PORT5   //OC2
#define LEFT_FINGER_MOTOR_PIN       PWM_PORT6   //OC3
#define MIDDLE_FINGER_MOTOR_PIN     PWM_PORT9   //OC4
#define WRIST_MOTOR_PIN             PWM_PORT10  //OC5

// Hand Motor Direction control pins
#define RIGHT_FINGER_DIRECTION_TRIS     TRISDbits.TRISD11   //UNO:35
#define LEFT_FINGER_DIRECTION_TRIS      TRISDbits.TRISD5    //UNO:34
#define MIDDLE_FINGER_DIRECTION_TRIS    TRISDbits.TRISD6    //UNO:36
#define WRIST_DIRECTION_TRIS            TRISDbits.TRISD7    //UNO:37

#define RIGHT_FINGER_DIRECTION_PIN      LATDbits.LATD11     //UNO:35
#define LEFT_FINGER_DIRECTION_PIN       LATDbits.LATD5      //UNO:34
#define MIDDLE_FINGER_DIRECTION_PIN     LATDbits.LATD6      //UNO:36
#define WRIST_DIRECTION_PIN             LATDbits.LATD7      //UNO:37

// Hand Motor current feedback pins
#define RIGHT_FINGER_CURRENT_PIN        AN5                 //UNO:A7
#define LEFT_FINGER_CURRENT_PIN         AN4                 //UNO:A1
#define MIDDLE_FINGER_CURRENT_PIN       AN3                 //UNO:A6
#define WRIST_CURRENT_PIN               AN2                 //UNO:A0

#endif	/* PINALLOCSLAVE_H */

