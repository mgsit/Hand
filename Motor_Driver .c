/* 
 * File:   Taylors Tests.c
 * Author: tfurtado
 *
 * Created on February 11, 2014, 8:51 PM
 *
 * UPDATED:
 * File: Motor_Driver.c
 * Updated to use new PWM library
 *
 * April 27, 2014
 */

#include "Motor_Driver.h"

//#define TEST


static uint16_t right, left, middle, wrist;

/*bools*/

/*
 * 
 */


void InitMotorDriver (void)
{
    //Init PWM pins
//    PWM_Init(RIGHT_FINGER_MOTOR_PIN | LEFT_FINGER_MOTOR_PIN | MIDDLE_FINGER_MOTOR_PIN,
//            PWM_10KHZ);
    PWM_Init();
    PWM_SetFrequency(PWM_10KHZ);
    PWM_AddPins(RIGHT_FINGER_MOTOR_PIN | LEFT_FINGER_MOTOR_PIN | MIDDLE_FINGER_MOTOR_PIN
            | WRIST_MOTOR_PIN);

    //Init ADC pins


    //Set direction of digital pins
    RIGHT_FINGER_DIRECTION_TRIS = 0;
    LEFT_FINGER_DIRECTION_TRIS = 0;
    MIDDLE_FINGER_DIRECTION_TRIS = 0;
    WRIST_DIRECTION_TRIS = 0;

}

void SetPWM (uint8_t motor, unsigned int direction, unsigned int dutyCycle)
{
    switch (motor) {
        case RIGHT_FINGER:
            PWM_SetDutyCycle(RIGHT_FINGER_MOTOR_PIN, dutyCycle);
            RIGHT_FINGER_DIRECTION_PIN = direction;
            break;
        case LEFT_FINGER:
            PWM_SetDutyCycle(LEFT_FINGER_MOTOR_PIN, dutyCycle);
            LEFT_FINGER_DIRECTION_PIN = direction;
            break;

        case MIDDLE_FINGER:
            PWM_SetDutyCycle(MIDDLE_FINGER_MOTOR_PIN, dutyCycle);
            MIDDLE_FINGER_DIRECTION_PIN = direction;
            break;

        case WRIST:
            PWM_SetDutyCycle(WRIST_MOTOR_PIN, dutyCycle);
            WRIST_DIRECTION_PIN = direction;

        default:
            break;
    }
}

uint16_t GetCurrent (uint8_t motor)
{
    switch (motor) {
        case RIGHT_FINGER:
            return ReadADPin(RIGHT_FINGER_CURRENT_PIN) * 11;
            
        case LEFT_FINGER:
            return ReadADPin(LEFT_FINGER_CURRENT_PIN) * 11;
            
        case MIDDLE_FINGER:
            return ReadADPin(MIDDLE_FINGER_CURRENT_PIN) * 11;
            
        case WRIST:
            return ReadADPin(WRIST_CURRENT_PIN) * 11;
    }
}


void ShutDown (void);



#ifdef TEST
int main(void) {
    BOARD_Init();
    InitMotorDriver();

    while (1) {
        if(IsTimerExpired(ADC_TIMER)) {
            InitTimer(ADC_TIMER, ADC_TIME);
            SetPWM(RIGHT_FINGER, CLOSE, GetCurrent(RIGHT_FINGER));
            SetPWM(LEFT_FINGER, CLOSE, GetCurrent(LEFT_FINGER));
            SetPWM(MIDDLE_FINGER, CLOSE, GetCurrent(MIDDLE_FINGER));
            SetPWM(WRIST, CLOSE, GetCurrent(WRIST));
            printf("RIGHT: %d\t", GetCurrent(RIGHT_FINGER));
            printf("LEFT: %d\t", GetCurrent(LEFT_FINGER));
            printf("MIDDLE: %d\t", GetCurrent(MIDDLE_FINGER));
            printf("WRIST: %d\n", GetCurrent(WRIST));
        }
    }
    return (EXIT_SUCCESS);
}
#endif
