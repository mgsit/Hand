/* 
 * File:   Motor_Driver.h
 * Author: tfurtado
 *
 * Created on February 12, 2014, 1:27 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "BOARD.h"
#include "timers.h"
#include <xc.h>
#include "pwm.h"
#include "PinAllocSlave.h"
#include "AD.h"
#include "HandFSM.h"

#ifndef MOTOR_DRIVER_H
#define	MOTOR_DRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif

/*Motors*/

/*MACROS*/
#define STOP(x)     SetPWM(x, 0, 0)

#define FULL_PWM    254
#define HALF_PWM    500
#define QUARTER_PWM 750
#define ZERO_PWM    1000



/**
 * Function: InitMotorDriver
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/13/2014   */
void InitMotorDriver (void);

/**
 * Function: SetPWM
 * @param: motor
 * @param: direction
 * @param: dutyCycle
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/13/2014   */
void SetPWM (uint8_t motor, unsigned int direction, unsigned int dutyCycle);

/**
 * Function: GetCurrent
 * @param: motor
 * @return: uint16_t
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/13/2014   */
uint16_t GetCurrent (uint8_t motor);

/**
 * Function: ShutDown
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/13/2014   */
void ShutDown (void);


#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_DRIVER_H */

