/* 
 * File:   FSR_Sensors.h
 * Author: tfurtado
 *
 * Created on February 21, 2014, 12:59 PM
 */

#ifndef FSR_SENSORS_H
#define	FSR_SENSORS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "BOARD.h"
#include "timers.h"
#include "pwm.h"
#include "PinAllocSlave.h"
#include "AD.h"
#include "HandFSM.h"

#define FSR_1           0
#define FSR_2           1
#define FSR_3           2

/**
 * Function: InitFSR
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/21/2014   */
void InitFSR (void);

/**
 * Function: GetFSR
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/21/2014   */
uint16_t GetFSR (uint8_t sensor);

/**
 * Function: CalculateForce
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/21/2014   */
void CalculateForce (void);

#ifdef	__cplusplus
}
#endif

#endif	/* FSR_SENSORS_H */

