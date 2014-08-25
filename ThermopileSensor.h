/* 
 * File:   ThermopileSensor.h
 * Author: tfurtado
 *
 * Created on February 20, 2014, 5:01 PM
 */

#ifndef THERMOPILESENSOR_H
#define	THERMOPILESENSOR_H

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

/**
 * Function: InitThermopile
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/20/2014   */
void InitThermopile (void);

/**
 * Function: GetTemperature
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/20/2014   */
uint16_t GetTemperature (void);

/**
 * Function: CalculateTemperature
 * @param: None
 * @return: None
 * @remark:
 * @author: Taylor Furtado
 * @date: 2/20/2014   */
void CalculateTemperature (void);



#ifdef	__cplusplus
}
#endif

#endif	/* THERMOPILESENSOR_H */

