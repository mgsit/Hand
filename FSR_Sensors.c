/*
 * File:   FSR_Sensor.c
 * Author: tfurtado
 *
 * Created on February 21, 2014
 */

#include "FSR_Sensors.h"
#include "PinAllocSlave.h"

/*#DEFINES*/

#define BETA            0.90

/*GLOBAL VARIABLES*/
static uint16_t force[3];

/*PRIVATE FUNCTION PROTOTYPES*/


/*PUBLIC FUNCTIONS DEFINES*/

uint16_t GetFSR (uint8_t sensor)
{
    return force[sensor];
}

void CalculateForce (void)
{
    uint16_t tempForce[3];
    tempForce[FSR_1] = ReadADPin(FSR_1_PIN);
    tempForce[FSR_2] = ReadADPin(FSR_2_PIN);
    tempForce[FSR_3] = ReadADPin(FSR_3_PIN);

    int i;
    for (i = FSR_1; i <= FSR_3; i++) {
        force[i] = (BETA * tempForce[i]) + (1.0 - BETA) * force[i];

        if (force[i] >= 1023)
            force[i] = 1023;
        if (force[i] <= 0)
            force[i] = 0;
    }
}