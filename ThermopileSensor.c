/*
 * File:   ThermopileSensor.c
 * Author: tfurtado
 *
 * Created on February 20, 2014
 */


#include "ThermopileSensor.h"

/*#DEFINES*/
#define ALPHA   0.5

/*GLOBAL VARIABLES*/
static uint16_t temperature;
static uint16_t celcius;


/*PRIVATE FUNCTION PROTOTYPES*/


/*PUBLIC FUNCTION DEFINES*/
uint16_t GetTemperature (void)
{
    return temperature;
    //return ReadADPin(THERMOPILE_PIN);
}

void CalculateTemperature (void)
{
    uint16_t temp = ReadADPin(THERMOPILE_PIN);
    temperature = (ALPHA * temp) + (1.0 - ALPHA) * temperature;
    if(temperature > 1023)
        temperature = 1023;
    if(temperature < 0)
        temperature = 0;

    // Convert to Actual Temperature value in celcius
//    celcius = (temperature - 169.26)/11.408;

}

/*PRIVATE FUNCTION DEFINES*/
