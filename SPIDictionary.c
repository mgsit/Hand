/*
 * File:   SPIDictionary.c
 * Author: Ivan
 *
 * Created on February 10, 2014, 6:24 PM
 */


#include "xc.h"
#include "SPIDictionary.h"
#include "stdint.h"
#include "stdio.h"

uint16_t getAnalogValue(uint16_t wholeWord){
    uint16_t sensorReading = 0;
    //sensorReading = wholeWord & ANVALUE; // Remove the upper bits, isolate analov value
    return sensorReading;
}

uint8_t getSensorName(uint16_t wholeWord){
    uint8_t sensorName = wholeWord>>10;
    return sensorName;
}
