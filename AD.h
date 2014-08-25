/*
 * File:   AD.h
 * Author: mdunne
 *
 * Software module to enable the Analog to Digital converter of the Uno32 boards.
 * All analog pins are are Port V and Port W, with an additional analog input for
 * the battery voltage (through a 10:1 divider).
 *
 * NOTE: Analog pins automatically take over digital I/O regardless of which TRIS
 *       state it is in. There remains an error in the ADC code such that if all 12
 *       pins are enabled, one of them does not respond.
 *
 * AD_TEST (in the .c file) conditionally compiles the test harness for the code. 
 * Make sure it is commented out for module useage.
 *
 * Created on November 22, 2011, 8:57 AM
 */

#ifndef AD_H
#define AD_H

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/


#define AN2 (1<<0) //AN2
#define AN3 (1<<1) //AN3
#define AN4 (1<<2) //AN4
#define AN5 (1<<3) //AN5

#define AN8 (1<<4) //AN8
#define AN9 (1<<5) //AN9
#define AN11 (1<<6) //AN11
#define AN10 (1<<7) //AN10
#define AN13 (1<<8) //AN13
#define AN12 (1<<9) //AN12
#define AN15 (1<<10) //AN15
#define AN14 (1<<11) //AN14
#define AN1 (1<<12) //AN1
#define AN0 (1<<13) //AN0


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * Function: AD_Init
 * @param Pins, used #defined AD_PORTxxx OR'd together for each A/D Pin
 * @return SUCCESS or ERROR
 * @remark Initializes the A/D pins requested into analog inputs and configures the A/D subsystem.
 * It then generates the mapping for correctly reading the pin and then starts the A/D system.
 * @author Max Dunne
 * @date 2011.12.10  */
unsigned char AD_Init(unsigned int Pins);


/**
 * Function: ReadADPin
 * @param Pin, used #defined AD_PORTxxx to select pin
 * @return 10-bit AD Value or ERROR
 * @remark Reads current value from buffer for given pin
 * @author Max Dunne
 * @date 2011.12.10  */
unsigned int ReadADPin(unsigned int Pin);



/**
 * Function: AD_End
 * @param None
 * @return None
 * @remark disables the A/D subsystem and release the pins used
 * @author Max Dunne
 * @date 2011.12.10  */
void AD_End(void);

#endif
