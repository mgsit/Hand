/*
 * File:   SlaveBurstExample.c
 * Author: Ivan
 *
 * Created on February 4, 2014, 6:16 PM
 */


#include "xc.h"
#include "plib.h" 
#include "SlaveSPI.h"
#include "serial.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
//#define SLAVESPI_DEBUG_VERBOSE
#ifdef SLAVESPI_DEBUG_VERBOSE
#define dbprintf(...) while(!IsTransmitEmpty()); printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif

/*******************************************************************************
 * PRIVATE DATATYPES                                                           *
 ******************************************************************************/


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/


/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/
static unsigned char commandReady = 0; // Flag to indicate command ready
static unsigned short commandInstruction = 0x0000; // message received from master
static unsigned short receivedMessage = 0x0000; // Initializes the line to high
static unsigned short sendMessage = INITMSG;
static unsigned char snsIndex = 0;
static snsMsg encodedValue = {.words = 0x0000};
static unsigned short sensorData[TOTAL_SENSORS] = {0}; // Initialize a sensor array to zero

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * Function: SPISlaveInit
 * @param: None
 * @return: None
 * @remark: Initializes SPI channel 2. Sets baud rate to 1MHz
 * @author: Ivan Romero
 * @date: 2/1/2014   */
int SPISlaveInit(void) {
    OpenSPI2(ENABLE_SDO_PIN | SPI_CKE_OFF | SLAVE_ENABLE_OFF | MASTER_ENABLE_OFF | PRI_PRESCAL_4_1 | SPI_MODE16_ON,
            SPI_ENABLE);
    ConfigIntSPI2(SPI_RX_INT_EN | SPI_INT_SUB_PRI_2 | SPI_INT_PRI_5 | SPI_FAULT_INT_DIS | SPI_TX_INT_DIS);

    INTEnable(INT_SPI2, INT_ENABLED);
    INTSetVectorPriority(INT_SPI_2_VECTOR, INT_PRIORITY_LEVEL_5);
    INTSetVectorSubPriority(INT_SPI_2_VECTOR, INT_SUB_PRIORITY_LEVEL_2);
    SpiChnSetBrg(2, 19); // Sets baud rate BR = Fpb/(2*(SPIBRG+1))
    encodedValue.msgTyp = STSMSG;
    putcSPI2(INITMSG);
    //    EnableIntSPI2;
    //prepareMessage();
    return 0;
}

/**
 * Function: commandReceived
 * @param: None
 * @return: unsigned char Flag indicating received command 
 * @remark: If there has been a received message a flag is raised to indicate so
 * @author: Ivan Romero
 * @date: 2/1/2014   */
unsigned char commandReceived(void) {
    return commandReady;
}

/**
 * Function: getCommand
 * @param: None
 * @return: unsigned short Command from the master controller
 * @remark: Acknowledges there has been a receoved messsage nad has been processed
 * @author: Ivan Romero
 * @date: 2/1/2014   */
unsigned short getCommand(void) {
    commandReady = 0; // Sets the flag low once the command has been retrieved 
    return commandInstruction;
}

/**
 * Function: updateSensorVals
 * @param: unsigned char snsID Sensor value to update
 *         unsigned short adVal Analog read value from sensor
 * @return: None
 * @remark: Upadtes the array to prepare transmissions
 * @author: Ivan Romero
 * @date: 3/21/2014   */
unsigned short updateSensorVals(unsigned char snsID, unsigned short adVal) {
    if (snsID > TOTAL_SENSORS - 1) { // Value should remain within array size
        dbprintf("Not a registered sensor\n");
    } else {
        encodedValue.snsName = snsID;
        encodedValue.snsVal = adVal;
        sensorData[snsID] = encodedValue.words; // Adds the encoded value to the array
        //dbprintf("ID:%d\tNAME:%d\tVAL:%d\n", encodedValue.msgTyp, encodedValue.snsName, encodedValue.snsVal );
    }
    return 0;
}

/**
 * Function: prepareMessage
 * @param: Configures the messages to send upon then ext received message
 * Master sends out (5)0x0000, slave sends out s1 (on the next packet), s2, s3, s4, 0xFFFF
 * IVAN Says printf will not work in interrrupts
 * @return: None
 * @remark: Bitshifts the ID to choose the proceeding message
 * @author: Ivan Romero
 * @date: 3/21/2014   */
void prepareMessage(void) {
    if (snsIndex < TOTAL_SENSORS) { // Grabs the sensor values from the array
        encodedValue.words = sensorData[snsIndex]; // May be able to remove
        sendMessage = sensorData[snsIndex++];
    } else {
        snsIndex = 0;
        sendMessage = 0xFFFF; // Once the array has sent, rest index and send done.
    }
    if(receivedMessage >> 14 == CMDMSG) { // check the two MSBs
            commandReady = 1; // Alert flag to indicate new master command received
            commandInstruction = receivedMessage; // Saves the message
    }
    putcSPI2(sendMessage);
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                          *
 ******************************************************************************/

/**
 * Function: getMessage
 * @param: None
 * @return: receivedMessage
 * @remark: Returns most recent message.
 * @author: Ivan Romero
 * @date: 3/21/2014   */
static unsigned short getMessage(void) {
    return receivedMessage;
}

/**
 * Function:
 * @param:
 * @return:
 * @remark:
 * @author: Ivan Romero
 * @date: 3//2014   */
void __ISR(_SPI_2_VECTOR, ipl5)__SPI2Interrupt(void) {
    receivedMessage = getcSPI2(); // Grabs the message as soon as it comes in.
    prepareMessage();
    // before exiting the service routine.
    SpiChnClrRovIntFlag(2);
    INTClearFlag(INT_SPI2);
}
