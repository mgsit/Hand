/* 
 * File:   SlaveBurstExample.h
 * Author: Ivan
 *
 * Created on February 4, 2014, 6:16 PM
 */

#ifndef SLAVESPI_H
#define	SLAVESPI_H


/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
// Communication Protocols
//Status Messages:  [_ _][_ _ _ _][_ _ _ _ _ _ _ _ _ _]
//                  [MSG][SNSRID ][    ANALOG VALUE   ]

//Command Messages: [_ _][E _ R _ L][I D  E][_ _  _][_ _  _] E = Enable, R = Right
//                  [MSG][WRIST    ][FGRL  ][FGRM  ][FGRR  ]

// Command Messages[15:0]: msgTyp[15:14]wCom[13:9]fgrLCom[8:6]fgrMCom[5:3]fgrRCom[2:0]

typedef union {
    struct {
        unsigned short snsVal : 10; // lowest ten bits correspond to value
        unsigned short snsName : 4; // correspond to sensor name
        unsigned short msgTyp : 2; // what type of messsage we've received
    };
    unsigned short words;
} snsMsg;

typedef union {
    struct {
        unsigned short fgrRCom : 3; // lowest ten bits correspond to value
        unsigned short fgrMCom : 3; // lowest ten bits correspond to value
        unsigned short fgrLCom : 3; // lowest ten bits correspond to value
        unsigned short wCom : 5; // correspond wrist commands
        unsigned short msgTyp : 2; // what type of messsage we've received
    };
    unsigned short words;
} comMsg;

#define LINEHIGH 0xFFFF // Message comes in if there is nothing written to SPI
#define INITMSG 0xACAC // data to acknowledge message commencement

/*******************************************************************************
 *MESSAGE IDs                                                                  *
 ******************************************************************************/


#define STSMSG  0x00 // Send sensor messages to the command module
#define SYSMSG  0x01 // Systm messages to turn off, standby, or turn on module
#define CMDMSG  0x02 // Tell the hand to open or close fingers, or rotate wrist
#define MSGFAIL 0x03
/*******************************************************************************
 *COMMAND TYPES                                                                *
 ******************************************************************************/
#define OPEN    0x1 // Using bitfields, will right directly to the finger
#define CLOSE   0x0// (cont) corrresponding
#define S_RIGHT 0x14// 0b0001 0100
#define S_LEFT  0x11// 0b0001 0001
#define NO_SPIN 0x00// 0b0000 0000 Better to keep lines low, active high means no msg

/*******************************************************************************
 *SENSOR NAMES                                                                 *
 ******************************************************************************/

// Stores the sensor names for all of the sensors on the hand
typedef enum {
    THERVAL = 0,
    PRS1VAL,
    PRS2VAL,
    PRS3VAL
} sensorNames_t;

#define TOTAL_SENSORS 4

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * Function: SPISlaveInit
 * @param: None
 * @return: None
 * @remark: Initializes SPI channel 2. Sets baud rate to 115200
 * @author: Ivan Romero
 * @date: 2/1/2014   */
int SPISlaveInit(void);

/**
 * Function: commandReceived
 * @param: None
 * @return: unsigned char Flag indicating received command
 * @remark: If there has been a received message a flag is raised to indicate so
 * @author: Ivan Romero
 * @date: 2/1/2014   */
unsigned char commandReceived(void);

/**
 * Function: getCommand
 * @param: None
 * @return: unsigned short Command from the master controller
 * @remark: Acknowledges there has been a receoved messsage nad has been processed
 * @author: Ivan Romero
 * @date: 2/1/2014   */
unsigned short getCommand(void);



/**
 * Function: updateSensorVals
 * @param: unsigned char snsID Sensor value to update
 *         unsigned short adVal Analog read value from sensor
 * @return: None
 * @remark: Upadtes the array to prepare transmissions
 * @author: Ivan Romero
 * @date: 3/21/2014   */
unsigned short updateSensorVals(unsigned char snsID, unsigned short int adVal);

/**
 * Function: prepareMessage
 * @param: Configures the messages to send upon then ext received message
 * @return: None
 * @remark: Bitshifts the ID to choose the proceeding message
 * @author: Ivan Romero
 * @date: 3/21/2014   */
void prepareMessage(void);

#endif	/*SLAVESPI_H */

