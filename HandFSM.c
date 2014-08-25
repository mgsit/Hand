/* 
 * File:   HandFSM.c
 * Author: tfurtado
 *
 * Created on February 14, 2014, 3:25 PM
 */

#include "HandFSM.h"
#include "Motor_Driver.h"
#include "ThermopileSensor.h"
#include "BOARD.h"
#include "FSR_Sensors.h"
#include "SPIDictionary.h"
#include "SlaveSPI.h"
#include "PinAllocSlave.h"
#include "serial.h"

#pragma config FPLLIDIV 	= DIV_2		//PLL Input Divider
#pragma config FPLLMUL 		= MUL_20	//PLL Multiplier
#pragma config FPLLODIV 	= DIV_1 	//System PLL Output Clock Divid
#pragma config FNOSC 		= FRCPLL	//Oscillator Selection Bits
#pragma config FSOSCEN 		= OFF		//Secondary Oscillator Enable
#pragma config IESO 		= OFF		//Internal/External Switch O
#pragma config POSCMOD 		= XT		//Primary Oscillator Configuration
#pragma config OSCIOFNC 	= OFF		//CLKO Output Signal Active on the OSCO Pin
#pragma config FPBDIV 		= DIV_2		//Peripheral Clock Divisor
#pragma config FCKSM 		= CSECMD	//Clock Switching and Monitor Selection
#pragma config WDTPS 		= PS1		//Watchdog Timer Postscaler
#pragma config FWDTEN		= OFF		//Watchdog Timer Enable
#pragma config ICESEL		= ICS_PGx2	//ICE/ICD Comm Channel Select
#pragma config PWP 			= OFF		//Program Flash Write Protect
#pragma config BWP 			= OFF		//Boot Flash Write Protect bit
#pragma config CP 			= OFF		//Code Protect
//#pragma FPBDIV                  = DIV_2  //Peripheral clock, unsure why it is not included in the ds30 settings

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif

/*#DEFINES*/
/*Flags*/
#define JAMMED              1       // Values for JamFlag[]
#define UNJAMMED            0
#define CHANGE              1       // Possible values for ChangeFlag
#define DONT_CHANGE         0

/*Scalers*/
#define GAMMA               0.01    //Scaler for exponential weighted average filter for current sense

/*Used Motors*/
#define FIRST_MOTOR         RIGHT_FINGER
#define LAST_MOTOR          WRIST

/*STATE VARIABLES*/
typedef enum {
    Init, Idle, Updating
} HandFSMStates_t;
static HandFSMStates_t currentHandFSMState = Init;

/*FLAGS*/
static bool ChangeFlag = CHANGE; //Flag for setting motor position and sending sensor values
static bool JamFlag[NUM_MOTORS]; //Flag for breaking static friction

/*GLOBAL VARIABLES*/
static uint8_t maintainGripCounter[NUM_MOTORS]; // Used to determine time between grip maintain function
static uint8_t changeCounter = 0; // Number of times sensors are read before reporting values
static uint16_t accumulatedCurrent[NUM_MOTORS]; // Accumulator for current, used in UpdateSensors()
static comMsg MasterMsg = {.words = 0x0000};

/*PRIVATE FUNCTION PROTOTYPES*/
/**
/**
 * Function: HandInit
 * @param: None
 * @return: None
 * @remark: Initializes the peripherals of the Hand module. Calls: BOARD_Init(),
 * TIMERS_Init(), AD_Init(), InitMotorDriver(), SPISlaveInit()
 * @author: Taylor Furtado
 * @date: 2/21/2014   */
void HandInit(void);

/**
 * Function: RunHandFSM
 * @param: state: currentHandState
 * @return: None
 * @remark: Call in while(1) within main().
 * Runs the main control loop for the hand module.
 * States:
 *      Init:   Starts Update timer and sendign timer, initializes commandPos amd
 *              JamFlag[] for each finger. Goes directly to Idle.
 *      Idle:   Waits for an event to trigger. Possible events are: Update timeout
 *              <goes to Update state>, Status timeout <goes to Sending>, or SPISuccess
 *              which signifies an SPI message in the buffer needing to be decoded
 *      Update: Has two options bepending on the status of ChangeFlag. If ChangeFlag
 *              is DONT_CHANGE, the sensors are read and their values are accumulated
 *              and averaged using UpdateSensors(). If the status of ChangeFlag is CHANGE,
 *              the motors are set using SetMotors()
 *      Sending:Displays the current value (accumulated & averaged) of the sensors
 *              Will later send SPI messages to the master.
 * @author: Taylor Furtado
 * @date: 2/14/2014   */
void RunHandFSM(HandFSMStates_t state);

/**
 * Function: SetMotors
 * @return: None
 * @remark: Calls the routine to  set motor position baised on current reading
 * from UpdateSensors(). Calls ToggleFingerPos() when commandPos and
 * currentPos are different. Calls MaintainGrip() every MAINTAIN_GRIP time
 *  when commandPos and currentPos are both CLOSE. Calls SetPWM with PWM value
 * determined by ToggleFingerPos() and MaintainGrip() before returning
 * @author: Taylor Furtado
 * @date: 2/14/2014   */
void SetMotors(void);

/**
 * Function: ToggleFinerPos
 * @param: finger: motor to be toggled (RIGHT_FINGER, LEFT_FINGER, MIDDLE_FINGER)
 * @return: None
 * @remark: Determines PWM value to be set in SetMotors(). Based on commandPos
 * vs currentPos, OPEN vs CLOSE state, JAMMED vs UNJAMMED. Default is FULL_PWM
 * @author: Taylor Furtado
 * @date: 2/20/2014   */
void ToggleFingerPos(int finger);

/**
 * Function: MaintainGrip
 * @param: finger: motor to be toggled (RIGHT_FINGER, LEFT_FINGER, MIDDLE_FINGER)
 * @return: None
 * @remark: Calls SetPWM() on an interval determined by MAINTAIN_COUNT to further
 * CLOSE all fingers to maintain the grip of the hand. Should only be called when
 * currentPos and commandPos are both CLOSE.
 * @author: Taylor Furtado
 * @date: 2/20/2014   */
void MaintainGrip(int finger);

/**
 * Function: UpdateSensors
 * @param: None
 * @return: None
 * @remark: Called within Updating state of FSM on interval determined by UPDATE_TIME.
 * Calls CalculateTemperature(), CalculateForce() and GetCurrent() for all motors.
 * Filters current using exponential weighted average filter and accumulatedCurrent[]
 * variables. current values are stored in motors[] struct.
 * @author: Taylor Furtado
 * @date: 2/20/2014   */
void UpdateSensors(void);

void delay(int wait){
    int index = 0;
    for(index = 0; index<wait; index ++){
        ;
    }
}

int main(void) {
    HandInit();
    SERIAL_Init();
    dbprintf("Serial INIT\n");
    while (1) {
        RunHandFSM(currentHandFSMState);
    }
    return (EXIT_SUCCESS);
}

void HandInit(void) {
    BOARD_Init();
    TIMERS_Init();
    AD_Init(RIGHT_FINGER_CURRENT_PIN | LEFT_FINGER_CURRENT_PIN | MIDDLE_FINGER_CURRENT_PIN
            | FSR_1_PIN | FSR_2_PIN | FSR_3_PIN
            | THERMOPILE_PIN);
    InitMotorDriver();
    SPISlaveInit();
}

void RunHandFSM(HandFSMStates_t state) {
    switch (state) {
            int finger;
        case Init:
            dbprintf("[HAND] INIT STATE\n");
            InitTimer(UPDATE_TIMER, UPDATE_TIME);

            for (finger = FIRST_MOTOR; finger <= LAST_MOTOR; finger++) {
                maintainGripCounter[finger] = 0;
                accumulatedCurrent[finger] = 0;
                motor[finger].currentPos = CLOSE;
                motor[finger].commandPos = OPEN;
                JamFlag[finger] = JAMMED;
            }
            currentHandFSMState = Idle;
            break; //END INIT
        case Idle:
            if (commandReceived()) {
                dbprintf("MSG Received:");
                MasterMsg.words = getCommand();
                dbprintf(" %x\n", MasterMsg.words);

                if (MasterMsg.msgTyp == CMDMSG) {
                    dbprintf("Move fingers MSG received:\tRight: %x\tLeft: %x\tMiddle %x\tWrist %x\n", MasterMsg.fgrRCom, MasterMsg.fgrLCom, MasterMsg.fgrMCom, MasterMsg.wCom);
                    motor[RIGHT_FINGER].commandPos = MasterMsg.fgrRCom;
                    motor[LEFT_FINGER].commandPos = MasterMsg.fgrLCom;
                    motor[MIDDLE_FINGER].commandPos = MasterMsg.fgrMCom;
                    motor[WRIST].commandPos = MasterMsg.wCom;
//                } else if (MasterMsg.msgTyp == SYSMSG) {
//                    // turn off and stand by modes
                } else {
                    dbprintf("INVALID MSG\n");
                }
            }
            if (IsTimerExpired(UPDATE_TIMER)) {
                currentHandFSMState = Updating;
            }
            break; //END IDLE
        case Updating:
//            dbprintf("[HAND] UPDATING STATE\n");
            if (ChangeFlag == DONT_CHANGE) {
                UpdateSensors();
                if (changeCounter++ >= NUM_CHANGES) {
                    ChangeFlag = CHANGE;
                    changeCounter = 0;
                }
            } else {
                SetMotors();
                ChangeFlag = DONT_CHANGE;

                for (finger = FIRST_MOTOR; finger <= LAST_MOTOR; finger++){
                    dbprintf("finger: %d\t", finger);
                    dbprintf("JAM: %d\t", JamFlag[finger]);
                    dbprintf("current: %u\t", motor[finger].currentDraw);
                    dbprintf("pwm: %d\t", motor[finger].pwm);
                    dbprintf("currentPos: %x\t", motor[finger].currentPos);
                    dbprintf("commandPos: %x\t", motor[finger].commandPos);
                    dbprintf("maintain: %d\n", maintainGripCounter[finger]);
                }
                dbprintf("\n");
            }
            updateSensorVals(THERVAL, GetTemperature());
            updateSensorVals(PRS1VAL, GetFSR(FSR_1));
            updateSensorVals(PRS2VAL, GetFSR(FSR_2));
            updateSensorVals(PRS3VAL, GetFSR(FSR_3));

            InitTimer(UPDATE_TIMER, UPDATE_TIME);
            currentHandFSMState = Idle;

            

            break; //END UPDATING
        default:
            dbprintf("[HANDFSM] IN DEFAULT\n");
            break; //END DEFAULT
    }
}

void SetMotors(void) {
    int finger;
    for (finger = FIRST_MOTOR; finger <= LAST_MOTOR; finger++) { //Iterate through all fingers & wrist
        /*Code to Set finger position*/                        
        if (motor[finger].currentPos != motor[finger].commandPos) { //Test: is finger at commanded pos (OPEN/CLOSE)
            ToggleFingerPos(finger);
        }
            /*Code to maintain grip when CLOSED*/
        else if (motor[finger].commandPos == CLOSE &&
                motor[finger].currentPos == CLOSE) {
            MaintainGrip(finger);
        } else {
            JamFlag[finger] = JAMMED;
            motor[finger].pwm = ZERO_PWM;
        }
        SetPWM(finger, motor[finger].commandPos, motor[finger].pwm);

        /*FOR TESTING*/
//        if (IsTimerExpired(9) && motor[finger].pwm == ZERO_PWM) {
//            motor[finger].commandPos ^= 1;
//            for (finger = FIRST_MOTOR; finger <= LAST_MOTOR; finger++){
//                dbprintf("finger: %d\t", finger);
//                dbprintf("JAM: %d\t", JamFlag[finger]);
//                dbprintf("current: %u\t", motor[finger].currentDraw);
//                dbprintf("pwm: %d\t", motor[finger].pwm);
//                dbprintf("currentPos: %x\t", motor[finger].currentPos);
//                dbprintf("commandPos: %x\t", motor[finger].commandPos);
//                dbprintf("maintain: %d\n", maintainGripCounter[finger]);
//            }
//            dbprintf("\n");
//        }
            
    }
}

void ToggleFingerPos(int finger) {
    motor[finger].pwm = FULL_PWM;

    uint16_t threshold;
    if (motor[finger].commandPos == OPEN)
        threshold = OPEN_THRESHOLD;
    else
        threshold = CLOSE_THRESHOLD;

    if (JamFlag[finger] == JAMMED) {
        if (motor[finger].currentDraw < FREE_THRESHOLD) {
            JamFlag[finger] = UNJAMMED;

        }
    } else if (motor[finger].currentDraw >= threshold && JamFlag[finger] == UNJAMMED) {
        motor[finger].pwm = ZERO_PWM;
        motor[finger].currentPos = motor[finger].commandPos;
        JamFlag[finger] = JAMMED;
                InitTimer(9, 1000);//FOR TESTING
    }
}

void MaintainGrip(int finger) {
    if (maintainGripCounter[finger]++ >= MAINTAIN_COUNT) {
        motor[finger].pwm = HALF_PWM;
        maintainGripCounter[finger] = 0;
    }
    if (motor[finger].currentDraw >= MAINTAIN_THRESHOLD) {
        motor[finger].pwm = ZERO_PWM;
    }
}

void UpdateSensors(void) {
    CalculateTemperature();
    CalculateForce();
    int finger;
    for (finger = FIRST_MOTOR; finger <= LAST_MOTOR; finger++) {
        motor[finger].currentDraw = (GAMMA * GetCurrent(finger)) + (1.0 - GAMMA) * motor[finger].currentDraw;
    }
}

