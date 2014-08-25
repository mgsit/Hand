/* 
 * File:   HandFSM.h
 * Author: tfurtado
 *
 * Created on February 14, 2014, 3:26 PM
 */

#ifndef HANDFSM_H
#define	HANDFSM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/*#DEFINES*/
/*Timers*/
#define UPDATE_TIMER                    1
#define UPDATE_TIME                     2

 /*MOTORS*/
#define NUM_MOTORS                      3
#define RIGHT_FINGER                    0
#define LEFT_FINGER                     1
#define MIDDLE_FINGER                   2
#define WRIST                           3

/*States*/
//#define OPEN                            1 //already defined in SlaveSPI.h
//#define CLOSE                           0
#define CW                              1
#define CCW                             0

/*Thresholds*/
#define CLOSE_THRESHOLD                 400
#define OPEN_THRESHOLD                  254
#define FREE_THRESHOLD                  150
#define MAINTAIN_THRESHOLD              100

/*Counts*/
#define MAINTAIN_COUNT                  30                  //Time for calling MaintainGrip() Units:0.1sec
#define NUM_CHANGES                     25                 //Number of samples before change occurs Units: Function Calls

/*Public Structs*/
struct Stats_t{
    uint16_t    currentDraw     :10;
    uint16_t    pwm             :10;
    bool        currentPos      :1;
    bool        commandPos      :1;
};
static struct Stats_t motor[NUM_MOTORS];
#define RIGHT_FINGER 0
#define LEFT_FINGER 1
#define MIDDLE_FINGER 2
#define WRIST 3




#ifdef	__cplusplus
}
#endif

#endif	/* HANDFSM_H */

