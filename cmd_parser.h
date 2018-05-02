/* 
 * File:   cmd_parser.h
 * Author: kolbi
 *
 * Created on February 28, 2018, 9:55 AM
 */

#ifndef CMD_PARSER_H
#define	CMD_PARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {CMD_NONE, CMD_ABS, CMD_REL, CMD_P,
                CMD_S, CMD_SETA, CMD_SETM, CMD_ZERO, 
                CMD_DRV_PEAK, CMD_DRV_GAIN, CMD_DRV_PIEZOONTIME,
                CMD_HEATER_PRETIME, CMD_HEATER_POSTTIME, CMD_HEATER_GAIN, CMD_HEATER_PWR,
                CMD_HEATER_TOGGLE,
                CMD_GET_ACTIVE, CMD_GET_MOVABLE, CMD_MOTION_STATUS, CMD_COMM_CHECK,
                CMD_PING
                } cmd_list;
int errorOut;
typedef union {
                char paramString[10]; // just in case... needed?
                int paramI;
} param;

struct CMD_STRUCT{
                cmd_list c;
                param p1;
                param p2;
};

int execCmd (struct CMD_STRUCT *s);
void parseCmd(char cmdString[], struct CMD_STRUCT *s);

/*
 * MOVEMENT
 */
void doMove(void);
int startRelMove(int steps);
int startAbsMove(int steps);
int stop(void);
int zeroPosition(void);
int getPosition(int a);

/*
 * DRV (PIEZO DRIVER)
 */
int setDrvPeak(int);
int setDrvGain(int);
int setDrvPiezoOnTime(int);

/*
 * HEATERS
 */
int setHeaterPreTime(int);
int setHeaterPostTime(int);
int setLEDBrightnessRange(int gain, int topOrBottom);
int setLEDPwr(int pwr, int topOrBottom);
int setHeaterToggle(int num, int topOrB);
/*
 * MISC
 */
int setActive(int probeNum, int onOff);
int setMoveMask(int probeNum, int onOff);
int getActive(void);
int getMoveMask(void);
int getMotionStatus(void);
int commCheck(void);

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* CMD_PARSER_H */

