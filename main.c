/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65
        Device            :  PIC16F18345
        Driver Version    :  2.00
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/


// https://os.mbed.com/users/akashvibhute/code/DRV2665/file/e2c726c628dc/drv2665.cpp/ --> drv code

/*
 * NOTES
 * 2/28/18: both LED banks are controllable, demo sequence, single-char serial, basic motion commands
 * 3/30/18: stable version. next, integrating piezo driver. stable version saved in zip
 */

/*
 * TODO:
 * check timing of piezo motion and interrupts
 * 
 */
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mcc_generated_files/LEDcomm.h"
#include "cmd_parser.h"
#include "drv2665.h"
#define DELAYMSAPPROX(x) for(int t=0; t<x; t++) __delay_ms(1)

// CONSTANTS -- UNEDITABLE
const float TMR0_RANGE = 255/(2621.44-10.24);
const float TMR2_RANGE = 255/(163.84-0.64);


// CONSTANTS -- SOME WILL BE EDITABLE/SETTABLE LATER!
#define TOTALNUMCHANS 24 // max number of channels
int heater_pre_time_ms = 10; // milliseconds
int heater_post_time_ms = 10; // milliseconds
long inter_step_interval_ms = 1000;


int piezo_on_time_ms = 1000;
const int LEDpower = 63;

/*
 SERIAL
 */
char readdata[20]; // max length of user command!
int cmdRead = 0;
int readIdx = 0;
struct CMD_STRUCT cmd_struct={0,0,0};

/*
 ACTUATOR
 */
unsigned long demoLong = 0x0001;
//          R1      R8(MSB)
// 0b000000001000000100000000;
//                            B1      G1       R1    R8
unsigned long moveVector_ups   = 0;//0b10000000100000001000000100000000;
unsigned long moveVector_downs = 0;//0b01000000010000000100000000000000;


/*
 MOTION CONTROL
 */
unsigned long activeMask=0x00;//0x80808100; // currently active actuators. activeMask must be a SUBSET of moveMask!!
unsigned long moveMask=0x00; // actuators that should move

/*
 * demo mode constants for toggling individual LEDs on/off
 */
unsigned long demoHeaterToggle_top = 0;
unsigned long demoHeaterToggle_bot = 0;

signed long currentPositions_steps[TOTALNUMCHANS]=0;
signed long remainingSteps=0;
signed long absCommand = 0; // command (in steps) (will eventually be sent over serial)
enum moveType{NONE, REL_MOVE, ABS_MOVE} motionType = NONE;

/* 
 FUNCTIONS
 */

void doAbsMove(void);
void doRelMove(void);
void main_initialize(void);
void calcHeaterPins(void);
void demoSequence(void);



/*
                         Main application
 */
void main(void)
{   
    // initialize the device
    SYSTEM_Initialize();
    __delay_ms(500);
    
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    printf("\n\nsystem done\n");
    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    main_initialize();
    
    while (1)
    {

        if (eusartRxCount!=0) {
                readdata[readIdx] = EUSART_Read();
                if(readdata[readIdx] == '\n'){
                    readdata[readIdx] = 0;
                    cmdRead = 1;
                }
                readIdx++;
        } 
        
        // parse commands here
        if(cmdRead){
            //printf("i = %d, FINAL STRING: %s \n",readIdx, readdata);
            parseCmd(readdata, &cmd_struct);
            //printf("parsed command: %d %d %d\n", cmd_struct.c, cmd_struct.p1.paramI, cmd_struct.p2.paramI);
            cmdRead = 0; // reset
            readIdx=0;
            execCmd(&cmd_struct);
            
        }
        

        switch(motionType){
            case REL_MOVE: doRelMove(); break;
            case ABS_MOVE: doAbsMove(); break;
            default: break;
        }
    }
}


void main_initialize(void){
    // LEDs
    LEDsPwr(LEDpower, 1); // set max power (top))
    LEDsPwr(LEDpower, 2); // set max power (bottom)
    
    LED_setBrightnessRange(1, 1); // set max range (top)
    LED_setBrightnessRange(1, 2); // set max range (bottom)
    LED_ENABLE_SetHigh(); // LEDs off
    
    // timers
    TMR2_LoadPeriodRegister((heater_post_time_ms*TMR2_RANGE)-1); // delay 1
    TMR0_Load8bitPeriod(((piezo_on_time_ms-heater_pre_time_ms)*TMR0_RANGE)-1); // delay 2
    
    // piezo driver
    drv_init(DRV2665_100_VPP_GAIN, DRV2665_20_MS_IDLE_TOUT);
    printf("initialized\n");
}



// DEMO FUNCTIONS!
/*
void doAbsMove(void){};
void doRelMove(void){};
int startRelMove(int steps){return 0;};
int startAbsMove(int steps){return 0;};
int stop(void){return 0;};
int zeroPosition(void){return 0;};
int getPosition(void){return 0;};

int setDrvPeak(int){return 0;};
int setDrvGain(int){return 0;};
int setDrvPiezoOnTime(int){return 0;};

int setHeaterPreTime(int){return 0;};
int setHeaterPostTime(int){return 0;};
int setLEDBrightnessRange(int gain, int topOrBottom){return 0;};
int setLEDPwr(int pwr, int topOrBottom){return 0;};
int setHeaterToggle(int num, int topOrB){return 0;};

int setActive(int probeNum, int onOff){return 0;};
int setMoveMask(int probeNum, int onOff){return 0;};
int getActive(void){return 0;};
int getMoveMask(void){return 0;};
int getMotionStatus(void){return 0;};
int commCheck(void){return 0;};
*/
// END DEMO FUNCTIONS

 int doMove(void){
    // over-simplified move code
    calcHeaterPins();
    LED_ENABLE_SetHigh();
    LEDsOn(first_top, 1);
    LEDsOn(first_bottom,2);
    LED_ENABLE_SetLow(); // LEDs on
    DELAYMSAPPROX(heater_pre_time_ms);
    TMR2_StartTimer(); // timer to turn off LEDs first time
    TMR0_StartTimer(); // timer to turn on LEDs second time
    
    drv_write_DC(drv_peak_val, piezo_on_time_ms); // piezo command
    
    DELAYMSAPPROX(heater_post_time_ms);

    LED_ENABLE_SetHigh(); // LEDs off
    
    // PUT IN 1S DELAY HERE?
    DELAYMSAPPROX(inter_step_interval_ms);
    return 0;
    
    
}


void calcHeaterPins(void){
    first_top = moveVector_downs & moveMask;
    first_bottom = (~moveVector_downs) & moveMask;
    second_top = moveVector_ups & moveMask;
    second_bottom = (~moveVector_ups) & moveMask;
}

// demo LED sequence on both top and bottom LEDs
void demoSequence(void){
    LEDsOn(demoLong, 1);
    LEDsOn(demoLong, 2);
    LED_ENABLE_SetLow(); // LEDs on
    __delay_ms(100);
    LED_ENABLE_SetHigh(); // LEDs off
    
    if(demoLong >= 0b10000000000000000000000000000000){
        demoLong = 0x0001;
    }
    else demoLong <<= 1;
}


int startRelMove(int steps){
    motionType = REL_MOVE;
    remainingSteps = steps;
    printf("A\n");
    return 0;
}
int startAbsMove(int steps){
    motionType = ABS_MOVE;
    absCommand = steps;
    printf("A\n");
    return 0;
}

// stop motion immediately
int stop(void){
    motionType = NONE;
    moveVector_ups = 0;
    moveVector_downs = 0;
    remainingSteps = 0;
    printf("A\n");
    return 0;
}

// all move to absolute position
void doAbsMove(void){
    
    
    int allThere=1; // 1 if all current positions are = command position
    for(int i=0;i<TOTALNUMCHANS;i++){
        if((activeMask & (1UL << i)) > 0)
            allThere &= (currentPositions_steps[i] == absCommand);
    }
    
    if(allThere){ // if all have reached command position -> stop moving
        printf("abs: all there\n");
        motionType = NONE;
        moveVector_ups = 0;
        moveVector_downs = 0;
        return;
    }
    else{
        for(int i=0;i<TOTALNUMCHANS;i++){
            //set vectors to 1 based on moveMask and delta position
            moveVector_ups |= (activeMask & (1UL << i)) & ((currentPositions_steps[i] > absCommand)<<i);
            moveVector_downs |= (activeMask & (1UL << i)) & ((currentPositions_steps[i] < absCommand)<<i);
            
        }
        doMove();
        
        for(int i=0;i<TOTALNUMCHANS;i++)
            currentPositions_steps[i] += ((moveVector_downs & (1UL << i)) > 0) - ((moveVector_ups & (1UL << i)) > 0);
        
    }
}

// all move by relative step
void doRelMove(void){
    if(remainingSteps == 0){
        motionType = NONE;
        moveVector_ups = 0;
        moveVector_downs = 0;
        return;
    }
    else{
        int j;
        if (remainingSteps > 0){ // move down
            moveVector_ups = 0;
            moveVector_downs = activeMask;
            
            doMove();
            remainingSteps--;
            
            // adjust current position array
            for(j = 0; j<TOTALNUMCHANS; j++)
                currentPositions_steps[j] += (activeMask & (1UL<<j) > 0);
            
        }
        else{ // move up
            moveVector_ups = activeMask;
            moveVector_downs = 0;

            doMove();
            remainingSteps++;
            for(j = 0; j<TOTALNUMCHANS; j++)
                currentPositions_steps[j] -= (activeMask & (1UL<<j) > 0);
            
        }
    }
}

// zero positions of active probes
int zeroPosition(void){
    for(int i = 0; i<TOTALNUMCHANS; i++){
        if((activeMask & (1UL << i)) > 0)
            currentPositions_steps[i]=0;
    }
    printf("A\n");
    return 0;
}

// prints position of actuator index a
int getPosition(int a){
    
    if (a>=0 && a < TOTALNUMCHANS){
        printf("%d\n", currentPositions_steps[a]);
        return 0;
    }
    
    return 1;
}

// sets piezo square wave peak value to [0, 127] range
int setDrvPeak(int pk){
    drv_peak_val = pk;
    printf("A\n");
    return 0;
};

// sets piezo square wave gain (25,50,75,100)
int setDrvGain(int gain){
    int setGain = 0;
    switch (gain){
        case 0: setGain = DRV2665_25_VPP_GAIN; break;
        case 1: setGain = DRV2665_50_VPP_GAIN; break;
        case 2: setGain = DRV2665_75_VPP_GAIN; break;
        case 3: setGain = DRV2665_100_VPP_GAIN; break;
        default: printf("E\n"); return 1;
    }
    drv_write(DRV2665_CTRL_1, setGain); // set register
    printf("A\n");
    return 0;
};

int setDrvPiezoOnTime(int t){
    piezo_on_time_ms = t;
    TMR0_Load8bitPeriod(((piezo_on_time_ms-heater_pre_time_ms)*TMR0_RANGE)-1); // adjust timer delay 2
    printf("A\n");
    return 0;
};


 //HEATERS

// set pre heat time (before piezo comes on), in milliseconds
int setHeaterPreTime(int t){
    heater_pre_time_ms = t;
    TMR0_Load8bitPeriod(((piezo_on_time_ms-heater_pre_time_ms)*TMR0_RANGE)-1); // delay 2
    printf("A\n");
    return 0;
};
int setHeaterPostTime(int t){
    heater_post_time_ms = t;
    TMR2_LoadPeriodRegister((heater_post_time_ms*TMR2_RANGE)-1); // delay 1
    printf("A\n");
    return 0;
};

//range: 0: low (default), 1: high
// topOrBottom (tOB): 1:top, 2:bottom;
// this is called by the "ledgain" command
int setLEDBrightnessRange(int r, int tOB){
    LED_setBrightnessRange(r, tOB);
    printf("A\n");
    return 0;
};

// pwr is int (0-63)
// topOrBottom: 1:top, 2:bottom;
int setLEDPwr(int pwr, int topOrBottom){
    LEDsPwr(pwr, topOrBottom);
    printf("A\n");
    return 0;
};


// only toggles on/off one heater at a time!
// topOrB: 1:top, 2:bottom;
int setHeaterToggle(int num, int topOrB){
    LED_ENABLE_SetLow();
    if(topOrB == 1){
        demoHeaterToggle_top ^= 1UL << num;
        LEDsOn(demoHeaterToggle_top, 1);
    }
    else if (topOrB == 2){
        demoHeaterToggle_bot ^= 1UL << num;
        LEDsOn(demoHeaterToggle_bot, 2);
    }

    printf("A\n");
    return 0;
};

// turns on ONLY ACTIVE LEDs for specified time period (ms)
// onTime: time for heater to be on (ms)
// topOrB: 1:top, 2:bottom;
int timedActiveHeatOn(long onTime, int topOrB){
    __delay_ms(2000);
    LED_ENABLE_SetLow();
    if(topOrB == 1){
        LEDsOn(activeMask, 1);
    }
    else if (topOrB == 2){
        LEDsOn(activeMask, 2);
    }
    DELAYMSAPPROX(onTime);
    LED_ENABLE_SetHigh();
    LEDsOn(0, 1);
    LEDsOn(0, 2);
    printf("A\n");
    return 0;
    

};

//MISC

// onOff: 1 to turn on (make active), 0 to turn off (make inactive)
int setActive(int probeNum, int onOff){
    // error out if this bit is not in the move mask
    // something like activeMask |= (1<<probeNum);
    if(moveMask & 1UL << probeNum){
        if (onOff)
            activeMask |= 1UL << probeNum; // set bit
        else
            activeMask &= ~(1UL << probeNum); // clear bit
        printf("A\n");
        return 0;
    }
    printf("E\n");
    return 1;
}

int setMoveMask(int probeNum, int onOff){
    // error out if probe num > TOTALNUMCHANNELS
    // something like moveMask |= (1<<probeNum);
    // note: probes are indexed 0:TOTALNUMCHANNELS-1!!!
    if (probeNum < TOTALNUMCHANS){
        if (onOff)
            moveMask |= (1UL<<probeNum);
        else
            moveMask &= ~(1UL<< probeNum); // clear bit
        printf("A\n");
        return 0;
    }
    printf("E\n");
    return 1;
}

// print active probe list (right to left) i.e. 0 0 1 0 1
int getActive(void){
    for(int i=TOTALNUMCHANS-1; i>=0; i--){
        printf("%d ", (activeMask >> i) & 1 );
    }
    printf("\n");
    return 0;
}

int getMoveMask(void){
    for(int i=TOTALNUMCHANS-1; i>=0; i--){
        printf("%d ", (moveMask >> i) & 1 );
    }
    printf("\n");
    return 0;
};
int getMotionStatus(void){
    switch(motionType){
            case REL_MOVE: printf("R"); break;
            case ABS_MOVE: printf("A"); break;
            default: printf("N"); break;
        }
};

int commCheck(void){    
    printf("\n----starting comm test----\n\n");
    
    
    printf("testing top LED driver...");
    if (!LED_testComm(1))
        printf("passed\n");
    else
        printf("FAILED\n");
    
    printf("opens: ");
    LED_test_openshort(LED_OPEN_DETECTION, 1);
    printf("\n\nshorts: ");
    LED_test_openshort(LED_SHORT_DETECTION, 1);
    
    
    printf("\ntesting bottom LED driver...\n");
    if (!LED_testComm(2))
        printf("passed\n");
    else
        printf("FAILED\n");
    
    printf("opens: ");
    LED_test_openshort(LED_OPEN_DETECTION, 2);
    printf("\n\nshorts: ");
    LED_test_openshort(LED_SHORT_DETECTION, 2);
    
    printf("\ntesting piezo driver...");
    if(drv_read(DRV2665_CTRL_1) > 0)
        printf("passed\n\n");
    else
        printf("FAILED\n\n");
    printf("----finished comm test----\n");
    
}
