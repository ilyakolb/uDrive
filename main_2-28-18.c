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
 */

#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mcc_generated_files/LEDcomm.h"

// CONSTANTS
#define TOTALNUMCHANS 24 // max number of channels
#define HEATER_ON_TIME 1000 // milliseconds
#define PIEZO_ON_TIME  1000 // milliseconds

/*
 SERIAL
 */
char readdata[10];
char dummy;
int cmdRead = 0;
int i = 0;

/*
 ACTUATOR
 */
unsigned long demoLong = 0x0001;
//          R1      R8(MSB)
// 0b000000001000000100000000;
//                            B1      G1       R1    R8
long moveVector_ups   = 0;//0b10000000100000001000000100000000;
long moveVector_downs = 0;//0b01000000010000000100000000000000;
long first_top = 0;
long second_top = 0;
long first_bottom = 0;
long second_bottom = 0;

/*
 MOTION CONTROL
 */
long activeMask=0x80808100; // currently active actuators. activeMask must be a SUBSET of moveMask!!
long moveMask=0xFFFFFFFF; // actuators that should move

int currentPositions_steps[TOTALNUMCHANS]=0;
int remainingSteps=0;
int relDist = -5; // num steps to move (will eventually be sent over serial)
int absCommand = -8; // command (in steps) (will eventually be sent over serial)
enum moveType{NONE, REL_MOVE, ABS_MOVE} motionType = ABS_MOVE;

/* 
 FUNCTIONS
 */

void initialize(void);
void calcHeaterPins(void);
void demoSequence(void);

void doMove(void);
void startRelMove(int steps);
void startAbsMove(int steps);
void doAbsMove(void);
void doRelMove(void);
void stop(void);
void zeroPosition(void);
void getPosition(void);

/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    initialize();
    
    while (1)
    {
        // serial parser
        
        while (eusartRxCount!=0) { // TODO: MAKE THIS WORK WITH MULTI INPUTS!
                readdata[i] = EUSART_Read();
                
                //printf("%d", eusartRxCount);
                i++;
                cmdRead = 1;
        } 
        
        // do other stuff here
        if(cmdRead){
            //printf("i = %d, FINAL STRING: %s \n",i, readdata);
            
            cmdRead = 0; // reset
            i=0;
            // parse message
            switch (readdata[0]){
                case 'r': startRelMove(relDist); // rel move. relDist should be set here!
                break;
                case 'a': startAbsMove(absCommand); // abs move. absCommand should be set here!
                break;
                case 's' : stop(); break; // stop
                case 'z' : zeroPosition(); break;
                case 'm' : break; // set active mask
                // make sure active mask is a SUBSET of moveMask
                case 'x' : printf("!"); break; // status
                    default: 
                            printf("unknown command");
                    break;

                }
            
        }
        

        switch(motionType){
            case REL_MOVE: doRelMove(); break;
            case ABS_MOVE: doAbsMove(); break;
            default: break;
        }
    }
}


void initialize(void){
    //send_2bytes_latchless((unsigned int)demoLong);
    //send_last_2bytes((unsigned int)((demoLong>>16) & 0xFFFF), 15);
    LED_ENABLE_SetHigh();
    send_2length_2bytes((unsigned int)demoLong, (unsigned int)((demoLong>>16) & 0xFFFF), 15, 1);
    send_2length_2bytes((unsigned int)demoLong, (unsigned int)((demoLong>>16) & 0xFFFF), 15, 2);
    LED_ENABLE_SetLow();

}


void doMove(void){
    printf("doMove\n");
    // over-simplified move code
    calcHeaterPins();
    LED_ENABLE_SetHigh();
    LEDsOn(first_top, 1);
    LEDsOn(first_bottom,2);
    LED_ENABLE_SetLow(); // LEDs on
    __delay_ms(HEATER_ON_TIME);
    LED_ENABLE_SetHigh(); // LEDs off
    
    __delay_ms(PIEZO_ON_TIME);     // MOVE PIEZEO

    LEDsOn(second_top, 1);
    LEDsOn(second_bottom,2);
    LED_ENABLE_SetLow(); // LEDs on
    __delay_ms(HEATER_ON_TIME);
    LED_ENABLE_SetHigh(); // LEDs off

    
    
    
}
/**
 End of File
*/

void calcHeaterPins(void){
    first_top = moveVector_downs & moveMask;
    first_bottom = (~moveVector_downs) & moveMask;
    second_top = moveVector_ups & moveMask;
    second_bottom = (~moveVector_ups) & moveMask;
}

// demo LED sequence
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


void startRelMove(int steps){
    motionType = REL_MOVE;
    remainingSteps = relDist;
}
void startAbsMove(int steps){
    motionType = ABS_MOVE;
}

// stop motion immediately
void stop(void){
    motionType = NONE;
    moveVector_ups = 0;
    moveVector_downs = 0;
    remainingSteps = 0;
}

// all move to absolute position
void doAbsMove(void){
    
    
    int allThere=1; // 1 if all current positions are = command position
    for(int i=0;i<TOTALNUMCHANS;i++){
        if((activeMask & (1 << i)) > 0)
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
            moveVector_ups |= (activeMask & (1 << i)) & ((currentPositions_steps[i] > absCommand)<<i);
            moveVector_downs |= (activeMask & (1 << i)) & ((currentPositions_steps[i] < absCommand)<<i);
            
        }
        doMove();
        
        for(int i=0;i<TOTALNUMCHANS;i++)
            currentPositions_steps[i] += ((moveVector_downs & (1 << i)) > 0) - ((moveVector_ups & (1 << i)) > 0);
        
    }
}

// all move by relative step
void doRelMove(void){
    printf("doRelMove\n");
    if(remainingSteps == 0){
        printf("rel: all there\n");
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
                currentPositions_steps[j] += (activeMask & (1<<j) > 0);
            
        }
        else{ // move up
            moveVector_ups = activeMask;
            moveVector_downs = 0;

            doMove();
            remainingSteps++;
            for(j = 0; j<TOTALNUMCHANS; j++)
                currentPositions_steps[j] -= (activeMask & (1<<j) > 0);
            
        }
    }
}
void zeroPosition(void){
    for(int i = 0; i<TOTALNUMCHANS; i++){
        currentPositions_steps[i]=0;
    }
}

// write positions of active actuators to USART
void getPosition(void){
    
    //TODO: THESE SHOULD WRITE TO USART!
    for(int i = 0; i<TOTALNUMCHANS; i++){
        if((activeMask & (1 << i)) > 0)
            printf("%d ", currentPositions_steps[i]);
    }
    printf("\n");
}
/* SINGLE CHAR SERIAL PARSER: WORKS
     while (1)
    {
        if (eusartRxCount!=0) { // TODO: MAKE THIS WORK WITH MULTI INPUTS!
            readdata = EUSART_Read();
            switch (readdata){
                    case '1': IO_RA4_SetHigh();
                              IO_RA5_SetLow();
                    break;
                    case '2': IO_RA4_SetLow();
                              IO_RA5_SetHigh();
                    break;
                    default: IO_RA4_SetHigh();
                             IO_RA5_SetHigh();
                            printf("test");
                    break;

                }
        }*/