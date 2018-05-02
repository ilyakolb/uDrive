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

#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "mcc_generated_files/LEDcomm.h"

// CONSTANTS
#define TOTALNUMCHANS 24 // max number of channels
#define HEATER_ON_TIME 1000 // milliseconds
#define PIEZO_ON_TIME  1000 // milliseconds

/*
 VARIABLES
 */
char readdata[10];
unsigned long demoLong = 0x0001;
int cmdRead = 0;
//          R1      R8(MSB)
// 0b000000001000000100000000;
//                        B1      G1       R1    R8
long moveVector_ups   = 0b10000000100000001000000100000000;
long moveVector_downs = 0b01000000010000000100000000000000;
long first_top = 0;
long second_top = 0;
long first_bottom = 0;
long second_bottom = 0;

// serial
int i = 0;

/* 
 FUNCTIONS
 */
void doMove(void);
void initialize(void);
void calcHeaterPins(void);
void demoSequence(void);

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
        /*
        while (eusartRxCount!=0) { // TODO: MAKE THIS WORK WITH MULTI INPUTS!
                readdata[i] = EUSART_Read();
                
                //printf("%d", eusartRxCount);
                i++;
                cmdRead = 1;
        } */
        
        __delay_ms(100);
        demoSequence();
        //doMove();
        // do other stuff here
        if(cmdRead){
            printf("i = %d, FINAL STRING: %s \n",i, readdata);
            cmdRead = 0; // reset
            i=0;
        }
        // Add your application code
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
    printf("doMove");
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
    first_top = moveVector_downs;
    first_bottom = ~moveVector_downs;
    second_top = moveVector_ups;
    second_bottom = ~moveVector_ups;
}

void append(char* s, char c)
{
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
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
/*
 TODO:
 * demo sequence that lights up all LEDs
 */

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