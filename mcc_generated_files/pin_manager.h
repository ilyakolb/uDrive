/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.35
        Device            :  PIC16F18346
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set LED_CLK aliases
#define LED_CLK_TRIS               TRISAbits.TRISA2
#define LED_CLK_LAT                LATAbits.LATA2
#define LED_CLK_PORT               PORTAbits.RA2
#define LED_CLK_WPU                WPUAbits.WPUA2
#define LED_CLK_OD                ODCONAbits.ODCA2
#define LED_CLK_ANS                ANSELAbits.ANSA2
#define LED_CLK_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define LED_CLK_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define LED_CLK_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define LED_CLK_GetValue()           PORTAbits.RA2
#define LED_CLK_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define LED_CLK_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define LED_CLK_SetPullup()      do { WPUAbits.WPUA2 = 1; } while(0)
#define LED_CLK_ResetPullup()    do { WPUAbits.WPUA2 = 0; } while(0)
#define LED_CLK_SetPushPull()    do { ODCONAbits.ODCA2 = 0; } while(0)
#define LED_CLK_SetOpenDrain()   do { ODCONAbits.ODCA2 = 1; } while(0)
#define LED_CLK_SetAnalogMode()  do { ANSELAbits.ANSA2 = 1; } while(0)
#define LED_CLK_SetDigitalMode() do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set LED_LE1 aliases
#define LED_LE1_TRIS               TRISBbits.TRISB4
#define LED_LE1_LAT                LATBbits.LATB4
#define LED_LE1_PORT               PORTBbits.RB4
#define LED_LE1_WPU                WPUBbits.WPUB4
#define LED_LE1_OD                ODCONBbits.ODCB4
#define LED_LE1_ANS                ANSELBbits.ANSB4
#define LED_LE1_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define LED_LE1_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define LED_LE1_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define LED_LE1_GetValue()           PORTBbits.RB4
#define LED_LE1_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define LED_LE1_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define LED_LE1_SetPullup()      do { WPUBbits.WPUB4 = 1; } while(0)
#define LED_LE1_ResetPullup()    do { WPUBbits.WPUB4 = 0; } while(0)
#define LED_LE1_SetPushPull()    do { ODCONBbits.ODCB4 = 0; } while(0)
#define LED_LE1_SetOpenDrain()   do { ODCONBbits.ODCB4 = 1; } while(0)
#define LED_LE1_SetAnalogMode()  do { ANSELBbits.ANSB4 = 1; } while(0)
#define LED_LE1_SetDigitalMode() do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set SDA1 aliases
#define SDA1_TRIS               TRISBbits.TRISB5
#define SDA1_LAT                LATBbits.LATB5
#define SDA1_PORT               PORTBbits.RB5
#define SDA1_WPU                WPUBbits.WPUB5
#define SDA1_OD                ODCONBbits.ODCB5
#define SDA1_ANS                ANSELBbits.ANSB5
#define SDA1_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define SDA1_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define SDA1_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define SDA1_GetValue()           PORTBbits.RB5
#define SDA1_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define SDA1_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define SDA1_SetPullup()      do { WPUBbits.WPUB5 = 1; } while(0)
#define SDA1_ResetPullup()    do { WPUBbits.WPUB5 = 0; } while(0)
#define SDA1_SetPushPull()    do { ODCONBbits.ODCB5 = 0; } while(0)
#define SDA1_SetOpenDrain()   do { ODCONBbits.ODCB5 = 1; } while(0)
#define SDA1_SetAnalogMode()  do { ANSELBbits.ANSB5 = 1; } while(0)
#define SDA1_SetDigitalMode() do { ANSELBbits.ANSB5 = 0; } while(0)

// get/set SCL1 aliases
#define SCL1_TRIS               TRISBbits.TRISB6
#define SCL1_LAT                LATBbits.LATB6
#define SCL1_PORT               PORTBbits.RB6
#define SCL1_WPU                WPUBbits.WPUB6
#define SCL1_OD                ODCONBbits.ODCB6
#define SCL1_ANS                ANSELBbits.ANSB6
#define SCL1_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define SCL1_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define SCL1_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define SCL1_GetValue()           PORTBbits.RB6
#define SCL1_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define SCL1_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define SCL1_SetPullup()      do { WPUBbits.WPUB6 = 1; } while(0)
#define SCL1_ResetPullup()    do { WPUBbits.WPUB6 = 0; } while(0)
#define SCL1_SetPushPull()    do { ODCONBbits.ODCB6 = 0; } while(0)
#define SCL1_SetOpenDrain()   do { ODCONBbits.ODCB6 = 1; } while(0)
#define SCL1_SetAnalogMode()  do { ANSELBbits.ANSB6 = 1; } while(0)
#define SCL1_SetDigitalMode() do { ANSELBbits.ANSB6 = 0; } while(0)

// get/set RB7 procedures
#define RB7_SetHigh()    do { LATBbits.LATB7 = 1; } while(0)
#define RB7_SetLow()   do { LATBbits.LATB7 = 0; } while(0)
#define RB7_Toggle()   do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define RB7_GetValue()         PORTBbits.RB7
#define RB7_SetDigitalInput()   do { TRISBbits.TRISB7 = 1; } while(0)
#define RB7_SetDigitalOutput()  do { TRISBbits.TRISB7 = 0; } while(0)
#define RB7_SetPullup()     do { WPUBbits.WPUB7 = 1; } while(0)
#define RB7_ResetPullup()   do { WPUBbits.WPUB7 = 0; } while(0)
#define RB7_SetAnalogMode() do { ANSELBbits.ANSB7 = 1; } while(0)
#define RB7_SetDigitalMode()do { ANSELBbits.ANSB7 = 0; } while(0)

// get/set LED_LE2 aliases
#define LED_LE2_TRIS               TRISCbits.TRISC0
#define LED_LE2_LAT                LATCbits.LATC0
#define LED_LE2_PORT               PORTCbits.RC0
#define LED_LE2_WPU                WPUCbits.WPUC0
#define LED_LE2_OD                ODCONCbits.ODCC0
#define LED_LE2_ANS                ANSELCbits.ANSC0
#define LED_LE2_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define LED_LE2_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define LED_LE2_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define LED_LE2_GetValue()           PORTCbits.RC0
#define LED_LE2_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define LED_LE2_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define LED_LE2_SetPullup()      do { WPUCbits.WPUC0 = 1; } while(0)
#define LED_LE2_ResetPullup()    do { WPUCbits.WPUC0 = 0; } while(0)
#define LED_LE2_SetPushPull()    do { ODCONCbits.ODCC0 = 0; } while(0)
#define LED_LE2_SetOpenDrain()   do { ODCONCbits.ODCC0 = 1; } while(0)
#define LED_LE2_SetAnalogMode()  do { ANSELCbits.ANSC0 = 1; } while(0)
#define LED_LE2_SetDigitalMode() do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set LED_ENABLE aliases
#define LED_ENABLE_TRIS               TRISCbits.TRISC1
#define LED_ENABLE_LAT                LATCbits.LATC1
#define LED_ENABLE_PORT               PORTCbits.RC1
#define LED_ENABLE_WPU                WPUCbits.WPUC1
#define LED_ENABLE_OD                ODCONCbits.ODCC1
#define LED_ENABLE_ANS                ANSELCbits.ANSC1
#define LED_ENABLE_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define LED_ENABLE_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define LED_ENABLE_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define LED_ENABLE_GetValue()           PORTCbits.RC1
#define LED_ENABLE_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define LED_ENABLE_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define LED_ENABLE_SetPullup()      do { WPUCbits.WPUC1 = 1; } while(0)
#define LED_ENABLE_ResetPullup()    do { WPUCbits.WPUC1 = 0; } while(0)
#define LED_ENABLE_SetPushPull()    do { ODCONCbits.ODCC1 = 0; } while(0)
#define LED_ENABLE_SetOpenDrain()   do { ODCONCbits.ODCC1 = 1; } while(0)
#define LED_ENABLE_SetAnalogMode()  do { ANSELCbits.ANSC1 = 1; } while(0)
#define LED_ENABLE_SetDigitalMode() do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set LED_MOSI aliases
#define LED_MOSI_TRIS               TRISCbits.TRISC2
#define LED_MOSI_LAT                LATCbits.LATC2
#define LED_MOSI_PORT               PORTCbits.RC2
#define LED_MOSI_WPU                WPUCbits.WPUC2
#define LED_MOSI_OD                ODCONCbits.ODCC2
#define LED_MOSI_ANS                ANSELCbits.ANSC2
#define LED_MOSI_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define LED_MOSI_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define LED_MOSI_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define LED_MOSI_GetValue()           PORTCbits.RC2
#define LED_MOSI_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define LED_MOSI_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define LED_MOSI_SetPullup()      do { WPUCbits.WPUC2 = 1; } while(0)
#define LED_MOSI_ResetPullup()    do { WPUCbits.WPUC2 = 0; } while(0)
#define LED_MOSI_SetPushPull()    do { ODCONCbits.ODCC2 = 0; } while(0)
#define LED_MOSI_SetOpenDrain()   do { ODCONCbits.ODCC2 = 1; } while(0)
#define LED_MOSI_SetAnalogMode()  do { ANSELCbits.ANSC2 = 1; } while(0)
#define LED_MOSI_SetDigitalMode() do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set LED_MISO aliases
#define LED_MISO_TRIS               TRISCbits.TRISC6
#define LED_MISO_LAT                LATCbits.LATC6
#define LED_MISO_PORT               PORTCbits.RC6
#define LED_MISO_WPU                WPUCbits.WPUC6
#define LED_MISO_OD                ODCONCbits.ODCC6
#define LED_MISO_ANS                ANSELCbits.ANSC6
#define LED_MISO_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define LED_MISO_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define LED_MISO_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define LED_MISO_GetValue()           PORTCbits.RC6
#define LED_MISO_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define LED_MISO_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define LED_MISO_SetPullup()      do { WPUCbits.WPUC6 = 1; } while(0)
#define LED_MISO_ResetPullup()    do { WPUCbits.WPUC6 = 0; } while(0)
#define LED_MISO_SetPushPull()    do { ODCONCbits.ODCC6 = 0; } while(0)
#define LED_MISO_SetOpenDrain()   do { ODCONCbits.ODCC6 = 1; } while(0)
#define LED_MISO_SetAnalogMode()  do { ANSELCbits.ANSC6 = 1; } while(0)
#define LED_MISO_SetDigitalMode() do { ANSELCbits.ANSC6 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()    do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()   do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()   do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()         PORTCbits.RC7
#define RC7_SetDigitalInput()   do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()  do { TRISCbits.TRISC7 = 0; } while(0)
#define RC7_SetPullup()     do { WPUCbits.WPUC7 = 1; } while(0)
#define RC7_ResetPullup()   do { WPUCbits.WPUC7 = 0; } while(0)
#define RC7_SetAnalogMode() do { ANSELCbits.ANSC7 = 1; } while(0)
#define RC7_SetDigitalMode()do { ANSELCbits.ANSC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/