/*
 * File:   newmain.c
 * Author: smithr13
 *
 * Created on May 18, 2016, 11:28 AM
 */

#define _XTAL_FREQ 32000000
#define RECEIVE_BUFFER_SIZE 80
#define TRANSFER_BUFFER_SIZE 10


//#define IDSMOSI   RA0
//#define ISCLK     RA1
//#define LED2      RA2        EP2
//#define MCLR      RA3
//#define SMISO     RA4
//#define SSS       RA5
//
//#define SDO2      RB4        LEDG   0x10   comms
//#define EP2       RB5        VTemp
//#define LE1       RB6        T2
//#define LEDCLK    RB7        LE2
//
//#define T2        RC0        SDO2 
//#define LE2       RC1 //+    LEDB   0x02   idle   
//#define ASDO      RC2 //~~   LEDR   0x04   heat/therm
//#define EP1       RC3        LEDCLK
//#define T1        RC4        SDI1
//#define LED1      RC5        LE1
//#define SDI1      RC6        EP1
//#define DOAS/LED3 RC7 //~~   T1


//#include <xc.h>
#include <pic16f18345.h>
#include <math.h>
#include <htc.h>
#include <stdio.h>
#include <stdlib.h>

// PIC16F18345 Configuration Bit Settings
// <editor-fold defaultstate="collapsed" desc="PIC16F18345 Configuration Bit Settings">
// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="Global Variable Intializations">
unsigned char message_complete = 0; //Flag to indicate a full message has been                                      //received over SPI
unsigned char input_buffer[RECEIVE_BUFFER_SIZE] = 0;
unsigned char output_buffer[TRANSFER_BUFFER_SIZE] = 0;
unsigned char buffer_position = 0;
unsigned char reply_mode = 0XFF; //Reply from (1) buffer 
                                //or (0) with previously received byte
unsigned char message_length = 0;
unsigned char received_bytes = 0;
unsigned char message_end = 0;

unsigned char board_delay = 0;
unsigned char highDelay = 0;
unsigned char lowDelay = 0;
unsigned int partOne = 0X00;
unsigned int partTwo = 0X00;
  unsigned long demoLong = 0x100;//0x400;
//unsigned long demoLong = 0b11111111111111111111111111111111;//0x400;
unsigned int demoLong2;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Function Declarations">
void init(void);
void SPI_setup(void);
void ADC_setup(void);
void EPWM_setup(void);
void Interrupt_setup(void);
void Go_Timer_setup(void);
void Delay_Timer_setup(void);
void Config_setup(void);
void Brightness_setup(void);
void SPI_ISR(void);
void GO_ISR(void);
void DELAY_ISR(void);
void send_2bytes_latchless(unsigned int data);
void send_last_2bytes(unsigned int data, unsigned char latch_start);
void send_2length_2bytes(unsigned int data1, unsigned int data2, unsigned char latch_start);
void load_2length_switch_settings(unsigned int data1, unsigned int data2);
//void calculate_heattime_timer(unsigned char numerals, unsigned char magnitude);
void configure_heattime_timer(unsigned char compare_value, unsigned char scalars);
void configure_delaytime_timer(unsigned char configs, unsigned char highStart, unsigned char lowStart);
void go(void);
void read_temp_voltages(void);
void temp_read_delay(void);
void tmr2_Setup(void);
void peripheral_disable(void);
void utility_functions(unsigned char case_byte0, unsigned char case_byte1); 
void demoLEDdisplay(void);
// </editor-fold>

void main(void) {
    init();
    char i = 0;
    char loop_var = 0;
    char spacer = 0;
    char message_position = 0;
    

    
    while(1){
        
        
        //demoLEDdisplay();

    }

}

void init(void){
    // LATC |= 0x04; //Turn on LEDR
//    Might be tricky to define this with 'variable' pin allocation
    ////  ld
    //// TRISA = 0b00101011;// 7x, 6x, +5SS, 4MISO, +3MCLR, 2LED2, +1CLK, +0MOSI 
    //// TRISB = 0b00010000;// 7LEDCLK, 6LE1, 5EP2, +4SDO2, 3x, 2x, 1x, 0x
    //// TRISC = 0b00010001;// 7LED3, 6SDI1, 5LED1, +4T1, 3EP1, 2ADS, 1LE2, +0T2
//  EIB Pinout  
    TRISA = 0b00101011;// 7x, 6x, +5SS, 4MISO, +3MCLR, 2EP2, +1CLK, +0MOSI 
    TRISB = 0b01000000;// 7LE2, +6T2, 5VTemp, 4LEDG, 3x, 2x, 1x, 0x
    TRISC = 0b10000001;// +7T1, 6EP1, 5LE1, 4SDI1, 3LEDCLK, 2LEDR, 1LEDB, +0SDO2
    
    
    LATA = 0;
    LATB = 0;
    LATC = 0;
//    LATC |= 0x01; //Disable LEDB
//    LATC |= 0x10; //Disable LEDR
//    LATB |= 0x02; //Disable  LEDG

    send_2length_2bytes((unsigned int)demoLong, (unsigned int)((demoLong>>16) & 0xFFFF), 15);

    LATA = 0;
    LATB = 0;
    LATC = 0;
    
    //for(int z = 0; z<100000; z++)
    
//    PPSLOCK = 0x55;
//    PPSLOCK = 0xAA;
//    PPSLOCKbits.PPSLOCKED = 0; //Unlock PPS Selections
    peripheral_disable();
    SPI_setup();
    Interrupt_setup();
    //tmr2_Setup();
    //lock disabled for taking over MISO pin
//    PPSLOCK = 0x55;
//    PPSLOCK = 0xAA;
//    PPSLOCKbits.PPSLOCKED = 1; //Lock PPS Selections
    for(int i=0;i<10000;i++); //give time for voltage to rise. 10000 is ~20 ms
                            // may only need 10
    //Config_setup();
    //Brightness_setup();
    //// LATC &= ~0x20; //Turn off LED1
    //LATB &= ~0x10; //Turn off LED1
    //LATC &= ~0x04; //Turn on LEDR
    
    
}

// turn on LED number num (0-23)
void LEDOn(int num){
    
}
// blink each LED for 1 s
void demoLEDdisplay(void){
        __delay_ms(1000); 
        //printf("Hello world/r/n");
        LATC |= 0x01; //Disable LEDB
        LATC |= 0x10; //Disable LEDR
        LATB |= 0x02; //Disable  LEDG
        // set up LEDs
        
        demoLong2 = (demoLong>>16) & 0xFFFF;
        send_2length_2bytes((unsigned int)demoLong, (unsigned int)demoLong2, 15);
        
        if(demoLong >= 0x80000000){
            demoLong = 0x1;
        }
        else{
            demoLong = demoLong <<1;
        }

        LATA = 0;
        LATB = 0;
        LATC = 0;
        
}
void Interrupt_setup(void){
    INTCON = 0b11000000; //Enable Global and Peripheral Interrupts
                            //interrupt on falling? edge
}

void ADC_setup(void){
    ADCON0 = 0; //ADC Initially Off
    ADCON1 = 0b10010000; //Left Justified, ADCCLK is FOSC/64 according to the table, 
                        //Vref- is VSS, Vref+ is VDD
    //// ANSELA = 0;
    //// ANSELB = 0;
    //// ANSELC = 0b00010101; //Set C4T1, C2ADS, C0T2 as ADC Inputs
    ANSELA = 0;
    ANSELB = 0b01100000; //Set B6T2, B5ADS as ADC Inputs
    ANSELC = 0b10000000; //Set C7T1as ADC Input
    ADACT = 0; //No autoconversion Trigger
}

void EPWM_setup(void){ //Use CLKR to produce the LEDCLK signals fed to the
                        //led driver PWM registers
    //// RC3PPS = 0b00011110; //RC3 EP1 Outputs CLKR
    //// RB5PPS = 0b00011110; //RB5 EP2 Outputs CLKR
    RC6PPS = 0b00011110; //RC6 EP1 Outputs CLKR
    RA2PPS = 0b00011110; //RA2 EP2 Outputs CLKR
    CLKRCON = 0b10010010; //CLKR Starts off, 50% Duty Cycle, 1:2 Fosc division
    
}

void SPI_setup(void){ //Setup for the SPI Functionality and Pins
    //// SSP1CLKPPS = 0b00000001; //SPI CLK
    //// SSP1DATPPS = 0b00000000; //SPI MOSI
    //// SSP1SSPPS =  0b00000101; //SPI SS
    //// RA4PPS = 0b00011001; //SDO1 / MISO
    // THese didn't change
    SSP1CLKPPS = 0b00000001; //SPI CLK
    SSP1DATPPS = 0b00000000; //SPI MOSI
    SSP1SSPPS =  0b00000101; //SPI SS
    RA4PPS = 0b00011001; //SDO1 / MISO

    //Mode 0 from Arduino CLK normally low and 
    //Sample on leading/rising edge 
    SSP1STAT = 0b01000000; //Put in SPI Slave Mode
    SSP1CON1 |= 0b00100100; //Enable SSP and put into SPI Mode
    SSP1CON2 |= 0b00000000; //Just I2C Stuff it seems
    SSP1CON3 |= 0b00010000; // The data buffer is overwritten when new data 
                                //is received 
    //SSP1MSK = 0x00; //I2C only addressing
    //SSP1ADD = 0x00; //More I2C only stuff
//    SSP1STATbits.BF //readable buffer status
//    SSP1CON1bits.SSPOV //indicates more data before read
    

    SSP1BUF = 0; //Buffer of all zeros
    
    //Need Interrupt settings 
    PIE1bits.SSP1IE = 1;
}

void Go_Timer_setup(void){ //set up to be initialized for 2 ms heat time
    TMR0H = 0b11111001; //249,   0.002=(249+1)*8*8*4/32000000
    T0CON0 = 0b00000111; //TIMER0 disabled,and in 8-bit mode/ 1:8 postscalar
    T0CON1 = 0b01000011; //TIMER0 uses the 8MHz source of FOSC/4, 1:8 prescalar
                            //and is set to be in synchronous mode with FOSC/4    
}

void Delay_Timer_setup(void){    
    board_delay = 0; //No Delay Initially
    
    //Use the 8 MHz instruction clock for the timer clock
    T1CONbits.TMR1CS0 = 0;
    T1CONbits.TMR1CS0 = 0;
    
    T1CON = 0;
    T1CON |= 0b00000000; //Fosc/4 clock, 1:1 prescale, SOsc disabled,
                            // ignore sync, Timer1 starts off
    
    T1GCON = 0; //No gate usage
    
    TMR1H = 0; //start with cleared count register
    TMR1L = 0;
    
    PIE1 &= ~(0b00000001); //Roll over Interrupt Disabled

}

void Config_setup(void){
//    send_2length_2bytes( 0x0024, 0x0024, 9); //Auto Off Shutdown and SDO Delay
//    send_2length_2bytes( 0x0024, 0x0024, 9); //Output twice as SDO delay wasn't active the first time
    send_2length_2bytes(((unsigned int)0)<<8 | ((unsigned char)36), ((unsigned int)0)<<8 | ((unsigned char)36), 9); //The hex doesn't translate properly
    send_2length_2bytes(((unsigned int)0)<<8 | ((unsigned char)36), ((unsigned int)0)<<8 | ((unsigned char)36), 9);
//    send_2length_2bytes( 0x0000, 0x0000, 15); //Zero out the switches in order to force the sleep mode
}

void Brightness_setup(void){
    for(int j = 0; j<15; j++){ 
        send_2length_2bytes( 0xFFFF, 0xFFFF, 12);    
    }
    send_2length_2bytes( 0xFFFF, 0xFFFF, 10);
           
}


void interrupt isr(void){ //Initial Interrupt Vector
    //LATA |= 0x04;
    if(PIR1bits.SSP1IF){ //SPI Interrupt Called
        SPI_ISR();
        PIR1bits.SSP1IF = 0;
    }
    else if(PIR0bits.TMR0IF){ //'Go'-Timer Interrupt Called
        GO_ISR();
        PIR0bits.TMR0IF = 0;
    }
    else if(PIR1bits.TMR1IF) {
        DELAY_ISR();
        PIR1bits.TMR1IF = 0;
    }
    //LATA &= ~0x04;
}

void SPI_ISR(void){ //SPI Interrupt Routine 
    //LATA |= 0x04;
    ////LATC |= 0x20; //Turn on LED1 when message begins
    LATB |= 0x10; //Turn on LED1 when message begins
    input_buffer[buffer_position] = SSP1BUF; //Get the received byte from
                                                //the buffer
    
    //Output the previously received byte for possible error checking
    //or output from a buffer for sending new information back, 
    //e.g. thermistor voltages
    if (reply_mode) 1+1;//SSP1BUF = input_buffer[buffer_position]; 
    else SSP1BUF = output_buffer[buffer_position];
    
    //either set the flag indicating that a full message has been received 
    //from the master or increment the buffer position in preparation for the 
    //next byte. The flag is triggered by receiving a newline character (10),
    //which is the last character of messages sent. 
    
    if (received_bytes == 0){
        switch(input_buffer[0]){
        case 'g':
          message_end = 1;
          break;
          
        case 's':
          message_end = 5;
          break;
          
        case 'k':
          message_end = 5;
          break;
          
        case 'c':
          message_end = 5;
          break;
          
        case 'b':
          message_end = 65;
          break;
          
        case 't':
          message_end = 3+3;
          break;
          
        case 'r':
          message_end = 1;
          break;
          
        case 'v':
          message_end =  4;
          break;
          
        case 'z':
          message_end = 1;
          break;
          
        case '@':
          message_end = 1;
          break;
  
        case 'e':
          message_end = 1;
          break;
  
        case 'f':
          message_end = 1;
          break;
          
        case 'u':
          message_end = 3;
          break;
          
        default:
          message_end = 1;
          break;
            
      }
    }
    //received_bytes++;
    
    
    if (received_bytes == message_end) {
        message_complete = 1;
        reply_mode = 0xFF; //sets the reply mode back to the error checking mode 
                            //after the end of sending an output buffer message
        message_length = buffer_position;
        SSP1BUF = input_buffer[0];
        //// LATA &= ~0x04; //Turn off LED2 when message ends
        LATC &= ~0x02; //Turn off LED2 when message ends
    }
    else {buffer_position++;
    //LATA &= ~0x04;
    received_bytes++;}
    
}

void GO_ISR(void){ //Need Timer Interrupt for 'Go' Timing
    //// LATB &= ~0b01000000; //LE1 Low
    //// LATC &= ~0b00000010; //LE2 Low
    LATC &= ~0x20; //LE1 Low
    LATB &= ~0x80; //LE2 Low
    
    //Take over the MISO pin to send a pin change signal to the base station
    //to inform it of the end of the heating period, so that the piezo
    //can be changed
    
    //won't work because the SS or Clock needs to be flipped apparently
//    char temp = SSP1BUF;
//    //SSP1BUFbits.SSP1BUF7 = ~SSP1BUFbits.SSP1BUF7;
//    for(int z = 0; z<1000; z++) SSP1BUF = 0b11111111;
//    //SSP1BUFbits.SSP1BUF7 = ~SSP1BUFbits.SSP1BUF7;
//    for(int z = 0; z<1000; z++) SSP1BUF = 0b00000000;
//    SSP1BUF = temp;
    
    //works, but it need the PPS lock to be turned off
//    SSP1CON1bits.SSPEN = 0; //turn off SPI to take over MISO pin
//    RA4PPS = 0b00000000; //LAT Control
    LATAbits.LATA4 = ~SSP1BUFbits.SSP1BUF7; //Flip the pin to send a signal to the base station
    for(int i=0;i<1;i++);   // Give the AVR time to see the pin change
//    for(int z = 0; z<1000; z++) LATA |= 0b00010000; //MISO High;
//    for(int z = 0; z<1000; z++); //MISO High;
    LATAbits.LATA4 = SSP1BUFbits.SSP1BUF7; //return the state of the bit
    for(int i=0;i<1;i++);   // Time to return the pin to its previous state
//    for(int z = 0; z<1000; z++) LATA &= ~0b00010000; //MISO Low
//    for(int z = 0; z<1000; z++); //MISO Low
    RA4PPS = 0b00011001; //SDO1 / MISO
    SSP1CON1bits.SSPEN = 1; //turn SPI back on
    
    //// LATC &= ~0b10000000; //Turn off LED3
    LATC &= ~0x04; //Turn off LED3
    
    T0CON0 &= ~0b10000000; //Disable the timer
    TMR0IF = 0; //Turn off the interrupt
}

void DELAY_ISR(void){
    //// LATB &= ~0b01000000; //LE1 Low
    //// LATC &= ~0b00000010; //LE2 Low
    LATC &= ~0x20; //LE1 Low
    LATB &= ~0x80; //LE2 Low

    PIE1 &= ~(0b00000001);
    T1CONbits.TMR1ON = 0; 
        
    //Load the off-switch settings
    //just 31 clock pulses with SDI Low
    //// LATC &= ~0b01000000; //SDI1 Low
    LATC &= ~0x10; //SDI1 Low
    for(int j = 0; j<31; j++){ //Loop through the 16 bits
        //// LATB &= ~0b10000000; //LEDCLK Low
        //// LATB |= 0b10000000; //LEDCLK High
        LATC &= ~0x08; //LEDCLK Low
        LATC |= 0x08; //LEDCLK High
    }

    //Last bit and clock pulse with the latch
    //// LATB &= ~0b10000000; //LEDCLK Low
    LATC &= ~0x08; //LEDCLK Low
    //turn on the extended latch
    //// LATB |= 0b01000000; //LE1 High
    //// LATC |= 0b00000010; //LE2 High 
    //// LATB |= 0b10000000; //LEDCLK High
    LATC |= 0x20; //LE1 High
    LATB |= 0x80; //LE2 High
    LATC |= 0x08; //LEDCLK High

    //// LATB &= ~0b10000000; //LEDCLK Low
    LATC &= ~0x08; //LEDCLK Low
    //Ends in a state such that lowering the latches will turn off all switches
    
    //pre-load the delay timer for the next go sequence 
    TMR1H = highDelay; //Most significant bits of the stored compare vale
    TMR1L = lowDelay; //Least Significant its of the stored Compare value
    
}

void send_2bytes_latchless(unsigned int data){
    //// LATB &= ~0b01000000; //LE1 Low     //necessary?  
    //// LATC &= ~0b00000010; //LE2 Low     //necessary?
    LATC &= ~0x20; //LE1 Low
    LATB &= ~0x80; //LE2 Low
    
  for(int j = 0; j<16; j++){ //Loop through the 16 bits
    //// LATB &= ~0b10000000; //LEDCLK Low
    LATC &= ~0x08; //LEDCLK Low
    
    if (data & 1 << j){  //trying to compare to jth value of state for determining LED state
      //// LATC |= 0b01000000; //SDI1 High
        LATC |= 0x10; //SDI1 High
    } //// else  LATC &= ~0b01000000; //SDI1 Low
    else LATC &= ~0x10; //SDI1 Low
        
    //// LATB |= 0b10000000; //LEDCLK High
    LATC |= 0x08; //LEDCLK High

    }
    //// LATB &= ~0b10000000; //LEDCLK Low
    LATC &= ~0x08; //LEDCLK Low
}

void send_last_2bytes(unsigned int data, unsigned char latch_start){ //general method for sending 16 bits with variable latch length to the led driver
    //// LATB &= ~0b01000000; //LE1 Low     //necessary?  
    //// LATC &= ~0b00000010; //LE2 Low     //necessary?
    LATC &= ~0x20; //LE1 Low
    LATB &= ~0x80; //LE2 Low
    
  for(int j = 0; j<16; j++){ //Loop through the 16 bits
    //// LATB &= ~0b10000000; //LEDCLK Low
    LATC &= ~0x08; //LEDCLK Low
    
    if (data & 1 << j){  //trying to compare to jth value of state for determining LED state
      //// LATC |= 0b01000000; //SDI1 High
      LATC |= 0x10; //SDI1 High
    } //// else  LATC &= ~0b01000000; //SDI1 Low
    else LATC &= ~0x10; //SDI1 Low
      
    if( j == latch_start) { //turn on the extended latch
        //// LATB |= 0b01000000; //LE1 High
        //// LATC |= 0b00000010; //LE2 High
        LATC |= 0x20; //LE1 High
        LATB |= 0x80; //LE2 High
    }
    
    //// LATB |= 0b10000000; //LEDCLK High
    LATC |= 0x08; //LEDCLK High

    }
    //// LATB &= ~0b10000000; //LEDCLK Low
    //// LATB &= ~0b01000000; //LE1 Low
    //// LATC &= ~0b00000010; //LE2 Low
    //// LATC &= ~0b01000000; //SDI1 Low     //necessary?
    LATC &= ~0x08; //LEDCLK Low
    LATC &= ~0x20; //LE1 Low
    LATB &= ~0x80; //LE2 Low
    LATC &= ~0x10; //SDI1 Low     //necessary?
}

void send_2length_2bytes(unsigned int data1, unsigned int data2, unsigned char latch_start){
    send_2bytes_latchless( data1);
    send_last_2bytes( data2, latch_start);
}

void load_2length_switch_settings(unsigned int data1, unsigned int data2){ //function for getting the switch settings into the array without the final latch and clock pulse
    send_2bytes_latchless( data1);
    
    
    //// LATB &= ~0b01000000; //LE1 Low     //necessary?  
    //// LATC &= ~0b00000010; //LE2 Low     //necessary?
    LATC &= ~0x20; //LE1 Low
    LATB &= ~0x80; //LE2 Low
  for(int j = 0; j<16; j++){ //Loop through the 16 bits
    //// LATB &= ~0b10000000; //LEDCLK Low
    LATC &= ~0x08; //LEDCLK Low
    
    if (data2 & 1 << j){  //trying to compare to jth value of state for determining LED state
      //// LATC |= 0b01000000; //SDI1 High
      LATC |= 0x10; //SDI1 High
    } ////else  LATC &= ~0b01000000; //SDI1 Low
    else LATC &= ~0x10; //SDI1 Low
      
    if (j == 15) { //turn on the extended latch
      //// LATB |= 0b01000000; //LE1 High
      //// LATC |= 0b00000010; //LE2 High 
      //// LATB |= 0b10000000; //LEDCLK High     
      //// LATB &= ~0b10000000; //LEDCLK Low
      LATC |= 0x20; //LE1 High
      LATB |= 0x80; //LE2 High
      LATC |= 0x08; //LEDCLK High
      LATC &= ~0x08; //LEDCLK Low
      
    } //// else LATB |= 0b10000000; //LEDCLK High
    else LATC |= 0x08; //LEDCLK High
  }
}


//Need 'Go' function
void go(void){
    
    T0CON0 |= 0b10000000; //Enable the timer
    
    //need some sort of if statement above that allows distinguishing between 
    // a go with both, or a go for just one initially with a delay to the other
    //This would then mean that a short timer interrupt would be needed to 
    // trigger the other, though it could be written just just put them both in
    // the latched state
    //This probably also means that the Go signal must become three different 
    // signals, or must be sent in two bytes, with the second being the byte
    // that has the value distinguishing among the three types. Kind of a waste
    // two have two bytes information wise, but it is relatively straightforward
    
    //Timer 2 is more or less set up and could be usable as the intermediate
    
    //If the intermediate value is zero, that could
    // internally imply that the synchrnous go version should be used. 
    switch(board_delay){
        case(0): //both are to start at the same time
            TMR0L = 0; //Clear the timer register
            //// LATB &= ~0b01000000; //LE1 Low - Turn on top
            //// LATC &= ~0b00000010; //LE2 Low - Turn on Bottom
            LATC &= ~0x20; //LE1 Low - Turn on top
            LATB &= ~0x80; //LE2 Low - Turn on Bottom
            PIE0 |= 0b00100000; //Enable the timer interrupt
                        
            SSP1CON1bits.SSPEN = 0; //turn off SPI to take over MISO pin
            RA4PPS = 0b00000000; //LAT Control
            LATAbits.LATA4 = SSP1BUFbits.SSP1BUF7; //return the state of the bit
            
            //// LATC |= 0b10000000; //Turn on LED3
            LATC |= 0x04; //Turn on LEDR
            
            //Load the off-switch settings
            //just 31 clock pulses with SDI Low
            //// LATC &= ~0b01000000; //SDI1 Low
            LATC &= ~0x10; //SDI1 Low
            for(int j = 0; j<31; j++){ //Loop through the 16 bits
                //// LATB &= ~0b10000000; //LEDCLK Low
                //// LATB |= 0b10000000; //LEDCLK High
                LATC &= ~0x08; //LEDCLK Low
                LATC |= 0x08; //LEDCLK High
            }
           
            //Last bit and clock pulse with the latch
            //// LATB &= ~0b10000000; //LEDCLK Low
            LATC &= ~0x08; //LEDCLK Low
            
            //turn on the extended latch
            //// LATB |= 0b01000000; //LE1 High
            //// LATC |= 0b00000010; //LE2 High 
            //// LATB |= 0b10000000; //LEDCLK High
            //// LATB &= ~0b10000000; //LEDCLK Low
            LATC |= 0x20; //LE1 High
            LATB |= 0x80; //LE2 High
            LATC |= 0x08; //LEDCLK High
            LATC &= ~0x08; //LEDCLK Low
            
            
            //Ends in a state such that lowering the latches will turn off all switches
            break;
    
        case(1): //Top First
            TMR0L = 0; //Clear the timer register
            T1CONbits.TMR1ON = 1; //start timer 1
            // zero the intermediate timer
            //// LATB &= ~0b01000000; //LE1 Low
            LATC &= ~0x20; //LE1 Low
            PIE0 |= 0b00100000; //Enable the timer interrupt
            //Enable the intermediate timer interrupt
            PIE1 |= 0b00000001; //Enable the delay interrupt
            
            SSP1CON1bits.SSPEN = 0; //turn off SPI to take over MISO pin
            RA4PPS = 0b00000000; //LAT Control
            LATAbits.LATA4 = SSP1BUFbits.SSP1BUF7; //return the state of the bit
            
            //// LATC |= 0b10000000; //Turn on LED3
            LATC |= 0x04; //Turn on LEDR
            break;
        
    
        case(2): //Bottom First
            TMR0L = 0; //Clear the timer register
            // zero the intermediate timer
            T1CONbits.TMR1ON = 1; //start timer 1
            //// LATC &= ~0b00000010; //LE2 Low
            LATB &= ~0x80; //LE2 Low
            PIE0 |= 0b00100000; //Enable the timer interrupt
            //Enable the intermediate timer interrupt
            PIE1 |= 0b00000001; //enable the delay interrupt
            
            SSP1CON1bits.SSPEN = 0; //turn off SPI to take over MISO pin
            RA4PPS = 0b00000000; //LAT Control
            LATAbits.LATA4 = SSP1BUFbits.SSP1BUF7; //return the state of the bit
            
            //// LATC |= 0b10000000; //Turn on LED3
            LATC |= 0x04; //Turn on LEDR
    }
}
    


void peripheral_disable(void){
    PMD0 = 0b01000001; //FVR, IOC
    PMD1 = 0b11111000; //nco,tmr6,tmr5,tmr4,tmr3
    PMD2 = 0b01000110; //adc, cmp2, cmp1
    PMD3 = 0b11111111; //cwg,pwm,ccp
    PMD4 = 0b00100100; // euart, mssp2
    PMD5 = 0b00011111; //clc,dsm
}


//Functions to use the error detection modes of the led drivers

//Need SPI Reception function?

//Need configuration function?

//Need brightness function?

