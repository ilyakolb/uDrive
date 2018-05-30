#include "mcc_generated_files/LEDcomm.h"
#include "mcc_generated_files/mcc.h"
#include <stdio.h>

// turn on specific LEDs
void LEDsOn(unsigned long LEDpins, int topOrBottom){ // 1: top, 2: bottom
    send_2length_2bytes_IK((unsigned int)((LEDpins>>16) & 0xFFFF),(unsigned int)LEDpins, LED_DATA_LATCH, topOrBottom); // 15th bit high to enable
}

// pwr is int (0-63)
// topOrBottom: 1:top, 2:bottom;
void LEDsPwr(int pwr, int topOrBottom){
    pwr = (pwr>63) ? 63 : pwr; // condition just in case
    unsigned long pwrAll = pwr | (pwr<<6) | (pwr<<12);
    send_2length_2bytes_IK((unsigned int)((pwrAll>>16) & 0xFFFF),(unsigned int)pwrAll, LED_WRITE_GAIN, topOrBottom); // 9th bit high to enable
};

// IK 4/3/18: flipping order of sent bits (for gain setting)
void send_last_2bytes_IK(unsigned int data, unsigned char latch_start, int topOrBottom){ 
    
    
    if (topOrBottom == 1){LED_LE1_SetLow();} else LED_LE2_SetLow();
    
    for(int j = 15; j>=0; j--){ //Loop through the 16 bits
    //// LATB &= ~0b10000000; //LEDCLK Low
    LED_CLK_SetLow(); //LATC &= ~0x08; //LEDCLK Low
    
    if (data & 1 << j){  //trying to compare to jth value of state for determining LED state
      LED_MOSI_SetHigh(); //SDI1 High
    }
    else LED_MOSI_SetLow(); //SDI1 Low
      
    if( j == latch_start) { //turn on the extended latch

        if (topOrBottom == 1){LED_LE1_SetHigh();} else LED_LE2_SetHigh();
        //LATB |= 0x80; //LE2 High //FUTURE
    }

    LED_CLK_SetHigh(); //LATC |= 0x08; //LEDCLK High

    }

    LED_CLK_SetLow();//LATC &= ~0x08; //LEDCLK Low
    if (topOrBottom == 1){LED_LE1_SetLow();} else LED_LE2_SetLow();
    
}

// modified 4/3/18 to reverse LSB/MSB order
void send_2bytes_latchless_IK(unsigned int data){

  LED_LE1_SetLow(); //LE1 Low

  for(int j = 15; j>=0; j--){ //Loop through the 16 bits
    LED_CLK_SetLow();// LATC &= ~0x08; //LEDCLK Low
    
    if (data & 1 << j){  //trying to compare to jth value of state for determining LED state
        LED_MOSI_SetHigh();//LATC |= 0x10; //SDI1 High
    } 
    else LED_MOSI_SetLow();//LATC &= ~0x10; //SDI1 Low
        
    
    LED_CLK_SetHigh();//LATC |= 0x08; //LEDCLK High

  }
    
    LED_CLK_SetLow(); //LATC &= ~0x08; //LEDCLK Low
}

// modified 4/3/18 to reverse LSB/MSB order
void send_2length_2bytes_IK(unsigned int data1, unsigned int data2, unsigned char latch_start, int topOrBottom){
    send_2bytes_latchless_IK( data1);
    send_last_2bytes_IK( data2, latch_start, topOrBottom);
}

//range: 0: low (default), 1: high
// topOrBottom: 1:top, 2:bottom;
void LED_setBrightnessRange(int range, int topOrBottom){
    
    long LED_configReg = (topOrBottom == 1) ? LED_configReg_top : LED_configReg_bot;
    LED_configReg |= ((range>0) | (range>0)<<1 | (range>0) << 2); // set three first registers
    send_2length_2bytes_IK((unsigned int)((LED_configReg>>16) & 0xFFFF),(unsigned int)LED_configReg, LED_WRITE_CR, topOrBottom); // 9th bit high to enable
    
    // update the correct configRegs with new info
    if (topOrBottom == 1) 
        LED_configReg_top = LED_configReg;
    else
        LED_configReg_bot = LED_configReg;
}

// TODO: make everything compatible with both top and bottom LEDs!
// prints results to PC USART
/*
 * type: open or short detection
 */
void LED_test_openshort(int type, int topOrBottom){
    
    LED_ENABLE_SetHigh(); // LEDs off
    LEDsOn(0xFFFFFFFF, topOrBottom); // all LEDs primed
    send_2length_2bytes_IK(0,0,type,topOrBottom); // short/open command

    LED_ENABLE_SetLow(); // LEDs on
    
    unsigned long readData = 0;
    
    for(int j = 23; j>=0; j--){ //Loop through the 16 bits
      LED_CLK_SetHigh();
      __delay_us(10);
      LED_CLK_SetLow();
      // HOW TO BEST OUTPUT RESULT SINCE I CAN'T OUTPUT 2 LONGS?
      if (LED_MISO_GetValue()) printf("%d ", j);
      //printf("%d\n", LED_MISO_GetValue());
      
      //__delay_us(100);
      
      // on first pulse, turn LEDs off
      if (j == 23) LED_ENABLE_SetHigh(); // LEDs off
    }
    printf("\n");
    //printf("read data: %lu\n", readData);
}

// NOTE: THIS WILL ONLY READ BACK 16 BITS OF DATA!
long LED_readConfig(int topOrBottom){
    LED_ENABLE_SetHigh(); // LEDs off
    send_2length_2bytes_IK(0,0,LED_READ_CR,topOrBottom);
    return LED_getData();
}

long LED_getData(void){
    
    long readData = 0;
    LED_LE1_SetLow(); //LE1 Low

    for(int j = 23; j>=0; j--){ //Loop through the 16 bits
      LED_CLK_SetHigh();
      readData |= (LED_MISO_GetValue() << j);
      
      LED_CLK_SetLow();
      __delay_us(100);

    }

    LED_CLK_SetLow();
    printf("read data: %lu\n", readData);
    return readData;
}


// write and then read data to config register
// topOrBottom: 1:top, 2:bottom
int LED_testComm(int topOrBottom){
    
    long oldConfigReg = (topOrBottom == 1) ? LED_configReg_top : LED_configReg_bot; // save previous brightness
    LED_setBrightnessRange(1, topOrBottom);
    int readResult = LED_readConfig(topOrBottom);
    // readResult = 7 (0b111) means everything is working
    LED_setBrightnessRange(oldConfigReg > 0, topOrBottom); // sets the brightness to what it was before
    
    return (readResult != 7); // 0: no error. 1: error
}