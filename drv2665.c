#include "drv2665.h"

// ello DRV branch
void drv_init(int output_gain, int idle_timeout){
    
    i2c_setAddress(drv_address);
    i2c_open(drv_address);
    __delay_ms(10);
    
    drv_reset();
    __delay_ms(10);
    drv_write(DRV2665_CTRL_2, 0); //clear standby
    __delay_ms(10);
    drv_write(DRV2665_CTRL_1, DRV2665_DIGITAL_IN | output_gain); //set mode digital, set gain
    //drv_write(DRV2665_CTRL_1, output_gain); //set mode digital, set gain
    __delay_ms(10);
    drv_write(DRV2665_CTRL_2, idle_timeout); //set timeout period
    //printf("drv_read: %d\n", drv_read(DRV2665_STATUS));
    __delay_ms(10);
    calcUpstroke(drv_peak_val);
    calcDownstroke(drv_peak_val);
    //printf("drv initialized\n");
}

void drv_reset(void){
    drv_write(DRV2665_CTRL_2, DRV2665_DEV_RST);
}

void drv_outputSine(int hz){
    int8_t waveform_byte;
    uint16_t length;
    uint8_t repeat = 1;
    length = (100 * 80) / hz;
    
    for(int j=0; j<repeat; j++) {
 
        for(int i=0; i<length; i++) {
            waveform_byte = 100.0 * sin( 2*M_PI * i/(length-1) ); //102 is the max +ve / -ve value, typecasting in uint8 turns -ve values into 2's complement required by the driver
 
            drv_write(DRV2665_FIFO, (uint8_t)waveform_byte);
            __delay_us(20);
        }
    }
}


void drv_outputWave(int waveform[], int length){
    
    int repeats = 5;
    for(int j=0; j<repeats; j++){
        for(int i=0; i<length; i++) {
            drv_write(DRV2665_FIFO, drv_sine[i]);
            __delay_us(100);
        }
    }
    
}

void drv_write(uint8_t reg, uint8_t data){
    //Create a temporary buffer
    //printf("i2c addr: %d\n", i2c_addr);
    //printf("drv_write\n");
    //char buff[2];
    uint8_t buff[2];
    //Load the register address and 16-bit data
    buff[0] = reg;
    buff[1] = data;
 
    //Write the data
    i2c_setBuffer(buff, 2);
    if(i2c_masterOperation(0) != I2C_NOERR)
        printf("drv_write error!\n");
    //printf("write: %d\n", ); // write..?

}

void drv_write_wvfrm(char upOrDown){
    
    //Write the data
    if (upOrDown) // upstroke
        i2c_setBuffer(&drv_fifo_upstroke, NELEMS(drv_fifo_upstroke));
    else // downstroke
        i2c_setBuffer(&drv_fifo_downstroke, NELEMS(drv_fifo_downstroke));
    i2c_masterOperation(0);
}

int drv_read(char reg){
    //Create a temporary buffer
    char buff=0;
 
    //Select the register
    i2c_setBuffer(&reg, 1);
    if(i2c_masterOperation(0) != I2C_NOERR)
        printf("drv_write error!\n");
    
    
    __delay_us(100);
    
    //Read the 8-bit register
    i2c_setBuffer(&buff, 1);

    if(i2c_masterOperation(1) != I2C_NOERR)
        printf("drv_read error!\n");

    return buff;
};

// write DC signal to piezo
void drv_write_DC(int val, int duration_ms){
    
    //drv_write(DRV2665_FIFO, 0x00);
    /*
    drv_write(DRV2665_FIFO, val/8);
    drv_write(DRV2665_FIFO, val/4);
    drv_write(DRV2665_FIFO, val*3/8);
    drv_write(DRV2665_FIFO, val/2);
    drv_write(DRV2665_FIFO, val * 3/4);
    */
    for(int i=0;i<duration_ms/DCONTIME_MS; i++){
        drv_write(DRV2665_FIFO, val);
        __delay_us(20); // sometimes get weird latchup if i don't have this wait...
        
    }
    /*
    drv_write(DRV2665_FIFO, val * 7/8);
    drv_write(DRV2665_FIFO, val/2);
    drv_write(DRV2665_FIFO, val*3/8);
    drv_write(DRV2665_FIFO, val/4);
    drv_write(DRV2665_FIFO, val/8);
    */
    
    //__delay_us(100);
    //while(!fifo_check());
    //if(!fifo_check()) printf("fifo not ready!\n");
    
    //drv_write(DRV2665_FIFO, 0x00);
    
}

// 0: not ready; 1: ready
// "is fifo empty? [y/n]
bit fifo_check(void)
{
    uint8_t reply;
    reply = drv_read(DRV2665_STATUS);
    //printf("reply: %d\n", reply);
    return !(reply & 0x01); // tests the DRV2665_FIFO_FULL (0x00) register

};

void calcUpstroke(int maxVal){
    
    drv_fifo_upstroke[0] = DRV2665_FIFO;
    for(int i = 1; i < STROKELENGTH; i++){
        drv_fifo_upstroke[i] = (char)round(i*maxVal/STROKELENGTH);
    }
};

void calcDownstroke(int maxVal){
    drv_fifo_downstroke[0] = DRV2665_FIFO;
    for(int i = 1; i < STROKELENGTH; i++){
        drv_fifo_downstroke[i] = (char)(maxVal - round(i*maxVal/STROKELENGTH));
    }
};