/* 
 * File:   drv2665.h
 * Author: kolbi
 *
 * Created on February 24, 2018, 4:51 PM
 */

#ifndef DRV2665_H
#define	DRV2665_H

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

#include "mcc_generated_files/drivers/i2c_master.h"
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "mcc_generated_files/pin_manager.h"



/* Contol registers */
#define DRV2665_STATUS  0x00
#define DRV2665_CTRL_1  0x01
#define DRV2665_CTRL_2  0x02
#define DRV2665_FIFO    0x0b
 
/* Status Register */
#define DRV2665_FIFO_FULL       0x00
#define DRV2665_FIFO_EMPTY      0x02
 
/* Control 1 Register */
#define DRV2665_25_VPP_GAIN     0x00
#define DRV2665_50_VPP_GAIN     0x01
#define DRV2665_75_VPP_GAIN     0x02
#define DRV2665_100_VPP_GAIN    0x03
#define DRV2665_DIGITAL_IN      0x38//0xfb //0x00
#define DRV2665_ANALOG_IN       0x04 
 
#define DRV2665_GAIN_RD_MASK    0x03
#define DRV2665_IN_RD_MASK      0x04 
#define DRV2665_CHIPID_RD_MASK  0x78
 
 
/* Control 2 Register */
#define DRV2665_BOOST_EN            0x02
#define DRV2665_STANDBY             0x40
#define DRV2665_DEV_RST             0x80
#define DRV2665_5_MS_IDLE_TOUT      0x00
#define DRV2665_10_MS_IDLE_TOUT     0x04
#define DRV2665_15_MS_IDLE_TOUT     0x08
#define DRV2665_20_MS_IDLE_TOUT     0x0c
 
#define DCONTIME_MS                 5 // time between commands when sending out DC pulses

int drv_peak_val = 0x7F; // value from 0 to 127
void drv_init(int output_gain, int idle_timeout);
void drv_reset(void);
void drv_outputSine(int hz);
void drv_write(uint8_t reg, uint8_t data);
void drv_outputWave(int waveform[], int length);
void drv_write_wvfrm(void);
void drv_write_DC(int val, int duration_ms);
bit fifo_check(void);
int drv_read(char reg);

static const int drv_sine[] = {
    0x00, 0x10, 0x20, 0x2e, 0x3c, 0x48, 0x53, 0x5b, 0x61, 0x65, 0x66,
    0x65, 0x61, 0x5b, 0x53, 0x48, 0x3c, 0x2e, 0x20, 0x10,
    0x00, 0xf0, 0xe0, 0xd2, 0xc4, 0xb8, 0xad, 0xa5, 0x9f, 0x9b, 0x9a,
    0x9b, 0x9f, 0xa5, 0xad, 0xb8, 0xc4, 0xd2, 0xe0, 0xf0, 0x00,
    0x10, 0x20, 0x2e, 0x3c, 0x48, 0x53, 0x5b, 0x61, 0x65, 0x66,
    0x65, 0x61, 0x5b, 0x53, 0x48, 0x3c, 0x2e, 0x20, 0x10,
    0x00, 0xf0, 0xe0, 0xd2, 0xc4, 0xb8, 0xad, 0xa5, 0x9f, 0x9b, 0x9a,
    0x9b, 0x9f, 0xa5, 0xad, 0xb8, 0xc4, 0xd2, 0xe0, 0xf0, 0x00,
    0x10, 0x20, 0x2e, 0x3c, 0x48, 0x53, 0x5b, 0x61, 0x65, 0x66,
    0x65, 0x61, 0x5b, 0x53, 0x48, 0x3c, 0x2e, 0x20, 0x10,
    0x00, 0xf0, 0xe0, 0xd2, 0xc4, 0xb8, 0xad, 0xa5, 0x9f, 0x9b, 0x9a,
    0x9b, 0x9f, 0xa5, 0xad, 0xb8, 0xc4, 0xd2, 0xe0, 0xf0, 0x00,
};

// including FIFO address
static const int drv_fifo_sine[] = {
    DRV2665_FIFO, 0x00, 0x10, 0x20, 0x2e, 0x3c, 0x48, 0x53, 0x5b, 0x61, 0x65, 0x66,
    0x65, 0x61, 0x5b, 0x53, 0x48, 0x3c, 0x2e, 0x20, 0x10,
    0x00, 0xf0, 0xe0, 0xd2, 0xc4, 0xb8, 0xad, 0xa5, 0x9f, 0x9b, 0x9a,
    0x9b, 0x9f, 0xa5, 0xad, 0xb8, 0xc4, 0xd2, 0xe0, 0xf0, 0x00,
    0x10, 0x20, 0x2e, 0x3c, 0x48, 0x53, 0x5b, 0x61, 0x65, 0x66,
    0x65, 0x61, 0x5b, 0x53, 0x48, 0x3c, 0x2e, 0x20, 0x10,
    0x00, 0xf0, 0xe0, 0xd2, 0xc4, 0xb8, 0xad, 0xa5, 0x9f, 0x9b, 0x9a,
    0x9b, 0x9f, 0xa5, 0xad, 0xb8, 0xc4, 0xd2, 0xe0, 0xf0, 0x00,
    0x10, 0x20, 0x2e, 0x3c, 0x48, 0x53, 0x5b, 0x61, 0x65, 0x66,
    0x65, 0x61, 0x5b, 0x53, 0x48, 0x3c, 0x2e, 0x20, 0x10,
    0x00, 0xf0, 0xe0, 0xd2, 0xc4, 0xb8, 0xad, 0xa5, 0x9f, 0x9b, 0x9a,
    0x9b, 0x9f, 0xa5, 0xad, 0xb8, 0xc4, 0xd2, 0xe0, 0xf0, 0x00,
};
#endif	/* DRV2665_H */

