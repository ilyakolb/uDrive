/* 
 * File:   LEDcomm.h
 * Author: kolbi
 *
 * Created on February 13, 2018, 9:04 PM
 */

#ifndef LEDCOMM_H
#define	LEDCOMM_H

// from LED2472 datasheet - clock pulses to hold latch high for each command
// it is just 16-X where X is w/e clock pulse is on datasheet
// is there a 1-bit shift..?
#define LED_DATA_LATCH 1
#define LED_WRITE_CR 3
#define LED_READ_CR 5
#define LED_WRITE_GAIN 7
#define LED_READ_GAIN 9
#define LED_OPEN_DETECTION 10
#define LED_SHORT_DETECTION 11
#define LED_OPENSHORT_DETECTION 12


unsigned long second_top;
unsigned long first_bottom;
unsigned long first_top;
unsigned long second_bottom;
unsigned long LED_configReg_top; // config register for top LED driver
unsigned long LED_configReg_bot; // config register for bottom LED driver

void send_2bytes_latchless(unsigned int data);
void send_last_2bytes(unsigned int data, unsigned char latch_start, int topOrBottom);
void send_2length_2bytes(unsigned int data1, unsigned int data2, unsigned char latch_start, int topOrBottom);
void LEDsOn(unsigned long LEDpins, int topOrBottom); // 1: top, 2: bottom
void LEDsPwr(int pwr, int topOrBottom);
void send_last_2bytes_IK(unsigned int data, unsigned char latch_start, int topOrBottom);
void send_2bytes_latchless_IK(unsigned int data);
void send_2length_2bytes_IK(unsigned int data1, unsigned int data2, unsigned char latch_start, int topOrBottom);
void LED_setBrightnessRange(int range, int topOrBottom);
void LED_test_openshort(int, int);
long LED_getData(int topOrBottom);
int LED_testComm(int);
long LED_readConfig(int);
#endif	/* LEDCOMM_H */

