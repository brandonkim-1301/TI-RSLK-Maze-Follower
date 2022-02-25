// Lab15_ADCmain.c
// Runs on MSP432, RSLK1.1
// Test the operation of the GP2Y0A21YK0F infrared distance
// sensors by repeatedly taking measurements.  Either
// perform statistical analysis of the data, stream data
// directly from all three channels, or stream calibrated
// measurements from all three channels.  In this case, the
// results are sent through the UART to a computer running
// TExaSdisplay or another terminal program.
// Daniel Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// 5V  connected to all three Pololu #136 GP2Y0A21YK0F Vcc's (+5V)
// ground connected to all three Pololu #136 GP2Y0A21YK0F grounds
// MSP432 P6.1 (J3.23) (analog input A14 to MSP432) connected to center GP2Y0A21YK0F Vout
// MSP432 P9.1 (J5) (analog input A16 to MSP432) connected to left GP2Y0A21YK0F Vout

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/IRDistance.h"
#include "../inc/TimerA1.h"
#include "../inc/UART0.h"
#include "../inc/LaunchPad.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "../inc/Nokia5110.h"

uint32_t FilteredRight, FilteredCenter, FilteredLeft;
uint32_t IsSamplingDone;


// ***********************************************************
//                     Lab15 Program15_1
// ***********************************************************

void LCDClear(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xA8;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("Lab 15: ADC");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("   LPF  dist");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("L=");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("C=");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("R=");
}

// Display ADC data and calculated distances
void LCDOut(void){
    Nokia5110_SetCursor(2,2); Nokia5110_OutUDec(FilteredLeft);
    Nokia5110_SetCursor(7,2);  Nokia5110_OutUDec(LeftConvert(FilteredLeft));
    Nokia5110_SetCursor(2,3); Nokia5110_OutUDec(FilteredCenter);
    Nokia5110_SetCursor(7,3);  Nokia5110_OutUDec(CenterConvert(FilteredCenter));
    Nokia5110_SetCursor(2,4); Nokia5110_OutUDec(FilteredRight);
    Nokia5110_SetCursor(7,4);  Nokia5110_OutUDec(RightConvert(FilteredRight));
}


// Transmit ADC data and calculated distances to PC via UART0.
void UART0Out(void) {
    // left
    UART0_OutUDec5(FilteredLeft); UART0_OutChar(','); UART0_OutUDec5(LeftConvert(FilteredLeft)); UART0_OutChar(',');
    // center
    UART0_OutUDec5(FilteredCenter); UART0_OutChar(','); UART0_OutUDec5(CenterConvert(FilteredCenter)); UART0_OutChar(',');
    // right
    UART0_OutUDec5(FilteredRight); UART0_OutChar(','); UART0_OutUDec5(RightConvert(FilteredRight));
    // need \n\r for CCS Terminal, need \n for most other serial terminals such as Putty.
    UART0_OutChar('\n'); UART0_OutChar('\r');
}


void AdcSampling(void){     // Runs at 2000 Hz by TimerA1 periodic interrupt.
    uint32_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16);  // sample
    FilteredRight = LPF_Calc(raw17);   // right is channel 17 P9.0
    FilteredCenter = LPF_Calc2(raw14);  // center is channel 14, P4.1
    FilteredLeft = LPF_Calc3(raw16);  // left is channel 16, P9.1
    IsSamplingDone = 1;     // semaphore
}


// 1. Test ADC0_InitSWTriggerCh17_14_16()
// 2. Test ADC_In17_14_16()
// 3. Find calibration coefficients for convert
// 4. Test IRDistance.c
int Program15_1(void) { // example program 15.1

    Clock_Init48MHz();
    DisableInterrupts();
    LaunchPad_Init();
    Nokia5110_Init();
    LCDClear();

    UART0_Init();          // initialize UART0 115,200 baud rate
    UART0_OutString("Program 15_1 Calibration test\n\rLeft, Center, Right\n\r");

    // Initialize ADC
    uint32_t raw17, raw14, raw16;
    ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,14,16
    ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample

    // Initialized Digital Low Pass Filters
    uint32_t filterLength = 256; // replace with your choice
    LPF_Init(raw17, filterLength);     // channel 17
    LPF_Init2(raw14, filterLength);    // channel 14
    LPF_Init3(raw16, filterLength);    // channel 16

    // Initialize TimerA1 for periodic interrupt.
    uint16_t period_2us = 250;  // T = 500us --> f = 2000Hz
    TimerA1_Init(&AdcSampling, period_2us);

    IsSamplingDone = 0;

    EnableInterrupts();

    while(1) {
        // wait until 1000 samples are captured.
        for(int n = 0; n < 1000; n++) {
            while(!IsSamplingDone); // wait until 1 data is sampled.
            IsSamplingDone = 0;
        }
        // show every 1000th point
        LCDOut();
        UART0Out();
    }
}

int main(void){
	Program15_1();
}
