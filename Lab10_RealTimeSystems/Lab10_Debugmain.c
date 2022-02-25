// Lab10_Debugmain.c
// Runs on MSP432
// Student version to Debug lab
// Daniel and Jonathan Valvano
// September 4, 2017
// Interrupt interface for QTRX reflectance sensor array
// Pololu part number 3672.
// Debugging dump, and Flash black box recorder

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
// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include "msp.h"
#include "..\inc\Bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\UART0.h"

#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
volatile uint8_t Data; // QTR-8RC
volatile uint8_t Bump; // 6 bump sensors
volatile uint32_t Time_ms;

void SysTick_Handler(void){ // called every 1ms
    // Starts reflectance sensors every 10 ms
    if(Time_ms % 10 == 0) Reflectance_Start();

    // Reads reflectance into Data and bump sensors into Bump
    if(Time_ms % 10 == 1) {
        Data = Reflectance_End();
        Bump = Bump_Read();
    }

    // 1ms later
    Time_ms++;
}


/* Main program that tests functionality for bump and
*    reflectance sensors
*/
int Program10_1(void){
    Clock_Init48MHz();
    LaunchPad_Init();
    Reflectance_Init();
    Bump_Init();
    Time_ms = 0;
    UART0_Init();

    // Initializes SysTick to run every 1 ms with priority 2
    // Calls SysTick_Init (declared in SysTicksInts.c) with appropriate inputs
    uint32_t period_48th_us = 48000;
    uint32_t priority = 2;
    SysTick_Init(period_48th_us, priority);

    // solution

    // Wait until SW1 is pressed
    while(LaunchPad_Input() != 0x01) {
        LEDOUT ^= 0x01;
        Clock_Delay1ms(1000);
    }

    EnableInterrupts();
    while(1){
        WaitForInterrupt();
        LEDOUT ^= 0x01; // foreground thread
        UART0_OutUDec(Time_ms);
        UART0_OutChar(',');
        UART0_OutUHex(Data);
        UART0_OutChar(',');
        UART0_OutUHex(Bump);
        UART0_OutString("\n\r");
    }

}


void main(void) {
    Program10_1();
}


