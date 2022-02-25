// Lab09_SysTickmain.c
// Runs on MSP432
// Student name: C2C Brandon Kim
// Date: 26 Sep 2021
// Edited by Capt Beyer to include assembly implementation
// 9 Sep 2019
// Daniel and Jonathan Valvano
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


// built-in LED1 connected to P1.0
// negative logic built-in Button 1 connected to P1.1
// negative logic built-in Button 2 connected to P1.4
// built-in red LED connected to P2.0
// built-in green LED connected to P2.1
// built-in blue LED connected to P2.2
// RC circuit connected to P2.6, to create DAC
#include <stdint.h>
#include "msp.h"
#include "..\inc\TExaS.h"
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTick.h"
#include "..\inc\LaunchPad.h"


// Code to test the SysTick_Wait1us function.
// Utilizes PWM to pulse LEDs on and off.
int Program9_1(void) {
    Clock_Init48MHz();  // makes bus clock 48 MHz
    SysTick_Init();
    LaunchPad_Init();   // buttons and LEDs
    while(1){
        LaunchPad_Output(RED);     // Red LED on
        SysTick_Wait1us(6070);
        LaunchPad_Output(GREEN);   // Green LED on
        SysTick_Wait1us(3930);
    }
}


// Now we are using a variable, high,
// to adjust the duty cycle. Remember, DutyCycle = high/(high+low).
//  In this example, hig = 7500, low = 10000-7500 = 2500, so
//  DutyCycle = 7500/(7500 + 2500) = 75%.
int Program9_2(void){
    Clock_Init48MHz();  // makes bus clock 48 MHz
    SysTick_Init();
    P2->SEL0 &= ~0x40;
    P2->SEL1 &= ~0x40; // 1) configure P2.6 as GPIO
    P2->DIR |= 0x40;   // P2.6 output
    uint32_t high_us = 7550;
    uint32_t low_us = 10000 - high_us;
    while(1){
        P2->OUT |= 0x40;   // on
        SysTick_Wait1us(high_us);
        P2->OUT &= ~0x40;  // off
        SysTick_Wait1us(low_us);
    }
}


#define LED_ON       1
#define LED_OFF      0
#define BUFF_SIZE    100

const uint32_t PulseBuf[BUFF_SIZE]={
    5000, 5308, 5614, 5918, 6219, 6514, 6804, 7086, 7361, 7626,
    7880, 8123, 8354, 8572, 8776, 8964, 9137, 9294, 9434, 9556,
    9660, 9746, 9813, 9861, 9890, 9900, 9890, 9861, 9813, 9746,
    9660, 9556, 9434, 9294, 9137, 8964, 8776, 8572, 8354, 8123,
    7880, 7626, 7361, 7086, 6804, 6514, 6219, 5918, 5614, 5308,
    5000, 4692, 4386, 4082, 3781, 3486, 3196, 2914, 2639, 2374,
    2120, 1877, 1646, 1428, 1224, 1036,  863,  706,  566,  444,
     340,  254,  187,  139,  110,  100,  110,  139,  187,  254,
     340,  444,  566,  706,  863, 1036, 1224, 1428, 1646, 1877,
    2120, 2374, 2639, 2914, 3196, 3486, 3781, 4082, 4386, 4692
};


// Operation
// The heartbeat starts when the operator pushes Button 1
// The heartbeat stops when the operator pushes Button 2
// When beating, the P1.0 LED oscillates at 100 Hz (too fast to see with the eye)
//  and the duty cycle is varied sinusoidally once a second
int Program9_3(void){
    Clock_Init48MHz(); // makes it 48 MHz
    LaunchPad_Init();   // initializes buttons and LEDs
    SysTick_Init();

    // variables to use in the while loop
    uint32_t high_us, low_us;
    uint32_t period_us = 10000;
    uint8_t i = 0;
    uint8_t sw1, sw2;
    int enable = 0;

    while(1) {
        // Get values of sw1 (P1.1) and sw2 (P1.4)
        sw1 = LaunchPad_Input() & 0x01;
        sw2 = LaunchPad_Input() & 0x02;

        // If sw1 is pressed, enables heartbeat;
        // If sw2 is pressed, disables heartbeat
        if(sw1) {
            enable = 1;
        } else if(sw2) {
            enable = 0;
        }

        // If heartbeat is disabled, turns off LED for one period,
        // and skips to the next iteration.
        if(!enable) {
            LaunchPad_LED(LED_OFF);
            SysTick_Wait10ms(1);
            continue;
        }

        // If heartheat is enabled, pulses LED
        high_us = PulseBuf[i];              // Determines high for duty cycle
        low_us = period_us - high_us;       // Calculates low for duty cycle
        P1->OUT |= 0x01;                    // Sets P1.0
        SysTick_Wait1us(high_us);
        P1->OUT &= ~0x01;                   // Clears P1.0
        SysTick_Wait1us(low_us);
        i++;
        if(i % 100 == 0) i = 0;             // If last high is selected, resets counter
    }
}

void main(void){
    // Run one of these
//    Program9_1();
//    Program9_2();
    Program9_3();
}




