// Lab08_LED_Switchesmain.c
// Runs on MSP432
// Student name: Brandon Kim
// Date:
// Edited by Capt Beyer to include assembly implementation
// 9 Sep 2019
// Daniel and Jonathan Valvano
// February 28, 2017

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

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

// P6.4 is positive logic Activate, Toggle switch, internal pull-down
// P6.5 is positive logic Window1, momentary switch, internal pull-down
// P6.6 is positive logic Window2, momentary switch, internal pull-down
// P6.7 is positive logic alarm, LED
// Activate    Window                Alarm
//   off       doesn't matter        LED should be off
//   on    either window not pressed LED flashes 5Hz
//   on,   both sensors are pressed  LED should be off
#include <stdint.h>
#include "msp.h"
#include "../inc/TExaS.h"
#include "../inc/Clock.h"

extern void LED_Oscillate(void);
extern void LED_Init(void);
extern void LED_Toggle(void);
extern void LED_On(void);
extern void LED_Off(void);
uint8_t sensor;
/*
 * Code to read switch values on P6.4
 * TODO: Demo code working with switch connected to P6.4
*/
void Program8_1(void){
  
    Clock_Init48MHz();  // makes bus clock 48 MHz
    P6->SEL0 &= ~0x10;  // configure P6.4 GPIO
    P6->SEL1 &= ~0x10;
    P6->DIR &= ~0x10;   // make P6.4 in
    P6->REN |= 0x10;    // enable pull resistors on P6.4
    P6->OUT &= ~0x10;   // Initialize to low
    while(1){
        sensor = P6->IN & 0x10; // read switch
    }
}

/*
 * Code to toggle LED on P6.7
 * TODO: Implement LED_Toggle and LED_Oscillate() within the LEDs.asm file, and demo
*/
void Program8_2(void){
    Clock_Init48MHz();  // makes bus clock 48 MHz
    LED_Init();         // activate output for LED
    LED_Oscillate();
}

/*
 * Code to initialize 3 buttons connected to P6.4 - P6.6
 * TODO: Configure P6.4 - P6.6 as GPIO, inputs, with
 *     pull down resistors
 */
void Button_Init(void) {
    P6->SEL0 &= ~0x70;  // configure P6.4-6 GPIO
    P6->SEL1 &= ~0x70;
    P6->DIR &= ~0x70;   // make P6.4-6 in
    P6->REN |= 0x70;    // enable pull resistors on P6.4-6
    P6->OUT &= ~0x70;   // Initialize to low
}

/*
 * Initialize security system
 */
void Security_Init(void) {
    Button_Init();
    LED_Init();
    LED_Off();
}

/*
 *  Read activate alarm switch (arm/disarm)
 *  TODO: return input of P6.4 (true if armed, false if disarmed)
 */
uint8_t Security_InputActivate(void){
    return (P6->IN & 0x10) >> 4;
}

/*
 * Read window switches input
 *  0x00 both not pressed
 *  0x01 one pressed
 *  0x02 the other pressed
 *  0x03 both pressed
 */
uint8_t Security_InputSensors(void){
    return (P6->IN & 0x60) >> 5;
}
/*
 * Write to Alarm
 */
void Security_OutputAlarm(uint8_t data){
    P6->OUT = (P6->OUT & ~0x80) | data << 7;
}

/*
 * Toggle LED alarm
 */
void Security_ToggleAlarm(void){

}

/*
 * Alarm operation
 * TODO: Initialize ports, wait 100 ms, read activate switch,
 *      read window switches, if the alarm is armed and windows
 *      not secure then toggle LED, else if the alarm is not
 *      armed or the windows are secure do not toggle LED
 */
void Program8_3(void){
    uint8_t arm, sensors;
    Clock_Init48MHz(); // makes it 48 MHz

    // Initializes ports
    Security_Init();

    while(1){
        // Waits 100 ms
        Clock_Delay1ms(100);

        // Reads activate switch
        arm = Security_InputActivate();

        // Reads windows switches
        sensors = Security_InputSensors();

        /* If the alarm is armed and windows not secure then toggle
        * LED, else if the alarm is not armed or the windows are
        * secure do not toggle LED
        */
        if(arm && sensors != 0x3) {
            LED_Toggle();
        } else {
            LED_Off();
        }

    }
}

void main(void){
  // run one of these
//    Program8_1();
//  Program8_2();
  Program8_3();
}
