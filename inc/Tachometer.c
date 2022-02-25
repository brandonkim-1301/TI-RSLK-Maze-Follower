// Tachometer.c
// Runs on MSP432
// Provide mid-level functions that initialize ports, take
// angle and distance measurements, and report total travel
// statistics.
// Daniel Valvano
// December 20, 2018

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

// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

#include <stdint.h>
#include "../inc/Clock.h"
#include "../inc/TA3InputCapture.h"
#include "msp.h"
#include "Tachometer.h"

uint16_t PreviousRightTime = 0;
uint16_t CurrentRightTime = 0;
uint16_t PreviouseLeftTime = 0;
uint16_t CurrentLeftTime = 0;
int32_t TachoRightSteps = 0;     // incremented with every step forward; decremented with every step backward
int32_t TachoLeftSteps = 0;      // incremented with every step forward; decremented with every step backward

enum TachDirection TachoRightDir = STOPPED;
enum TachDirection TachoLeftDir = STOPPED;


// This function is called by TA3CCR0 ISR.
// input: currenttime will be the value loaded in
// CCR0 when the interrupt is triggered.
static void tachometerRightInt(uint16_t currenttime){
    PreviousRightTime = CurrentRightTime;
    CurrentRightTime = currenttime;
    if(P5->IN & 0x01) {
        // Encoder B is high, so this is a step forward
        // increment steps, 1 step = 1 deg
        TachoRightSteps++;
        TachoRightDir = FORWARD;
    }else{
        // Encoder B is low, so this is a step backward
        // decrement steps, 1 step = 1 deg
        TachoRightSteps--;
        TachoRightDir = REVERSE;
    }
}


// This function is called by TA3CCR1 ISR.
// input: currenttime will be the value loaded in
// CCR1 when the interrupt is triggered.
static void tachometerLeftInt(uint16_t currenttime){
    PreviouseLeftTime = CurrentLeftTime;
    CurrentLeftTime = currenttime;
    if(P5->IN & 0x04){
        // Encoder B is high, so this is a step forward
        // increment steps, 1 step = 1 deg
        TachoLeftSteps++;
        TachoLeftDir = FORWARD;
    }else{
        // Encoder B is low, so this is a step backward
        // decrement steps, 1 step = 1 deg
        TachoLeftSteps--;
        TachoLeftDir = REVERSE;
    }
}


// ------------Tachometer_Init------------
// Initialize P5.0 and P5.2 as GPIO input, which will
// be used to determine the direction of rotation.
// Initialize TimerA3 for the input capture interface,
// which will be used to measure the speed of rotation.
// Input: none
// Output: none
void Tachometer_Init(void){

    // initialize P5.0 and P5.2 and make them GPIO inputs
    // configure P5.0 and P5.2 as GPIO
    P5->SEL0 &= ~0x05;
    P5->SEL1 &= ~0x05;
    // make P5.0 and P5.2 inputs
    P5->DIR &= ~0x05;

    // initialize TimerA3.
    TimerA3Capture_Init(&tachometerRightInt, &tachometerLeftInt);
}


// ------------Tachometer_ResetSteps------------
// Reset the displacements measured by Tachometer
// Input: none
// Output: none
void Tachometer_ResetSteps(void) {
    TachoRightSteps = 0;     // incremented with every step forward; decremented with every step backward
    TachoLeftSteps = 0;
}


// ------------Tachometer_Get------------
// Get the most recent tachometer measurements.
// Input: leftPeriod_2_3rd_us   is pointer to store last measured tachometer period of left wheel (units of 2/3 us)
//        leftDir               is pointer to store enumerated direction of last movement of left wheel
//        leftSteps_deg         is pointer to store total number of forward steps measured for left wheel (360 steps per ~220 mm circumference)
//        rightPeriod_2_3rd_us  is pointer to store last measured tachometer period of right wheel (units of 2/3 us)
//        rightDir              is pointer to store enumerated direction of last movement of right wheel
//        rightSteps_deg        is pointer to store total number of forward steps measured for right wheel (360 steps per ~220 mm circumference)
// Output: none
// Assumes: Tachometer_Init() has been called
// Assumes: Clock_Init48MHz() has been called
void Tachometer_Get(uint16_t *leftPeriod_2_3rd_us,
                    enum TachDirection *leftDir,
                    int32_t *leftSteps_deg,
                    uint16_t *rightPeriod_2_3rd_us,
                    enum TachDirection *rightDir,
                    int32_t *rightSteps_deg) {
    *leftPeriod_2_3rd_us = (CurrentLeftTime - PreviouseLeftTime);
    *leftDir = TachoLeftDir;
    *leftSteps_deg = TachoLeftSteps;
    *rightPeriod_2_3rd_us = (CurrentRightTime - PreviousRightTime);
    *rightDir = TachoRightDir;
    *rightSteps_deg = TachoRightSteps;
}





