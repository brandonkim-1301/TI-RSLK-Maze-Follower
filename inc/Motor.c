// Motor.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot. Lab 13 solution
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

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

// *******Lab 13 solution*******

// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
    // write this as part of Lab 13

    // Configures P5.4 and P5.5 for PH
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

    // Configures P3.6 and P3.7 for SLEEP
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    // Initializes PWM with 0% duty cycle
    PWM_Init34(15000, 0, 0);

}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Coast(void){
    // write this as part of Lab 13
    // Note: setting nSleep = 0 and PWM (EN) = 0 makes the robot "coast"

    // Sets motors to sleep
    P3->OUT &= ~0xC0;

    // Update left PWM to stop motor
    PWM_DutyLeft(0);

    // Update right PWM to stop motor
    PWM_DutyRight(0);

}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 9999)
//        rightDuty duty cycle of right wheel (0 to 9999)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty_permyriad, uint16_t rightDuty_permyriad){

    // write this as part of Lab 13

    // Sets direction of motors
    P3->OUT |= 0xC0;
    P5->OUT &= ~0x30;

	// Updates PWMs
    PWM_DutyLeft(leftDuty_permyriad);
    PWM_DutyRight(rightDuty_permyriad);

	// Enables motors
    P2->OUT |= 0xC0;

  	// The motors run until software issues another command (don't turn off)

}

// ------------Motor_TurnRight------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 9999)
//        rightDuty duty cycle of right wheel (0 to 9999)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_TurnRight(uint16_t leftDuty_permyriad, uint16_t rightDuty_permyriad){

    // write this as part of Lab 13

    // set direction of motors
    P3->OUT |= 0xC0;
    P5->OUT &= ~0x10;   // Sets Left forward
    P5->OUT |= 0x20;    // Sets Right reverse

	// update PWMs
    PWM_DutyLeft(leftDuty_permyriad);
    PWM_DutyRight(rightDuty_permyriad);

	// Enable motors
    P2->OUT |= 0xC0;

  	// The motors run until software issues another command (don't turn off)

}

// ------------Motor_TurnLeft------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 9999)
//        rightDuty duty cycle of right wheel (0 to 9999)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_TurnLeft(uint16_t leftDuty_permyriad, uint16_t rightDuty_permyriad){

    // write this as part of Lab 13

    // set direction of motors
    P3->OUT |= 0xC0;
    P5->OUT &= ~0x20;   // Sets Right forward
    P5->OUT |= 0x10;    // Sets Left reverse

    // update PWMs
    PWM_DutyLeft(leftDuty_permyriad);
    PWM_DutyRight(rightDuty_permyriad);

    // Enable motors
    P2->OUT |= 0xC0;

    // The motors run until software issues another command (don't turn off)

}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 9999)
//        rightDuty duty cycle of right wheel (0 to 9999)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty_permyriad, uint16_t rightDuty_permyriad){

    // write this as part of Lab 13

    // Sets motors in reverse
    P3->OUT |= 0xC0;
    P5->OUT |= 0x30;

    // update PWMs
    PWM_DutyLeft(leftDuty_permyriad);
    PWM_DutyRight(rightDuty_permyriad);

    // Enable motors
    P2->OUT |= 0xC0;

    // The motors run until software issues another command (don't turn off)

}