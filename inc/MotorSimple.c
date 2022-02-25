// MotorSimple.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.
// Student starter code for Lab 12, uses Systick software delay to create PWM
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

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTick.h"
#include "../inc/Bump.h"

//**************RSLK Max***************************
// Left motor direction connected to P5.4
// Left motor PWM connected to P2.7
// Left motor enable connected to P3.7
// Right motor direction connected to P5.5
// Right motor PWM connected to P2.6
// Right motor enable connected to P3.6
// Initializes the 6 GPIO lines and puts driver to sleep
void Motor_InitSimple(void) {

    // write this as part of Lab 12

    // Configure P2.6 and P2.7 (EN) as outputs
    // and disable them for brake/coast.
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;

    // Configure P5.4 and P5.5 (PH) as outputs
    // and set them low for the forward direction.
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

    // Configure P3.6 and P3.7 (SLEEP) as outputs
    // and set them low for coast.
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    // initializes SysTick
    SysTick_Init();

}


// Stops both motors, puts driver to sleep mode for coasting.
// Returns right away
void Motor_StopSimple(void) {
    // Sets Right and Left to sleep for coasting
    P3->OUT &= ~0xC0;
}


// Drives both motors forward at duty (100 to 9900).
// The unit of duty cycle is permyriad, which is one percent of one percent.
// A very rarely used term permyriad means parts per ten thousand (0/000).
// 50 % = 5000 permyriad (0/000).
// Runs for duration_10ms (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after duration_10ms or if a bumper switch is hit
void Motor_ForwardSimple(uint16_t duty_permyriad, uint32_t duration_10ms) {
    // write this as part of Lab 12

	// Check input errors. 
	// If the input range is incorrect, do nothing and return. 
	if (duty_permyriad < 100 || duty_permyriad > 9900) {
		return;
	}

	// Drives both motors forward at duty (100 to 9900)
	// Runs for time duration (units=10ms), and then stops
	// Stop the motors and return if any bumper switch is active
	// Returns after the time duration or if a bumper switch is hit

	// Set Right and Left for forward direction
	P3->OUT |= 0xC0;
	P5->OUT &= ~0x30;

    // Drives for time duration or until hits bumper
	for(uint32_t i = 0; i < duration_10ms; i++) {
	    if(Bump_Read()) break;
	    P2->OUT |= 0xC0;
	    SysTick_Wait1us(duty_permyriad);
	    P2->OUT &= ~0xC0;
	    SysTick_Wait1us(10000-duty_permyriad);
	}
}


// Drives both motors backward at duty (100 to 9900)
// The unit of duty cycle is permyriad, which is one percent of one percent.
// A very rarely used term permyriad means parts per ten thousand (0/000).
// 50 % = 5000 permyriad (0/000).
// Runs for duration_10ms (units=10ms), and then stops
// Returns after time*10ms
void Motor_BackwardSimple(uint16_t duty_permyriad, uint32_t duration_10ms) {

	// write this as part of Lab 12
	// Do not stop the motors even if any bumper switch is active

    // Set Right and Left for reverse direction
    P3->OUT |= 0xC0;
    P5->OUT |= 0x30;

    // Drives for time duration
    for(uint32_t i = 0; i < duration_10ms; i++) {
        P2->OUT |= 0xC0;
        SysTick_Wait1us(duty_permyriad);
        P2->OUT &= ~0xC0;
        SysTick_Wait1us(10000-duty_permyriad);
   }
}


// Drives just the left motor forward at duty (100 to 9900)
// The unit of duty cycle is permyriad, which is one percent of one percent.
// A very rarely used term permyriad means parts per ten thousand (0/000).
// 50 % = 5000 permyriad (0/000).
// Right motor is stopped (sleeping)
// Runs for duration_10ms (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after duration_10ms or if a bumper switch is hit
void Motor_LeftSimple(uint16_t duty_permyriad, uint32_t duration_10ms) {

    // write this as part of Lab 12
    // Sets Right to sleep
    P3->OUT &= ~0x40;

    // Sets Left to move forward
    P3->OUT |= 0x80;
    P5->OUT &= ~0x10;

    // Drives for time duration
    for(uint32_t i = 0; i < duration_10ms; i++) {
        if(Bump_Read()) break;
        P2->OUT |= 0x80;
        SysTick_Wait1us(duty_permyriad);
        P2->OUT &= ~0x80;
        SysTick_Wait1us(10000-duty_permyriad);
    }
}


// Drives just the right motor forward at duty (100 to 9900)
// The unit of duty cycle is permyriad, which is one percent of one percent.
// A very rarely used term permyriad means parts per ten thousand (0/000).
// 50 % = 5000 permyriad (0/000).
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after duration_10ms or if a bumper switch is hit
void Motor_RightSimple(uint16_t duty_permyriad, uint32_t duration_10ms) {

    // write this as part of Lab 12
    // Sets Left to sleep
    P3->OUT &= ~0x80;

    // Sets Right to move forward
    P3->OUT |= 0x40;
    P5->OUT &= ~0x20;

    // Drives for time duration
    for(uint32_t i = 0; i < duration_10ms; i++) {
        if(Bump_Read()) break;
        P2->OUT |= 0x40;
        SysTick_Wait1us(duty_permyriad);
        P2->OUT &= ~0x40;
        SysTick_Wait1us(10000-duty_permyriad);
    }
}
