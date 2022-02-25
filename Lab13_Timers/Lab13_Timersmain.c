 // Lab13_Timersmain.c
// Runs on MSP432
// Student starter code for Timers lab
// Daniel and Jonathan Valvano
// July 11, 2019
// PWM output to motor
// Second Periodic interrupt

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

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "..\inc\TimerA1.h"
#include "..\inc\Nokia5110.h"

// bit-banding for LEDs
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define GREENLED (*((volatile uint8_t *)(0x42098064)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))


uint16_t Time_100ms = 0;
uint8_t IsNewDataDisplayed = 0;   // semaphore

void Task(void){
    Time_100ms = Time_100ms + 1;
    LEDOUT ^= 0x01;       // toggle P2.0
    IsNewDataDisplayed = 0;
}

void LCDClear(void){
    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("Lab13:Timers");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("Time:");
    Nokia5110_SetCursor(7,3); Nokia5110_OutString("00 ms");
}

// displays Time_100ms
void LCDOut(void){
    Nokia5110_SetCursor(2,3); Nokia5110_OutUDec(Time_100ms);
    IsNewDataDisplayed = 1;
}


// Test TimerA1 periodic interrupt
// Red LED blinks at 10 Hz by periodic interrupt.
// Blue LED blinks at 4 Hz in the foreground thread
// LCD displays time elapsed at 4 Hz in the foreground thread.
void Program13_1(void){
    Clock_Init48MHz();
    LaunchPad_Init();  // built-in switches and LEDs
    Nokia5110_Init();
    LCDClear();

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    IsNewDataDisplayed = 1;
    uint16_t period_2us = 50000;        // T = 100,000 us -> f = 10 Hz
    TimerA1_Init(&Task, period_2us);    // periodic interrupt
    EnableInterrupts();
    while(1) {              // foreground thread
        BLUELED ^= 0x01;    // toggle Blue LED
        if (!IsNewDataDisplayed) {
            LCDOut();           // display time elapsed on LCD
        }
        Clock_Delay1ms(50);
    }
}


// Delay for the given time in ms and then
// wait for any bump switch to be touched.
void Wait4Bump_ms(uint32_t delay_ms){
    Clock_Delay1ms(delay_ms);          // run for a while and stop
    Motor_Coast();
    while(!Bump_Read());    // wait for collision
    while(Bump_Read());     // wait for release
}


// Test PWM.c and Motors.c
// Use motor functions in Motor.c to move the robot.
// Use a bump switch to start the next motor function.
// The bump switches cannot stop the motors.
// Do not modify this function.
void Program13_2(void){
    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    Bump_Init();      // bump switches
    Motor_Init();     // your function

    uint16_t leftDuty_permyriad = 0; //0 %
    uint16_t rightDuty_permyriad = 0; //0 %
    uint32_t delay_ms = 2000;

    while(1){

        Wait4Bump_ms(delay_ms);
        leftDuty_permyriad = 5000;     // 50%
        rightDuty_permyriad = 5000;    // 50%
        Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);

        Wait4Bump_ms(delay_ms);
        Motor_Backward(leftDuty_permyriad, rightDuty_permyriad);

        Wait4Bump_ms(delay_ms);
        leftDuty_permyriad = 3500;     // 35%
        rightDuty_permyriad = 3500;    // 35%
        Motor_TurnLeft(leftDuty_permyriad, rightDuty_permyriad);

        Wait4Bump_ms(delay_ms);
        Motor_TurnRight(leftDuty_permyriad, rightDuty_permyriad);
    }
}


// Semaphore
// 0 means stopped
// 1 means running
int IsRunning = 0;  // initially stopped

// Creates a pause without the need of pressing the button
void TimedPause_ms(uint32_t delay_ms){
	Clock_Delay1ms(delay_ms);
	Motor_Coast();
	IsRunning = 0;  // stopped
}


// Timer task that will check if the robot is running
// If robot is running (IsRunning) and a bump switch is pressed,
// stop the motors and reset the semaphore (motor is Not Running).
// This function will be called by TimerA1 ISR periodically at 10 Hz.
void CheckBumper(void){
	// Write this for Lab 13
    if(IsRunning && Bump_Read()) {
        P3->IN &= ~0xC0;
        IsRunning = 0;
    }
}


// Write this program that uses PWM to move the robot
// like Program13_2, but uses TimerA1 to periodically
// check the bump switches, stopping the robot on a collision
int Program13_3(void){
	Clock_Init48MHz();
	LaunchPad_Init();
	Bump_Init();
	Motor_Init();

	// Write this for Lab 13

	// Initialize Timer A1 to run CheckBumper at 10 Hz
	uint16_t period_2us = 100; // T = 50,000 * 2us = 100ms --> 10 Hz
	TimerA1_Init(&CheckBumper, period_2us);  // 10 Hz

    // Enable Interrupts
	EnableInterrupts();


    // Accomplish the same actions as Program13_2 using
    // TimedPause_ms() with a 3 second delay between actions
    // Remember to set the semaphore prior to each movement
	// Forward/backward direction: 50% duty cycle
	// turns: 35% duty cycle.
    uint16_t leftDuty_permyriad = 0;
    uint16_t rightDuty_permyriad = 0;
	uint32_t time_ms = 3000;   // 3 seconds
    while(1){

        leftDuty_permyriad = 5000;     // 50%
        rightDuty_permyriad = 5000;    // 50%
        IsRunning = 1;
        Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);
        TimedPause_ms(time_ms);

        IsRunning = 1;
        Motor_Backward(leftDuty_permyriad, rightDuty_permyriad);
        TimedPause_ms(time_ms);

        leftDuty_permyriad = 3500;     // 35%
        rightDuty_permyriad = 3500;    // 35%
        IsRunning = 1;
        Motor_TurnLeft(leftDuty_permyriad, rightDuty_permyriad);
        TimedPause_ms(time_ms);

        IsRunning = 1;
        Motor_TurnRight(leftDuty_permyriad, rightDuty_permyriad);
        TimedPause_ms(time_ms);
    }
}

int main(void){
	Program13_1();
//	Program13_2();
//	Program13_3();
}
