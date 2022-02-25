// Lab16_Tachmain.c
// Runs on MSP432
// Test the operation of the tachometer by implementing
// a simple DC motor speed controller.
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

// See Bump.c for bumper connections (Port 8 or Port 4)

// Debug heartbeat connected to P2.0 (built-in red LED)
// Debug heartbeat connected to P2.4

// Pololu kit v1.1 connections:
// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

// Negative logic bump sensors defined in Bump.c (use Port 4)
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// Debug heartbeat connected to P2.0 (built-in red LED)
// Debug heartbeat connected to P1.0 (built-in LED1)

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA1.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Bump.h"


// Debug heartbeat connected to red LED
// Debug heartbeat connected to RGB LED
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define GREENLED (*((volatile uint8_t *)(0x42098064)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))



// ***********************************************************
//                     Lab16 Program16_1
// ***********************************************************

#define PULSE2RPM   250000    // clock divider is 8

uint16_t PeriodRight;   // (8/SMCLK) units = 2/3 us
uint16_t PeriodLeft;    // (8/SMCLK) units = 2/3 us
uint16_t PrevRight;     // TimerA3 first edge, P10.4
uint16_t PrevLeft;      // TimerA3 first edge, P10.5
uint8_t IsRightDone;    // Set after a tachometer measurement is taken.
uint8_t IsLeftDone;     // Set after a tachometer measurement is taken.

void LCDClear1(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xA0;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_SetCursor(0, 0);
    Nokia5110_OutString("Lab16_1 Demo");
    Nokia5110_SetCursor(0, 1);
    Nokia5110_OutString("Period      ");
    Nokia5110_SetCursor(0, 2);
    Nokia5110_OutString("L     R     ");
    Nokia5110_SetCursor(0, 3);
    Nokia5110_OutString("Speed  (RPM)");
    Nokia5110_SetCursor(0, 4);
    Nokia5110_OutString("L     R     ");
}


void LCDSpeed(void){
    Nokia5110_SetCursor(1, 2);
    Nokia5110_OutUDec(PeriodLeft);
    Nokia5110_SetCursor(7, 2);
    Nokia5110_OutUDec(PeriodRight);
    Nokia5110_SetCursor(1, 4);
    Nokia5110_OutUDec(PULSE2RPM/PeriodLeft);
    Nokia5110_SetCursor(7, 4);
    Nokia5110_OutUDec(PULSE2RPM/PeriodRight);
}


// This function is called by TA3CCR0 ISR.
// The ISR will pass the CCR0 value to this function.
// input: currenttime will be the value loaded in
// CCR0 when the interrupt is triggered.
void PeriodMeasureRight(uint16_t currenttime) {
    PeriodRight = currenttime - PrevRight; // 16 bits, 2/3 us resolution
    PrevRight = currenttime;                          // setup for next
    IsRightDone = 1;
}


// This function is called by TA3CCR1 ISR.
// The ISR will pass the CCR0 value to this function.
// input: currenttime will be the value loaded in
// CCR1 when the interrupt is triggered.
void PeriodMeasureLeft(uint16_t currenttime){
    PeriodLeft = currenttime - PrevLeft;   // 16 bits, 2/3 us resolution
    PrevLeft = currenttime;                           // setup for next
    IsLeftDone = 1;
}


#define BUFFER_SIZE 500

uint16_t PeriodBufferR[BUFFER_SIZE];    // f = 8/12 MHz --> T = 2/3 us
uint16_t PeriodBufferL[BUFFER_SIZE];    // f = 8/12 MHz --> T = 2/3 us
uint16_t SpeedBufferR[BUFFER_SIZE];     // RPM
uint16_t SpeedBufferL[BUFFER_SIZE];     // RPM
uint16_t DutyBuffer[BUFFER_SIZE];


// This function is called by TA1 ISR every 10 ms.
void Collect(void) {

    static uint16_t time_10ms = 0;

    // If interrupt is not triggered, the semaphore will remain 0.
    // It means the wheel did not move at all since the last measurement.
    if(IsRightDone == 0) { PeriodRight = 65534; } // stopped
    if(IsLeftDone == 0)  { PeriodLeft = 65534;  } // stopped

    IsRightDone = IsLeftDone = 0;     // Reset semaphores

    uint16_t duty_permyriad;        // 0 to 10000

    if(time_10ms == 0) {            // update duty at 1 sec
        duty_permyriad = 2500;      // 25%
        Motor_Forward(duty_permyriad, duty_permyriad);
    } else if(time_10ms == 100) {   // update duty at 1 sec
        duty_permyriad = 5000;      // 50%
        Motor_Forward(duty_permyriad, duty_permyriad);
    } else if(time_10ms == 200) {   // update duty at 2 sec
        duty_permyriad = 7500;      // 75%
        Motor_Forward(duty_permyriad, duty_permyriad);
    } else if(time_10ms == 300) {   // update duty at 3 sec
        duty_permyriad = 9999;      // 99.99%
        Motor_Forward(duty_permyriad, duty_permyriad);
    } else if(time_10ms == 400) {   // update duty at 4 sec
        duty_permyriad = 2500;      // 25%
        Motor_Forward(duty_permyriad, duty_permyriad);
    }

    if(time_10ms < 500){            // take measurement for 5 sec
        PeriodBufferR[time_10ms] = PeriodRight;
        PeriodBufferL[time_10ms] = PeriodLeft;
        SpeedBufferR[time_10ms] = PULSE2RPM/PeriodRight;
        SpeedBufferL[time_10ms] = PULSE2RPM/PeriodLeft;
        DutyBuffer[time_10ms] = duty_permyriad;
        time_10ms = time_10ms + 1;
        LCDSpeed();
    }

    if (time_10ms % 20 == 0) {  // heartbeat
        BLUELED ^= 1;
    }

    // stop taking measurements, stop the motors. and stop TimerA1.
    if((time_10ms == 500) || Bump_Read()){
        duty_permyriad = 0;
        Motor_Coast();
        TimerA1_Stop();
    }
}


int Program16_1(void){

    DisableInterrupts();

    Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
    LaunchPad_Init();
    Bump_Init();
    Motor_Init();        // activate Lab 13 software
    Nokia5110_Init();
    LCDClear1();
    TimerA3Capture_Init(&PeriodMeasureRight, &PeriodMeasureLeft);

    uint16_t period_2us = 5000;             // T = 10 ms
    TimerA1_Init(&Collect, period_2us);     // f = 100 Hz

    PrevRight = PrevLeft = 0;       // Prev will be wrong at the beginning
    IsRightDone = IsLeftDone = 0;   // Set semaphores

    EnableInterrupts();

    while(1){
        WaitForInterrupt();
    }
}



// ***********************************************************
//                     Lab16 Program16_2
// ***********************************************************

// variables for motors
uint16_t DesiredL_rpm = 50;  // desired motor RPM left, set initially to 50
uint16_t DesiredR_rpm = 50;  // desired motor RPM right, set initially to 50
uint16_t ActualL_rpm;       // measured motor RPM
uint16_t ActualR_rpm;       // measured motor RPM
int32_t LeftSteps_deg;      // left wheel steps in deg
int32_t RightSteps_deg;     // right wheel steps in deg


// The wheel circumference is 220 mm.
// gear ratio = 120 : 1, 3 N poles in tachometer --> 360 pulses per rotation.
// 1 wheel step = 1 deg rotation of the wheel
// 1 wheel revolution (360 deg rotation) = 220 mm displacement.
#define STEPS2DISTANCE  220/360 // no parentheses should be added, e.g., (220/360)


void LCDClear2(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB8;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_SetCursor(0, 0);
    Nokia5110_OutString("Desired(RPM)L     R     ");
    Nokia5110_SetCursor(0, 2);
    Nokia5110_OutString("Actual (RPM)L     R     ");
    Nokia5110_SetCursor(0, 4);
    Nokia5110_OutString("Distance(mm)");

}

void LCDDesired(void){
    Nokia5110_SetCursor(1, 1);         // one leading space, second row
    Nokia5110_OutUDec(DesiredL_rpm);
    Nokia5110_SetCursor(7, 1);         // seven leading spaces, second row
    Nokia5110_OutUDec(DesiredR_rpm);
}

void LCDActual(void){
    Nokia5110_SetCursor(1, 3);       // one leading space, fourth row
    Nokia5110_OutUDec(ActualL_rpm);
    Nokia5110_SetCursor(7, 3);       // seven leading spaces, fourth row
    Nokia5110_OutUDec(ActualR_rpm);
    Nokia5110_SetCursor(0, 5);  Nokia5110_OutSDec(LeftSteps_deg*STEPS2DISTANCE);
    Nokia5110_SetCursor(6, 5);  Nokia5110_OutSDec(RightSteps_deg*STEPS2DISTANCE);
}

// tachometer period of left wheel (number of 2/3 us cycles to rotate 1/360 of a wheel rotation)
// variables to average tachometer readings
#define TACHBUFF_SIZE 10

uint16_t LeftTachPeriod[TACHBUFF_SIZE];
uint16_t RightTachPeriod[TACHBUFF_SIZE];
enum TachDirection LeftDir;
enum TachDirection RightDir;


// ------------average------------
// Simple math function that returns the average
// value of the data array.
// Input: data is an array of 16-bit unsigned numbers
//        data_length is the number of elements in data
// Output: the average value of the data
// Note: overflow is not considered
uint16_t average(uint16_t *data, int data_length) {

    uint32_t sum = 0;

    for(int i=0; i < data_length; i++) {
        sum += data[i];
    }

    return sum/data_length;

}

#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max) ? (Max) : (X) ) )

#define DESIREDMAX 120
#define DESIREDMIN 30
#define MAXDUTY 10000
#define MINDUTY 100

uint8_t IsControllerEnabled = 0;


// This function is called by TA1 ISR every 20 ms.
void Controller(void) {

    static uint8_t nData = 0; // number of tachometer data read.
    static uint16_t leftDuty_permyriad = 0;
    static uint16_t rightDuty_permyriad = 0;

    // Controller is disabled.  Do nothing
    if(!IsControllerEnabled) { return; }

    // If a bump switch is pressed, disable Controller
    if (Bump_Read()) {
        IsControllerEnabled = 0;
    }

    // Get values from the tachometer and store into variables
    Tachometer_Get(&LeftTachPeriod[nData], &LeftDir, &LeftSteps_deg,
                   &RightTachPeriod[nData], &RightDir, &RightSteps_deg);

    nData = (nData + 1) % TACHBUFF_SIZE;

    // omega = PULSE2RPM/n, where n is the number of timer pulses
    // Use the average of the last 10 measurements to reduce the noise.
    // write code to get actual left and right values

    ActualL_rpm = PULSE2RPM/average(LeftTachPeriod, TACHBUFF_SIZE);
    ActualR_rpm = PULSE2RPM/average(RightTachPeriod, TACHBUFF_SIZE);

    // very simple, very stupid controller
    // If actual is greater than (desired + 3), subtract 20 from Duty
    // If actual is less than (desired - 3), add 20 to Duty

    // solution for left wheel
    if(ActualL_rpm > (DesiredL_rpm+3)) {
        leftDuty_permyriad -= 20;
    } else if (ActualL_rpm < (DesiredL_rpm-3)) {
        leftDuty_permyriad += 20;
    }

    // repeat for right wheel
    if(ActualR_rpm > (DesiredR_rpm+3)) {
        rightDuty_permyriad -= 20;
    } else if (ActualR_rpm < (DesiredR_rpm-3)) {
        rightDuty_permyriad += 20;
    }


    // If actual is less than MINDUTY, actual = MINDUTY
    // If actual is greater than MAXDUTY, actual = MAXDUTY
    // use MINMAX defined above.

    leftDuty_permyriad = MINMAX(MINDUTY, MAXDUTY, leftDuty_permyriad);
    rightDuty_permyriad = MINMAX(MINDUTY, MAXDUTY, rightDuty_permyriad);

    // drive motors forward
    Motor_Forward(leftDuty_permyriad, rightDuty_permyriad);
    // update the screen
    LCDActual();

}


void Program16_2(void){

    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Nokia5110_Init();
    LCDClear2();
    LCDDesired();
    Motor_Init();
	Tachometer_Init();

    // Initialize TimerA1 for periodic interrupts at 50 Hz.

	uint16_t period_2us = 10000;                // T = 20 ms
    TimerA1_Init(&Controller, period_2us);      // f = 50 Hz

    IsControllerEnabled = 0;

	EnableInterrupts();

	while(1) {

        // low power mode while waiting for the next interrupt
        WaitForInterrupt();

        // If controller is enabled, skip the next and go to low power mode.
	    if (IsControllerEnabled) {
	        continue;
	    }

        // If the program reaches here, controller is disabled.
	    // We can update the desired speeds.
	    LaunchPad_Output(0); // turn off RGB LED
	    Motor_Coast();
	    Clock_Delay1ms(300);

	    // Press SW1 or SW2 to update desired speeds.
	    // Hit a bump switch when done.
		while(!Bump_Read()) {

            // update the screen
			LCDDesired();

			// SW1 has been pressed
			if((LaunchPad_Input() & 0x01)) {
			    // Increase desired right speed by 10 and roll over if max is reached
			    //solution

			    DesiredR_rpm = (DesiredR_rpm + 10) % (DESIREDMAX - 10);
			}

			// SW2 has been pressed
			if((LaunchPad_Input() & 0x02)){
			    // Increase desired left speed by 10 and roll over if max is reached
			    // solution

			    DesiredL_rpm = (DesiredL_rpm + 10) % (DESIREDMAX - 10);
			}

            // flash the blue LED while desired speeds are updated.
			BLUELED ^= 1;
			Clock_Delay1ms(200);
		}

        // desired speeds are updated now. flash yellow LED for 2 sec.
		for(int k = 0; k < 10; k++) {
			LaunchPad_Output(0x03);
			Clock_Delay1ms(100);
			LaunchPad_Output(0x00);
			Clock_Delay1ms(100);
		}

        // Enable controller
		IsControllerEnabled = 1;
	}
}

void main(void){
//    Program16_1();
    Program16_2();
}
