// Lab14_EdgeInterruptsmain.c
// Runs on MSP432, interrupt version
// Main test program for interrupt driven bump switches the robot.
// Daniel Valvano and Jonathan Valvano
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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/BumpInt.h"
#include "../inc/TimerA1.h"
#include "../inc/Nokia5110.h"


#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define GREENLED (*((volatile uint8_t *)(0x42098064)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))


// ***********************************************************
//                     Lab14 Program14_1
// ***********************************************************

uint16_t NumSW1Pressed = 0;
uint16_t NumSW2Pressed = 0;

void LCDClear1(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();      // erase entire display
    Nokia5110_OutString("Program14_1");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("SW1:");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("SW2:");
}


// Display Bump data and number of collisions
void LCDOut1(void){
    Nokia5110_SetCursor(4,2); Nokia5110_OutUDec(NumSW1Pressed);
    Nokia5110_SetCursor(4,3); Nokia5110_OutUDec(NumSW2Pressed);
}


void PORT1_IRQHandler(void) {   // P1 interrupt
    if (P1->IFG & 0x02) { // interrupt from P1.1
        P1->IFG &= ~0x02;   // clear interrupt pending
        NumSW1Pressed++;
    }
    if (P1->IFG & 0x10) { // interrupt from P1.4
        P1->IFG &= ~0x10;   // clear interrupt pending
        NumSW2Pressed++;
    }
}


int Program14_1(void){ // test of interrupt-driven bump interface

    DisableInterrupts();
    Clock_Init48MHz(); // 48 MHz clock;

    Nokia5110_Init();

    NumSW1Pressed = 0;
    NumSW2Pressed = 0;
    LCDClear1();

    LaunchPad_Init();
    // P1.1 & P1.4 are already initialized as input in LanchPad_Init();
    P1->IES |= 0x12;  // falling edge event
    P1->IFG &= ~0x12;  // clear flag: no interrupt pending
    P1->IE |= 0x12;   // enable interrupt on P1.1 & P1.4
    NVIC->IP[35] = 0x80;        // IRQ35 for P1, priority 4: 0b10000000
    NVIC->ISER[1] = 0x00000008; // enable interrupt on P1

    EnableInterrupts();

    while(1){
        LEDOUT ^= 0x01;
        WaitForInterrupt();
        LCDOut1();
    }
}


// ***********************************************************
//                     Lab14 Program14_2
// ***********************************************************

uint8_t BumpData;
uint16_t NumCollisions;

void LCDClear2(void) {

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();      // erase entire display
    Nokia5110_OutString("Program14_2");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("Bump:");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("nCol:");
}


// Display Bump data and number of collisions
void LCDOut2(void) {
    Nokia5110_SetCursor(5,2); Nokia5110_OutUHex7(BumpData);
    Nokia5110_SetCursor(5,3); Nokia5110_OutUDec(NumCollisions);
}


void HandleCollision(uint8_t bumpSensor) {
    LEDOUT ^= 0x01;
    LEDOUT ^= 0x01;
	NumCollisions++;
	LEDOUT ^= 0x01;
	BumpData = bumpSensor;
}


int Program14_2(void) { // test of interrupt-driven bump interface

    DisableInterrupts();
    Clock_Init48MHz(); // 48 MHz clock; 12 MHz Timer A clock
    Nokia5110_Init();
    LCDClear2();
	NumCollisions = 0;
	BumpInt_Init(&HandleCollision);

	EnableInterrupts();

	while(1){
		BLUELED ^= 0x01;
		WaitForInterrupt();
		LCDOut2();
	}
}


// ***********************************************************
//                     Lab14 Program14_3
// ***********************************************************

// Struct to send a command to the robot
typedef struct command {
    uint16_t dutyRight_permyriad;   // 0 to 9999
    uint16_t dutyLeft_permyriad;    // 0 to 9999
    void (*MotorFunction)(uint16_t, uint16_t);
    uint32_t duration_ms;           // time to run in ms
} command_t;

// Write this as part of Lab 14
// Code three high-level commands
// 1. Back up slowly for 2 second (~30-60% duty cycle)
// 2. Turn right slowly for 4 seconds (90 degrees)
// 3. Go forward quickly for 10 second (~60-80% duty cycle)
//#define NUM_STEPS   3
//const command_t Control[NUM_STEPS]={
//    {2000,	2000,	&Motor_Backward,	2000},
//    {750,   750,    &Motor_TurnRight,   4000},
//    {6000,  5500,   &Motor_Forward,     8000},
//};

// Controls to drive in a square
#define NUM_STEPS   11
const uint16_t dutyRight_Forward = 3000;
const uint16_t dutyLeft_Forward = 2900;
const uint16_t dutyRight_Turn = 750;
const uint16_t dutyLeft_Turn = 750;
const uint16_t durationForward = 5200;
const uint16_t durationTurn = 3800;

const command_t Control[NUM_STEPS]={
//    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft
//    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnRight,   durationTurn},   // 90 deg right turn
//    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft
//    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnRight,   durationTurn},  // 90 deg right turn
//    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft
//    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnRight,   durationTurn},  // 90 deg right turn
//    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft
//
//    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnRight,   2*durationTurn},  // 180 deg right turn

    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft
    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Coast,       durationTurn},
    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnLeft,    durationTurn},  // 90 deg left turn
    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Coast,       durationTurn},
    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft
    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Coast,       durationTurn},
    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnLeft,    durationTurn},  // 90 deg left turn
    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft
    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnLeft,    durationTurn},  // 90 deg left turn
    {dutyRight_Forward,     dutyLeft_Forward,   &Motor_Forward,     durationForward},  // move forward 4 ft

    {dutyRight_Turn,        dutyLeft_Turn,      &Motor_TurnRight,   2*durationTurn},  // 180 deg right turn
};

uint32_t ElapsedTime_ms;
uint32_t CurrentStep;   // 0, 1, 2...,NUM-1

//uint32_t firstSquare = 1;
//uint32_t sidesToSquare = 4;

// The Controller will run using TimerA1 at a rate of 1000 Hz or every 1 ms.
// If the elapsed time is greater than the duration of the current command,
// move to the next command in the Controller (using CurrentStep)
// Reset the CurrentStep if it is greater than the number of steps (NUM_STEPS) using modulo math.
void Controller(void) {
    // Write this as part of Lab 14
    // Controller should increment a timer (ElapsedTime_ms)

//    if((ElapsedTime_ms >= Control[CurrentStep].duration_ms) && firstSquare) {
//        if(!(firstSquare % (sidesToSquare + 1))) {
//            firstSquare = 0;    // Acknowledge semaphore for first square
//            CurrentStep = 3;    // Set for 180 deg right turn
//            Control[CurrentStep].MotorFunction(Control[CurrentStep].dutyLeft_permyriad, Control[CurrentStep].dutyRight_permyriad);
//        } else if(CurrentStep == 1) {  // if it completed a side of the square (ie moved forward)
//            firstSquare++;
//        }
//        ElapsedTime_ms = 0;
//        CurrentStep = ++CurrentStep % (NUM_STEPS - 2);          // only use first two controls
//        Control[CurrentStep].MotorFunction(Control[CurrentStep].dutyLeft_permyriad, Control[CurrentStep].dutyRight_permyriad);
//    } else if(ElapsedTime_ms >= Control[CurrentStep].duration_ms) {
//        ElapsedTime_ms = 0;
//        if()
//        CurrentStep = ++CurrentStep % (NUM_STEPS - 2);    // only use second and third controls
//        Control[CurrentStep].MotorFunction(Control[CurrentStep].dutyLeft_permyriad, Control[CurrentStep].dutyRight_permyriad);
//    }

    if(ElapsedTime_ms >= Control[CurrentStep].duration_ms) {
        ElapsedTime_ms = 0;
        CurrentStep = ++CurrentStep % NUM_STEPS;
        Control[CurrentStep].MotorFunction(Control[CurrentStep].dutyLeft_permyriad, Control[CurrentStep].dutyRight_permyriad);
    }
    ElapsedTime_ms++;
}


// On a collision, you stop and restart the simple set of commands.
// 1. On a collision, stop the robot (coast mode).
// 2. Restart the commands with the backward command.
// 3. Reset elapsed time
void Collision(uint8_t bumpSensor) {
    // Write this as part of Lab 14
    // Note: After collision, the robot must move backward.
    if(bumpSensor) {
        P3->OUT &= ~0xC0;
        CurrentStep = 0;
        Control[CurrentStep].MotorFunction(Control[CurrentStep].dutyLeft_permyriad, Control[CurrentStep].dutyRight_permyriad);
        ElapsedTime_ms = 0;
    }
}


int Program14_3(void) {
	DisableInterrupts();
	Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
	LaunchPad_Init();
	Motor_Init();

	// write this as part of Lab 14, Integrated Robotic System
	// Initialize Bump with the Collision() function you wrote
	BumpInt_Init(&Collision);

	// Initialize Timer A1 with the Controller() function you wrote at 1000 Hz
	uint16_t period_2us = 500;
	TimerA1_Init(&Controller, period_2us);

	// Initialize Step to the first command
    CurrentStep = 0;

	// Run first command
    Control[CurrentStep].MotorFunction(Control[CurrentStep].dutyLeft_permyriad, Control[CurrentStep].dutyRight_permyriad);

	// Initialize Elapsed Time
	ElapsedTime_ms = 0;

	EnableInterrupts();
	while(1) {
        WaitForInterrupt();
    }
}


int main(void){
//    Program14_1();
//    Program14_2();
	Program14_3();
}
