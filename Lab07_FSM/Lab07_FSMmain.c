/* Lab07_FSMmain.c
 * Runs on MSP432
 *
 * Student name: Brandon Kim
 * Date:
 * Redesigned by Capt Steven Beyer
 * Date: 9 September 2020 
 * Student version of FSM lab, FSM with 2 inputs and 2 outputs.
 * Utilizes center IR sensors and motors.
 * Daniel and Jonathan Valvano
 * July 11, 2019
*/
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
#include <stdio.h>
#include "msp.h"
#include "../inc/clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Texas.h"
#include "../inc/Motor.h"
#include "../inc/Reflectance.h"
#include "../inc/SysTickInts.h"
#include "../inc/Bump.h"
#include "../inc/Nokia5110.h"


enum State {
    Center = 0,
    Left = 1,
    Right = 2,
};
typedef enum State State_t;

// ***********testing of FSM*********
#define DEFAULT_SPEED 2500

char StateNames[3][7] = {"Center","Left  ","Right "};

uint8_t Input;
uint8_t Sensor;
int32_t Position;

State_t CurrentState = Center;
State_t NextState = Center;

/********Helper functions******/
void CheckBump(void) {

    if(!Bump_Read()) {
        return; // no collision
    }

    Motor_Stop();       // stop

    uint8_t bump = Bump_Read();

    while(Bump_Read()) {        // wait for release
        Clock_Delay1ms(200);
        LaunchPad_Output(0);    // off
        Clock_Delay1ms(200);
        LaunchPad_Output(1);    // red
    }

    while(!Bump_Read()) {       // wait for touch
        Clock_Delay1ms(100);
        LaunchPad_Output(0);    // off
        Clock_Delay1ms(100);
        LaunchPad_Output(3);    // red/green
    }

    while(Bump_Read()) {        // wait for release
        Clock_Delay1ms(100);
        LaunchPad_Output(0);    // off
        Clock_Delay1ms(100);
        LaunchPad_Output(4);    // blue
    }

    for(int i = 500; i > 100; i = i - 100) {
        Clock_Delay1ms(i);
        LaunchPad_Output(0); // off
        Clock_Delay1ms(i);
        LaunchPad_Output(2); // green
    }
}

void LCDClear(void){
    Nokia5110_Init();
	
    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_OutString("Lab 7 FSM");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("Line follow");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("In="); Nokia5110_OutHex7(Input);
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("St=");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("IR=");
    Nokia5110_SetCursor(0,5); Nokia5110_OutString("Pos=");
}

void LCDOut(void){
    Nokia5110_SetCursor(3,2); Nokia5110_OutHex7(Input);
    Nokia5110_SetCursor(3,3); Nokia5110_OutString(StateNames[CurrentState]);
    Nokia5110_SetCursor(3,4); Nokia5110_OutUHex7(Sensor);
    Nokia5110_SetCursor(3,5); Nokia5110_OutSDec(Position);
}

/**********MAIN************/
 /*Run FSM continuously
a) Output depends on State (Motors)
b) Wait depends on State
c) Input (Center reflectance sensors)
d) Next depends on (Input,State)
 */

int main(void){
	// Initialize all systems
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Motor_Init();
	Reflectance_Init();
    Bump_Init();
    LCDClear();
    Motor_Stop();
    EnableInterrupts();

    uint16_t SpeedL = DEFAULT_SPEED;
    uint16_t SpeedR = DEFAULT_SPEED;

    while(1) {

        LCDOut();

        // delay
        uint32_t time_ms = 500;
        Clock_Delay1ms(time_ms);

        // Use middle line sensors (3 and 4) to set input
        uint32_t time_us = 1000;
        Input = Reflectance_Center(time_us);
        Sensor = Reflectance_Get();
		
        // State Transitions, Moore FSM
        switch(CurrentState) {
            // If the robot is last recorded to be centered
            case Center:
                if(Input == 0x3) {
                    NextState = Center;
                } else if(Input == 0x1) {
                    NextState = Left;
                } else {
                    NextState = Right;
                }
                Motor_Forward(SpeedL, SpeedR);
                break;
            // If the robot is last recorded to be off to the left
            case Left:
                if(Input == 0x2) {
                    NextState = Right;
                } else if (Input == 0x0) {
                    NextState = Left;
                } else {
                    NextState = Center;
                }
                Motor_Forward(SpeedL, 0);
                break;
            // If the robot is last recorded to be off to the right
            case Right:
                if(Input == 0x1) {
                    NextState = Left;
                } else if(Input == 0x0) {
                    NextState = Right;
                } else {
                    NextState = Center;
                }
                Motor_Forward(0, SpeedR);
                break;
            // Default case
            default:
                NextState = Right;
                Motor_Forward(SpeedL, SpeedR);
                break;
        }
        // Overwrite the current state as the next state
        CurrentState = NextState;

        Position = Reflectance_Position(Sensor);
        CheckBump();
    }

}
