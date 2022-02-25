// Lab12_Motorsmain.c
// Runs on MSP432
// Solution to Motors lab
// Daniel and Jonathan Valvano
// December 17, 2018

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

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\MotorSimple.h"


// Driver test
void WaitForSW(void){

    while(LaunchPad_Input()){     // wait for release
        Clock_Delay1ms(200); LaunchPad_Output(0); // off
        Clock_Delay1ms(200); LaunchPad_Output(3); // yellow
    }

    while(!LaunchPad_Input()) {  // wait for touch
        Clock_Delay1ms(100); LaunchPad_Output(0); // off
        Clock_Delay1ms(100); LaunchPad_Output(3); // yellow
    }

    while(LaunchPad_Input()) {     // wait for release
        Clock_Delay1ms(100); LaunchPad_Output(0); // off
        Clock_Delay1ms(100); LaunchPad_Output(3); // yellow
    }
}


void Program12_1(void){
    Clock_Init48MHz();
    LaunchPad_Init();               // buttons and LEDs
    Bump_Init();                    // bump switches
    Motor_InitSimple();             // your function

    // A very rarely used term permyriad means parts per ten thousand (0/000).
    // 5000 permyriad (0/000) = 500 permille (0/00) = 50 percent (%).
    uint32_t duty_permyriad = 2500;     // 25%

    uint32_t t_forward_10ms = 500;      // 5.0s
    uint32_t t_backward_10ms = 250;     // 2.5s
    uint32_t t_turning_10ms =  300;     // 3.0s


    while(1){
        WaitForSW();
        Motor_ForwardSimple(duty_permyriad, t_forward_10ms);
        WaitForSW();
        Motor_BackwardSimple(duty_permyriad, t_backward_10ms);
        WaitForSW();
        Motor_LeftSimple(duty_permyriad, t_turning_10ms);
        WaitForSW();
        Motor_RightSimple(duty_permyriad, t_turning_10ms);
    }
}


void WaitForBump(void) {

    while(Bump_Read()){     // wait for release
        Clock_Delay1ms(200); LaunchPad_Output(0); // off
        Clock_Delay1ms(200); LaunchPad_Output(1); // red
    }

    while(!Bump_Read()) {  // wait for touch
        Clock_Delay1ms(100); LaunchPad_Output(0); // off
        Clock_Delay1ms(100); LaunchPad_Output(3); // yellow
    }

    while(Bump_Read()) {     // wait for release
        Clock_Delay1ms(100); LaunchPad_Output(0); // off
        Clock_Delay1ms(100); LaunchPad_Output(4); // blue
    }

    for(int i = 0; i < 10; i++) {
        Clock_Delay1ms(100); LaunchPad_Output(0); // off
        Clock_Delay1ms(100); LaunchPad_Output(2); // green
    }
}


void Program12_2(void){
    Clock_Init48MHz();
    LaunchPad_Init();               // buttons and LEDs
    Bump_Init();                    // bump switches
    Motor_InitSimple();             // your function

    // A very rarely used term permyriad means parts per ten thousand (0/000).
    // 5000 permyriad (0/000) = 500 permille (0/00) = 50 percent (%).
    uint32_t duty_permyriad = 2500;     // 25%

    uint32_t t_forward_10ms = 500;      // 5.0s
    uint32_t t_backward_10ms = 250;     // 2.5s
    uint32_t t_turning_10ms =  300;     // 3.0s

    WaitForBump();

    while(1) {
        Motor_ForwardSimple(duty_permyriad, t_forward_10ms);
        Motor_BackwardSimple(duty_permyriad, t_backward_10ms);
        Motor_LeftSimple(duty_permyriad, t_turning_10ms);
        Motor_RightSimple(duty_permyriad, t_turning_10ms);
    }
}


void main(void){
//    Program12_1();
    Program12_2();
}
