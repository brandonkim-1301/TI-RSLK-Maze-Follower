// TA3InputCapture.c
// Runs on MSP432
// Use Timer A3 in capture mode to request interrupts on rising
// edges of P10.4 (TA3CCP0) and P8.2 (TA3CCP2) and call user
// functions.
// Daniel Valvano
// May 30, 2017

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

// external signal connected to P10.5 (TA3CCP1) (trigger on rising edge)
// external signal connected to P10.4 (TA3CCP0) (trigger on rising edge)

#include <stdint.h>
#include "msp.h"


void (*CaptureTask0)(uint16_t time); // user function
void (*CaptureTask1)(uint16_t time); // user function

//------------TimerA3Capture_Init01------------
// Initialize Timer A3 in edge time mode to request interrupts on
// the rising edges of P10.4 (TA3CCP0) and P10.5 (TA3CCP1).  The
// interrupt service routines acknowledge the interrupt and call
// a user function.
// Input: task0 is a pointer to a user function called when P10.4 (TA3CCP0) edge occurs
//              parameter is 16-bit up-counting timer value when P10.4 (TA3CCP0) edge occurred
//              (units of 2/3 usec for /8)
//        task1 is a pointer to a user function called when P10.5 (TA3CCP1) edge occurs
//              parameter is 16-bit up-counting timer value when P10.5 (TA3CCP1) edge occurred
//              (units of 2/3 usec for /8)
// Output: none
// Assumes: SMCLK is 12 MHz
void TimerA3Capture_Init(void(*task0)(uint16_t time), void(*task1)(uint16_t time)){
	// write this as part of lab 16
	CaptureTask0 = task0;       // user function
	CaptureTask1 = task1;       // user function

	// initialize P10.4 in secondary mode as input (P10.4 TA3CCP0)
	// initialize P10.5 in secondary mode as input (P10.5 TA3CCP1)
	P10->SEL0 |= 0x30;
	P10->SEL1 &= ~0x30;
	P10->DIR &= ~0x30;

	// halt Timer A3
	TIMER_A3->CTL &= ~0x0030;

    // clock source to SMCLK, input clock divider /1, TACLR = 0, stop mode,
    // interrupt disabled, no interrupt pending
    // bits15-10=XXXXXX,    reserved
    // bits9-8,             clock source to SMCLK
    // bits7-6,             input clock divider /1
    // bits5-4,             stop mode
    // bit3,                reserved
    // bit2,                set this bit to reset TAxR
    // bit1,                interrupt disable
    // bit0,                clear interrupt pending
	TIMER_A3->CTL = 0x0200;

    // capture on rising edge, capture/compare input on CCI3A, synchronous capture source
    // capture mode, output mode, enable capture/compare interrupt, no interrupt pending
    // bits15-14,   capture on rising edge
    // bits13-12,   capture/compare input on CCI3A
    // bit11,       synchronous capture source
    // bit10,       synchronized capture/compare input
    // bit9,        reserved
    // bit8,        capture mode
    // bits7-5,     output mode
    // bit4,        enable capture/compare interrupt
    // bit3,        read capture/compare input from here
    // bit2,        output this value in output mode 0
    // bit1,        capture overflow status
    // bit0,        clear capture/compare interrupt pending
	TIMER_A3->CCTL[0] = 0x4910;
	TIMER_A3->CCTL[1] = 0x4910;

	// configure for input clock divider /8
	TIMER_A3->EX0 = 0x7;

    // TA3_0 priority 2
	// TA3_N priority 2
	NVIC->IP[14] |= 0x40;
	NVIC->IP[15] |= 0x40;

	// enable interrupt 15 and 14 in NVIC
	NVIC->ISER[0] = 0x0000C000;

    // Continuous mode and reset TAxR
    // bits15-10,   reserved
    // bits7-6,     input clock divider /1
    // bits5-4,     continuous count up mode
    // bit3,        reserved
    // bit2,        set this bit to reset TAxR
    // bit1,        interrupt disable (no interrupt on rollover)
    // bit0,        clear interrupt pending
	TIMER_A3->CTL |= 0x0024;

}

// This ISR is called when TA3CCR0 interrupt is triggered (rising edge of P10.4).
void TA3_0_IRQHandler(void){
	// write this as part of lab 16
    // acknowledge capture/compare interrupt 0
	TIMER_A3->CCTL[0] &= ~0x0001;
    // execute user task
	(*CaptureTask0)(TIMER_A3->CCR[0]);

}

// This ISR is called when TA3CCR1 interrupt is triggered (rising edge of P10.5).
void TA3_N_IRQHandler(void){
    // write this as part of lab 16
    // acknowledge capture/compare interrupt 1
	TIMER_A3->CCTL[1] &= ~0x0001;
    // execute user task
	(*CaptureTask1)(TIMER_A3->CCR[1]);

}
