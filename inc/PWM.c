// PWM.c
// Runs on MSP432
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// PWM on P2.6 using TimerA0 TA0.CCR3
// PWM on P2.7 using TimerA0 TA0.CCR4
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
// Derived from msp432p401_portmap_01.c in MSPware
// Jonathan Valvano
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

#include "msp.h"

//***************************PWM_Init1*******************************
// PWM outputs on P2.4
// Inputs:  period (166.67ns)
//          duty (0<=duty<period-1)
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 1/12MHz = 83.33ns
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// Period of P2.4 is period*166.67ns, duty cycle is duty/period
void PWM_Init1(uint16_t period, uint16_t duty) {
    if(duty >= period) return;     // bad input
    P2->DIR |= 0x10;               // P2.4 output
    P2->SEL0 |= 0x10;              // P2.4 Timer0A functions
    P2->SEL1 &= ~0x10;             // P2.4 Timer0A functions
    TIMER_A0->CCTL[0] = 0x0080;    // CCI0 toggle
    TIMER_A0->CCR[0] = period;     // Period is 2*period*8*83.33ns is 1.333*period
    TIMER_A0->EX0 = 0x0000;        //    divide by 1
    TIMER_A0->CCTL[1] = 0x0040;    // CCR1 toggle/reset
    TIMER_A0->CCR[1] = duty;       // CCR1 duty cycle is duty1/period
    TIMER_A0->CTL = 0x0230;        // SMCLK=12MHz, divide by 1, up-down mode
    // bit  mode
    // 9-8  10    TASSEL, SMCLK=12MHz
    // 7-6  00    ID, divide by 1
    // 5-4  11    MC, up-down mode
    // 2    0     TACLR, no clear
    // 1    0     TAIE, no interrupt
    // 0          TAIFG
}

//***************************PWM_Init12*******************************
// PWM outputs on P2.4, P2.5
// Inputs:  period (1.333us)
//          duty1
//          duty2
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
// Period of P2.4 is period*1.333us, duty cycle is duty1/period
// Period of P2.5 is period*1.333us, duty cycle is duty2/period
void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2){
    if(duty1 >= period) return; // bad input
    if(duty2 >= period) return; // bad input
    P2->DIR |= 0x30;          // P2.4, P2.5 output
    P2->SEL0 |= 0x30;         // P2.4, P2.5 Timer0A functions
    P2->SEL1 &= ~0x30;        // P2.4, P2.5 Timer0A functions
    TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
    TIMER_A0->CCR[0] = period;       // Period is 2*period*8*83.33ns is 1.333*period
    TIMER_A0->EX0 = 0x0000;        //    divide by 1
    TIMER_A0->CCTL[1] = 0x0040;      // CCR1 toggle/reset
    TIMER_A0->CCR[1] = duty1;        // CCR1 duty cycle is duty1/period
    TIMER_A0->CCTL[2] = 0x0040;      // CCR2 toggle/reset
    TIMER_A0->CCR[2] = duty2;        // CCR2 duty cycle is duty2/period
    TIMER_A0->CTL = 0x02F0;        // SMCLK=12MHz, divide by 8, up-down mode
    // bit  mode
    // 9-8  10    TASSEL, SMCLK=12MHz
    // 7-6  11    ID, divide by 8
    // 5-4  11    MC, up-down mode
    // 2    0     TACLR, no clear
    // 1    0     TAIE, no interrupt
    // 0          TAIFG
}

//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on P2.4
// Inputs:  duty1
// Outputs: none
// period of P2.4 is 2*period*666.7ns, duty cycle is duty1/period
void PWM_Duty1(uint16_t duty1){
    if(duty1 >= TIMER_A0->CCR[0]) return; // bad input
    TIMER_A0->CCR[1] = duty1;        // CCR1 duty cycle is duty1/period
}

//***************************PWM_Duty2*******************************
// change duty cycle of PWM output on P2.5
// Inputs:  duty2
// Outputs: none// period of P2.5 is 2*period*666.7ns, duty cycle is duty2/period
void PWM_Duty2(uint16_t duty2){
    if(duty2 >= TIMER_A0->CCR[0]) return; // bad input
    TIMER_A0->CCR[2] = duty2;        // CCR2 duty cycle is duty2/period
}

//***************************PWM_Init34*******************************
// PWM outputs on P2.6, P2.7
// Inputs:  period (1.333us)
//          duty_right
//          duty_left 
// Outputs: none
// Set up TimerA0 with SMCLK=12MHz, divide by 8, up-down mode 
// f_clk = SMCLK = 48MHz/4 = 12 MHz, T_clk = 1/12 us
// Counter counts up to TA0CCR0 and back down - up/down mode
// Let Timer clock frequency: f_c_TA0 = f_clk/8 = 12/8 MHz, T_c_TA0 = 8/12 us = 2/3 us
// P2.6=1 when timer equals TA0CCR3 on way down, P2.6=0 when timer equals TA0CCR3 on way up
// P2.7=1 when timer equals TA0CCR4 on way down, P2.7=0 when timer equals TA0CCR4 on way up
// TimerA0 period: T_TA0 = T_c_TA0 * CCR0 = CCR0 * 2/3 us.
// PWM period: T_PWM = T_TA0 * 2 = CCR0 * 4/3 us.
// The PWM period of P2.6: CCR0*4/3 us, duty cycle(%) is duty_right/CCR0
// The PWM period of P2.7: CCR0*4/3 us, duty cycle(%) is duty_right/CCR0
void PWM_Init34(uint16_t period, uint16_t duty_right, uint16_t duty_left){

    if(duty_right >= period)return;  // bad input
    if(duty_left >= period)return;  // bad input

    // write this as part of Lab 13
    // Initialize P2.7 and P2.6 - EN PWML/PWMR


    P2->SEL0 |= 0xC0;               // Sets to primary function
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;                // Sets to output
    TIMER_A0->CCTL[0] = 0x0080;		// CCI0 toggle
    TIMER_A0->CCR[0] = period;		// T_PWM is CCR0*4/3 us, if CCR0 = 15000, T_PWM = 20 ms.
    TIMER_A0->EX0 = 0x0000;			// divide by 1
    TIMER_A0->CCTL[3] = 0x0040; 	// CCR3 toggle/reset
    TIMER_A0->CCR[3] = duty_right;  // CCR3 duty cycle is duty1/period
    TIMER_A0->CCTL[4] = 0x0040;     // CCR4 toggle/reset
    TIMER_A0->CCR[4] = duty_left;   // CCR4 duty cycle is duty2/period
    TIMER_A0->CTL = 0x02F0;			// SMCLK=12MHz, divide by 8, up-down mode

}

//***************************PWM_DutyRight*******************************
// change duty cycle of PWM output on P2.6
// Inputs:  duty_per10k, duty cycle is in per 10,000 (0.01%)
// Outputs: none
void PWM_DutyRight(uint16_t duty_permyriad){
    // write this as part of Lab 13

    // convert duty_permyriad (0 - 9999) to duty in TA0 unit (0 - 14998)
    // duty = duty_permyriad * 15000/10000 (or 3/2)
    uint16_t duty = (duty_permyriad * 3) >> 1;

    // if new duty cycle is greater than period, return immediately
    if(duty > TIMER_A0->CCR[0]) return;

    // assign new duty cycle to CCR[3]
    TIMER_A0->CCR[3] = duty;
}

//***************************PWM_DutyLeft*******************************
// change duty cycle of PWM output on P2.7
// Inputs:  duty_per10k, duty cycle is in per 10,000 (0.01%)
// Outputs: none
void PWM_DutyLeft(uint16_t duty_permyriad){
    // write this as part of Lab 13

    // convert duty_permyriad (0 - 9999) to duty in TA0 unit (0 - 14998)
    // duty = duty_permyriad * 15000/10000 (or 3/2)
    uint16_t duty = (duty_permyriad * 3) >> 1;

    // if new duty cycle is greater than period, return immediately
    if(duty > TIMER_A0->CCR[0]) return;

    // assign new duty cycle to CCR[4]
    TIMER_A0->CCR[4] = duty;
}
