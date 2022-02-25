// SysTick.c
// Runs on MSP432
// Provide functions that initialize the SysTick module, wait at least a
// designated number of clock cycles, and wait approximately a multiple
// of 10 milliseconds using busy wait.  After a power-on-reset, the
// MSP432 gets its clock from the internal digitally controlled
// oscillator, which is set to 3 MHz by default.  One distinct advantage
// of the MSP432 is that it has low-power clock options to reduce power
// consumption by reducing clock frequency.  This matters for the
// function SysTick_Wait10ms(), which will wait longer than 10 ms if the
// clock is slower.
// Daniel Valvano
// February 18, 2017

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


// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
      SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
      SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
}
// Time delay using busy wait.
// The delay parameter is in units of the core clock. 
// assumes 48 MHz bus clock
void SysTick_Wait(uint32_t delay_48th_us){

    if(delay_48th_us <= 1){
        // without this step:
        // if delay == 0, this function will wait 0x00FFFFFF cycles
        // if delay == 1, this function will never return (because COUNTFLAG is set on 1->0 transition)
        return;                   // do nothing; at least 1 cycle has already passed anyway
    }

    // write this as part of Lab 9

    // 1) Set Reload Value Register,
    SysTick->LOAD = delay_48th_us - 1;

    // 2) Clear Current Value Register,
    // any write to CVR clears it and COUNTFLAG in CSR
    SysTick->VAL = 0;

    // 3) Poll COUNTFLAG in Control and Status Register
    while((SysTick->CTRL&0x00010000) == 0);

}


// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait1us(uint32_t delay_us){

    // write this as part of Lab 9

    if(delay_us <= 1){
        // without this step:
        // if delay_us == 0, this function will wait 0x00FFFFFF cycles
        return;
    }


    // 1) Set Reload Value Register,
    SysTick->LOAD = delay_us*48 - 1;

    // 2) Clear Current Value Register,
    // any write to CVR clears it and COUNTFLAG in CSR
    SysTick->VAL = 0;

    // 3) Poll COUNTFLAG in Control and Status Register
     while((SysTick->CTRL & 0x00010000) == 0);

}


// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait1ms(uint32_t delay_ms){

    for(uint32_t i = 0; i < delay_ms; i++){
        SysTick_Wait(48000);  // wait 10ms (assumes 48 MHz clock)
    }
}

// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait10ms(uint32_t delay_10ms){

    for(uint32_t i = 0; i < delay_10ms; i++){
        SysTick_Wait(480000);  // wait 10ms (assumes 48 MHz clock)
    }
}




