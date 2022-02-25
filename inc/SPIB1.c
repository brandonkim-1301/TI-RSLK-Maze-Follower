// SPIB1.c
// Runs on MSP432
// Use eUSCI_B1 to send data via SPI B1

/*
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


#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "SPIB1.h"

//********SPIB1_Init*****************
// Initialize SPI B1. One feature of the
// MSP432 is that its SSIs can get their baud clock from
// either the auxiliary clock (ACLK = REFOCLK/1 = 32,768 Hz
// see ClockSystem.c) or from the low-speed subsystem
// master clock (SMCLK <= 12 MHz see ClockSystem.c).  The
// SSI can further divide this clock signal by using the
// 16-bit Bit Rate Control prescaler Register, UCAxBRW.
// Inputs: none
// Outputs: none
// Assumes: low-speed subsystem master clock 12 MHz
void SPIB1_Init(void) {

    EUSCI_B1->CTLW0 = 0x0001;             // hold the eUSCI module in reset mode

    // configure UCB13CTLW0 for:
    // bit15      UCCKPH = 1; data shifts in on first edge, out on following edge
    // bit14      UCCKPL = 0; clock is low when inactive
    // bit13      UCMSB = 1; MSB first
    // bit12      UC7BIT = 0; 8-bit data
    // bit11      UCMST = 1; master mode
    // bits10-9   UCMODEx = 2; UCSTE active low
    // bit8       UCSYNC = 1; synchronous mode
    // bits7-6    UCSSELx = 2; eUSCI clock SMCLK
    // bits5-2    reserved
    // bit1       UCSTEM = 1; UCSTE pin enables slave
    // bit0       UCSWRST = 1; reset enabled
    EUSCI_B1->CTLW0 = 0xAD83;


    // set the baud rate for the eUSCI which gets its clock from SMCLK
    // Clock_Init48MHz() from ClockSystem.c sets SMCLK = HFXTCLK/4 = 12 MHz
    // if the SMCLK is set to 12 MHz, divide by 3 for 4 MHz baud clock
    EUSCI_B1->BRW = 3;

    // configure P6.2, P6.3, and P6.4 as primary module function
    P6->SEL0 |= 0x1C;
    P6->SEL1 &= ~0x1C;

    EUSCI_B1->CTLW0 &= ~0x0001;           // enable eUSCI module
    EUSCI_B1->IE &= ~0x0003;              // disable interrupts

}


//********SPIB1_OutChar*****************
// Print a character to the SPI channel.
// Inputs: data  character to print
// Outputs: none
void SPIB1_OutChar(char data) {
    // you write this as part of Lab 11

    // Wait for transmitter to be empty (UCTXIFG)
    while((EUSCI_B1->IFG&0x02) == 0);

    // Send data using TXBUF
    EUSCI_B1->TXBUF = data;
}


//********SPIB1_OutString*****************
// Print a string of characters to the SPI channel.
// Inputs: ptr  pointer to NULL-terminated ASCII string
// Outputs: none
void SPIB1_OutString(char *ptr){
// you write this as part of Lab 11

    // While there is data (*ptr)
    while(*ptr) {
        SPIB1_OutChar(*ptr);    // Send data casting as unsigned char
        ptr++;                  // Increment pointer
    }

}




