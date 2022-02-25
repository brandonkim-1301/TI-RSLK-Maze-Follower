// UART2.c
// Runs on MSP432
// Busy-wait device driver for the UART UCA2.
// Daniel Valvano
// July 11, 2019
// Modified by EE345L students Charlie Gough && Matt Hawk
// Modified by EE345M students Agustinus Darmawan && Mingjie Qiu

// Modified by Stan Baek at US Air Force Academy

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

// UCA2RXD (VCP receive) connected to P3.2
// UCA2TXD (VCP transmit) connected to P3.3

#include <stdint.h>
#include <stdio.h>
#include "UART2.h"
#include "msp.h"


//------------UART2_Init------------
// Initialize the UART for the given baud rate (assuming 12 MHz SMCLK clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void UART2_Init(uint16_t baudrate){

    EUSCI_A2->CTLW0 = 0x0001; // hold the USCI module in reset mode

    // bit15=0,      no parity bits
    // bit14=x,      not used when parity is disabled
    // bit13=0,      LSB first
    // bit12=0,      8-bit data length
    // bit11=0,      1 stop bit
    // bits10-8=000, asynchronous UART mode
    // bits7-6=11,   clock source to SMCLK
    // bit5=0,       reject erroneous characters and do not set flag
    // bit4=0,       do not set flag for break characters
    // bit3=0,       not dormant
    // bit2=0,       transmit data, not address (not used here)
    // bit1=0,       do not transmit break (not used here)
    // bit0=1,       hold logic in reset state while configuring
    EUSCI_A2->CTLW0 = 0x00C1;

    // set the baud rate
    // N = clock/baud rate = 12,000,000/115,200 = 104.1667
    // UCBR = baud rate = int(N) = 104
    EUSCI_A2->BRW = 12000000/baudrate;

    // clear first and second modulation stage bit fields
    EUSCI_A2->MCTLW &= ~0xFFF1;

    // configure P3.3 and P3.2 as primary module function
    P3->SEL0 |= 0x0C;
    P3->SEL1 &= ~0x0C;

    // enable the USCI module
    EUSCI_A2->CTLW0 &= ~0x0001;

    // disable interrupts (transmit ready, start received, transmit empty, receive full)
    EUSCI_A2->IE &= ~0x000F;
}


//------------UART2_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
char UART2_InChar(void){
    while((EUSCI_A2->IFG&0x01) == 0);   // busy-wait for receive data
    return((char)(EUSCI_A2->RXBUF));    // get new input
}


//------------UART2_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART2_OutChar(char letter){
    while((EUSCI_A2->IFG&0x02) == 0);   // busy-wait for previous output
    EUSCI_A2    ->TXBUF = letter;           // start transmission
}


//------------UART2_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART2_OutString(char *ptr){

    // While there is data (*ptr)
    while(*ptr) {
        UART2_OutChar(*ptr); // Send data casting as unsigned char.
        ptr++;               // Increment pointer
    }
}

