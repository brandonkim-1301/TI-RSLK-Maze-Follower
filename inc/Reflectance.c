// Reflectance.c
// Provide functions to take measurements using the kit's built-in
// QTRX reflectance sensor array.  Pololu part number 3672. This works by outputting to the
// sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Daniel and Jonathan Valvano
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

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "..\inc\Clock.h"

uint8_t ReflectanceResult = 0;

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Initialize even LED Illuminate connected to P5.3
// Initialize odd LED Illuminate connected to P9.2
// Initialize reflectance sensors connected to P7.0-P7.7
// Input: none
// Output: none
void Reflectance_Init(void){
    // write this as part of Lab 6

    // 1. configure P5.3 as GPIO
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;

    // 2. make P5.3 pin out
    P5->DIR |= 0x08;

    // 3. turn off even IR LEDs
    P5->OUT &= ~0x08;

    // 4. configure P9.2 as GPIO
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;

    // 5. make P9.2 pin out
    P9->DIR |= 0x04;

    // 6. turn off odd IR LEDs
    P9->OUT &= ~0x04;

    // 7. configure P7 as GPIO
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;

    // 8. make P7 pins in
    P7->DIR &= ~0xFF;
    P7->REN &= ~0xFF;
}

// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time_us){
    // write this as part of Lab 6

    // 1. Turn on the 8 IR LEDs
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;

    // 1a. Charge capacitors
    P7->DIR |= 0xFF;
    P7->OUT |= 0xFF;

    // 2. Keep the 8 sensors high for 10 us
    Clock_Delay1us(10);         // wait 10 us

    // 3. Make the sensor pins input and wait time_us
    P7->DIR &= ~0xFF;           // make P7.7-P7.0 in
    Clock_Delay1us(time_us);    // wait time_us

    // 5. Read sensors
    ReflectanceResult = P7->IN;       // convert input to digital


    // 6. Turn off the 8 IR LEDs
    P5->OUT &= ~0x08;       // turn off odd IR LEDs
    P9->OUT &= ~0x04;       // turn off even IR LEDs

    return ReflectanceResult;
}


#define LOST_OFFSET 50
const int32_t Weight[8] = {-334, -238, -142, -48, 48, 142, 238, 334};
const int32_t Mask[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){

    // static variables preserve their value until the program ends.
    static int32_t prevSign = 1;    // need to initialize once.

    // write this as part of Lab 6
    int32_t Position = 0;
    int32_t readCount = 0;

    // determine its position on the line, relative to its center
    readCount = 0;
    for(int i = 0; i < 8; i++) {
        if(Mask[i] & data) {
             Position += Weight[i];
             readCount++;
           }
    }
    Position /= readCount;

    // determine if it is completely off the line
    if(data == 0) {    // off line
        Position = prevSign * (Weight[7] + LOST_OFFSET);
    } else {
        // determine if it is off to the left or right, from its center
        if(Position >= 0) {
            prevSign = 1;
        } else {
            prevSign = -1;
        }
    }

    return Position;
}



// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          just left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time_us){

    // write this as part of Lab 7
    // Hint: Use Reflectance_Read().
    return (Reflectance_Read(time_us) & 0x18) >> 3;

}

// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// 1. Turn on the 8 IR LEDs
// 2. Pulse the 8 sensors high for 10 us
// 3. Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // Sets P5.3 and P9.2 high, to turn on 8 IR LED
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;

    // Makes P7 outputs, and set them all high
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR |= 0xFF;
    P7->OUT |= 0xFF;

    // Waits 10us
    Clock_Delay1us(10);

    // Makes P7 inputs
    P7->DIR &= ~0xFF;
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    // Reads 8-bit sensor result
    // Stores data into a shared global variable
    ReflectanceResult = P7->IN;

    // Sets P5.3 and P9.2 low, to turn off 8 IR LEDs
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;

    return ReflectanceResult;

}

// Return last reading
// Input: none
// Output: 8-bit result
// Assumes: Either Reflectance_Read() or Reflectance_End has been called.
uint8_t Reflectance_Get(void) {
    return ReflectanceResult;
}
