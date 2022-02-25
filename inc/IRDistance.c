// IRDistance.c
// Runs on MSP432
// Provide mid-level functions that convert raw ADC
// values from the GP2Y0A21YK0F infrared distance sensors to
// distances in mm.
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

// 5V  connected to all three Pololu #136 GP2Y0A21YK0F Vcc's (+5V)
// ground connected to all three Pololu #136 GP2Y0A21YK0F grounds
// MSP432 P9.0 (J5) (analog input A17 to MSP432) connected to right GP2Y0A21YK0F Vout
// MSP432 P6.1 (J3.23) (analog input A14 to MSP432) connected to center GP2Y0A21YK0F Vout
// MSP432 P9.1 (J5) (analog input A16 to MSP432) connected to left GP2Y0A21YK0F Vout


#include <stdint.h>
#include "../inc/ADC14.h"
#include "msp.h"

// Max and Min distances
#define MaxDist 800
#define MinDist 70

#define Al 		929000		// Calibration coefficient
#define Bl 		606 		// Calibration coefficient
#define Cl 		70		    // Distance from the center of robot to IR sensor
#define IRmaxl  12000       // maximum IR value
#define IRminl  2300        // minimum IR value

 // returns left distance in mm
int32_t LeftConvert(int32_t nl) {
    // Checks for min or max distances
    if(nl >= IRmaxl) {
        return MinDist+Cl;
    } else if(nl <= IRminl) {
        return MaxDist+Cl;
    }

    return Al/(nl-Bl)+Cl;
}


#define Ac 		835340		// Calibration coefficient
#define Bc 		995 		// Calibration coefficient
#define Cc 		65	    	// Distance from the center of robot to IR sensor
#define IRmaxc  11500       // maximum IR value
#define IRminc  2650        // minimum IR value


 // returns center distance in mm
int32_t CenterConvert(int32_t nc) {
    // Checks for min or max distances
    if(nc >= IRmaxc) {
        return MinDist+Cc;
    } else if(nc <= IRminc) {
        return MaxDist+Cc;
    }

    return Ac/(nc-Bc)+Cc;
}


#define Ar 		977210		// Calibration coefficient
#define Br 		439 		// Calibration coefficient
#define Cr 		70	    	// Distance from the center of robot to IR sensor
#define IRmaxr  12000   	// maximum IR value
#define IRminr  1630        // minimum IR value


 // returns right distance in mm
int32_t RightConvert(int32_t nr) {
    // Checks for min or max distances
    if(nr >= IRmaxr) {
        return MinDist+Cr;
    } else if(nr <= IRminr) {
        return MaxDist+Cr;
    }

    return Ar/(nr-Br)+Cr;

}
