/*
 * Classifier.c
 * Runs on MSP432
 *
 *
 * Conversion function for a GP2Y0A21YK0F IR sensor and classification function
 *  to take 3 distance readings and determine what the current state of the robot is.
 *
 *  Created on: Jul 24, 2020
 *  Author: Captain Steven Beyer
 *
 * This example accompanies the book
 * "Embedded Systems: Introduction to Robotics,
 * Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 * For more information about my classes, my research, and my books, see
 * http://users.ece.utexas.edu/~valvano/
 *
 * Simplified BSD License (FreeBSD License
 *
 * Copyright (c) 2019, Jonathan Valvano, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */

#include <stdint.h>
#include "../inc/Classifier.h"

// Complete the following lines
#define IRSlope 1195172
#define IROffset 1058
#define IRMax 2552
#define MaxDist 800


/* Convert
* Calculate the distance in mm given the 14-bit ADC value
* D = 195172/(n - 1058)
*
* The maximum measurement distance for the sensor is 800 mm,
* so if the ADC value is less than 2552 (IRMAX), your
* function should return 800.
*
* Input
*   int32_t n:  14-bit ADC data
* Output
*   int32_t     Distance in mm
*/
int32_t Convert(int32_t adc_value){


    if(adc_value < IRMax) {
        return MaxDist;
    }

    int32_t dist = IRSlope/(adc_value - IROffset);

    return dist;

}

// Complete the following lines
#define SIDEMAX 160     // largest side distance to wall in mm
#define SIDEMIN 110     // smallest side distance to wall in mm
#define CENTEROPEN 200  // distance to wall between open/blocked
#define CENTERMIN 140   // min distance to wall in the front
#define IRMIN 50        // min possible reading of IR sensor
#define IRMAX 800       // max possible reading of IR sensor

/* Classify
* Utilize three distance values from the left, center, and right
* distance sensors to determine and classify the situation into one
* of many scenarios (enumerated by scenario)
*
* Input
*   int32_t Left: distance measured by left sensor
*   int32_t Center: distance measured by center sensor
*   int32_t Right: distance measured by right sensor
*
* Output
*   scenario_t: value representing the scenario the robot
*       currently detects (e.g. RightTurn, LeftTurn, etc.)
*/
scenario_t Classify(int32_t left_mm, int32_t center_mm, int32_t right_mm) {

    scenario_t result = Error;

    // Checks if any of the inputs are outside the range of the IR sensors
    if(left_mm < IRMIN || right_mm < IRMIN || center_mm < IRMIN || left_mm > IRMAX || right_mm > IRMAX || center_mm > IRMAX) {
        return result;
    }

    // Checks if any of the inputs (or combination of them) are too close to an obstacle
    if(left_mm < SIDEMIN) {
        result |= LeftTooClose;
    }
    if(right_mm < SIDEMIN) {
        result |= RightTooClose;
    }
    if(center_mm < CENTERMIN) {
        result |= CenterTooClose;
    }

    // If it is too close to an obstacle, it returns it
    if(result != Error) {
        return result;
    }

    // Considers potential turns (left, right, or both)
    int32_t turnOptions = 0;
    if(left_mm >= SIDEMAX) {
        turnOptions |= 1;
    }
    if(right_mm >= SIDEMAX) {
        turnOptions |= 2;
    }

    // Classifies scenarios, by considering turn options and if something is in front
    if(center_mm < CENTEROPEN) {
        switch(turnOptions) {
            case 0:
                return Blocked;
            case 1:
                return LeftTurn;
            case 2:
                return RightTurn;
            case 3:
                return TeeJoint;
            default:
                break;
        }
    } else {
        switch(turnOptions) {
            case 0:
                return Straight;
            case 1:
                return LeftJoint;
            case 2:
                return RightJoint;
            case 3:
                return CrossRoad;
            default:
                break;
        }
    }

    return result;
}

