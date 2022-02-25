// Lab04_SoftwareDesignmain.c
// Runs on MSP432
// Solution to Software Design lab
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
#include <stdint.h>
#include "../inc/Classifier.h"

// ***********testing of Convert*********
int32_t const ADCBuffer[16] = {2000, 2733, 3466, 4199, 4932, 5665, 6398, 7131, 7864, 8597, 9330, 10063, 10796, 11529, 12262, 12995};
int32_t const DistanceBuffer_mm[16] = {800, 713, 496, 380, 308, 259, 223, 196, 175, 158, 144, 132, 122, 114, 106, 100};


void Program5_1(void){

    int32_t adc_value, distance_mm, diff_mm;
    int32_t errors = 0;

    for(int i = 0; i < 16; i++) {
        adc_value = ADCBuffer[i];
        distance_mm = Convert(adc_value);    // call to your function
        diff_mm = distance_mm - DistanceBuffer_mm[i];
        if((diff_mm < -1) || (diff_mm > 1)) {
            errors++;
        }
    }

    while(1);
}

// ***********end of testing of Convert*********
// ***********testing of classify
scenario_t Solution(int32_t Left, int32_t Center, int32_t Right);
int32_t const CornerCases[18] = {49,50,51,149,150,151,211,212,213,353,354,355,599,600,601,799,800,801};

void Program5_2(void) {

    scenario_t result, truth;
    int32_t left_mm, right_mm, center_mm; // sensor readings
    int32_t errors = 0;

    for(int i = 0; i < 18; i++) {
        left_mm = CornerCases[i];
        for(int j = 0; j < 18; j++) {
            center_mm = CornerCases[j];
            for(int k=0; k < 18; k++) {
                right_mm = CornerCases[k];
                result = Classify(left_mm, center_mm, right_mm); // your solution
                truth = Solution(left_mm, center_mm, right_mm);  // correct answer
                if(result != truth) {
                    errors++;
                }
            }
        }
    }

    while(1);

}


void Program5_3(void){ // will take over 16 hours to complete

    enum scenario result, truth;
    int32_t errors = 0;

    for(int32_t left_mm = 0; left_mm < 1000; left_mm++) {
        for(int32_t center_mm = 0; center_mm < 1000;  center_mm++) {
            for(int32_t right_mm = 0; right_mm < 1000; right_mm++) {
                result = Classify(left_mm, center_mm, right_mm); // your solution
                truth = Solution(left_mm, center_mm, right_mm);  // correct answer
                if(result != truth) {
                    errors++;
                }
            }
        }
    }

    while(1);
}

void main(void){
    // run one of these
//    Program5_1();
    Program5_2();
//    Program5_3();
}
