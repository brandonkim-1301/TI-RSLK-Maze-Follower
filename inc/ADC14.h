/**
 * @file      ADC14.h
 * @brief     Initialize 14-bit ADC0
 * @details   ADC input, software trigger, 14-bit conversion<br>
 * The ADC14 allows two possible references: 2.5V or 3.3V.
 * The internal 2.5V reference is lower noise, but limits the range to 0 to 2.5V.>
 * The other possibility is to use the analog voltage supply at 3.3V,
 * making the ADC range 0 to 3.3V. In this driver, the 3.3V range is selected.
 * There are several configurations (each with initialization and a
 * read ADC with software trigger, busy-wait function):<br>
 * - sample just P4.7/A6 <br>
 * - sample P4.6/A7 and P4.7/A6 <br>
 * - sample just P6.1/A14 <br>
 * - sample P9.0/A17, P6.1/A14, and P9.1/A16<br>
 * - sample just P8.2/A23 <br>
 * @version   TI-RSLK MAX v1.1
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2019 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 28, 2019
<table>
<caption id="ADCpins">ADC pins used on the TI-RSLK MAX</caption>
<tr><th>Pin  <th>ADC channel<th>Sensor
<tr><td>P9.0 <td>A17<th>right GP2Y0A21YK0F
<tr><td>P6.1 <td>A14<th>center GP2Y0A21YK0F
<tr><td>P9.1 <td>A16<th>left GP2Y0A21YK0F
</table>
******************************************************************************/

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

/*!
 * @defgroup ADC14
 * @brief
 * @{*/

#ifndef ADC14_H_
#define ADC14_H_


/**
 * Initialize 14-bit ADC0 in software-triggered mode to take
 * measurements when the associated function is called.  These
 * channels are measured:<br>
 * - Sample P9.0/A17(first)<br>
 * - Sample P6.1/A14<br>
 * - Sample P9.1/A16(last)
 * @param none
 * @return none
 * @note  The 3.3V analog supply is used as reference
 * @brief  Initialize 14-bit ADC0
 */
void ADC0_InitSWTriggerCh17_14_16(void);


/**
 * Trigger a single ADC measurement on P9.0/A17, P6.1/A14, and
 * P9.1/A16, wait for it to complete, and put the results in
 * the pointers given.  The pointers are 32-bit integers
 * because the ADC registers are 32 bits.  In this case, bits
 * 31-16 are undefined, and bits 15-14 are zero.
 * Busy-wait synchronization used.
 * The ADC input voltage range is 0 to 3.3V.
 * Uses return by reference parameter passing.
 * @param ch17 is a pointer to store 32-bit P9.0/A17 conversion result<br>
 * @param ch14 is a pointer to store 32-bit P6.1/A14 conversion result<br>
 * @param ch16 is a pointer to store 32-bit P9.1/A16 conversion result
 * @return none
 * @note  Assumes ADC0_InitSWTriggerCh17_14_16() has been called.
 * @brief  Trigger ADC measurement on channels 17+14+16 and wait for result.
 */
void ADC_In17_14_16(uint32_t *ch17, uint32_t *ch14, uint32_t *ch16);


#endif /* ADC14_H_ */
