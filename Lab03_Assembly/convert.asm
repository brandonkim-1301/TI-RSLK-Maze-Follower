; Convert.asm
; Runs on any Cortex M
; Student version to lab
; Conversion function for a GP2Y0A21YK0F IR sensor
; Jonathan Valvano
; July 11, 2019

; This example accompanies the book
;   "Embedded Systems: Introduction to Robotics,
;   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
; For more information about my classes, my research, and my books, see
; http://users.ece.utexas.edu/~valvano/
;
;Simplified BSD License (FreeBSD License)
;Copyright (c) 2019, Jonathan Valvano, All rights reserved.
;
;Redistribution and use in source and binary forms, with or without modification,
;are permitted provided that the following conditions are met:
;
;1. Redistributions of source code must retain the above copyright notice,
;   this list of conditions and the following disclaimer.
;2. Redistributions in binary form must reproduce the above copyright notice,
;   this list of conditions and the following disclaimer in the documentation
;   and/or other materials provided with the distribution.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
;USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;The views and conclusions contained in the software and documentation are
;those of the authors and should not be interpreted as representing official
;policies, either expressed or implied, of the FreeBSD Project.

       .thumb
       .text
       .align 2
       .global Convert

;------------Convert------------
; Calculate the distance in mm given the 14-bit ADC value
; D = 1195172/(n - 1058)
; Input: R0 is n, ADC value, 14-bit unsigned number 2552 to 16383
; Output: R0 is distance in mm
; If the ADC input is less than 2552, return 800 mm
; Modifies: R1, R2
Convert:   .asmfunc
		LDR R2, ADCLim		; R2 = 2552, loads the limit of the ADC into R2
		CMP R2, R0			; R2 =? R0, compares the ADC limit to the input
		BGE RetMin			; if R2 >= R0, then branches to return min value of the ADC
		LDR R1, IROffset	; R1 = 1058, loads the IR sensor's offset into R1
		SUB R0, R0, R1		; R0 = R0 - R1, subtracts the offset from the input
		LDR R1, IRSlope		; R1 = 1195172, loads the IR sensor's slope into R1
		UDIV R0, R1, R0		; R0 = R1 / R0, calculates the calibration for input
		B Done
RetMin	LDR R0, MinRet		; R0 = 800, returns the minimum ADC value for input
Done	BX LR				; branches to the address in link address, or main

      .endasmfunc
      .align 4

IRSlope  	.word	1195172	; slope for the ADC conversion calculation
IROffset	.word	1058	; offset for the ADC conversion calculation
ADCLim		.word	2552	; input limit for the ADC
MinRet		.word	800		; ADC reading, if the input is less than or equal to limit

    .end
