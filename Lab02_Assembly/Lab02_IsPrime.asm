; main.s
; Runs on any Cortex M processor
; A very simple first project implementing a random number generator
; Daniel Valvano
; May 12, 2015

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

; we align 32 bit variables to 32-bits
; we align op codes to 16 bits

		.thumb
        .data			; following lines are placed in RAM
        .align 2		; forces machine code to be halfword-aligned
Res     .space 16 		; reserves 16 bytes in RAM to store results, one byte for each result
        .text			; following lines are placed in Flash ROM
        .align 4		; forces machine code to be word-aligned
        ; variable
Nums    .word 1, 2, 7, 10, 15, 62, 97, 282, 408, 467, 880, 967, 0
		; prime numbers are 2, 7, 97, 467, and 967

	    ; pointers to variables
ResAddr	 .word	Res
NumsAddr .word	Nums
        .global  main

main:  	.asmfunc		 	; begin assembly function
       	LDR R2, ResAddr		; R2 = [ResAddr]
		LDR R3, NumsAddr	; R3 = [NumsAddr]

Loop1
		LDR R0, [R3], #4	; Read from R3 and R3 += 4 , 4 bytes/word to inc via Nums
		CMP R0, #0			; R0 == 0?  (n == 0?) , end of the Nums array?
		BEQ	Exit			; EOF, exit

		CMP R0, #1			; (n == 1)?
		BEQ	False			; if so, go to False

		ASR	R5, R0, #1		; R5 = R0/2 (m = n/2)
		MOV R6, #2			; R6 = 2 (i = 2)

; add your code here
Loop2
		CMP		R6, R5		; i == m ?, loop conditional
		BGT		True		; if so, go to True

		UDIV	R7, R0, R6	; R7 = R0/R6 (n/i)
		MUL		R8, R7, R6	; R8 = R7*R6 (int(n/i)*i)

		CMP		R0, R8		; n == int(n/i)*i ?, n is divisible by i if R0 == R8
		BEQ		False		; if so, go to False
		ADD		R6, #1		; otherwise, i++
		B		Loop2		; and loop back on inner loop
False
		MOV		R9, #0			; R9 = 0, number is not prime
		STRB	R9, [R2], #1	; R2 = 0
		B		Loop1
True
		MOV		R9, #1			; R9 = 1, number is prime
		STRB	R9, [R2], #1	; R2 = 1
		B		Loop1

; do not modify anything below this line
Exit	B	Exit		; stall here and observe the registers and memory


       .endasmfunc
       .end

