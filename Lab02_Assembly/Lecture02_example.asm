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
       .align 2			; forces machine code to be halfword-aligned
data1  .space 32		;
data2  .space 32		;
       .text			; following lines are placed in Flash ROM
       .align 2			; forces machine code to be word-aligned
       ; variable
Msg    .word 0x12345678, 0x24681012, 0x87654321, 0x1210642
Count  .word 20

	    ; pointers to variables
D1Addr  .word 	data1
D2Addr  .word 	data2
MsgAddr	.word	Msg

       .global  main	; imports main between functions
;main:  .asmfunc			; begin assembly function
       MOV R0, #100    	; R0 = 100, immediate addressing
       MOV R1, R0		; R1 = R0
       ADD R1, #2		; R1 = R1 + 2
       ADD R2, R0, R1	; R2 = R0 + R1

       LDR R0, Count	; R0 = Count (R0 = 20)
       ADD R2, R0, R1
	   LDR R0, MsgAddr
	   LDR R1, [R0]

stall  B	stall		; stall here
       .endasmfunc

       .end			;



