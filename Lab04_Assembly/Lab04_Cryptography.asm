; Cryptography.asm
; Runs on any Cortex M
; Student name: Brandon Kim
; Date: 25 Aug 2021
; Documentation statement: Refer to Gradescope.
; Cryptography function to encrypt and decrypt a message using a 1 byte key.
; Capt Steven Beyer
; July 23 2020
;
;Simplified BSD License (FreeBSD License)
;Copyright (c) 2020, Steven Beyer, All rights reserved.
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
       .data
       .align 2
		; variables in RAM
encrypted_message .space 32	; reserve 32 bytes of memory to store the encrypted message
decrypted_message .space 32	; reserve 32 bytes of memory to store the decrypted message
       .text
       .align 2
		;variables in ROM
message1 .string "To be is to do#"	; string message 1
message2 .string "To do is to be#" 	; string message 2
message3 .string "Do be do be do#" 	; string message 3

EMessage .word 	0x196F696F, 0x430A1812, 0x45490A59, 0x00094645

key1	.byte	0xab	; byte key
key2	.byte	0xcd	; byte key
key3	.byte	0xef	; byte key
key4 	.byte  	42		; Key for Deliverable 3.

       	.global Encrypt
       	.global Decrypt
       	.global main

main:  .asmfunc
		; loads registers
	LDR R0, EMsgAddr
	LDR R1, key4Addr
	LDR R2, encMsgAddr

     	; encrypts the message
	BL Encrypt

     	; loads registers
	LDR R0, encMsgAddr
	LDR R1, key4Addr
	LDR R2, decMsgAddr

     	; decrypts the message
	BL Decrypt

Loop	B Loop	; stall here and observe the registers

    .endasmfunc

    ; 32-bit pointers to each of the variables in memory
msg1Addr	.word message1
msg2Addr	.word message2
msg3Addr	.word message3
EMsgAddr	.word EMessage
encMsgAddr	.word encrypted_message
decMsgAddr	.word decrypted_message
key1Addr	.word key1
key2Addr	.word key2
key3Addr	.word key3
key4Addr	.word key4
    .end
