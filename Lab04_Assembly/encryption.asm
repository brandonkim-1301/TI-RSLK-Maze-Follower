; Encryption.asm
; Runs on any Cortex M
; Student name: Brandon Kim
; Date:
; Basic XOR encryption and decryption functions for cryptography.
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
       .text
       .align 2
       .global Encrypt
       .global Decrypt
       .global XOR_bytes

;------------Encrypt------------
; Takes in the location of a plain text message and key and
;	xors the message to encrypt it. It stores the
;	result at a passed in memory location. Stops
;	when the ASCII character # is encountered.
; Input: R0 is message address, R1 is key address,
;	R2 is location to store encrypted message
; Output: encrypted message stored at memory
;	located ate R2
; Modifies: R0, R1
Encrypt:   .asmfunc
	PUSH {R4-R7, LR}		; creates stack
	MOV	 R4, R0				; R4 = R0, original message
	MOV	 R5, R1				; R5 = R1, original key
	LDR  R6, EOM			; loads end of message key
	LDRB R1, [R5]			; reads first byte of key
EncLoop
	LDRB R0, [R4], #1		; reads next byte of message
	MOV	 R7, R0				; R7 = R0
	BL 	 XOR_bytes			; branch to XOR bytes, encrypt message
	STRB R0, [R2], #1		; stores encrypted message by byte
	CMP	 R7, R6				; check if end of message
	BNE	 EncLoop			; if not end of message, loop back
	POP {R4-R7, LR}			; empties stack
	BX LR

		.endasmfunc

;------------Decrypt------------
; Takes in the location of an encrypted message and key and
;	xors the message to decrypt it it. It stores the
;	result at a passed in memory location. Stops
;	when the ASCII character # is encountered.
; Input: R0 is message address, R1 is key address,
;	R2 is location to store decrypted message
; Output: decrypted message stored at memory
;	located ate R2
; Modifies: R0, R1
Decrypt:	.asmfunc
	; save registers
	; select first byte of key and clear the rest of the bytes
	; loop
	; retrieve next byte of message
	; XOR_Bytes
	; Compare byte from decrypted with end of message
	; loop or end
	; restore messages
	PUSH {R4-R7, LR}		; creates stack
	MOV	 R4, R0				; R4 = R0, original message
	MOV	 R5, R1				; R5 = R1, original key
	LDR  R6, EOM			; loads end of message key
	LDRB R1, [R5]			; reads first byte of key
DecLoop
	LDRB R0, [R4], #1		; reads next byte of message
	BL XOR_bytes			; branch to XOR bytes, decrypt message
	STRB R0, [R2], #1		; stores decrypted message by byte
	CMP	R0, R6				; check if end of message
	BNE DecLoop				; if not end of message, loop back
	POP {R4-R7, LR}			; empties stack
	BX LR
		.endasmfunc


;------------XOR_Bytes------------
; Takes in two bytes, XORs them, returns the result
; Input: R0 message byte, R1 is key byte,
; Output: R0 is XOR result
; Modifies: R0
XOR_bytes:	.asmfunc
	EOR	R0, R0, R1	; R0 = R0^R1
	BX LR
	.endasmfunc

	.align 4
	;End of Message character, #
EOM	.word	'#'
    .end
