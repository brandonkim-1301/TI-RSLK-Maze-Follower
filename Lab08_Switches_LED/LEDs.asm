; LEDs.asm
; Runs on MSP432
; Capt Steven Beyer
; September 9, 2019


;	Code to activate LED on P6.7. This code accompanies the Lab08_LED_Switchesmain.c
;
   	.thumb
   	.text
   	.align 2
   	.global LED_Init
   	.global LED_Off
   	.global LED_On
   	.global LED_Toggle
   	.global LED_Oscillate

; function to initialize P6.7
LED_Init:	.asmfunc
	LDR R1, P6SEL0
	LDRB R0, [R1]
	BIC R0, R0, #0x80	; GPIO
	STRB R0, [R1]
	LDR R1, P6SEL1
	LDRB R0, [R1]
	BIC R0, R0, #0x80
	STRB R0, [R1]		; GPIO
	LDR R1, P6DIR
	LDRB R0, [R1]
	ORR R0, R0, #0x80	; output
	STRB R0, [R1]
	BX LR
	.endasmfunc

; function to turn off P6.7
LED_Off:		.asmfunc
	LDR R1, P6OUT
	LDRB R0, [R1]		; 8-bit read
	BIC R0, R0, #0x80	; turn off
	STRB R0, [R1]
	BX LR
	.endasmfunc

; function to turn on P6.7
LED_On:	.asmfunc
	LDR R1, P6OUT
	LDRB R0, [R1]		; 8-bit read
	ORR R0, R0, #0x80	; turn on
	STRB R0, [R1]
	BX LR
	.endasmfunc

; function to toggle P6.7
LED_Toggle: .asmfunc
	LDR R1, P6OUT
	LDRB R0, [R1]		; 8-bit read
	EOR R0, R0, #0x80	; toggle bits
	STRB R0, [R1]		; 8-bit write
	BX LR
	.endasmfunc

; function to continuously toggle P6.7 every half second
; use a loop as a timer
LED_Oscillate:	.asmfunc
switch	BL LED_Toggle	; toggle LEDs
		MOV R1, #0		; [R1] = 0
pause	ADD R1, R1, #1	; [R1] += 1
		LDR R2, DELAY	; R2 = DELAY
		CMP R2, R1		; [R2] =? [R1]
		BNE	pause		; If R2 != 0, then loop to increment counter again
		B switch		; If R2 <= 0, then done counting and toggle
	.endasmfunc


; addresses for Port 6 registers
	.align 4
P6SEL0 .word 0x40004C4B
P6SEL1 .word 0x40004C4D
P6DIR  .word 0x40004C45
P6OUT  .word 0x40004C43
DELAY  .word 05300000
	.end
