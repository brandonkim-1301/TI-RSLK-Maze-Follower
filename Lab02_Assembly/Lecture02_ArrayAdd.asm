; Array_Add.asm
; by George York
;    let array Y equal array X plus 3
; Aug 20 2020

       .thumb
       .data
       .align 2
Y_array .space 32   ; create 32 bytes in RAM to hold the output Y array

       .text
       .align 2
X_array .byte 0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08, 0x09, 0x0A   ; initialize X_array to 10 numbers in ROM
N .byte 0x0A ; N is the length of the array

       .global main

;main:  .asmfunc
	 ; load pointer registers
     LDR R0, YAddr
     LDR R1, XAddr
     ; load constant number "3"
     LDR R6, three

     LDR R2, NAddr
     LDRB R3, [R2]  ; get the length of the array... Use LDRB since it is only a byte, not a 32-bit word

     ; Loop for the length of the array, letting Y = 3 + X
loop LDRB R4, [R1], #1  ; powerful instruction. loads the byte R1 is pointing to into R4, and then increments R1 to point in next byte in array
     ADD R5, R4, R6    ; let y = x + 3
	 STRB R5, [R0], #1  ; stores the byte in R5 into RAM where R0 is pointing, and then increments R0 to point in next byte in array
	 ; let's downcount R3 by one until it is zero to know when we are done
     SUB R3, R3, #1
     CMP R3, #0
	 BNE loop       ; if not done, loop back to do next byte, else quit

quit B quit             ; stall here and observe the registers

    .endasmfunc
YAddr .word Y_array ; get address or pointer to Y_array
XAddr .word X_array ; get address or pointer to X_array
NAddr .word N
three .word 3
    .end
