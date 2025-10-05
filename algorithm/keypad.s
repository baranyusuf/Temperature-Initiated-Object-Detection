	AREA main_area, CODE, READONLY
	THUMB
	EXPORT keypad


GPIO_PORTD_DATA EQU 0x400073FC  ; All pins data address for Port D


GPIO_PORTB_DATA EQU 0x400053FC  ; All pins data address for Port B
GPIO_PORTB_DIR  EQU 0x40005400  ; Direction register for Port B
GPIO_PORTB_DEN  EQU 0x4000551C  ; Digital enable for Port B
SYSCTL_RCGCGPIO EQU 0x400FE608  ; Clock Gating Control for GPIO
R2_Storage EQU 0x20000910      ; Set address to a writable location in RAM

    ; Define the I/O mask
LOW_NIBBLE      EQU 0x0F        ; Mask for low nibble B3-B0 (LEDs)
HIGH_NIBBLE     EQU 0xF0        ; Mask for high nibble B7-B4 (Push buttons)    

keypad
    
	LDR R7, =R2_Storage  ; Load address of the R2_Storage variable
	MOV R5, #3
	LDR R1, =GPIO_PORTB_DATA
	LDR R0, [R1]
	BIC R0 , R0 , #0x0C
	STR R0 , [R1]
	LDR R8, =GPIO_PORTD_DATA
	LDR R0, [R1]
	BIC R0 , R0 , #0x0C
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	
	
main_lobby	
	
    LDR R0, [R1]
	MOV R2, R0
	BIC R2 , R2, #0xC3
	BIC R0 , R0 , #0xFF
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0, [R8]
	ORR R0, R0 , #0x04
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0 , [R1]
	MOV R3 , R0
	BIC R3 , R3 , #0xC3
	MOV R10, #0x04
	CMP R3 , R2
	BNE sensed
	
	LDR R0, [R1]
	MOV R2, R0
	BIC R2 , R2, #0xC3
	BIC R0 , R0 , #0xFF
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0, [R8]
	ORR R0, R0 , #0x08
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0 , [R1]
	MOV R3 , R0
	BIC R3 , R3 , #0xC3
	MOV R10, #0x08
	CMP R3 , R2
	BNE sensed
	
	LDR R0, [R1]
	MOV R2, R0
	BIC R2 , R2, #0xC3
	BIC R0 , R0 , #0xFF
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0, [R8]
	ORR R0, R0 , #0x40
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0 , [R1]
	MOV R3 , R0
	BIC R3 , R3 , #0xC3
	MOV R10, #0x01
	CMP R3 , R2
	BNE sensed
	
	LDR R0, [R1]
	MOV R2, R0
	BIC R2 , R2, #0xC3
	BIC R0 , R0 , #0xFF
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0, [R8]
	ORR R0, R0 , #0x80
	STR R0 , [R8]
	NOP
	NOP
	NOP
	NOP
	LDR R0 , [R1]
	MOV R3 , R0
	BIC R3 , R3 , #0xC3
	MOV R10, #0x02
	CMP R3 , R2
	BNE sensed

	
	B main_lobby 
	
	
delay_200ms
    LDR     R0, =4800000     ; Load the loop count for 200ms (16MHz clock / 5 instructions)
delay_loop
    SUBS    R0, R0, #1       ; Decrement the counter
    BNE     delay_loop       ; If counter is not zero, branch back to delay_loop
    B       main_lobby             ; Return from the subroutine

	
sensed
	BIC R0 , R0, #0xC3
	LSL R0 , R0 ,#2
	ORR R0 , R0, R10
	STR R0, [R7]
	NOP
	NOP
	NOP
	ADD R7 , R7 ,#4
	SUB R5 , R5, #1
	MOV R6 , #0
	CMP R5,R6
	BEQ finished
	B delay_200ms


	
finished	

    BX LR     ; Return to C
END
