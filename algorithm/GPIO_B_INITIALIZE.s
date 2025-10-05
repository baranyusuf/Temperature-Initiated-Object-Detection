	AREA main_area, CODE, READONLY
	THUMB
	EXPORT GPIO_B_INITIALIZE
		
GPIO_PORTD_DATA EQU 0x400073FC  ; All pins data address for Port D
GPIO_PORTD_DIR  EQU 0x40007400  ; Direction register for Port D
GPIO_PORTD_DEN  EQU 0x4000751C  ; Digital enable for Port D

GPIO_PORTD_LOCK EQU 0x40007520  ; Lock register for Port D
GPIO_PORTD_CR   EQU 0x40007524  ; Commit register for Port D
	
GPIO_PORTB_DATA EQU 0x400053FC  ; All pins data address for Port B
GPIO_PORTB_DIR  EQU 0x40005400  ; Direction register for Port B
GPIO_PORTB_DEN  EQU 0x4000551C  ; Digital enable for Port B
SYSCTL_RCGCGPIO EQU 0x400FE608  ; Clock Gating Control for GPIO
R2_Storage EQU 0x20000010  ; Set address to a writable location in RAM

LOW_NIBBLE      EQU 0x0F        ; Mask for low nibble B3-B0 (LEDs)
HIGH_NIBBLE     EQU 0xF0        ; Mask for high nibble B7-B4 (Push buttons)    

GPIO_B_INITIALIZE
	NOP         ; No operation in ARM Assembly
    LDR R1, =SYSCTL_RCGCGPIO
    LDR R0, [R1]
    ORR R0, R0, #0x0A           ; Enable clock for Port B
    STR R0, [R1]
    NOP                          ; Small delay for stabilization
    NOP
    NOP
	
	LDR R1, =GPIO_PORTD_LOCK     ; Load the address of the lock register
    LDR R0, =0x4C4F434B          ; Unlock key for GPIO registers
    STR R0, [R1]                 ; Write the unlock key

    LDR R1, =GPIO_PORTD_CR       ; Load the address of the commit register
    MOV R0, #0x80                ; Enable commit for PD7 (bit 7)
    STR R0, [R1]                 ; Write to the commit register

    ; Configure Port B
    LDR R1, =GPIO_PORTB_DIR
    LDR R0, [R1]
    BIC R0, R0, #0xFF     ; Set B7-B4 as inputs
    STR R0, [R1]	
	LDR R1, =0x40005514      ; GPIO_PORTB_PUR register address
	LDR R0, [R1]
	ORR R0, R0, #0x3F        ; Enable pull-up resistors on B7-B4 (input columns)
	STR R0, [R1]
	
	LDR R1, =GPIO_PORTD_DIR
    LDR R0, [R1]
    ORR R0, R0, #0xCC      ; Set B3-B0 as outputs
    STR R0, [R1]	


    LDR R1, =GPIO_PORTB_DEN
    LDR R0, [R1]
    ORR R0, R0, #0x3C            ; Enable digital functionality for all B pins
    STR R0, [R1]
	
	LDR R1, =GPIO_PORTD_DEN
    LDR R0, [R1]
    ORR R0, R0, #0xCC            ; Enable digital functionality for all B pins
    STR R0, [R1]
	
	
	LDR R1, =GPIO_PORTD_DATA
	LDR R0, [R1]
    ORR R0, R0, #0xCC            ; Enable digital functionality for all B pins
    STR R0, [R1]
	
	
	
	
	BX LR     ; Return to C

END