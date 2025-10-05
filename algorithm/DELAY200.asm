        AREA D_DELAY_200_AREA, DATA, READWRITE    ; Define data area (if needed)
        AREA C_DELAY_200_AREA, CODE, READONLY     ; Define a code area for the subroutine

        EXPORT DELAY200                           ; Export the subroutine so it can be called externally

; DELAY200 - Creates an approximate 200 ms delay
DELAY200    PROC
            LDR R0, =830122                       ; Load R0 with a count of 400,000
DELAY_LOOP  SUBS R0, R0, #1                       ; Decrement R0 by 1
            NOP                                   ; No operation (1 cycle delay, helps tuning)
            BNE DELAY_LOOP                        ; Repeat until R0 reaches 0
            BX LR                                 ; Return from subroutine
            ENDP

	END                                       ; End of the assembly file
