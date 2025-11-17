;--------------------------------------------------------------------------
; LCD Control Subroutines

; Initialize the LCD
INIT_LCD        BSET  DDRB,%11111111            ; configure pins PS7,PS6,PS5,PS4 for output
                BSET  DDRJ,%11000000            ; configure pins PE7,PE4 for output
                LDY   #2000                     ; wait for LCD to be ready
                JSR   DELAY_50US                  ; -"-
                LDAA  #$28                      ; set 4-bit data, 2-line display
                JSR   SEND_COMMAND                   ; -"-
                LDAA  #$0C                      ; display on, cursor off, blinking off
                JSR   SEND_COMMAND                   ; -"-
                LDAA  #$06                      ; move cursor right after entering a character
                JSR   SEND_COMMAND                   ; -"-
                RTS

; Clear the contents of the LCD
CLEAR_DISPLAY   LDAA  CLEAR_HOME                      ; clear cursor and return to home position
                JSR   SEND_COMMAND                   ; -"-
                LDY   #40                       ; wait until "clear cursor" command is complete
                JSR   DELAY_50US                  ; -"-
                RTS

; Send Command to LCD
SEND_COMMAND:   BCLR  LCD_CNTR,LCD_RS           ; select the LCD Instruction Register (IR)
                JSR   SEND_LCD_DATA                   ; send data to IR
                RTS

; Print String to LCDD
PRINT_STRING    LDAA  1,X+                      ; get one character from the string
                BEQ   DONE_PRINT_STR            ; reach NULL character?
                JSR   PRINT_CHAR
                BRA   PRINT_STRING
DONE_PRINT_STR  RTS

; Print Character to LCD
PRINT_CHAR      BSET  LCD_CNTR,LCD_RS           ; select the LCD Data register (DR)
                JSR   SEND_LCD_DATA                   ; send data to DR
                RTS

; Delay subroutine - provides ~50 microsecond delay
DELAY_50US      PSHX                            ; (2 E-clk) Protect the X register

OUTER_LOOP      LDX   #300                      ; (2 E-clk) Initialize the inner loop counter

INNER_LOOP      NOP                             ; (1 E-clk) No operation
                DBNE  X,INNER_LOOP              ; (3 E-clk) If the inner cntr not 0, loop again
                DBNE  Y,OUTER_LOOP              ; (3 E-clk) If the outer cntr not 0, loop again
                PULX                            ; (3 E-clk) Restore the X register
                RTS                             ; (5 E-clk) Else return

; Send data byte to LCD in 4-bit mode (sends upper 4 bits, then lower 4 bits)
SEND_LCD_DATA   BSET  LCD_CNTR,LCD_E            ; pull the LCD E-signal high
                STAA  LCD_DAT                   ; send the upper 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.
              
                LSLA                            ; match the lower 4 bits with the LCD data pins
                LSLA                            ; -"-
                LSLA                            ; -"-
                LSLA                            ; -"-
              
                BSET  LCD_CNTR,LCD_E            ; pull the LCD E signal high
                STAA  LCD_DAT                   ; send the lower 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.
              
                LDY   #1                        ; adding this delay will complete the internal
                JSR   DELAY_50US                ; operation for most instructions
                RTS

;--------------------------------------------------------------------------
; Analog to Digital Conversion

; Initialize Analog-to-Digital converter with 8-bit resolution
INIT_AD_CONV    MOVB  #$C0,ATDCTL2              ; power up AD, select fast flag clear
                JSR   DELAY_50US                ; wait for 50 us
                MOVB  #$00,ATDCTL3              ; 8 conversions in a sequence
                MOVB  #$85,ATDCTL4              ; res=8, conv-clks=2, prescal=12
                BSET  ATDDIEN,$0C               ; configure pins AN03,AN02 as digital inputs
                RTS

; Convert 16-bit binary number to BCD (Binary Coded Decimal) digits
BIN_TO_BCD      XGDX                            ; Save the binary number into .X
                LDAA  #0                        ; Clear the BCD_BUFFER
                STAA  TEN_THOUS
                STAA  THOUSANDS
                STAA  HUNDREDS
                STAA  TENS
                STAA  UNITS
                STAA  BCD_SPARE
                STAA  BCD_SPARE+1
                
                CPX   #0                        ; Check for a zero input
                BEQ   CONV_EXIT                 ; and if so, exit
                
                XGDX                            ; Not zero, get the binary number back to .D as dividend
                LDX   #10                       ; Setup 10 (Decimal!) as the divisor
                IDIV                            ; Divide: Quotient is now in .X, remainder in .D
                STAB  UNITS                     ; Store remainder
                CPX   #0                        ; If quotient is zero,
                BEQ   CONV_EXIT                 ; then exit
                
                XGDX                            ; else swap first quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  TENS
                CPX   #0
                BEQ   CONV_EXIT
                
                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  HUNDREDS
                CPX   #0
                BEQ   CONV_EXIT
                
                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  THOUSANDS
                CPX   #0
                BEQ   CONV_EXIT
                
                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  TEN_THOUS
                
CONV_EXIT       RTS                             ; We're done the conversion

; Convert BCD digits to ASCII with leading space suppression
BCD_TO_ASCII    LDAA  #0                        ; Initialize the blanking flag
                STAA  NO_BLANK
                
CHK_TENTHOUS    LDAA  TEN_THOUS                 ; Check the 'ten_thousands' digit
                ORAA  NO_BLANK
                BNE   CONV_TENTHOUS
                
BLANK_TENTHOUS  LDAA  #' '                      ; It's blank
                STAA  TEN_THOUS                 ; so store a space
                BRA   CHK_THOUS                 ; and check the 'thousands' digit
                
CONV_TENTHOUS   LDAA  TEN_THOUS                 ; Get the 'ten_thousands' digit
                ORAA  #$30                      ; Convert to ascii
                STAA  TEN_THOUS
                LDAA  #$1                       ; Signal that we have seen a 'non-blank' digit
                STAA  NO_BLANK
                
CHK_THOUS       LDAA  THOUSANDS                 ; Check the thousands digit for blankness
                ORAA  NO_BLANK                  ; If it's blank and 'no-blank' is still zero
                BNE   CONV_THOUS
                
BLANK_THOUS     LDAA  #' '                      ; Thousands digit is blank
                STAA  THOUSANDS                 ; so store a space
                BRA   CHK_HUNDS                 ; and check the hundreds digit
                
CONV_THOUS      LDAA  THOUSANDS                 ; (similar to 'ten_thousands' case)
                ORAA  #$30
                STAA  THOUSANDS
                LDAA  #$1
                STAA  NO_BLANK
                
CHK_HUNDS       LDAA  HUNDREDS                  ; Check the hundreds digit for blankness
                ORAA  NO_BLANK                  ; If it's blank and 'no-blank' is still zero
                BNE   CONV_HUNDS
                
BLANK_HUNDS     LDAA  #' '                      ; Hundreds digit is blank
                STAA  HUNDREDS                  ; so store a space
                BRA   CHK_TENS                  ; and check the tens digit
                
CONV_HUNDS      LDAA  HUNDREDS                  ; (similar to 'ten_thousands' case)
                ORAA  #$30
                STAA  HUNDREDS
                LDAA  #$1
                STAA  NO_BLANK
                
CHK_TENS        LDAA  TENS                      ; Check the tens digit for blankness
                ORAA  NO_BLANK                  ; If it's blank and 'no-blank' is still zero
                BNE   CONV_TENS
                
BLANK_TENS      LDAA  #' '                      ; Tens digit is blank
                STAA  TENS                      ; so store a space
                BRA   CONV_UNITS                ; and check the units digit
                
CONV_TENS       LDAA  TENS                      ; (similar to 'ten_thousands' case)
                ORAA  #$30
                STAA  TENS
                
CONV_UNITS      LDAA  UNITS                     ; No blank check necessary, convert to ascii.
                ORAA  #$30
                STAA  UNITS
                
                RTS