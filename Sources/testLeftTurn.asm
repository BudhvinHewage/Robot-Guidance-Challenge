;-------------------------------------------------------------------
;*              Lab 6: Robot Guidance Challenge (9S32C)               *
;-------------------------------------------------------------------

; export symbols
                        XDEF Entry, _Startup  ; export 'Entry' symbol
                        ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
                        INCLUDE 'derivative.inc'

; The sensor value can show when exactly the robot will line up with the tape, adjust the value and play with it identify when it stop on the line, or as close as possible

; The robot chosen for this challenge has the following serial number: 102933

;-------------------------------------------------------------------
; Equates Section
;-------------------------------------------------------------------

; Directions and Headings
DIR_UNKNOWN             EQU 0
DIR_NORTH               EQU 1
DIR_EAST                EQU 2
DIR_SOUTH               EQU 3
DIR_WEST                EQU 4

; The possible states of the robot
STATE_IDLE              EQU 0                               ; Waiting for the start signal (bumper)
STATE_SEARCH            EQU 1                               ; Following the line normally at the start of the maze and then looks for the first intersection
STATE_AT_INTERSECTION   EQU 2                               ; Intersection logic: detect options, decide turn
STATE_MOVING_BRANCH     EQU 3                               ; Move down the selected path (until next intersection or dead end)
STATE_BUMPER_COLLIDE    EQU 4                               ; Dead end detected via front bumper (turnaround logic)
STATE_BACKTRACKING      EQU 5                               ; Retrace back to last intersection and correct stored decision
STATE_RETRACING         EQU 6                               ; Full retrace back to the start (following stored solutions only)

; Thresholds for the sensors                                                                                   
THRESH_CENTER_RIGHT     EQU $85                             ; If below this, robot is veering to the RIGHT
THRESH_CENTER_LEFT      EQU $D4                             ; If above this, robot is veering to the LEFT
THRESH_BOW              EQU $CF                             ; Front sensor
THRESH_MID              EQU $B5                             ; Middle sensor
THRESH_PORT             EQU $B5                             ; Left sensor
THRESH_STBD             EQU $B5                             ; Right sensor 

; Motor Timing Intervals
FWD_INT                 EQU 10                              ; Forward movement interval
REV_INT                 EQU 10                              ; Reverse movement interval
T_RIGHT_INT             EQU 5                               ; Right turn interval
T_LEFT_INT              EQU 5                               ; Left turn interval

; Path status values
PATH_UNKNOWN            EQU 0
PATH_FAILED             EQU 1
PATH_SUCCESS            EQU 2

;LCD ADDRESSES
LCD_DAT                 EQU   PORTB                         ;LCD data port, bits - PB7,...,PB0
LCD_CNTR                EQU   PTJ                           ;LCD control port, bits - PE7(RS),PE4(E)
LCD_E                   EQU   $80                           ;LCD E-signal pin
LCD_RS                  EQU   $40                           ;LCD RS-signal pin

;LCD DISPLAY EQUATES
CLEAR_HOME              EQU $01
INTERFACE               EQU $38
CURSOR_OFF              EQU $0C
SHIFT_OFF               EQU $06
LCD_SEC_LINE            EQU 64

;-------------------------------------------------------------------
; Variable and Data Section
;-------------------------------------------------------------------      

                        ORG $3800                           ; Where our TOF counter register lives

; Temporary Storage 
EXIT_LIST:              DS.B 2                             ; List of available exits at current intersections
TEMP_DIRECTION          DS.B 1
TEMP_EXIT_COUNT         DS.B 1
TEMP_FIRST_EXIT         DS.B 1
TEMP_SECOND_EXIT        DS.B 1

; Sensors, beginning from $3806
SENSOR_LINE             FCB $00                             ; Line Sensor
SENSOR_BOW              FCB $00                             ; Front ("bow") sensor
SENSOR_PORT             FCB $00                             ; Left sensor
SENSOR_MID              FCB $00                             ; Center sensor
SENSOR_STBD             FCB $00                             ; Right sensor

SENSOR_NUM              RMB 1                               ; The currently selected sensor

; Storage for Maze Mapping, beginning from $380C
MAZE_TABLE:             DS.B 24                             ; Maze table for up to 8 intersections (3 bytes each) where byte 0 = entry dir, byte 1 = first exit tried, byte 2 = second exit tried

; Variables for Maze Navigation, beginning from $3824
SCRATCH_DIR             DS.B 1                              ; Temporary storage for calculated directions
TEMP                    DS.B 1                              ; Temporary storage for second exit
CURRENT_INTERSECTION:   DS.B 1                              ; Index of current intersection (1..MAX)
HEADING:                DS.B 1                              ; 0..3 (N,E,S,W) or use DIR_*
ENTRY_DIRECTION:        DS.B 1                              ; Direction we entered current intersection from
STATE:                  DS.B 1
STACK_PTR:              DS.B 1                              ; Simple stack top for backtracking (store inter. indices)
CURRENT_STATE           DC.B 0                              ; Current state register

NULL                    EQU 00

; Tof Counter
TOF_COUNTER             DC.B 0                              ; The timer, incremented at 23Hz
CC                      DC.B 8

; LCD Display Variables
ALIVE_COUNTER           DS.B 1                              ; Counter to toggle ALIVE display
STR_IDLE                DC.B "IDLE        ",0
STR_MOVING              DC.B "MOVING      ",0 
STR_INTERSECTION        DC.B "INTERSECTION",0
STR_BACKTRACK           DC.B "BACKTRACK   ",0 
STR_UNKNOWN             DC.B "UNKNOWN     ",0   
SENSOR_LABELS:          DC.B "PCPCSBS:"    ,0 

;LCD CURSOR POSITIONS FOR DISPLAY
TOP_LINE                RMB 20
                        FCB NULL
BOT_LINE                RMB 40
                        FCB NULL
DP_FRONT_SENSOR         EQU TOP_LINE+3
DP_PORT_SENSOR          EQU BOT_LINE+0
DP_MID_SENSOR           EQU BOT_LINE+3
DP_STBD_SENSOR          EQU BOT_LINE+6
DP_LINE_SENSOR          EQU BOT_LINE+9

; Found Intersection Flag
FOUND_INTERSECTION      DS.B 1                              ; Set to $01 when intersection is found

;-------------------------------------------------------------------
; Initialization
;-------------------------------------------------------------------   

                        ORG $4000
                  
Entry:
 _Startup:
                        LDS #$4000                          ; Initialize the stack pointer
                        CLI                                 ; Enable interrupt
                        
                        JSR INIT_PORTS
                        JSR INIT_ATD
                        JSR initLCD
                        JSR clrLCD         
                        BRA MAIN

;-------------------------------------------------------------------
; Main Program Section
;-------------------------------------------------------------------

MAIN:                   BRSET PORTAD0,$04,NO_START          ; Check front bumper - if NOT pressed, skip to NO_START
                        BRA   START_NAVIGATION              ; Bumper IS pressed, start navigation

NO_START:               JSR   INIT_ALL_STP                  ; Keep motors stopped while waiting
                        BRA   MAIN                          ; Loop back and keep checking bumper

START_NAVIGATION:       JSR   LINE_NAVIGATION               ; Execute line following
                        LDAA  FOUND_INTERSECTION
                        CMPA  #$01
                        BEQ   LEFT_TURN_TEST                ; An intersection found, end main program
                        BRA   START_NAVIGATION              ; Keep navigating

LEFT_TURN_TEST          JSR   LEFT_WAIT_FOR_STRIP           ; Wait for strip fade/rise
                        LDY   #20000                        ; 
                        JSR   DELAY_50US
                        BRA   FINAL_NAVIGATION
                        
FINAL_NAVIGATION        JSR   LINE_NAVIGATION 
                        BRA   FINAL_NAVIGATION

END_MAIN:               SWI                                 ; Software Interrupt - end of main prograM
                        
;-------------------------------------------------------------------------- 
; Movement along an indicated line until intersection or bumper is detected
;--------------------------------------------------------------------------

LINE_NAVIGATION:        JSR   SENSOR_READ                   ; Refresh sensor values

                        ; Check if MID sensor has the main line
                        LDAA  SENSOR_MID
                        CMPA  #THRESH_MID
                        BLO   NO_INTERSECTION               ; MID doesn't see line = not at intersection

                        ; MID sees line - now check for branch lines
                        LDAA  SENSOR_PORT
                        CMPA  #THRESH_PORT
                        BHS   INTERSECTION_FOUND            ; PORT high + MID high = real intersection
                        
                        LDAA  SENSOR_STBD
                        CMPA  #THRESH_STBD
                        BHS   INTERSECTION_FOUND            ; STBD high + MID high = real intersection

NO_INTERSECTION:        ; No intersection - do line following
                        LDAA  SENSOR_LINE                   ; Use differential for positioning
                        CMPA  #THRESH_CENTER_RIGHT
                        BLO   LINE_NAV_LEFT                ; Line to right
                        CMPA  #THRESH_CENTER_LEFT
                        BHS   LINE_NAV_RIGHT                 ; Line to left
                        BRA   LINE_NAV_FORWARD              ; Centered

INTERSECTION_FOUND:     JSR   INIT_ALL_STP
                        LDAA  #$01
                        STAA  FOUND_INTERSECTION
                        JMP   LINE_NAV_EXIT

LINE_NAV_LEFT:          JSR   INIT_SOFT_LEFT
                        LDY   #1000                          ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        JSR   INIT_ALL_STP
                        LDY   #50
                        JSR   DELAY_50US
                        BRA   LINE_NAV_EXIT

LINE_NAV_RIGHT:         JSR   INIT_SOFT_RIGHT
                        LDY   #1000                          ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        JSR   INIT_ALL_STP
                        LDY   #50
                        JSR   DELAY_50US
                        BRA   LINE_NAV_EXIT

LINE_NAV_FORWARD:       JSR   INIT_FWD
                        LDY   #2000                         ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        JSR   INIT_ALL_STP
                        LDY   #50
                        JSR   DELAY_50US
                        BRA   LINE_NAV_EXIT

LINE_NAV_EXIT:          RTS                   

;---------------------------------------------------------------------------
; Wait for Strip Subroutine - Left Turn
;---------------------------------------------------------------------------

LEFT_WAIT_FOR_STRIP:    JSR  INIT_FWD
                        LDY  #10000
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #1000
                        JSR  DELAY_50US
                        
LEFT_WAIT_STRIP_FADE:   JSR  INIT_LEFT_TRN
                        LDY  #500
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BHS  LEFT_WAIT_STRIP_FADE                 ; Still on old strip, wait

LEFT_WAIT_STRIP_RISE:   JSR  INIT_LEFT_TRN
                        LDY  #500
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BLO  LEFT_WAIT_STRIP_RISE                ; New strip not detected yet, keep waitingg
                                        
LEFT_WAIT_STRIP_DONE:  RTS

;--------------------------------------------------------------------------
; Initialize Ports
;--------------------------------------------------------------------------

INIT_PORTS              BCLR DDRAD,$FF                      ; Make PORTAD an input (DDRAD@$0272)
                        BSET DDRA, $FF                      ; Make PORTA an output (DDRA@$0002)
                        BSET DDRB, $FF
                        BSET DDRJ, $C0
                        BSET DDRT, $30                      ; STAR_SPEED, PORT_SPEED 00110000
                        RTS
                        
;--------------------------------------------------------------------------
; Motor Control Subroutines
;--------------------------------------------------------------------------

; Turn motors on to move forward
INIT_FWD                BCLR  PORTA,%00000011               ; Set FWD direction for both motors
                        BSET  PTT,%00110000                 ; Turn on the drive motors
                        RTS
                    
; Turn motors on to move backward
INIT_REV                BSET  PORTA,%00000011               ; Set REV direction for both motors
                        BSET  PTT,%00110000                 ; Turn on the drive motors
                        RTS
                    
; Turn motors off
INIT_ALL_STP            BCLR  PTT,%00110000                 ; Turn off the drive motors
                        RTS
                    
; Turn motors on to rotate right
INIT_RIGHT_TRN          BCLR  PORTA,%00000010
                        BSET  PORTA,%00000001               ; Set REV dir. for STARBOARD (right) motor
                        BSET  PTT,  %00110000
                        RTS
                    
; Turn motors on to rotate left
INIT_LEFT_TRN           BCLR  PORTA,%00000001               ; Set FWD dir. for STARBOARD (right) motor
                        BSET  PORTA,%00000010
                        BSET  PTT,%00110000
                        RTS

; Turn motors on to pivot right (Port FWD, Starboard OFF)
INIT_SOFT_RIGHT:        BSET  PORTA,%00000010               ; Set Port (Left) direction FWD (0) (Assuming BCLR for FWD)
                        BCLR  PORTA,%00000001               ; Set Starboard (Right) direction FWD (0) - doesn't matter since it's off
                        BCLR  PTT,  %00010000               ; Turn OFF Starboard motor (bit 4)
                        BSET  PTT,  %00100000               ; Turn ON Port motor (bit 5)
                        RTS                    

; Turn motors on to pivot left (Starboard FWD, Port OFF)
INIT_SOFT_LEFT:         BCLR  PORTA,%00000010               ; Set Port (Left) direction FWD (0) - doesn't matter since it's off
                        BSET  PORTA,%00000001               ; Set Starboard (Right) direction FWD (0)
                        BSET  PTT,  %00010000               ; Turn ON Starboard motor (bit 4)
                        BCLR  PTT,  %00100000               ; Turn OFF Port motor (bit 5)
                        RTS

;---------------------------------------------------------------------------
; Guider Sensor Read Subroutine
;---------------------------------------------------------------------------

SENSOR_READ             JSR G_LEDS_ON     ; Enable the guider LEDs
                        JSR READ_SENSORS  ; Read the 5 guider sensors
                        JSR G_LEDS_OFF    ; Disable the guider LEDs
                        JSR UPDT_LCD
                        RTS

;---------------------------------------------------------------------------
; Guider LED Control Subroutines
;---------------------------------------------------------------------------

; Turn on guider LEDs (to measure reflected light via illumination)
G_LEDS_ON               BSET  PORTA,%00100000               ; Set bit 5 of PORTA to 1 (enable LEDs)
                        RTS

; Turn off guider LEDs (to measure ambient light or conserve power)
G_LEDS_OFF              BCLR  PORTA,%00100000               ; Clear bit 5 of PORTA to 0 (disable LEDs)
                        RTS

;---------------------------------------------------------------------------
; Sensor Reading Subroutine
;---------------------------------------------------------------------------

READ_SENSORS
    
RS_MAIN_LOOP            CLR   SENSOR_NUM                    ; Initialize sensor index to 0
                        LDX   #SENSOR_LINE                  ; Point X at the start of sensor RAM array
                        
RS_LOOP                 LDAA  SENSOR_NUM                    ; Load current sensor number
                        JSR   SELECT_SENSOR                 ; Select corresponding physical sensor

                        LDY   #400                        ; Wait ~20ms to stabilize sensor reading
                        JSR   DELAY_50US                    ; (400 * 50Ã‚Âµs = 20ms)

                        LDAA  #%10000001                    ; Configure ATD: single scan, channel AN1
                        STAA  ATDCTL5                       ; Start analog-to-digital conversion

                        BRCLR ATDSTAT0,$80,*                ; Loop until conversion complete (SCF=1)

                        LDAA  ATDDR0L                       ; Read 8-bit ADC result from ATDDR0L
                        STAA  0,X                           ; Store result at current sensor location

                        CPX   #SENSOR_STBD                  ; Check if last sensor (index 4)
                        BEQ   RS_DONE                       ; If yes, exit
                        
                        INC   SENSOR_NUM                    ; Increment sensor index
                        INX                                 ; Increment pointer to next RAM location
                        BRA   RS_LOOP                       ; Repeat for next sensor
RS_DONE:
                                               
RS_EXIT                 RTS                                 ; Return from subroutine

;---------------------------------------------------------------------------
; Sensor Selector Subroutine
;---------------------------------------------------------------------------

SELECT_SENSOR           PSHA                                ; Save ACCA (sensor number) temporarily

                        LDAA  PORTA                         ; Read PORTA
                        ANDA  #%11100011                    ; Clear bits 2-4 (sensor select bits)
                        STAA  TEMP                          ; Store modified value in TEMP
                        
                        PULA                                ; Restore sensor number to ACCA
                        ASLA                                ; Shift sensor number left twice (bits 0-2 to 2-4)
                        ASLA
                        ANDA  #%00011100                    ; Mask off irrelevant bits
                        
                        ORAA  TEMP                          ; Merge with stored PORTA bits (update sensor bits)
                        STAA  PORTA                         ; Write back to PORTA (select sensor)
                        RTS
;--------------------------------------------------------------------------
; initialize LCD subroutine
;--------------------------------------------------------------------------
;--------------------------------------------------------------------------
; Initialize the LCD  (8-bit mode)
;--------------------------------------------------------------------------
initLCD
                        LDY  #2000              ; Wait ~100 ms for LCD power-up
                        JSR  DELAY_50US

                        LDAA #INTERFACE         ; 8-bit, 2-line, 5x8 font
                        JSR  cmd2LCD

                        LDAA #CURSOR_OFF        ; Display ON, cursor OFF
                        JSR  cmd2LCD

                        LDAA #SHIFT_OFF         ; Increment cursor, no shift
                        JSR  cmd2LCD

                        LDAA #CLEAR_HOME        ; Clear display + return home
                        JSR  cmd2LCD

                        LDY  #40                ; Wait >1.6ms for clear to finish
                        JSR  DELAY_50US

                        RTS
                       
;--------------------------------------------------------------------------
; clrLCD  subroutine
;--------------------------------------------------------------------------
clrLCD
                       LDAA #$01                            ; clear cursor and return to home position
                       JSR cmd2LCD                          ; -"-
                       LDY #40                              ; wait until "clear cursor" command is complete
                       JSR DELAY_50US                         ; -"-
                       RTS


;--------------------------------------------------------------------------
; cmd2LCD  subroutine
;--------------------------------------------------------------------------
cmd2LCD
                       BCLR LCD_CNTR,LCD_RS                 ; select the LCD Instruction Register (IR)
                       JSR dataMov                          ; send data to IR
                       RTS
 
;--------------------------------------------------------------------------
; putsLCD  subroutine
;--------------------------------------------------------------------------
putsLCD
                       LDAA 1,X+                            ; get one character from the string
                       BEQ donePS                           ; reach NULL character?
                       JSR putcLCD
                       BRA putsLCD
donePS                 RTS

;--------------------------------------------------------------------------
; putcLCD  subroutine
;--------------------------------------------------------------------------
putcLCD                BSET LCD_CNTR,LCD_RS                 ; select the LCD Data register (DR)
                       JSR dataMov                          ; send data to DR
                       RTS

;--------------------------------------------------------------------------
; datamov  subroutine (8-bit mode)
;--------------------------------------------------------------------------
dataMov
                       BSET LCD_CNTR,LCD_E      ; E = 1
                       STAA LCD_DAT             ; Output full 8 bits
                       NOP
                       NOP
                       NOP
                       BCLR LCD_CNTR,LCD_E      ; E = 0 (latch)
                       LDY #1
                       JSR DELAY_50US
                       RTS

;--------------------------------------------------------------------------
; initAD  subroutine
;--------------------------------------------------------------------------               
initAD                 MOVB #$C0,ATDCTL2                    ;power up AD, select fast flag clear
                       JSR DELAY_50US                        ; wait for 50 us
                       MOVB #$00,ATDCTL3                    ;8 conversions in a sequence
                       MOVB #$85,ATDCTL4                    ;res=8, conv-clks=2, prescal=12
                       BSET ATDDIEN,$0C                     ;configure pins AN03,AN02 as digital inputs
                       RTS


;---------------------------------------------------------------------------
; Delay subroutine - provides ~50 microsecond delay
;---------------------------------------------------------------------------

DELAY_50US              PSHX                                ; (2 E-clk) Protect the X register

OUTER_LOOP              LDX   #300                          ; (2 E-clk) Initialize the inner loop counter

INNER_LOOP              NOP                                 ; (1 E-clk) No operation
                        DBNE  X,INNER_LOOP                  ; (3 E-clk) If the inner cntr not 0, loop again
                        DBNE  Y,OUTER_LOOP                  ; (3 E-clk) If the outer cntr not 0, loop again
                        PULX                                ; (3 E-clk) Restore the X register
                        RTS                                 ; (5 E-clk) Else return     

;---------------------------------------------------------------------------
; Initialize ATD
;---------------------------------------------------------------------------

INIT_ATD                MOVB #$C0, ATDCTL2
                        LDY #1
                        JSR DELAY_50US
                        MOVB #$00, ATDCTL3
                        MOVB #$85,ATDCTL4
                        BSET ATDDIEN,$0C
                        RTS
;---------------------------------------------------------------------------
; UPDATE LCD  (Same working version but WITHOUT LABELS)
;---------------------------------------------------------------------------
UPDT_LCD              
                        PSHB
                        PSHX

;---------------------------------------------------------
; 1. Clear LCD and print STATE on Line 1
;---------------------------------------------------------
                        LDAA #CLEAR_HOME
                        JSR  cmd2LCD
                        LDY  #40
                        JSR  DELAY_50US

                        ; Print robot state
                        LDAA CURRENT_STATE
                        CMPA #STATE_IDLE
                        BEQ  LCD_STATE_IDLE
                        CMPA #STATE_SEARCH_LINE
                        BEQ  LCD_STATE_SEARCH
                        CMPA #STATE_AT_INTERSECTION
                        BEQ  LCD_STATE_INT
                        CMPA #STATE_MOVING_BRANCH
                        BEQ  LCD_STATE_MOVING
                        CMPA #STATE_BACKTRACKING
                        BEQ  LCD_STATE_BACK
                        CMPA #STATE_RETRACING
                        BEQ  LCD_STATE_RETRACE
                        BRA  LCD_STATE_UNKNOWN

LCD_STATE_IDLE:         LDY STR_IDLE
                        BRA PRINT_STATE
LCD_STATE_SEARCH:       LDY STR_MOVING
                        BRA PRINT_STATE
LCD_STATE_INT:          LDY STR_INTERSECTION
                        BRA PRINT_STATE
LCD_STATE_MOVING:       LDY STR_MOVING
                        BRA PRINT_STATE
LCD_STATE_BACK:         LDY STR_BACKTRACK
                        BRA PRINT_STATE
LCD_STATE_RETRACE:      LDY STR_BACKTRACK
                        BRA PRINT_STATE
LCD_STATE_UNKNOWN:      LDY STR_UNKNOWN

PRINT_STATE:
                        JSR putsLCD


;---------------------------------------------------------
; 2. Build second line in BOT_LINE buffer (NO LABELS)
;---------------------------------------------------------

                        LDX #BOT_LINE

; Start writing values at BOT_LINE+0 now
FIRST_VAL  EQU 0

; PORT sensor
                        LDAA SENSOR_PORT
                        JSR  BIN2ASC
                        STAA BOT_LINE+FIRST_VAL
                        STAB BOT_LINE+FIRST_VAL+1
                        LDAA #'/'
                        STAA BOT_LINE+FIRST_VAL+2

; MID sensor
                        LDAA SENSOR_MID
                        JSR  BIN2ASC
                        STAA BOT_LINE+FIRST_VAL+3
                        STAB BOT_LINE+FIRST_VAL+4
                        LDAA #'/'
                        STAA BOT_LINE+FIRST_VAL+5

; LINE sensor
                        LDAA SENSOR_LINE
                        JSR  BIN2ASC
                        STAA BOT_LINE+FIRST_VAL+6
                        STAB BOT_LINE+FIRST_VAL+7
                        LDAA #'/'
                        STAA BOT_LINE+FIRST_VAL+8

; BOW sensor
                        LDAA SENSOR_BOW
                        JSR  BIN2ASC
                        STAA BOT_LINE+FIRST_VAL+9
                        STAB BOT_LINE+FIRST_VAL+10
                        LDAA #'/'
                        STAA BOT_LINE+FIRST_VAL+11

; STBD sensor
                        LDAA SENSOR_STBD
                        JSR  BIN2ASC
                        STAA BOT_LINE+FIRST_VAL+12
                        STAB BOT_LINE+FIRST_VAL+13

; Null terminator
                        LDAA #0
                        STAA BOT_LINE+FIRST_VAL+14


;---------------------------------------------------------
; 3. Display BOT_LINE on LCD line 2
;---------------------------------------------------------
                        LDAA #LCD_SEC_LINE
                        JSR  LCD_POS_CRSR

                        LDX #BOT_LINE
                        JSR putsLCD


;---------------------------------------------------------
; Cleanup
;---------------------------------------------------------
                        PULX
                        PULB
                        RTS




;--------------------------------------------------------------------------
; LCD_POS_CRSR
;--------------------------------------------------------------------------
LCD_POS_CRSR           ORAA #%10000000
                       JSR cmd2LCD
                       RTS
                       
;--------------------------------------------------------------------------
; BIN2ASC
;--------------------------------------------------------------------------
                    
HEX_TABLE  FCC '0123456789ABCDE'
BIN2ASC    PSHA
           TAB
           ANDB #%00001111
           CLRA
           ADDD #HEX_TABLE
           XGDX
           LDAA 0,X
           PULB
           PSHA
           RORB
           RORB
           RORB
           RORB
           ANDB #%00001111
           CLRA
           ADDD #HEX_TABLE
           XGDX
           LDAA 0,X
           PULB
           RTS
           

                        

;--------------------------------------------------------------------------
; Interrupt Vector Table
;--------------------------------------------------------------------------

                        ORG   $FFFE
                        DC.W  Entry                         ; Reset Vector





