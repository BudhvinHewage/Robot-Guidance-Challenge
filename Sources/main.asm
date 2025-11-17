;*******************************************************************
;*              Lab 6: Robot Guidance Challenge (9S32C)               *
;*******************************************************************

; export symbols
                      XDEF Entry, _Startup  ; export 'Entry' symbol
                      ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
                      INCLUDE 'derivative.inc'

; Equates Section
;*******************************************************************

; LCD Control Addresses
LCD_DATA              EQU PORTB           ; LCD dataport, bits-PB7,...,PB0
LCD_CONTROL           EQU PTJ             ; LCD control port,bits-PJ7(E),PJ6(RS)
LCD_E_SIGNAL          EQU $80             ; LCD E-signal pin
LCD_RS_SIGNAL         EQU $40             ; LCD RS-signal pin

; LCD Commands
CLEAR_HOME            EQU $01             ; Clears the LCD screen and moves the cursor to home position (top-left).
INTERFACE             EQU $38             ; Tells the LCD to use 8-bit data mode, 2-line display, and 5×8 dot characters.
CURSOR_OFF            EQU $0C             ; Turns on the display, cursor off, without blinking
SHIFT_OFF             EQU $06             ; Makes the cursor move to the right automatically after each character. No display shift.
LCD_SEC_LINE          EQU 64              ; Address in LCD's memory where line 2 starts. Used to position the cursor on the second line.


; Other useful LCD codes
NULL                  EQU 00              ; The string ’null terminator’
CR                    EQU $0D             ; The ’Carriage Return’ character
SPACE                 EQU ' '             ; The ’space’character

; Directions and Headings
DIR_UNKNOWN           EQU 0
DIR_NORTH             EQU 1
DIR_EAST              EQU 2
DIR_SOUTH             EQU 3
DIR_WEST              EQU 4

; Turn deltas (heading arithmetic mod 4)
TURN_LEFT             EQU 3               ; -1 mod4
TURN_STRAIGHT         EQU 0
TURN_RIGHT            EQU 1
TURN_UTURN            EQU 2

; The possible states of the robot
STATE_IDLE            EQU 0               ; Waiting for the start signal (bumper)
STATE_SEARCH_LINE     EQU 1               ; Following the line normally; looking for intersection
STATE_AT_INTERSECTION EQU 2               ; Intersection logic: detect options, decide turn
STATE_MOVING_BRANCH   EQU 3               ; Move down the selected path (until next intersection or dead end)
STATE_BUMPER_COLLIDE  EQU 4               ; Dead end detected via front bumper (turnaround logic)
STATE_BACKTRACKING    EQU 5               ; Retrace back to last intersection and correct stored decision
STATE_RETRACING       EQU 6               ; Full retrace back to the start (following stored solutions only)

; Thresholds for the sensors (tune these)
THRESH_LIGHT          EQU $40                  ; Example 64 decimal (set by calibration)
THRESH_DARK           EQU $80                  ; Example 128 decimal

; Motor Timing Intervals
FWD_INT               EQU 10              ; Forward movement interval
REV_INT               EQU 10              ; Reverse movement interval
T_RIGHT_INT           EQU 5               ; Right turn interval
T_LEFT_INT            EQU 5               ; Left turn interval

; Variable and Data Section
;*******************************************************************      

                      ORG $3850                ; Where our TOF counter register lives
; Sensors
SENSOR_LINE           FCB $01                  ; Storageforguider sensorreadings
SENSOR_BOW            FCB $23                  ; Initializedto testvalues
SENSOR_PORT           FCB $45
SENSOR_MID            FCB $67
SENSOR_STBD           FCB $89
SENSOR_NUM            RMB 1                    ; The currently selected sensor

; LCD Display
TOP_LINE              RMB 20                   ; Top line of display
                      FCB NULL                 ; Terminated by null
BOT_LINE              RMB 20                   ; Bottom lineof display
                      FCB NULL                 ; Terminated by null
CLEAR_LINE            FCC ' '
                      FCB NULL                 ; Terminated by null
TEMP                  RMB 1                    ; Temporary location


; Tof Counter
TOF_COUNTER           DC.B 0                   ; The timer, incremented at 23Hz
CURRENT_STATE         DC.B 3                   ; Current state register

; Conversion Units
TEN_THOUS             DS.B 1                   ; 10,000 digit
THOUSANDS             DS.B 1                   ; 1,000 digit
HUNDREDS              DS.B 1                   ; 100 digit
TENS                  DS.B 1                   ; 10 digit
UNITS                 DS.B 1                   ; 1 digit
NO_BLANK              DS.B 1                   ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE             RMB  10                  ; Extra space for decimal point and string terminator

; Robot Guidance Values
MAZE_TABLE:           DS.B 6                   ; holds chosen direction per intersection (1..4), 0=unknown
MAZE_COUNT:           DS.B 1                   ; number of intersections discovered
CURRENT_INTERSECTION: DS.B 1                   ; index of current intersection (1..MAX)
HEADING:              DS.B 1                   ; 0..3 (N,E,S,W) or use DIR_*
STATE:                DS.B 1
STACK_PTR:            DS.B 1                   ; simple stack top for backtracking (store inter. indices)

; Robot Motion Time
T_FWD                 DS.B  1                  ; FWD time
T_REV                 DS.B  1                  ; REV time
T_RIGHT_TRN           DS.B  1                  ; RIGHT_TRN time - Was T_FWD_TRN
T_LEFT_TRN            DS.B  1                  ; LEFT_TRN time - Was T_REV_TRN

; Initialization
;*******************************************************************    

                ORG $4000
                  
Entry:
 _Startup:
                LDS #$4000         ; Initialize the stack pointer
                CLI                ; Enable interrupt
                
                JSR INIT_PORTS
                
                BRA MAIN

; Main Program Section
;*******************************************************************

MAIN:
                LDAA  CURRENT_STATE
                JSR   DISPATCHER
                BRA   MAIN


; Subroutine Section
;*******************************************************************

;--------------------------------------------------------------------------
; State Machine Dispatcher

DISPATCHER          CMPA  #STATE_IDLE                 ; If it’s the IDLE state 
                    BNE   NOT_IDLE                    ;
                    JSR   IDLE_ST                     ; then call IDLE_ST routine
                    BRA   DISP_EXIT                   ; and exit
                                                 
NOT_IDLE            CMPA  #STATE_SEARCH_LINE          ; Else if it's the SEARCH_LINE state
                    BNE   NOT_SEARCH_LINE             
                    JSR   SEARCH_LINE_ST              ; then call SEARCH_LINE_ST routine
                    BRA   DISP_EXIT                   ; and exit
              
NOT_SEARCH_LINE     CMPA  #STATE_AT_INTERSECTION      ; Else if it's the AT_INTERSECTION state
                    BNE   NOT_AT_INTERSECTION
                    JSR   AT_INTERSECTION_ST          ; then call AT_INTERSECTION_ST routine
                    BRA   DISP_EXIT                   ; and exit

NOT_AT_INTERSECTION CMPA  #STATE_MOVING_BRANCH        ; Else if it's the MOVING_BRANCH state
                    BNE   NOT_MOVING_BRANCH
                    JSR   MOVING_BRANCH_ST            ; then call MOVING_BRANCH_ST routine
                    BRA   DISP_EXIT                   ; and exit

NOT_MOVING_BRANCH   CMPA  #STATE_BUMPER_COLLIDE       ; Else if it's the BUMPER_COLLIDE state
                    BNE   NOT_BUMPER_COLLIDE
                    JSR   BUMPER_COLLIDE_ST           ; then call BUMPER_COLLIDE_ST routine
                    BRA   DISP_EXIT                   ; and exit

NOT_BUMPER_COLLIDE  CMPA  #STATE_BACKTRACKING         ; Else if it's the BACKTRACKING state
                    BNE   NOT_BACKTRACKING
                    JSR   BACKTRACKING_ST             ; then call BACKTRACKING_ST routine
                    BRA   DISP_EXIT                   ; and exit

NOT_BACKTRACKING    CMPA  #STATE_RETRACING            ; Else if it's the RETRACING state
                    BNE   NOT_RETRACING
                    JSR   RETRACING_ST                ; then call RETRACING_ST routine
                    BRA   DISP_EXIT                   ; and exit

NOT_RETRACING       SWI                               ; Else the STATE is not defined, so stop

DISP_EXIT           RTS                               ; Exit from the state dispatcher 

;--------------------------------------------------------------------------
; States

; The idle state
IDLE_ST
                    RTS
              
; The search line state
SEARCH_LINE_ST
                    RTS
              
; The at intersection state
AT_INTERSECTION_ST
                    RTS
              
; The moving branch state
MOVING_BRANCH_ST
                    RTS
              
; The bumper collide state
BUMPER_COLLIDE_ST
                    RTS
              
; The backtracking state
BACKTRACKING_ST
                    RTS
              
; The retracing state
RETRACING_ST
                    RTS
 
;--------------------------------------------------------------------------
; Initialize Ports
 
INIT_PORTS      BCLR DDRAD,$FF     ; Make PORTAD an input (DDRAD@$0272)
                BSET DDRA, $FF     ; Make PORTA an output (DDRA@$0002)
                BSET DDRB, $FF     ; Make PORTB an output (DDRB@$0003)
                BSET DDRJ, $C0     ; Make pins 7,6 of PTJ outputs (DDRJ @$026A)
                
                BSET DDRA, $03     ; STAR_DIR, PORT_DIR  00000011
                BSET DDRT, $30     ; STAR_SPEED, PORT_SPEED 00110000
                
                RTS

;--------------------------------------------------------------------------
; Motor Control Subroutines

; Turn motors on to move forward
INIT_FWD        BCLR  PORTA,%00000011           ; Set FWD direction for both motors
                BSET  PTT,%00110000             ; Turn on the drive motors
                ADDA  #FWD_INT
                STAA  T_FWD
                RTS
              
; Turn motors on to move backward
INIT_REV        BSET  PORTA,%00000011           ; Set REV direction for both motors
                BSET  PTT,%00110000             ; Turn on the drive motors
;               LDAA  TOF_COUNTER               ; Mark the fwd time Tfwd
                ADDA  #REV_INT
                STAA  T_REV
                RTS
              
; Turn motors off
INIT_ALL_STP    BCLR  PTT,%00110000             ; Turn off the drive motors
                RTS
              
; Turn motors on to rotate right
INIT_RIGHT_TRN  BSET  PORTA,%00000010           ; Set REV dir. for STARBOARD (right) motor
                LDAA  TOF_COUNTER               ; Mark the fwd_turn time Tfwdturn
                ADDA  #T_RIGHT_INT
                STAA  T_RIGHT_TRN
                RTS
              
; Turn motors on to rotate left
INIT_LEFT_TRN   BCLR  PORTA,%00000010           ; Set FWD dir. for STARBOARD (right) motor
                LDAA  TOF_COUNTER               ; Mark the fwd time Tfwd
                ADDA  #T_LEFT_INT 
                STAA  T_LEFT_TRN
                RTS


                
;--------------------------------------------------------------------------
; TOF Control Subroutines

; Initialize and enable Timer Overflow interrupt with prescaler
ENABLE_TOF    LDAA  #%10000000
              STAA  TSCR1                     ; Enable TCNT
              STAA  TFLG2                     ; Clear TOF
              LDAA  #%10000100                ; Enable TOI and select prescale factor equal to 16
              STAA  TSCR2
              RTS

; Timer Overflow Interrupt Service Routine - increments overflow counter
TOF_ISR       INC   TOF_COUNTER
              LDAA  #%10000000                ; Clear
              STAA  TFLG2                     ; TOF
              RTI




     