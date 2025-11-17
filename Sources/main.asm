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
TURN_LEFT             EQU 3               ; -1 mod 4
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
SENSOR_LINE           FCB $01                  ; Line Sensor
SENSOR_BOW            FCB $23                  ; Front ("bow") sensor
SENSOR_PORT           FCB $45                  ; Left sensor
SENSOR_MID            FCB $67                  ; Center sensor
SENSOR_STBD           FCB $89                  ; Right sensor
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
TEN_THOUSAND          DS.B 1                   ; 10,000 digit
THOUSANDS             DS.B 1                   ; 1,000 digit
HUNDREDS              DS.B 1                   ; 100 digit
TENS                  DS.B 1                   ; 10 digit
UNITS                 DS.B 1                   ; 1 digit
NO_BLANK              DS.B 1                   ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE             RMB  10                  ; Extra space for decimal point and string terminator

; Robot Guidance Values
MAZE_TABLE:           DS.B 8                   ; holds chosen direction per intersection (1..4), 0=unknown
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
IDLE_ST             BRCLR PORTAD0,$04,NO_IDLE_ST
                    JSR   INIT_FWD
                    MOVB  #STATE_SEARCH_LINE,CURRENT_STATE

NO_IDLE_ST          NOP

IDLE_ST_EXIT        RTS
              
; The searching for a line state 
SEARCH_LINE_ST:     JSR   SENSOR_READ             
                                
                    LDAA  SENSOR_BOW               
                    CMPA  #THRESH_DARK             
                    BLO   STILL_SEARCHING          
                
FOLLOW_LINE         LDAA  SENSOR_LINE              
                    CMPA  #$68                    
                    BEQ   ON_CENTER                
                    BLO   STEER_RIGHT              
                
STEER_LEFT          JSR   INIT_LEFT_TRN
                    BRA   SEARCH_EXIT
                
STEER_RIGHT         JSR   INIT_RIGHT_TRN
                    BRA   SEARCH_EXIT
                
ON_CENTER           JSR   INIT_FWD
                    JSR   SENSOR_READ
                    
                    LDAA  SENSOR_PORT
                    LDAB  SENSOR_STBD
                    
                    CMPA  #THRESH_DARK
                    BHS   FOUND_INTERSECTION   
                    
                    CMPB  #THRESH_DARK
                    BHS   FOUND_INTERSECTION
                
                    BRA   SEARCH_EXIT
               
STILL_SEARCHING     JSR   INIT_FWD 

FOUND_INTERSECTION  MOVB  #STATE_AT_INTERSECTION,CURRENT_STATE
                    JSR   INIT_ALL_STP
                    BRA   SEARCH_EXIT              
                
SEARCH_EXIT:        RTS
              
; The at intersection state
AT_INTERSECTION_ST:     JSR   SENSOR_READ

                        INC   CURRENT_INTERSECTION ; Increment intersection counter (we're at a new or revisited intersection
                        
                        LDX   #MAZE_TABLE
                        LDAB  CURRENT_INTERSECTION
                        ABX
                        LDAA  0,X
                        
                        CMPA  #0                       ; First visit?
                        BEQ   TRY_LEFT
                        
                        CMPA  #TURN_LEFT
                        BEQ   TRY_STRAIGHT             ; Already tried left, try straight
                        
                        CMPA  #TURN_STRAIGHT
                        BEQ   TRY_RIGHT                ; Already tried straight, try right                      

; Try turning left (highest priority)
TRY_LEFT:               LDAA  SENSOR_PORT              ; Check if left path exists
                        CMPA  #THRESH_DARK
                        BLO   TRY_STRAIGHT             ; No left path, try straight
                        
                        LDAA  #TURN_LEFT               ; Left exists! Take it
                        JSR   STORE_DECISION
                        JSR   UPDATE_HEADING
                        JSR   INIT_LEFT_TRN
                        
                        LDAA  #STATE_MOVING_BRANCH
                        STAA  CURRENT_STATE
                        RTS

; Try going straight (medium priority)
TRY_STRAIGHT:           LDAA  SENSOR_BOW               ; Check if straight path exists
                        CMPA  #THRESH_DARK
                        BLO   TRY_RIGHT                ; No straight path, try right
                        
                        LDAA  #TURN_STRAIGHT           ; Straight exists! Take it
                        JSR   STORE_DECISION
                        JSR   UPDATE_HEADING
                        JSR   INIT_FWD
                        
                        LDAA  #STATE_MOVING_BRANCH
                        STAA  CURRENT_STATE
                        RTS

; Try turning right (lowest priority)
TRY_RIGHT:              LDAA  SENSOR_STBD              ; Check if right path exists
                        CMPA  #THRESH_DARK
                        BLO   ALL_TRIED                ; No right path either!
                        
                        LDAA  #TURN_RIGHT              ; Right exists! Take it
                        JSR   STORE_DECISION
                        JSR   UPDATE_HEADING
                        JSR   INIT_RIGHT_TRN
                        
                        LDAA  #STATE_MOVING_BRANCH
                        STAA  CURRENT_STATE
                        RTS


AT_INTERSECTION_ST_EXIT RTS
              
; The moving branch state
MOVING_BRANCH_ST

MOVING_BRANCH_ST_EXIT                    RTS
              
; The bumper collide state
BUMPER_COLLIDE_ST

BUMPER_COLLIDE_ST_EXIT                   RTS
              
; The backtracking state
BACKTRACKING_ST

BACKTRACKING_ST_EXIT                    RTS
              
; The retracing state
RETRACING_ST

RETRACING_ST_EXIT                    RTS
 

;--------------------------------------------------------------------------
; Storing Decisions

; This stores the decision for each intersection found within the maze
STORE_DECISION: LDX   #MAZE_TABLE              ; Point to start of maze table
                LDAB  CURRENT_INTERSECTION     ; Get current intersection index
                ABX                            ; Add offset (X = X + B)
                STAA  0,X                      ; Store direction at MAZE_TABLE[index]
                
                LDAB  MAZE_COUNT               ; Increment intersection counter if this is a new intersection
                CMPB  CURRENT_INTERSECTION
                BHI   STORE_EXIT               ; Already been here
                
                INC   MAZE_COUNT               ; New intersection discovered!
                
STORE_EXIT:     RTS

UPDATE_HEADING 

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

;---------------------------------------------------------------------------
; Guider Sensor Read Subroutine
SENSOR_READ   JSR G_LEDS_ON     ; Enable the guider LEDs
              JSR READ_SENSORS  ; Read the 5 guider sensors
              JSR G_LEDS_OFF    ; Disable the guider LEDs
              RTS


;---------------------------------------------------------------------------
; Guider LED Control Subroutines

; Turn on guider LEDs (to measure reflected light via illumination)
G_LEDS_ON       BSET  PORTA,%00100000           ; Set bit 5 of PORTA to 1 (enable LEDs)
                RTS

; Turn off guider LEDs (to measure ambient light or conserve power)
G_LEDS_OFF      BCLR  PORTA,%00100000           ; Clear bit 5 of PORTA to 0 (disable LEDs)
                RTS

;---------------------------------------------------------------------------
; Sensor Reading Subroutine

; This routine reads all 5 guider sensors in sequence, storing their values
; into RAM variables SENSOR_LINE through SENSOR_STBD.
;
; Uses: A/D converter channel AN1. Sensors are selected via the guider board mux.
READ_SENSORS
    
RS_MAIN_LOOP     CLR   SENSOR_NUM               ; Initialize sensor index to 0
                 LDX   #SENSOR_LINE             ; Point X at the start of sensor RAM array
                 
RS_LOOP          LDAA  SENSOR_NUM               ; Load current sensor number
                 JSR   SELECT_SENSOR            ; Select corresponding physical sensor

                 LDY   #400                     ; Wait ~20ms to stabilize sensor reading
                 JSR   DELAY_50US               ; (400 * 50�s = 20ms)

                 LDAA  #%10000001               ; Configure ATD: single scan, channel AN1
                 STAA  ATDCTL5                  ; Start analog-to-digital conversion

                 BRCLR ATDSTAT0,$80,*           ; Loop until conversion complete (SCF=1)

                 LDAA  ATDDR0L                  ; Read 8-bit ADC result from ATDDR0L
                 STAA  0,X                      ; Store result at current sensor location

                 CPX   #SENSOR_STBD             ; Check if last sensor (index 4)
                 BEQ   RS_EXIT                  ; If yes, exit
                 
                 INC   SENSOR_NUM               ; Increment sensor index
                 INX                            ; Increment pointer to next RAM location
                 BRA   RS_LOOP                  ; Repeat for next sensor

RS_EXIT          RTS                            ; Return from subroutine

;---------------------------------------------------------------------------
; Sensor Selector Subroutine

; Selects one of the guider sensors using PORTA bits connected to a 74HC4051 mux.
; Sensor number (0-4) is passed in ACCA.
; Only bits 2�4 of PORTA are modified; bits 0,1,5,6,7 are preserved.
SELECT_SENSOR    PSHA                           ; Save ACCA (sensor number) temporarily

                 LDAA  PORTA                    ; Read PORTA
                 ANDA  #%11100011               ; Clear bits 2-4 (sensor select bits)
                 STAA  TEMP                     ; Store modified value in TEMP
                 
                 PULA                           ; Restore sensor number to ACCA
                 ASLA                           ; Shift sensor number left twice (bits 0-2 to 2-4)
                 ASLA
                 ANDA  #%00011100               ; Mask off irrelevant bits
                 
                 ORAA  TEMP                     ; Merge with stored PORTA bits (update sensor bits)
                 STAA  PORTA                    ; Write back to PORTA (select sensor)
                 RTS




; Delay subroutine - provides ~50 microsecond delay
DELAY_50US      PSHX                            ; (2 E-clk) Protect the X register

OUTER_LOOP      LDX   #300                      ; (2 E-clk) Initialize the inner loop counter

INNER_LOOP      NOP                             ; (1 E-clk) No operation
                DBNE  X,INNER_LOOP              ; (3 E-clk) If the inner cntr not 0, loop again
                DBNE  Y,OUTER_LOOP              ; (3 E-clk) If the outer cntr not 0, loop again
                PULX                            ; (3 E-clk) Restore the X register
                RTS                             ; (5 E-clk) Else return     