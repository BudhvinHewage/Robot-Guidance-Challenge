;-------------------------------------------------------------------
;*              Lab 6: Robot Guidance Challenge (9S32C)               *
;-------------------------------------------------------------------

; export symbols
                        XDEF Entry, _Startup  ; export 'Entry' symbol
                        ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
                        INCLUDE 'derivative.inc'

; The sensor value can show when exactly the robot will line up with the tape, adjust the value and play with it identify when it stop on the line, or as close as possible

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
STATE_SEARCH_LINE       EQU 1                               ; Following the line normally; looking for intersection
STATE_AT_INTERSECTION   EQU 2                               ; Intersection logic: detect options, decide turn
STATE_MOVING_BRANCH     EQU 3                               ; Move down the selected path (until next intersection or dead end)
STATE_BUMPER_COLLIDE    EQU 4                               ; Dead end detected via front bumper (turnaround logic)
STATE_BACKTRACKING      EQU 5                               ; Retrace back to last intersection and correct stored decision
STATE_RETRACING         EQU 6                               ; Full retrace back to the start (following stored solutions only)

; Thresholds for the sensors  
THRESH_CENTER_PORT      EQU $60                             ; If below this, robot is veering to the left
THRESH_CENTER_STBD      EQU $70                             ; If above this, robot is veering to the right
THRESH_BOW              EQU $B0                             ; Front sensor
THRESH_MID              EQU $B0                             ; Middle sensor
THRESH_PORT             EQU $AC                             ; Left sensor
THRESH_STBD             EQU $AC                             ; Right sensor 

; Motor Timing Intervals
FWD_INT                 EQU 10                              ; Forward movement interval
REV_INT                 EQU 10                              ; Reverse movement interval
T_RIGHT_INT             EQU 5                               ; Right turn interval
T_LEFT_INT              EQU 5                               ; Left turn interval

; Path status values
PATH_UNKNOWN            EQU 0
PATH_FAILED             EQU 1
PATH_SUCCESS            EQU 2

; Delay Intervals
THREE_SECOND_DELAY      EQU 69                              ; Approx 3 seconds at 23Hz
TWO_SECOND_DELAY        EQU 46                              ; Approx 2 seconds at 23Hz
SECOND_DELAY            EQU 25                              ; Approx 1 second at 23Hz
HALF_SECOND_DELAY       EQU 12                              ; Approx 0.5 seconds at 23Hz

; Temporal Constants

;-------------------------------------------------------------------
; Variable and Data Section
;-------------------------------------------------------------------      

                        ORG $3800                           ; Where our TOF counter register lives
; Sensors
SENSOR_LINE             FCB $00                             ; Line Sensor
SENSOR_BOW              FCB $00                             ; Front ("bow") sensor
SENSOR_PORT             FCB $00                             ; Left sensor
SENSOR_MID              FCB $00                             ; Center sensor
SENSOR_STBD             FCB $00                             ; Right sensor

SENSOR_NUM              RMB 1                               ; The currently selected sensor

; Tof Counter
TOF_COUNTER             DC.B 0                              ; The timer, incremented at 23Hz
CC                      DC.B 8
CURRENT_STATE           DC.B 0                              ; Current state register


; Storage for Maze Mapping
MAZE_TABLE:             DS.B 24                             ; Maze table for up to 8 intersections (3 bytes each) where byte 0 = entry dir, byte 1 = first exit tried, byte 2 = second exit tried
SCRATCH_DIR             DS.B 1                              ; Temporary storage for calculated directions
TEMP                    DS.B 1                              ; Temporary storage for second exit
MAZE_COUNT:             DS.B 1                              ; Number of intersections discovered
CURRENT_INTERSECTION:   DS.B 1                              ; Index of current intersection (1..MAX)
HEADING:                DS.B 1                              ; 0..3 (N,E,S,W) or use DIR_*
ENTRY_DIRECTION:        DS.B 1                              ; Direction we entered current intersection from
STATE:                  DS.B 1
STACK_PTR:              DS.B 1                              ; Simple stack top for backtracking (store inter. indices)

; Robot Motion Time
T_FWD                   DS.B  1                             ; FWD time
T_REV                   DS.B  1                             ; REV time
T_RIGHT_TRN             DS.B  1                             ; RIGHT_TRN time - Was T_FWD_TRN
T_LEFT_TRN              DS.B  1                             ; LEFT_TRN time - Was T_REV_TRN

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
                        
                        SWI

;-------------------------------------------------------------------
; Main Program Section
;-------------------------------------------------------------------

MAIN:                   BRSET PORTAD0,$04,NO_START          ; Check front bumper - if NOT pressed, skip to NO_START
                        BRA   START_NAVIGATION              ; Bumper IS pressed, start navigation

NO_START:               JSR   INIT_ALL_STP                  ; Keep motors stopped while waiting
                        BRA   MAIN                          ; Loop back and keep checking bumper

START_NAVIGATION:       JSR   LINE_NAVIGATION               ; Execute line following                    
                        BRA   START_NAVIGATION              ; Keep navigating

                        
;-------------------------------------------------------------------------- 
; Movement along an indicated line until intersection or bumper is detected
;--------------------------------------------------------------------------

LINE_NAVIGATION:        JSR   SENSOR_READ                   ; Refresh sensor values

                        JSR   LEFT_WAIT_FOR_STRIP           ; Wait for strip fade/rise
                        LDY   #20000                        ; 
                        JSR   DELAY_50US                    ;
                        
                        JSR   RIGHT_WAIT_FOR_STRIP          ; Wait for strip fade/rise    
                        LDY   #20000                        ; 
                        JSR   DELAY_50US                    ; 
                        
                        RTS
                                           

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
; Wait for Strip Subroutine - Left Turn
;---------------------------------------------------------------------------

LEFT_WAIT_FOR_STRIP:    
                
LEFT_WAIT_STRIP_FADE:   JSR  INIT_LEFT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #50
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BGE  LEFT_WAIT_STRIP_FADE                ; Still on old strip, wait

LEFT_WAIT_STRIP_RISE:   JSR  INIT_LEFT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #50
                        JSR  DELAY_50US

                        JSR SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BLO  LEFT_WAIT_STRIP_RISE                ; New strip not detected yet, keep waiting

LEFT_WAIT_STRIP_DONE:   RTS

;---------------------------------------------------------------------------
; Wait for Strip Subroutine - Right Turn
;---------------------------------------------------------------------------

RIGHT_WAIT_FOR_STRIP:   
    
RIGHT_WAIT_STRIP_FADE:  JSR  INIT_RIGHT_TRN
                        LDY  #500
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BGE  RIGHT_WAIT_STRIP_FADE                ; Still on old strip, wait

RIGHT_WAIT_STRIP_RISE:  JSR  INIT_RIGHT_TRN
                        LDY  #500
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BLO  RIGHT_WAIT_STRIP_RISE                ; New strip not detected yet, keep waitingg
                                        
RIGHT_WAIT_STRIP_DONE:  RTS

;---------------------------------------------------------------------------
; Guider Sensor Read Subroutine
;---------------------------------------------------------------------------

SENSOR_READ             JSR G_LEDS_ON     ; Enable the guider LEDs
                        JSR READ_SENSORS  ; Read the 5 guider sensors
                        JSR G_LEDS_OFF    ; Disable the guider LEDs
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
                        BEQ   RS_EXIT                       ; If yes, exit
                        
                        INC   SENSOR_NUM                    ; Increment sensor index
                        INX                                 ; Increment pointer to next RAM location
                        BRA   RS_LOOP                       ; Repeat for next sensor
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

;--------------------------------------------------------------------------
; Interrupt Vector Table
;--------------------------------------------------------------------------

                        ORG   $FFFE
                        DC.W  Entry                         ; Reset Vector
