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
STATE_BUMPER_COLLIDE    EQU 3                               ; Dead end detected via front bumper (turnaround logic)
STATE_BACKTRACKING      EQU 4                               ; Retrace back to last intersection and correct stored decision

; Thresholds for the sensors                                                                                   
THRESH_CENTER_RIGHT     EQU $A5                             ; If below this, robot is veering to the RIGHT
THRESH_CENTER_LEFT      EQU $C0                             ; If above this, robot is veering to the LEFT
THRESH_BOW              EQU $CF                             ; Front sensor
THRESH_MID              EQU $B5                             ; Middle sensor
THRESH_PORT             EQU $B5                             ; Left sensor
THRESH_STBD             EQU $CE                             ; Right sensor 

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

;-------------------------------------------------------------------
; Variable and Data Section
;-------------------------------------------------------------------      

                        ORG $3800                           ; Where our TOF counter register lives

EXIT_LIST:              DS.B 2                              ; List of available exits at current intersections
TEMP_STORAGE            DS.B 1
TEMP_EXIT_ACCOUNT       DS.B 1

; Sensors - This begins from $3805
SENSOR_LINE             FCB $00                             ; Line Sensor
SENSOR_BOW              FCB $00                             ; Front ("bow") sensor
SENSOR_PORT             FCB $00                             ; Left sensor
SENSOR_MID              FCB $00                             ; Center sensor
SENSOR_STBD             FCB $00                             ; Right sensor
SENSOR_NUM              RMB 1                               ; The currently selected sensor
NULL                    EQU 00

TOP_LINE                RMB 20
                        FCB NULL

BOT_LINE                RMB 40
                        FCB NULL

; Tof Counter
TOF_COUNTER             DC.B 0                              ; The timer, incremented at 23Hz
CC                      DC.B 8
CURRENT_STATE           DC.B 0                              ; Current state register


; Storage for Maze Mapping
MAZE_TABLE:             DS.B 24                             ; Maze table for up to 8 intersections (3 bytes each) where byte 0 = entry dir, byte 1 = first exit tried, byte 2 = second exit tried
SCRATCH_DIR             DS.B 1                              ; Temporary storage for calculated directions
TEMP                    DS.B 1                              ; Temporary storage for second exit
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


ALIVE_COUNTER           DS.B 1                              ; Counter to toggle ALIVE display
STR_IDLE                DC.B "IDLE        ",0
STR_MOVING              DC.B "MOVING      ",0 
STR_INTERSECTION        DC.B "INTERSECTION",0
STR_BACKTRACK           DC.B "BACKTRACK   ",0 
STR_UNKNOWN             DC.B "UNKNOWN     ",0   

SENSOR_LABELS:          DC.B "PCPCSBS:" ,0 
                        
;LCD CURSOR POSITIONS FOR DISPLAY
DP_FRONT_SENSOR         EQU TOP_LINE+3
DP_PORT_SENSOR          EQU BOT_LINE+0
DP_MID_SENSOR           EQU BOT_LINE+3
DP_STBD_SENSOR          EQU BOT_LINE+6
DP_LINE_SENSOR          EQU BOT_LINE+9

;-------------------------------------------------------------------
; Initialization
;-------------------------------------------------------------------   

                        ORG $4000
                  
Entry:
 _Startup:
                        LDS #$4000                          ; Initialize the stack pointer
                        CLI                                 ; Enable interrupt
                        
                        CLR   CURRENT_STATE
                        CLR   CURRENT_INTERSECTION
                        CLR   ENTRY_DIRECTION                        

                        LDX   #MAZE_TABLE
                        LDAA  #24
CLEAR_MAZE:             CLR   0,X
                        INX
                        DECA
                        BNE   CLEAR_MAZE                        

                        JSR INIT_PORTS
                        JSR INIT_ATD        
                        BRA MAIN

;-------------------------------------------------------------------
; Main Program Section
;-------------------------------------------------------------------

MAIN:                   LDAA  CURRENT_STATE
                        JSR   DISPATCHER
                        BRA   MAIN

;-------------------------------------------------------------------
; State Machine Dispatcher
;-------------------------------------------------------------------

DISPATCHER              CMPA  #STATE_IDLE                   ; If it's the IDLE state 
                        BNE   NOT_IDLE                      ;
                        JSR   IDLE_ST                       ; then call IDLE_ST routine
                        BRA   DISP_EXIT                     ; and exit
                                                    
NOT_IDLE                CMPA  #STATE_SEARCH                 ; Else if it's the SEARCH_LINE state
                        BNE   NOT_SEARCH             
                        JSR   SEARCH_ST                     ; then call SEARCH_LINE_ST routine
                        BRA   DISP_EXIT                     ; and exit
                
NOT_SEARCH              CMPA  #STATE_AT_INTERSECTION        ; Else if it's the AT_INTERSECTION state
                        BNE   NOT_AT_INTERSECTION
                        JSR   AT_INTERSECTION_ST            ; then call AT_INTERSECTION_ST routine
                        BRA   DISP_EXIT                     ; and exit

NOT_AT_INTERSECTION     CMPA  #STATE_BUMPER_COLLIDE          ; Else if it's the MOVING_BRANCH state
                        BNE   NOT_BUMPER_COLLIDE
                        JSR   BUMPER_COLLIDE_ST              ; then call MOVING_BRANCH_ST routine
                        BRA   DISP_EXIT                     ; and exit

NOT_BUMPER_COLLIDE      CMPA  #STATE_BACKTRACKING           ; Else if it's the BACKTRACKING state
                        BNE   DISP_EXIT
                        JSR   BACKTRACKING_ST               ; then call BACKTRACKING_ST routine
                        BRA   DISP_EXIT                     ; and exit

DISP_EXIT               RTS                                 ; Exit from the state dispatcher 

;--------------------------------------------------------------------------
; The Idle State - Waiting for Start Signal
;--------------------------------------------------------------------------

IDLE_ST                 BRSET  PORTAD0,$04,NO_CHANGE        ; The function BRSET tests if bit 2 (front bumper) of PORTAD0 is set and if it is not, branches to NO_IDLE_ST
                        MOVB   #STATE_SEARCH,CURRENT_STATE
                        
                        JSR    INIT_FWD
                        LDY    #10000
                        JSR    DELAY_50US
                        
                        JSR    INIT_ALL_STP
                        
                        BRA    IDLE_ST_EXIT
                        
NO_CHANGE               NOP                                 ; Do nothing while idle

IDLE_ST_EXIT            RTS

;-------------------------------------------------------------------------
; The Search State - Navigate until the first intersection is reached
;-------------------------------------------------------------------------

SEARCH_ST:              BRSET  PORTAD0,$04,NO_FRONT_BUMPER
                        LDAA   #STATE_BUMPER_COLLIDE
                        STAA   CURRENT_STATE

NO_FRONT_BUMPER         JSR   SENSOR_READ                   ; Refresh sensor values  

                        LDAA  SENSOR_MID                    ; Check if MID sensor has the main line
                        CMPA  #THRESH_MID
                        BLO   STILL_SEARCHING               ; MID doesn't see line = not at intersection

                        LDAA  SENSOR_PORT                   ; Check PORT sensor for an intersection
                        CMPA  #THRESH_PORT
                        BHS   FOUND_INTERSECTION            ; PORT high + MID high = real intersection
                        
                        BRA   STILL_SEARCHING

FOUND_INTERSECTION      LDAA  #STATE_AT_INTERSECTION        ; Update state to AT_INTERSECTION
                        STAA  CURRENT_STATE                 
                        BRA   SEARCH_ST_EXIT                   

STILL_SEARCHING         JSR   LINE_NAVIGATION               ; Move forward to find line  
                        BRA   SEARCH_ST_EXIT          
                
SEARCH_ST_EXIT:         RTS

;--------------------------------------------------------------------------   
; The At Intersection State - Decide which way to go
;---------------------------------------------------------------------------

AT_INTERSECTION_ST:     JSR   INIT_ALL_STP
                        LDY   #5000
                        JSR   DELAY_50US
                        
                        JSR   SENSOR_READ                   ; Refresh sensor values

                        LDAA  SENSOR_PORT                    ; Check BOW sensor for obstacle
                        CMPA  #THRESH_PORT

                        BHS   LEFT_TURN_DETECTED

LEFT_TURN_DETECTED:     JSR   INIT_ALL_STP
                        LDY   #1000
                        JSR   DELAY_50US

                        JSR   LEFT_WAIT_FOR_STRIP
                        LDAA  #STATE_SEARCH                 ; Update state to MOVING_BRANCH
                        STAA  CURRENT_STATE

                        BRA   AT_INTERSECTION_EXIT
                        
AT_INTERSECTION_EXIT:   RTS

;-------------------------------------------------------------------------
; The Bumper Collide State - Handle dead end via U-turn and indicate that this path failed and check it off on the maze table
;-------------------------------------------------------------------------

BUMPER_COLLIDE_ST:      JSR   INIT_ALL_STP                  ; Stop motors
                        LDY   #1000                       ; Small delay to settle
                        JSR   DELAY_50US

                        JSR   EXECUTE_U_TURN                ; Execute U-turn

                        LDAA  #STATE_BACKTRACKING           ; Update state to backtracking
                        STAA  CURRENT_STATE     

BUMPER_COLLIDE_EXIT:    RTS
                        
; -------------------------------------------------------------------------
; BACKTRACKING_ST - Follow line back to previous intersection
; -------------------------------------------------------------------------

BACKTRACKING_ST:        JSR   SENSOR_READ
    
                        LDAA  SENSOR_PORT                   ; Check PORT sensor for line to check if we're back at the previous intersection
                        CMPA  #THRESH_PORT                  ; Compare with dark threshold
                        BHS   BACK_AT_INTERSECTION          ; If above or equal, we're back at intersection
                                                
                        JSR   LINE_NAVIGATION               ; Move forward along line since no branching line has been detected yet
                        BRA   BACKTRACKING_EXIT             ; Exit to backtracking state

BACK_AT_INTERSECTION:   JSR   INIT_ALL_STP                  ; Arrived at the previous intersection
                        LDY   #1000                       ; Small delay to settle
                        JSR   DELAY_50US

                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE                      

BACKTRACKING_EXIT:      RTS

;------------------------------------------------------------------------
; Execute U-Turn - Perform a 180 degree turn
;------------------------------------------------------------------------

EXECUTE_U_TURN:         
                       
U_WAIT_STRIP_FADE:      JSR  INIT_LEFT_TRN
                        LDY  #800
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BHS  READJUST

READJUST                JSR  INIT_LEFT_TRN
                        LDY  #6000
                        JSR  DELAY_50US
                        
                        JSR  INIT_ALL_STP
                        LDY  #10000
                        JSR  DELAY_50US
                        
                        JSR  INIT_FWD
                        LDY  #5000
                        JSR  DELAY_50US                        
                        JSR  INIT_ALL_STP
                        LDY  #5000
                        JSR  DELAY_50US

U_WAIT_STRIP_RISE:      JSR  INIT_LEFT_TRN
                        LDY  #800
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BLO  U_WAIT_STRIP_RISE                ; New strip not detected yet, keep waitingg
                                        
U_WAIT_STRIP_DONE:      JSR  INIT_LEFT_TRN
                        LDY  #2000
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10000
                        JSR  DELAY_50US
                        
                        RTS
 
;-------------------------------------------------------------------------- 
; Movement along an indicated line until intersection or bumper is detected
;--------------------------------------------------------------------------

LINE_NAVIGATION:        JSR   SENSOR_READ                   ; Refresh sensor values

                        ; Check if MID sensor has the main line
                        LDAA  SENSOR_MID
                        CMPA  #THRESH_MID
                        BLO   NO_INTERSECTION               ; MID doesn't see line = not at intersection

                        JSR   SENSOR_READ
                        ; MID sees line - now check for branch lines
                        LDAA  SENSOR_PORT
                        CMPA  #THRESH_PORT
                        BHS   INTERSECTION_FOUND            ; PORT high + MID high = real intersection

NO_INTERSECTION:        LDAA  SENSOR_LINE                   ; Use differential for positioning
                        CMPA  #THRESH_CENTER_RIGHT
                        BLO   LINE_NAV_LEFT                ; Line to right
                        
                        CMPA  #THRESH_CENTER_LEFT
                        BHS   LINE_NAV_RIGHT                 ; Line to left
                        BRA   LINE_NAV_FORWARD              ; Centered

INTERSECTION_FOUND:     JSR   INIT_ALL_STP
                        
                        JSR   INIT_FWD
                        LDY   #1100
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #1200
                        JSR   DELAY_50US
                        
                        JSR   INIT_FWD
                        LDY   #1100
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #1200
                        JSR   DELAY_50US
                        
                        JSR   INIT_FWD
                        LDY   #500
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #500
                        JSR   DELAY_50US
                        
                        JMP   LINE_NAV_EXIT

LINE_NAV_LEFT:          JSR   INIT_SOFT_LEFT
                        LDY   #1200                          ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        JSR   INIT_ALL_STP
                        LDY   #500
                        JSR   DELAY_50US
                        BRA   LINE_NAV_EXIT

LINE_NAV_RIGHT:         JSR   INIT_SOFT_RIGHT
                        LDY   #1200                          ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        JSR   INIT_ALL_STP
                        LDY   #500
                        JSR   DELAY_50US
                        BRA   LINE_NAV_EXIT

LINE_NAV_FORWARD:       JSR   INIT_FWD
                        LDY   #1500                         ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        JSR   INIT_ALL_STP
                        LDY   #800
                        JSR   DELAY_50US
                        BRA   LINE_NAV_EXIT

LINE_NAV_EXIT:          RTS

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
INIT_LEFT_TRN           BCLR  PORTA,%00000010
                        BSET  PORTA,%00000001               ; Set REV dir. for STARBOARD (right) motor
                        BSET  PTT,  %00110000
                        RTS
                    
; Turn motors on to rotate left
INIT_RIGHT_TRN          BCLR  PORTA,%00000001               ; Set FWD dir. for STARBOARD (right) motor
                        BSET  PORTA,%00000010
                        BSET  PTT,  %00110000
                        RTS

; Turn motors on to pivot right (Port FWD, Starboard OFF)
INIT_SOFT_LEFT:        BCLR  PORTA,%00000010               ; Set Port (Left) direction FWD (0) (Assuming BCLR for FWD)
                        BCLR  PORTA,%00000001               ; Set Starboard (Right) direction FWD (0) - doesn't matter since it's off
                        BCLR  PTT,  %00010000               ; Turn OFF Starboard motor (bit 4)
                        BSET  PTT,  %00100000               ; Turn ON Port motor (bit 5)
                        RTS                    

; Turn motors on to pivot left (Starboard FWD, Port OFF)
INIT_SOFT_RIGHT:         BCLR  PORTA,%00000010               ; Set Port (Left) direction FWD (0) - doesn't matter since it's off
                        BCLR  PORTA,%00000001               ; Set Starboard (Right) direction FWD (0)
                        BSET  PTT,  %00010000               ; Turn ON Starboard motor (bit 4)
                        BCLR  PTT,  %00100000               ; Turn OFF Port motor (bit 5)
                        RTS
                
;--------------------------------------------------------------------------
; TOF Control Subroutines
;--------------------------------------------------------------------------

; Initialize and enable Timer Overflow interrupt with prescaler
ENABLE_TOF              LDAA  #%10000000
                        STAA  TSCR1                         ; Enable TCNT
                        STAA  TFLG2                         ; Clear TOF
                        LDAA  #%10000100                    ; Enable TOI and select prescale factor equal to 16
                        STAA  TSCR2
                        RTS

; Timer Overflow Interrupt Service Routine - increments overflow counter
TOF_ISR                 INC   TOF_COUNTER
                        LDAA  #%10000000                    ; Clear
                        STAA  TFLG2                         ; TOF
                        RTI

;---------------------------------------------------------------------------
; Guider Sensor Read Subroutine
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
                        JSR   DELAY_50US                    ; (400 * 50Ã¯Â¿Â½s = 20ms)

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
; Wait for Strip Subroutine - Left Turn
;---------------------------------------------------------------------------

LEFT_WAIT_FOR_STRIP:    JSR  INIT_FWD
                        LDY  #8000
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #1000
                        JSR  DELAY_50US
                        
LEFT_WAIT_STRIP_FADE:   JSR  INIT_LEFT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BHS  LEFT_WAIT_STRIP_FADE                 ; Still on old strip, wait

LEFT_WAIT_STRIP_RISE:   JSR  INIT_LEFT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BLO  LEFT_WAIT_STRIP_RISE                ; New strip not detected yet, keep waitingg
                                        
LEFT_WAIT_STRIP_DONE:   JSR  INIT_LEFT_TRN
                        LDY  #1000
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #5000
                        JSR  DELAY_50US
                        RTS

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
; Delay subroutine - provides ~50 microsecond delay
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
                        ORG   $FFDE
                        DC.W  TOF_ISR                       ; Timer Overflow Interrupt Vector               
