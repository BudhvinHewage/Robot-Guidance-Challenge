;-------------------------------------------------------------------
;*              Lab 6: Robot Guidance Challenge (9S32C)               *
;-------------------------------------------------------------------

; export symbols
                        XDEF Entry, _Startup  ; export 'Entry' symbol
                        ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
                        INCLUDE 'derivative.inc'

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

; Thresholds for the sensors (tune these)
THRESH_LIGHT            EQU $40                             ; Example 64 decimal (set by calibration)
THRESH_CENTER           EQU $80                             ; Center value for line sensor
THRESH_DARK             EQU $80                             ; Example 128 decimal

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
; Sensors
SENSOR_LINE             FCB $01                             ; Line Sensor
SENSOR_BOW              FCB $23                             ; Front ("bow") sensor
SENSOR_PORT             FCB $45                             ; Left sensor
SENSOR_MID              FCB $67                             ; Center sensor
SENSOR_STBD             FCB $89                             ; Right sensor
SENSOR_NUM              RMB 1                               ; The currently selected sensor

; Tof Counter
TOF_COUNTER             DC.B 0                              ; The timer, incremented at 23Hz
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
                        MOVB #2, HEADING          
                        BRA MAIN

;-------------------------------------------------------------------
; Main Program Section
;-------------------------------------------------------------------

MAIN:
                        LDAA  CURRENT_STATE
                        JSR   DISPATCHER
                        BRA   MAIN
;-------------------------------------------------------------------
; State Machine Dispatcher
;-------------------------------------------------------------------

DISPATCHER              CMPA  #STATE_IDLE                   ; If it’s the IDLE state 
                        BNE   NOT_IDLE                      ;
                        JSR   IDLE_ST                       ; then call IDLE_ST routine
                        BRA   DISP_EXIT                     ; and exit
                                                    
NOT_IDLE                CMPA  #STATE_SEARCH_LINE            ; Else if it's the SEARCH_LINE state
                        BNE   NOT_SEARCH_LINE             
                        JSR   SEARCH_LINE_ST                ; then call SEARCH_LINE_ST routine
                        BRA   DISP_EXIT                     ; and exit
                
NOT_SEARCH_LINE         CMPA  #STATE_AT_INTERSECTION        ; Else if it's the AT_INTERSECTION state
                        BNE   NOT_AT_INTERSECTION
                        JSR   AT_INTERSECTION_ST            ; then call AT_INTERSECTION_ST routine
                        BRA   DISP_EXIT                     ; and exit

NOT_AT_INTERSECTION     CMPA  #STATE_MOVING_BRANCH          ; Else if it's the MOVING_BRANCH state
                        BNE   NOT_MOVING_BRANCH
                        JSR   MOVING_BRANCH_ST              ; then call MOVING_BRANCH_ST routine
                        BRA   DISP_EXIT                     ; and exit

NOT_MOVING_BRANCH       CMPA  #STATE_BUMPER_COLLIDE         ; Else if it's the BUMPER_COLLIDE state
                        BNE   NOT_BUMPER_COLLIDE
                        JSR   BUMPER_COLLIDE_ST             ; then call BUMPER_COLLIDE_ST routine
                        BRA   DISP_EXIT                     ; and exit

NOT_BUMPER_COLLIDE      CMPA  #STATE_BACKTRACKING           ; Else if it's the BACKTRACKING state
                        BNE   NOT_BACKTRACKING
                        JSR   BACKTRACKING_ST               ; then call BACKTRACKING_ST routine
                        BRA   DISP_EXIT                     ; and exit

NOT_BACKTRACKING        CMPA  #STATE_RETRACING              ; Else if it's the RETRACING state
                        BNE   NOT_RETRACING
                        JSR   RETRACING_ST                  ; then call RETRACING_ST routine
                        BRA   DISP_EXIT                     ; and exit

NOT_RETRACING           SWI                                 ; Else the STATE is not defined, so stop

DISP_EXIT               RTS                                 ; Exit from the state dispatcher 

;--------------------------------------------------------------------------
; The Idle State - Waiting for Start Signal
;--------------------------------------------------------------------------

IDLE_ST                 BRCLR PORTAD0,$04,NO_IDLE_ST        ; Wait for front bumper press
                        MOVB  #STATE_SEARCH_LINE,CURRENT_STATE

NO_IDLE_ST              NOP                                 ; Do nothing while idle

IDLE_ST_EXIT            RTS
              
;-------------------------------------------------------------------------
; The Search Line State - Move forward until line is found
;-------------------------------------------------------------------------

SEARCH_LINE_ST:         JSR   SENSOR_READ                   ; Refresh sensor values
                                
                        LDAA  SENSOR_BOW                    ; Check bow sensor for line
                        CMPA  #THRESH_DARK                  ; Compare with dark threshold
                        BLO   STILL_SEARCHING               ; If below threshold, keep searching

FOUND_LINE              JSR   LINE_NAVIGATION               ; Move forward along line
                        BRA   SEARCH_LINE_ST_EXIT                   

STILL_SEARCHING         JSR   INIT_FWD                      ; Move forward to find line  
                        BRA   SEARCH_LINE_ST_EXIT          
                
SEARCH_LINE_ST_EXIT:    RTS

;--------------------------------------------------------------------------   
; The At Intersection State - Decide which way to go
;---------------------------------------------------------------------------

AT_INTERSECTION_ST:     JSR   INIT_ALL_STP                  ; Stop motors
                        JSR   SENSOR_READ                   ; Refresh sensor values

                        LDAA  HEADING                       ; Current heading   
                        ADDA  #2                            ; Add 180 degrees
                        ANDA  #$03                          ; Modulo 4
                        STAA  ENTRY_DIRECTION               ; Store into Accumulator A the entry direction where 1 to 4 = N, E, S, W

                        
                        JSR   GET_INTERSECTION_PTR          ; Get pointer to current intersection data where X = address of 3-byte block

                        LDAA  0,X                           ; Load entry direction stored at this intersection
                        BNE   RETURN_VISIT                  ; If non-zero, we've been here before

                        LDAA  ENTRY_DIRECTION               ; First time here: store entry direction
                        STAA  0,X                           ; Store entry direction
                        INC   MAZE_COUNT                    ; New intersection discovered!
                        
RETURN_VISIT:           JSR   GET_EXITS                     ; Get available exits (returns count in A, directions in B and scratch)
                        
                        CMPA  #1                            ; Is it an L-junction?
                        BEQ   HANDLE_L_JUNCTION       
                        CMPA  #2                            ; Is it a T-junction?
                        BEQ   HANDLE_T_JUNCTION       

HANDLE_L_JUNCTION:      LDAA  1,X                           ; Check if we've been here before
                        BNE   L_BEEN_HERE                   ; BNE checks if A is non-zero
                        
                        STAB   1,X                           ; Store the byte into 
                        TBA                                 ; Move first exit to A  
                        STAA  HEADING                       ; Set heading to first exit
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to new heading 
                        
                        LDAA  #STATE_MOVING_BRANCH
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

L_BEEN_HERE:            LDAA  ENTRY_DIRECTION               ; Load into Accumulator A the entry direction of the intersection
                        STAA  HEADING                       ; Set the desired heading to the entry direction to 'turn around' and revisit the previous intersection
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to face that direction

                        LDAA  #STATE_BACKTRACKING           ; Continue backtracking
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

HANDLE_T_JUNCTION:      LDAA  1,X                           ; Check what we've tried
                        BNE   T_TRIED_FIRST                 ; If non-zero, we've tried one option already

                        STAB   1,X                           ; Store first exit tried
                        TBA                                 ; Move first exit to A  
                        STAA  HEADING                       ; Set heading to first exit
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to new heading

                        LDAA  #STATE_MOVING_BRANCH
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

T_TRIED_FIRST:          LDAA  2,X                           ; Check second attempt                         
                        BNE   T_TRIED_BOTH                  ; If non-zero, we've tried both options

                        LDAA  TEMP                          ; Get right option
                        STAA  2,X                           ; Record right exit
                        STAA  HEADING                       ; Set heading to right exit
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to new heading

                        LDAA  #STATE_MOVING_BRANCH 
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

T_TRIED_BOTH:           LDAA  ENTRY_DIRECTION               ; Load into Accumulator A the entry direction of the intersection
                        STAA  HEADING                       ; Set the desired heading to the entry direction to 'turn around' and revisit the previous intersection
                        JSR   EXECUTE_TURN_TO_HEADING

                        LDAA  #STATE_BACKTRACKING           ; Continue backtracking
                        STAA  CURRENT_STATE                 ; Update current state to backtracking

AT_INTERSECTION_EXIT:   RTS

;--------------------------------------------------------------------------
; The Moving Branch State - Move down selected path until intersection or bumper
;--------------------------------------------------------------------------

MOVING_BRANCH_ST:       BRSET PORTAD0,$04,NO_FWD_BUMP       ; Check front bumper for collision
                        LDAA  #STATE_BUMPER_COLLIDE         ; Update state to bumper collide
                        STAA  CURRENT_STATE
                        BRA   MOVING_BRANCH_EXIT

NO_FWD_BUMP             BRSET PORTAD0,$08,NO_REAR_BUMP
                        LDAA  #STATE_RETRACING
                        STAA  CURRENT_STATE
                        BRA   MOVING_BRANCH_EXIT

NO_REAR_BUMP            JSR   READ_SENSORS                  ; Refresh sensor values
                        LDAA  SENSOR_PORT                   ; Check PORT sensor for line
                        CMPA  #THRESH_DARK                  ; Compare with dark threshold
                        BGE   NEW_BRANCH_DETECTED           ; If above or equal, new branch detected
                        LDAA  SENSOR_STBD                   ; Check STBD sensor for line
                        CMPA  #THRESH_DARK                  ; Compare with dark threshold
                        BGE   NEW_BRANCH_DETECTED           ; If above or equal, new branch detected                        

NEW_BRANCH_DETECTED:    JSR   INIT_ALL_STP                  ; Stop motors at intersection and then add new intersection count

                        LDAA  CURRENT_INTERSECTION          ; Load current intersection index
                        INCA                                ; Increment to next intersection
                        STAA  CURRENT_INTERSECTION          ; Store updated intersection index

                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE

MOVING_BRANCH_EXIT:     RTS

;-------------------------------------------------------------------------
; The Bumper Collide State - Handle dead end via U-turn and indicate that this path failed and check it off on the maze table
;-------------------------------------------------------------------------

BUMPER_COLLIDE_ST:      JSR   INIT_ALL_STP                  ; Stop motors

                        LDAA  HEADING                       ; Load current heading
                        ADDA  #2                            ; Add 180 degrees
                        ANDA  #$03                          ; Modulo 4
                        STAA  HEADING                       ; Store new heading

                        JSR   EXECUTE_U_TURN                ; Execute U-turn

                        LDAA  #STATE_BACKTRACKING           ; Update state to backtracking
                        STAA  CURRENT_STATE     

BUMPER_COLLIDE_EXIT:    RTS

;-------------------------------------------------------------------------
; The Retracing State - Retrace back to start following stored solutions
;-------------------------------------------------------------------------

RETRACING_ST            LDAA CURRENT_INTERSECTION
                        BEQ REACHED_START                   ; If at start, we're done retracing

                        JSR   SENSOR_READ                  ; Refresh sensor values

                        LDAA  SENSOR_PORT                   ; Check PORT sensor for line to check if we're at an intersection
                        CMPA  #THRESH_DARK                  ; Compare with dark threshold
                        BGE   RETRACE_INTERSECTION          ; If above or equal, we're at intersection
                        LDAA  SENSOR_STBD                   ; Check STBD sensor for line to check if we're at an intersection
                        CMPA  #THRESH_DARK                  ; Compare with dark threshold
                        BGE   RETRACE_INTERSECTION          ; If above or equal, we're at intersection
                        
                        JSR   LINE_NAVIGATION               ; Move forward along line since no branching line has been detected yet
                        BRA   RETRACING_EXIT

RETRACE_INTERSECTION:   JSR   INIT_ALL_STP                  ; Stop at intersection

                        LDY   #10                           ; Small delay to settle
                        JSR   DELAY_50US

                        JSR   GET_INTERSECTION_PTR          ; X points to this intersection's data
                        LDAA  0,X                           ; Load entry direction
                        
                        STAA  HEADING                       ; Set heading to entry direction since we're retracing and its where we came from
                        
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to face that direction

                        LDAA  CURRENT_INTERSECTION          ; Load current intersection index
                        DECA                                ; Decrement to previous intersection
                        STAA  CURRENT_INTERSECTION          ; Store updated intersection index

RETRACING_EXIT:         RTS

REACHED_START:          JSR   INIT_ALL_STP
                        
                        LDAA  #STATE_IDLE
                        STAA  CURRENT_STATE
                        RTS
                        
; -------------------------------------------------------------------------
; BACKTRACKING_ST - Follow line back to previous intersection
; -------------------------------------------------------------------------

BACKTRACKING_ST:        JSR   SENSOR_READ
    
                        LDAA  SENSOR_PORT                   ; Check PORT sensor for line to check if we're back at the previous intersection
                        CMPA  #THRESH_DARK                  ; Compare with dark threshold
                        BGE   BACK_AT_INTERSECTION          ; If above or equal, we're back at intersection
                        LDAA  SENSOR_STBD                   ; Check STBD sensor for line to check if we're back at the previous intersection
                        CMPA  #THRESH_DARK                  ; Compare with dark threshold
                        BGE   BACK_AT_INTERSECTION          ; If above or equal, we're back at intersection
                                                
                        JSR   LINE_NAVIGATION               ; Move forward along line since no branching line has been detected yet
                        BRA   BACKTRACKING_EXIT             ; Exit to backtracking state

BACK_AT_INTERSECTION:   JSR   INIT_ALL_STP                  ; Arrived at the previous intersection

                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE                      

BACKTRACKING_EXIT:      RTS

; ----------------------------------------------------------------------
; GET_INTERSECTION_PTR - Get pointer to current intersection data block in MAZE_TABLE
;----------------------------------------------------------------------

GET_INTERSECTION_PTR:   LDX   #MAZE_TABLE                   ; Base of maze table
                        LDAA  CURRENT_INTERSECTION          ; Get current intersection index
                        LDAB  #3                            ; 3 bytes per intersection
                        MUL                                 ; D = A × B
                        LEAX  D,X                           ; X = base + offset
                        RTS
 
;-----------------------------------------------------------------------
; GET_EXITS - Determine available exits at current intersection, returning A = count, B = first exit, TEMP = second exit (if any)
;-----------------------------------------------------------------------

EXIT_LIST:              DS.B 2                              ; Temp storage for found exits

GET_EXITS:              CLR   EXIT_LIST                     ; Clear exit list
                        CLR   EXIT_LIST+1
                        CLRA                                ; A will count exits found
                    
                        LDAB  SENSOR_PORT                   ; Check LEFT sensor by loading it into Accumulator B
                        CMPB  #THRESH_DARK                  ; Compare with dark threshold
                        BLO   CHECK_STRAIGHT_SENSOR         ; If the result is below threshold, no left path so skip to straight check
    
                        JSR   GET_LEFT_DIRECTION            ; Since left path exists, calculate its absolute direction
                        LDAB  SCRATCH_DIR                   ; Load calculated left direction
                        CMPB  ENTRY_DIRECTION               ; Is this where we came from?
                        BEQ   CHECK_STRAIGHT_SENSOR         ; Yes, skip it
                    
                        LDX   #EXIT_LIST                    ; No, it's a valid exit
                        ABX                                 ; X points to EXIT_LIST[A]
                        STAB   0,X                           ; Store this exit direction
                        INCA                                ; Count++
    
CHECK_STRAIGHT_SENSOR:  LDAB  SENSOR_BOW                    ; Check STRAIGHT sensor (SENSOR_BOW) 
                        CMPB  #THRESH_DARK                  ; Compare with dark threshold
                        BLO   CHECK_RIGHT_SENSOR            ; If below threshold, no straight path so skip to right check
                        
                        LDAB  HEADING                       ; Since straight path exists, its absolute direction is current heading
                        CMPB  ENTRY_DIRECTION               ; Is this where we came from?
                        BEQ   CHECK_RIGHT_SENSOR            ; Yes, skip it
                        
                        LDX   #EXIT_LIST                    ; No, it's a valid exit
                        ABX                                 ; X points to EXIT_LIST[A]
                        STAB   0,X                           ; Store this exit direction
                        INCA                                ; Count++
                    
CHECK_RIGHT_SENSOR:     LDAB  SENSOR_STBD                   ; Check RIGHT sensor (SENSOR_STBD)
                        CMPB  #THRESH_DARK                  ; Compare with dark threshold
                        BLO   EXITS_FOUND                   ; If below threshold, no right path so finish
                        
                        JSR   GET_RIGHT_DIRECTION           ; Returns direction in SCRATCH_DIR
                        LDAB  SCRATCH_DIR                   ; Load calculated right direction
                        CMPB  ENTRY_DIRECTION               ; Is this where we came from?
                        BEQ   EXITS_FOUND                   ; Yes, skip it
                        
                        LDX   #EXIT_LIST                    ; No, it's a valid exit
                        ABX                                 ; X points to EXIT_LIST[A]
                        STAB   0,X                           ; Store this exit direction
                        INCA                                ; Count++

EXITS_FOUND:            LDAB  EXIT_LIST                     ; Load first exit into B
               
                        CMPA  #2                            ; Did we find two exits?
                        BLO   GET_EXITS_DONE                ; If less than 2, we're done
                        LDAA  EXIT_LIST+1                   ; Load into Accumulator A the second exit
                        STAA  TEMP                          ; Store second exit in TEMP
                        LDAA  #2                            ; Restore count
                    
GET_EXITS_DONE:         RTS

; -----------------------------------------------------------------------
; GET_LEFT_DIRECTION - Calculate absolute direction of relative left and store in SCRATCH_DIR
; -----------------------------------------------------------------------

GET_LEFT_DIRECTION:     LDAA  HEADING                       ; Load current heading (1=N, 2=E, 3=S, 4=W)
                        DECA                                ; Heading - 1
                        CMPA  #0                            ; Did we go below 1? (0 is invalid)
                        BNE   LEFT_OK                       ; If not, we're good
                        LDAA  #4                            ; Wrap around to 4 (W)
                                        
LEFT_OK:                STAA  SCRATCH_DIR                   ; Store result
                        RTS

; -----------------------------------------------------------------------
; GET_RIGHT_DIRECTION - Calculate absolute direction of relative right and store in SCRATCH_DIR
; -----------------------------------------------------------------------
GET_RIGHT_DIRECTION:    LDAA  HEADING                       ; Load current heading (1=N, 2=E, 3=S, 4=W)
                        INCA                                ; Heading + 1                  
                        CMPA  #5                            ; Did we go above 4? (5 is invalid)
                        BLO   RIGHT_OK                      ; If not, we're good
                        LDAA  #1                            ; Wrap around to 1 (N)

RIGHT_OK:               STAA  SCRATCH_DIR                   ; Store result
                        RTS

;--------------------------------------------------------------------------
; Execute Turn to Heading - Turn robot to face specified absolute heading where on entry HEADING = desired heading and ENTRY_DIRECTION = direction we came from
;--------------------------------------------------------------------------

EXECUTE_TURN_TO_HEADING:LDAA  HEADING                       ; Get new heading
                        LDAB  ENTRY_DIRECTION               ; Get old heading (reverse of entry)
                        ADDB  #2                            ; Calculate old heading
                        CMPB  #5                            ; Did we go above 4?
                        BLO   OLD_HEAD_OK                   ; If not, we're good
                        SUBB  #4                            ; Wrap around to 1..4
    
OLD_HEAD_OK:            CBA                               ; Calculate turn delta: new - old
                        BPL   TURN_OK                       ; If positive or zero, proceed
                        ADDA  #4                            ; If negative, add 4 to get positive equivalent
    
    
TURN_OK:                ANDA  #$03                          ; Modulo 4 to get turn delta
    
                        CMPA  #1                            ; Is it a right turn?
                        BEQ   TURN_RIGHT      
                        CMPA  #3                            ; Is it a left turn?
                        BEQ   TURN_LEFT
                        
                        RTS
    
TURN_LEFT:              JSR   INIT_LEFT_TRN
                        JSR   WAIT_FOR_STRIP
                        RTS
    

TURN_RIGHT:             JSR   INIT_RIGHT_TRN
                        JSR   WAIT_FOR_STRIP
                        RTS

;------------------------------------------------------------------------
; Execute U-Turn - Perform a 180 degree turn
;------------------------------------------------------------------------

EXECUTE_U_TURN:         JSR   INIT_LEFT_TRN
                        JSR   WAIT_FOR_STRIP
                        JSR   INIT_LEFT_TRN
                        JSR   WAIT_FOR_STRIP
                        RTS

;-------------------------------------------------------------------------- 
; Movement along an indicated line until intersection or bumper is detected
;--------------------------------------------------------------------------

LINE_NAVIGATION:        JSR   SENSOR_READ                   ; Refresh sensor values

                        BRSET PORTAD0,$04,NO_FRONT_BUMPER   ; Check front bumper for collision
                        LDAA  #STATE_BUMPER_COLLIDE         ; Update state to bumper collide
                        STAA  CURRENT_STATE
                        BRA   LINE_NAV_RIGHT

NO_FRONT_BUMPER         LDAA  SENSOR_PORT 
                        CMPA  #THRESH_DARK
                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE
                        BGE   LINE_NAV_EXIT                 ; If sensor >= THRESH_DARK, exit
                        
                        LDAA  SENSOR_STBD
                        CMPA  #THRESH_DARK
                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE
                        BGE   LINE_NAV_EXIT                 ; If sensor >= THRESH_DARK, exit

                        LDAA  SENSOR_LINE
                        CMPA  #THRESH_CENTER
                        BEQ   LINE_NAV_FORWARD
                        BLO   LINE_NAV_RIGHT                ; Sensor low -> line to right

LINE_NAV_LEFT:          JSR   INIT_LEFT_TRN 
                        JSR   WAIT_FOR_STRIP
                        BRA   LINE_NAV_EXIT

LINE_NAV_RIGHT:         JSR   INIT_RIGHT_TRN
                        JSR   WAIT_FOR_STRIP
                        BRA   LINE_NAV_EXIT

LINE_NAV_FORWARD:       JSR   INIT_FWD
                        BRA   LINE_NAV_EXIT

LINE_NAV_EXIT:          RTS

;--------------------------------------------------------------------------
; Initialize Ports
;--------------------------------------------------------------------------

INIT_PORTS              BCLR DDRAD,$FF                      ; Make PORTAD an input (DDRAD@$0272)
                        BSET DDRA, $FF                      ; Make PORTA an output (DDRA@$0002)
                        BSET DDRB, $FF                      ; Make PORTB an output (DDRB@$0003)
                        BSET DDRJ, $C0                      ; Make pins 7,6 of PTJ outputs (DDRJ @$026A)
                        
                        BSET DDRA, $03                      ; STAR_DIR, PORT_DIR  00000011
                        BSET DDRT, $30                      ; STAR_SPEED, PORT_SPEED 00110000
                        
                        RTS
                        
;--------------------------------------------------------------------------
; Motor Control Subroutines
;--------------------------------------------------------------------------

; Turn motors on to move forward
INIT_FWD                BCLR  PORTA,%00000011               ; Set FWD direction for both motors
                        BSET  PTT,%00110000                 ; Turn on the drive motors
                        LDAA  TOF_COUNTER                   ; Mark the fwd time Tfwd
                        ADDA  #FWD_INT
                        STAA  T_FWD
                        RTS
                    
; Turn motors on to move backward
INIT_REV                BSET  PORTA,%00000011               ; Set REV direction for both motors
                        BSET  PTT,%00110000                 ; Turn on the drive motors
                        LDAA  TOF_COUNTER                   ; Mark the fwd time Tfwd
                        ADDA  #REV_INT
                        STAA  T_REV
                        RTS
                    
; Turn motors off
INIT_ALL_STP            BCLR  PTT,%00110000                 ; Turn off the drive motors
                        RTS
                    
; Turn motors on to rotate right
INIT_RIGHT_TRN          BSET  PORTA,%00000010               ; Set REV dir. for STARBOARD (right) motor
                        LDAA  TOF_COUNTER                   ; Mark the fwd_turn time Tfwdturn
                        ADDA  #T_RIGHT_INT
                        STAA  T_RIGHT_TRN
                        RTS
                    
; Turn motors on to rotate left
INIT_LEFT_TRN           BCLR  PORTA,%00000010               ; Set FWD dir. for STARBOARD (right) motor
                        LDAA  TOF_COUNTER                   ; Mark the fwd time Tfwd
                        ADDA  #T_LEFT_INT 
                        STAA  T_LEFT_TRN
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
                        JSR   DELAY_50US                    ; (400 * 50�s = 20ms)

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
; Wait for Strip Subroutine
;---------------------------------------------------------------------------

WAIT_FOR_STRIP:         JSR  G_LEDS_ON
                
WAIT_STRIP_FADE:        JSR  READ_SENSORS
                        LDAA SENSOR_BOW
                        CMPA #THRESH_LIGHT
                        BGE  WAIT_STRIP_FADE                ; Still seeing old strip, wait

WAIT_STRIP_RISE:        BGE  WAIT_STRIP_DONE                ; New strip under line sensor
                        LDAA SENSOR_BOW
                        CMPA #THRESH_DARK
                        BLO  WAIT_STRIP_RISE                ; No strip seen yet, keep waiting

WAIT_STRIP_DONE:        JSR  G_LEDS_OFF 
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

OUTER_LOOP              LDX   #300                        ; (2 E-clk) Initialize the inner loop counter

INNER_LOOP              NOP                                 ; (1 E-clk) No operation
                        DBNE  X,INNER_LOOP                  ; (3 E-clk) If the inner cntr not 0, loop again
                        DBNE  Y,OUTER_LOOP                  ; (3 E-clk) If the outer cntr not 0, loop again
                        PULX                                ; (3 E-clk) Restore the X register
                        RTS                                 ; (5 E-clk) Else return     

;--------------------------------------------------------------------------
; Interrupt Vector Table
;--------------------------------------------------------------------------

                        ORG   $FFFE
                        DC.W  Entry                         ; Reset Vector
                        ORG   $FFDE
                        DC.W  TOF_ISR                       ; Timer Overflow Interrupt Vector               