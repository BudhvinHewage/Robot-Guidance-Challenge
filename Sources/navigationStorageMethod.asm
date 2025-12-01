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
LCD_DAT                 EQU PORTB                           ;LCD data port, bits - PB7,...,PB0
LCD_CNTR                EQU PTJ                             ;LCD control port, bits - PE7(RS),PE4(E)
LCD_E                   EQU $80                             ;LCD E-signal pin
LCD_RS                  EQU $40                             ;LCD RS-signal pin

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
EXIT_LIST:              DS.B 2                              ; List of available exits at current intersections
TEMP_DIRECTION          DS.B 1
TEMP_EXIT_COUNT         DS.B 1
TEMP_FIRST_EXIT         DS.B 1
TEMP_SECOND_EXIT        DS.B 1
PORTCHECK:              DS.B 1
STBDCHECK:              DS.B 1
BOWCHECK:               DS.B 1

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

; Robot Motion Time
T_FWD                   DS.B 1                             ; FWD time
T_REV                   DS.B 1                             ; REV time
T_RIGHT_TRN             DS.B 1                             ; RIGHT_TRN time - Was T_FWD_TRN
T_LEFT_TRN              DS.B 1                             ; LEFT_TRN time - Was T_REV_TRN

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

;-------------------------------------------------------------------
; Initialization
;-------------------------------------------------------------------   

                        ORG $4000
                  
Entry:
 _Startup:
                        LDS #$4000                          ; Initialize the stack pointer
                        CLI                                  ; Enable interrupt
                        
                        CLR CURRENT_STATE
                        CLR CURRENT_INTERSECTION
                        CLR ENTRY_DIRECTION                        

                        LDX #MAZE_TABLE
                        LDAA #24
CLEAR_MAZE:             CLR 0,X
                        INX
                        DECA
                        BNE CLEAR_MAZE                        

                        JSR INIT_PORTS
                        JSR INIT_ATD
                        MOVB #2, HEADING          
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

IDLE_ST                 BRSET PORTAD0,$04,NO_CHANGE        ; The function BRSET tests if bit 2 (front bumper) of PORTAD0 is set and if it is not, branches to NO_IDLE_ST
                        MOVB  #STATE_SEARCH,CURRENT_STATE
                        
                        JSR   INIT_FWD
                        LDY   #2000
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #1000
                        JSR   DELAY_50US

                        BRA   IDLE_ST_EXIT
                        
NO_CHANGE               NOP                                 ; Do nothing while idle

IDLE_ST_EXIT            RTS

;-------------------------------------------------------------------------
; The Search State - Navigate until the first intersection is reached
;-------------------------------------------------------------------------

SEARCH_ST:              JSR   LINE_NAVIGATION               ; This now sets flags

                        LDAA  PORTCHECK                     ; Check if intersection found
                        ORAA  STBDCHECK
                        BEQ   SEARCH_ST_EXIT                ; No intersection, keep searching
                        
                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE

SEARCH_ST_EXIT:         RTS

;--------------------------------------------------------------------------   
; The At Intersection State - Decide which way to go
;---------------------------------------------------------------------------

AT_INTERSECTION_ST:     JSR   INIT_ALL_STP
                        LDY   #2000
                        JSR   DELAY_50US
                        
                        LDAA  HEADING                       ; Current heading (1-4)
                        SUBA  #1                            ; Convert to 0-3 range
                        ADDA  #2                            ; Add 180 degrees (2 in 0-3 system)
                        ANDA  #$03                          ; Modulo 4 by keeping lower 2 bits
                        ADDA  #1                            ; Convert back to 1-4 range
                        STAA  ENTRY_DIRECTION               ; Store into Accumulator A the entry direction where 1 to 4 = N, E, S, W
                        
                        JSR   GET_INTERSECTION_PTR          ; Get pointer to current intersection data where X = address of 3-byte block

                        LDAA  0,X                           ; Load entry direction stored at this intersection
                        BNE   RETURN_VISIT                  ; If non-zero, we've been here before, since the Z flag is set only if Accumulator A is zero

                        LDAA  ENTRY_DIRECTION               ; First time here: store entry direction
                        STAA  0,X                           ; Store entry direction
                        INC   CURRENT_INTERSECTION          ; New intersection discovered!
                        
RETURN_VISIT:           CLR   TEMP_EXIT_COUNT               ; Clear exit count
                        CLR   TEMP_FIRST_EXIT               ; Clear first exit
                        CLR   TEMP_SECOND_EXIT              ; Clear second exit
                        
                        JSR   GET_EXITS                     ; Get available exits (returns count in A, directions in B and scratch)
                        
                        JSR   GET_INTERSECTION_PTR          ; X points to this intersection's data as GET_EXITS resulted in X pointing torwards EXIT_LIST

                        LDAA  TEMP_EXIT_COUNT               ; Load number of exits found
                        CMPA  #1                            ; Is it an L-junction?
                        BEQ   HANDLE_L_JUNCTION       
                        CMPA  #2                            ; Is it a T-junction?
                        BEQ   HANDLE_T_JUNCTION       

; At this point, the pointer is pointing to the current intersection data in MAZE_TABLE
; So, the bits will be storing the following with a value that represents the absolute direction
; Byte 0: Entry Direction
; Byte 1: First Exit Tried
; Byte 2: Second Exit Tried

HANDLE_L_JUNCTION:      LDAA  1,X                           ; Check if we've been here before
                        BNE   L_BEEN_HERE                   ; BNE checks if A is non-zero
                        
                        LDAB  TEMP_FIRST_EXIT               ; Get first exit
                        STAB  1,X                           ; Store the byte into 
                        STAB  HEADING                       ; Set heading to first exit
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to new heading 
                        
                        LDAA  #STATE_MOVING_BRANCH
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

; Since we've been here before, we must have tried the only exit already, so backtrack
; This is accomplished by identifying the entry direction and turning in that direction to exit the junction

L_BEEN_HERE:            LDAA  ENTRY_DIRECTION               ; Load into Accumulator A the entry direction of the intersection
                        STAA  HEADING                       ; Set the desired heading to the entry direction to 'turn around' and revisit the previous intersection
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to face that direction

                        LDAA  #STATE_BACKTRACKING           ; Continue backtracking
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

; At this point, we know it's a T-junction with two available exits
; Therefore, the function call GET_EXITS must have populated TEMP_FIRST_EXIT and TEMP_SECOND_EXIT with valid exit directions
; Then it becomes a matter of checking which exits we've already tried by looking at bytes 1 and 2 of the intersection data in MAZE_TABLE

HANDLE_T_JUNCTION:      LDAA  1,X                           ; Check what we've tried
                        BNE   T_TRIED_FIRST                 ; If non-zero, we've tried one option already

                        LDAB  TEMP_FIRST_EXIT               ; Get first exit
                        STAB  1,X                           ; Store first exit that will be tried
                        STAB  HEADING                       ; Set heading to first exit
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to new heading

                        LDAA  #STATE_MOVING_BRANCH
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

T_TRIED_FIRST:          LDAA  2,X                           ; Check second attempt                         
                        BNE   T_TRIED_BOTH                  ; If non-zero, we've tried both options

                        LDAB  TEMP_SECOND_EXIT              ; Get right option
                        STAB  2,X                           ; Record right exit
                        STAB  HEADING                       ; Set heading to right exit
                        JSR   EXECUTE_TURN_TO_HEADING       ; Turn to new heading

                        LDAA  #STATE_MOVING_BRANCH 
                        STAA  CURRENT_STATE
                        BRA   AT_INTERSECTION_EXIT

T_TRIED_BOTH:           LDAB  ENTRY_DIRECTION               ; Load into Accumulator A the entry direction of the intersection
                        STAB  HEADING                       ; Set the desired heading to the entry direction to 'turn around' and revisit the previous intersection
                        JSR   EXECUTE_TURN_TO_HEADING

                        LDAA  #STATE_BACKTRACKING           ; Continue backtracking
                        STAA  CURRENT_STATE                 ; Update current state to backtracking
                        
                        LDAA  CURRENT_INTERSECTION          ; Load current intersection index
                        DECA                                ; Decrement to previous intersection
                        STAA  CURRENT_INTERSECTION          ; Store updated intersection index

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

NO_REAR_BUMP            JSR   LINE_NAVIGATION               ; This sets flags

                        LDAA  PORTCHECK
                        ORAA  STBDCHECK
                        BEQ   MOVING_BRANCH_EXIT            ; No intersection
                        
                        ; Intersection detected
                        JSR   INIT_ALL_STP
                        LDY   #1000
                        JSR   DELAY_50US

                        LDAA  CURRENT_INTERSECTION
                        INCA
                        STAA  CURRENT_INTERSECTION

                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE

MOVING_BRANCH_EXIT:     RTS

;-------------------------------------------------------------------------
; The Bumper Collide State - Handle dead end via U-turn and indicate that this path failed and check it off on the maze table
;-------------------------------------------------------------------------

BUMPER_COLLIDE_ST:      JSR   INIT_ALL_STP                  ; Stop motors

                        JSR   INIT_REV                      ; Move backward slightly
                        LDY   #2000                       ; Short reverse
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP                  ; Stop motors
                        LDY   #1000                       ; Small delay to settle
                        JSR   DELAY_50US
                        
                        LDAA  HEADING                       ; Load current heading
                        SUBA  #1                            ; Convert to 0-3 range
                        ADDA  #2                            ; Add 180 degrees
                        ANDA  #$03                          ; Modulo 4
                        ADDA  #1                            ; Convert back to 1-4 range
                        STAA  HEADING                       ; Store new heading

                        JSR   EXECUTE_U_TURN                ; Execute U-turn

                        LDAA  #STATE_BACKTRACKING           ; Update state to backtracking
                        STAA  CURRENT_STATE     

BUMPER_COLLIDE_EXIT:    RTS

;-------------------------------------------------------------------------
; The Retracing State - Retrace back to start following stored solutions
;-------------------------------------------------------------------------

RETRACING_ST            LDAA  CURRENT_INTERSECTION
                        BEQ   REACHED_START                 ; If at start, we're done retracing

                        LDAA  PORTCHECK
                        ORAA  STBDCHECK
                        BNE   RETRACE_INTERSECTION          ; If at intersection, handle it
                        
                        JSR   LINE_NAVIGATION               ; Move forward along line since no branching line has been detected yet
                        BRA   RETRACING_EXIT

RETRACE_INTERSECTION:   JSR   INIT_ALL_STP                  ; Stop at intersection

                        LDY   #1000                       ; Small delay to settle
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
                        JST   INIT_PORTS
                        LDAA  #STATE_IDLE
                        STAA  CURRENT_STATE
                        RTS
                        
; -------------------------------------------------------------------------
; BACKTRACKING_ST - Follow line back to previous intersection
; -------------------------------------------------------------------------

BACKTRACKING_ST:        JSR   SENSOR_READ
    
                        LDAA  PORTCHECK
                        ORAA  STBDCHECK
                        BNE   BACK_AT_INTERSECTION          ; If at intersection, handle it
                                                
                        JSR   LINE_NAVIGATION               ; Move forward along line since no branching line has been detected yet
                        BRA   BACKTRACKING_EXIT             ; Exit to backtracking state

BACK_AT_INTERSECTION:   JSR   INIT_ALL_STP                  ; Arrived at the previous intersection
                        LDY   #1000                       ; Small delay to settle
                        JSR   DELAY_50US

                        LDAA  #STATE_AT_INTERSECTION
                        STAA  CURRENT_STATE                      

BACKTRACKING_EXIT:      RTS

; ----------------------------------------------------------------------
; GET_INTERSECTION_PTR - Get pointer to current intersection data block in MAZE_TABLE
;----------------------------------------------------------------------

GET_INTERSECTION_PTR:   LDX   #MAZE_TABLE                   ; Base of maze table
                        LDAA  CURRENT_INTERSECTION          ; Get current intersection index
                        LDAB  #3                            ; 3 bytes per intersection
                        MUL                                 ; D = A Ã— B
                        LEAX  D,X                           ; X = base + offset
                        RTS
 
;-----------------------------------------------------------------------
; GET_EXITS - Determine available exits at current intersection, returning A = count, B = first exit, TEMP = second exit (if any)
;-----------------------------------------------------------------------

GET_EXITS:              JSR   SENSOR_READ

                        CLR   EXIT_LIST                     ; Clear exit list
                        CLR   EXIT_LIST+1                   ; Clear second exit

                        LDAA  PORTCHECK
                        BEQ   CHECK_STRAIGHT_SENSOR         ; If the value is equal to zero, no left path so skip to straight check
    
                        JSR   GET_LEFT_DIRECTION            ; Since left path exists, calculate its absolute direction
                        LDAB  SCRATCH_DIR                   ; Load calculated left direction
                        CMPB  ENTRY_DIRECTION               ; Is this where we came from?
                        BEQ   CHECK_STRAIGHT_SENSOR         ; Yes, skip it
                    
                        LDX   #EXIT_LIST                    ; No, it's a valid exit
                        STAB  ,X                            ; Store this exit direction, from Accumulator B to EXIT_LIST at index A which in this case is 0 since it is the first exit found
                        INC   TEMP_EXIT_COUNT               ; Increment Accumulator A by one to account for the fact that we found an exit on the left
    
CHECK_STRAIGHT_SENSOR:  LDAA  BOWCHECK                      ; Check STRAIGHT sensor (SENSOR_BOW) 
                        BEQ   CHECK_RIGHT_SENSOR            ; If the result is below threshold, no straight path so skip to right check
                        
                        LDAB  HEADING                       ; Since straight path exists, its absolute direction is current heading
                        CMPB  ENTRY_DIRECTION               ; Is this where we came from?
                        BEQ   CHECK_RIGHT_SENSOR            ; Yes, skip it
                        
                        LDX   #EXIT_LIST
                        STAB  TEMP_DIRECTION
                        LDAB  TEMP_EXIT_COUNT
                        ABX                                 ; X = EXIT_LIST + TEMP_EXIT_COUNT    
                        LDAA  TEMP_DIRECTION
                        STAA  0,X                           ; Store this exit direction at the the index pointed to by X
                        INC   TEMP_EXIT_COUNT               ; Increment Accumulator A by one to account for the fact that we found an exit on the straight path
                    
CHECK_RIGHT_SENSOR:     LDAA  STBDCHECK                     ; Check RIGHT sensor (SENSOR_STBD)
                        BEQ   EXITS_FOUND                   ; If the result is below threshold, no right path so skip to done
                        
                        JSR   GET_RIGHT_DIRECTION           ; Returns direction in SCRATCH_DIR
                        LDAB  SCRATCH_DIR                   ; Load calculated right direction
                        CMPB  ENTRY_DIRECTION               ; Is this where we came from?
                        BEQ   EXITS_FOUND                   ; Yes, skip it
                        
                        LDX   #EXIT_LIST                    ; No, it's a valid exit
                        STAB  TEMP_DIRECTION
                        LDAB  TEMP_EXIT_COUNT
                        ABX
                        LDAA  TEMP_DIRECTION
                        STAA  0,X
                        INC   TEMP_EXIT_COUNT               ; Increment Accumulator A by one to account for the fact that we found an exit on the right path

EXITS_FOUND:            LDAB  EXIT_LIST                     ; Load first exit into B
                        STAB  TEMP_FIRST_EXIT               ; Store first exit

                        LDAA  TEMP_EXIT_COUNT
                        CMPA  #2                            ; Did we find two exits?
                        BLO   GET_EXITS_DONE                ; If less than 2, we're done
                        
                        LDAA  EXIT_LIST+1                   ; Load into Accumulator A the second exit
                        STAA  TEMP_SECOND_EXIT              ; Store second exit 
                    
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

EXECUTE_TURN_TO_HEADING:LDAA  HEADING                       ; Get the heading we want to face
                        LDAB  ENTRY_DIRECTION               ; Get old heading (reverse of entry), where it could be the the original heading into the intersection or the entry direction when backtracking

; This is to recalculate the current heading since it is not tracked in the subroutine
; Heading is currently storing the direction we wish to head towards
; Therefore, we need to calculate the old heading based on the entry direction

                        ADDB  #2                            ; Calculate old heading + 2 (180 degree turn) where it is B + 2
                        CMPB  #5                            ; Did we go above 4?
                        BLO   OLD_HEAD_OK                   ; If not, we're good
                        SUBB  #4                            ; Wrap around to 1..4

; B now has the old heading (1-4) and A has the desired new heading (1-4)

OLD_HEAD_OK:            CBA                                 ; Calculate turn delta: A - B
                        BPL   TURN_OK                       ; If positive or zero, proceed
                        ADDA  #4                            ; If negative, add 4 to get positive equivalent
    
    
TURN_OK:                ANDA  #$03                          ; Modulo 4 to get turn delta
    
                        CMPA  #1                            ; Is it a left turn?
                        BEQ   TURN_RIGHT      
                        CMPA  #3                            ; Is it a right turn?
                        BEQ   TURN_LEFT

NO_TURN                 JSR   INIT_FWD                       ; Otherwise, it's a no-turn, so just go forward
                        LDY   #10000                        ; Long forward movement pulse
                        JSR   DELAY_50US
                        JSR   INIT_ALL_STP                  ; Stop after pulse
                        LDY   #1000                       ; Small delay to settle
                        JSR   DELAY_50US
                        BRA   TURN_COMPLETE
    
TURN_LEFT:              JSR   LEFT_WAIT_FOR_STRIP
                        BRA   TURN_COMPLETE
    

TURN_RIGHT:             JSR   RIGHT_WAIT_FOR_STRIP
                        BRA   TURN_COMPLETE

TURN_COMPLETE           RTS

;------------------------------------------------------------------------
; Execute U-Turn - Perform a 180 degree turn
;------------------------------------------------------------------------

EXECUTE_U_TURN:         

U_WAIT_FOR_STRIP:       JSR  INIT_REV
                        LDY  #2000
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #1000
                        JSR  DELAY_50US
                        
U_WAIT_STRIP_FADE:      JSR  INIT_LEFT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BHS  U_WAIT_STRIP_FADE                 ; Still on old strip, wait

U_WAIT_STRIP_RISE:      JSR  INIT_LEFT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BLO  U_WAIT_STRIP_RISE                ; New strip not detected yet, keep waitingg
                                        
U_WAIT_STRIP_DONE:      JSR  INIT_LEFT_TRN
                        LDY  #900
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #5000
                        JSR  DELAY_50US
                        
                        RTS
 
;-------------------------------------------------------------------------- 
; Movement along an indicated line until intersection or bumper is detected
;--------------------------------------------------------------------------

LINE_NAVIGATION:        CLR   PORTCHECK
                        CLR   STBDCHECK
                        CLR   BOWCHECK
                        
                        JSR   SENSOR_READ                   ; Refresh sensor values

                        LDAA  SENSOR_MID
                        CMPA  #THRESH_MID
                        BLO   NO_INTERSECTION               ; MID doesn't see line = not at intersection

                        LDAA  SENSOR_PORT
                        CMPA  #THRESH_PORT
                        BHS   INTERSECTION_FOUND            ; PORT high + MID high = real intersection
                        
                        LDAA  SENSOR_STBD
                        CMPA  #THRESH_STBD
                        BHS   INTERSECTION_FOUND            ; STBD high + MID high = real intersection

NO_INTERSECTION:        LDAA  SENSOR_LINE                   ; No intersection - do line following
                        
                        CMPA  #THRESH_CENTER_RIGHT
                        BLO   LINE_NAV_LEFT                 ; Line to right
                        
                        CMPA  #THRESH_CENTER_LEFT
                        BHS   LINE_NAV_RIGHT                ; Line to left
                        
                        BRA   LINE_NAV_FORWARD              ; Centered

INTERSECTION_FOUND:     JSR   SENSOR_READ                   ; Refresh sensor values
                        
                        LDAA  SENSOR_PORT
                        CMPA  #THRESH_PORT
                        BLO   PORT_NOT_HIGH                 ; PORT high = intersection
                        LDAA  #$01
                        STAA  PORTCHECK

PORT_NOT_HIGH:          LDAA  SENSOR_STBD
                        CMPA  #THRESH_STBD
                        BLO   STBD_NOT_HIGH                 ; STBD high = intersection
                        LDAA  #$01
                        STAA  STBDCHECK

STBD_NOT_HIGH:          JSR   INIT_FWD
                        LDY   #1000
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #1200
                        JSR   DELAY_50US
                        
                        JSR   INIT_FWD
                        LDY   #1000
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #1200
                        JSR   DELAY_50US
                        
                        JSR   INIT_FWD
                        LDY   #800
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #2000
                        JSR   DELAY_50US

                        JSR   SENSOR_READ                    ; Refresh sensor values

                        LDAA  SENSOR_BOW
                        CMPA  #THRESH_BOW
                        BLO   LINE_NAV_EXIT                  ; Bow not high = no intersection
                        LDAA  #$01
                        STAA  BOWCHECK
                        
                        JMP   LINE_NAV_EXIT

LINE_NAV_LEFT:          JSR   INIT_SOFT_LEFT
                        LDY   #900                          ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #500
                        JSR   DELAY_50US
                        
                        BRA   LINE_NAV_EXIT

LINE_NAV_RIGHT:         JSR   INIT_SOFT_RIGHT
                        LDY   #900                          ; <<< Adjust these values as needed
                        JSR   DELAY_50US
                        
                        JSR   INIT_ALL_STP
                        LDY   #500
                        
                        JSR   DELAY_50US
                        BRA   LINE_NAV_EXIT

LINE_NAV_FORWARD:       JSR   INIT_FWD
                        LDY   #1200                         ; <<< Adjust these values as needed
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
                        JSR   DELAY_50US                    ; (400 * 50ï¿½s = 20ms)

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
                        LDY  #6000
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
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #1000
                        JSR  DELAY_50US
                        RTS

;---------------------------------------------------------------------------
; Wait for Strip Subroutine - Right Turn
;---------------------------------------------------------------------------

RIGHT_WAIT_FOR_STRIP:   JSR  INIT_FWD
                        LDY  #6000
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #1000
                        JSR  DELAY_50US
    
RIGHT_WAIT_STRIP_FADE:  JSR  INIT_RIGHT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR  SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BHS  RIGHT_WAIT_STRIP_FADE                ; Still on old strip, wait

RIGHT_WAIT_STRIP_RISE:  JSR  INIT_RIGHT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #10
                        JSR  DELAY_50US

                        JSR SENSOR_READ
                        LDAA SENSOR_BOW
                        CMPA #THRESH_BOW
                        BLO  RIGHT_WAIT_STRIP_RISE                ; New strip not detected yet, keep waiting                        
                                        
RIGHT_WAIT_STRIP_DONE:  JSR  INIT_RIGHT_TRN
                        LDY  #700
                        JSR  DELAY_50US
                        JSR  INIT_ALL_STP
                        LDY  #1000
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
