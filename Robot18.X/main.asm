    list p=18f4620              
    #include <p18f4620.inc>     ;Set processor to PIC18F4620
    #include <lcd.inc>			;Import LCD functions from LCD.asm
    #include <rtc_macros.inc>   ;Import RTC functions

;; CONFIGURATION BITS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		CONFIG OSC=INTIO67, FCMEN=OFF, IESO=OFF
		CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
		CONFIG WDT = OFF, WDTPS = 32768
		CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF, CCP2MX = PORTC
		CONFIG STVREN = ON, LVP = OFF, XINST = OFF
		CONFIG DEBUG = OFF
		CONFIG CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
		CONFIG CPB = OFF, CPD = OFF
		CONFIG WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
		CONFIG WRTB = OFF, WRTC = OFF, WRTD = OFF
		CONFIG EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
		CONFIG EBTRB = OFF

;; DEFINITIONS AND DATA ALLOCATIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
cblock 0x0
	temp_w						;Temporary variables to store WREG and STATUS
	temp_status                 ;during interrupt code
    Machine_state				;Machine_state used for PIC state machine
    ;bit 0 - main menu
    ;bit 1 - machine is in operation
    ;bit 2 - motor is running
    ;bit 3 - displaying operation log at end of operation
    ;bit 4 - choosing permanent log to display
    ;bit 5 - end of file reached when viewing permanent logs
    ;bit 6 - displaying a single permanent log
    Motor_Step					;Sets the state of the motor
    Turn_counter
    FlashlightCounter
    Step_Counter				;Three counters used as named
    Zero						;Register holding binary value 00000000
    BCDL						;
    BCDH						;
    BCDU						;
    COUNT						;Four variables used for the BinToBCD macro
    BCDDISPLAY					;Temporary variable used to display BCD values
    TIMEH						;Variables used to count when a minute has
    TIMEL						;elapsed in real time
    StartTime           		;Time when operation started
    StopTime					;Time when operation finished
    RunStateBits                ;Bits used during the operation of the machine
    ;bit 0: high if operation has lasted longer than a minute
    ;bit 1: high if a flashlight has been detected
    ;bit 2: used to delay getting data from the photoresistor
    DataDisplay					;Controls what is being displayed from the logs
    Key_Pressed					;Determines what key on keypad was pressed
    ResistorDelay				;Delay for getting data from photoresistor
    Resistor1
    Resistor2
    Resistor3
    Resistor4
    Resistor5
    Resistor6					;A2D value of photoresistors
    Flashlight1
    Flashlight2
    Flashlight3
    Flashlight4
    Flashlight5
    Flashlight6
    Flashlight7
    Flashlight8
    Flashlight9					;Status of the flashlight in matching slot
    EEOffset					;Used to diplay the permanent logs
    NumTests					;Number of logs stored in permanent logs
    NumZeros					;Number of low signals from photoresistors
    ZeroBits    				;bit x is high if matching Resistor x is low
    ZeroTest					;Used in sensing logic
    PRSumH
    PRSumL						;Sum of all photoresistor values
endc

;; ENTRY VECTORS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    org         0x0000
	goto        Mainline			;Goto start of program

	org         0x08				;High priority ISR
    call        ISR_Key				;Calls keypad ISR which is the only high
									;priority ISR
    movf        temp_status,w
	movwf       STATUS
    movf        temp_w,w			;Restores WREG and STATUS
	retfie

	org         0x18				;Low priority ISR
    movwf       temp_w
	movf        STATUS,w
	movwf       temp_status			;Stores WREG and STAUS
    call        ISR_Timer0			;Calls timer0 ISR which is the only low
									;priority ISR
    movf        temp_status,w
	movwf       STATUS
    movf        temp_w,w			;Restores WREG and STATUS
	retfie

;; TABLES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Used to display messages on LCD
Line1Start
	db		"1:Start 2:Logs", 0
StartMsg
	db		"In Operation", 0
Number
    db      "0123456789", 0
DoneLine1
    db      "Operation Done!", 0
DoneLine2
	db		"1:Main 2:Results", 0
RetMsg
    db      "1:Main 2:Next", 0
DateMsg
    db      "Tested: ", 0
TimeMsg
    db      "Test Time: ",0
NumberMsg
    db      "No. of Lights: ", 0
FlashlightMsg
    db      "Slot", 0
EmptyMsg
    db      "Empty", 0
LED
    db      "3-LED Fail", 0
LEDD
    db      "2-LED Fail", 0
LEDDD
    db      "1-LED Fail", 0
LEDDDD
    db      "Pass", 0
NoLogMsg
    db      "End of Logs",0
FinalRetMsg
    db      "1:Main",0
LogRetMsg
    db      "2:Next 3:View",0

;; MACROS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;Display macro
;input: label - the name of the table that needs to be displayed
;output: displays message to LCD
Display macro label
    local       Again
	movlw		upper label
    movwf		TBLPTRU
	movlw		high label
	movwf		TBLPTRH
	movlw		low label
	movwf		TBLPTRL				;Initializes table pointer
	tblrd*
	movf		TABLAT, W           ;Reads table value and stores in WREG
Again
    call        WR_DATA
	tblrd+*
	movf		TABLAT, W
	bnz         Again				;Loop writes to LCD until a 0 is reached
endm

;BCD_Display macro
;input: num - a register holding a BCD value
;output: displays the BCD number (0-9) to the LCD
BCD_Display macro num
    local       Overflow1
    local       Overflow2
    local       Write

    movlw		upper Number
    movwf		TBLPTRU
	movlw		high Number
	movwf		TBLPTRH				;Initializes table pointer

    movlw		low Number           
    bcf         STATUS, OV
    addwf       num,w				;Offset Table Pointer by BCD
    movwf		TBLPTRL
    btfsc       STATUS, OV
    call        Overflow1
    goto        Write

Overflow1
    bcf         STATUS, OV
    incf        TBLPTRH				;Add if overflow occurs
    btfsc       STATUS, OV
    call        Overflow2
    return

Overflow2
    incf        TBLPTRU				;Add if overflow occurs
    return

Write
    tblrd*
	movf		TABLAT, W
    call        WR_DATA				;Reads table value and writes BCD to LCD
endm

;BinToBCD macro
;input: BINL - a register holding the low half of the binary number to be converted
;		BINH - a register holding the high half of the binary number to be converted
;output: BCDU,BCDH,BCDL - three registers holding the BCD value after conversion
;source: http://www.microchip.com/forums/m322713.aspx
BinToBCD macro BINL, BINH
    local       ConvertBit
    clrf        BCDL
    clrf        BCDH
    clrf        BCDU
    movlw       d'16'
    movwf       COUNT
    bcf         STATUS,C                ;Personal debug addition
ConvertBit
    rlcf        BINL,F
    rlcf        BINH,F
    movf        BCDL,W
    addwfc      BCDL,W 
    daw
    movwf       BCDL
    movf        BCDH,W
    addwfc      BCDH,W
    daw
    movwf       BCDH
    rlcf        BCDU,F
    decfsz      COUNT
    bra         ConvertBit
endm

;BinToBCD macro
;input: bcd - a register holding the BCD number to be converted
;output: WREG - the binary value of the number
;source: http://www.piclist.com/techref/microchip/math/radix/bp2b-2d8b.htm
BCDToBin macro  bcd
    swapf   bcd, W
    andlw   0x0F            ; W= tens
    rlncf   WREG, W         ; W= 2*tens
    subwf   bcd, F          ; 16*tens + ones - 2*tens
    subwf   bcd, F          ; 14*tens + ones - 2*tens
    subwf   bcd, W          ; 12*tens + ones - 2*tens
endm

;; MAIN ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Mainline
    call        Init				;Start of program
    call        LCD_Init
    call        RTC_init
    call        ISR_init
    call        EEPROM_Init			;Initializes required components
    banksel     Line1Start
    call        DispMainMenu		;Displays the main menu

MainMenu
    btfss       Machine_state,1
    goto        MainMenu			;Infinite loop if nothing happens
    goto        Start				;Starts the machine if user requests

;; INITIALIZATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init
    movlw       B'01110000'
    movwf		OSCCON              ;Set internal oscillator to 32MHz = 8MHz per
    bsf         OSCTUNE, 6          ;clock cycle. Enables scaler

    movlw       b'11111111'
    movwf       TRISB               ;Set PORTB to input - Keypad
    movlw       b'00011000'         ;Set PORTC 0:2 and 5:7 as output (0:2 and
    movwf       TRISC               ;5 for motor) RC3:4 are inputs for RTC
	clrf		TRISD               ;Set PORTD to output for LCD
    movlw       b'00111111'
    movwf       TRISA               ;RA0:1 input for sensors, rest output
    movlw       b'00000111'
    movwf       TRISE

    clrf        LATC
    clrf        LATD
    clrf        LATA

    movlw       b'00010111'
    movwf       ADCON1              ;Sets AN0:12 to analog, Uses Vref+
    movlw       b'00100110'         ;Sets 2Tad and 16Tosc and left justified
    movwf       ADCON2

    ;Initialize Timer for RTC display
    ;Ticks once every 60.014592 seconds.
    movlw       b'01001111'
    movwf       TIMEL
    movlw       b'00001110'
    movwf       TIMEH

    movlw       b'00000000'
    movwf       Zero				;Sets Zero Register to 0
    return

RTC_init
    call 	   i2c_common_setup		;Initializes RTC
    return

;Enables RB1 pin, Sets RB1 to high and TMR0 to low priority
ISR_init
    bsf         RCON,IPEN
	bsf         INTCON,GIEH

    bcf         INTCON3,INT1IF
    bsf         INTCON3,INT1IE
    bsf         INTCON3,INT1IP

    bcf         INTCON,TMR0IF
    bsf         INTCON,TMR0IE
    bcf         INTCON2,TMR0IP

;Sets Timer0 to 16bit and configures Timer0, prescales 1:8. At 16 bit, the timer
;overflows every 122Hz. At 1:2, it overflows every 61.035Hz
    movlw       b'00000000'
    movwf       T0CON
    bsf         INTCON,GIEL
    bsf         T0CON,TMR0ON   
	return

;Sets required EEPROM data
EEPROM_Init
    clrf        EEADRH
    clrf        EEADR
    call        EEPROM_Read
    bcf         STATUS,Z
    movf        EEDATA,w
    andlw       0x0F
    btfsc       STATUS,Z
    goto        EEPROM_Init2
    clrf        EEDATA
    call        EEPROM_Write
EEPROM_Init2
    incf        EEADR
    call        EEPROM_Read
    movlw       d'15'
    cpfsgt      EEDATA
    return
    clrf        EEDATA
    call        EEPROM_Write
    return

;; INTERRUPT SERVICES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ISR_Key
    movwf       temp_w
	movf        STATUS,w
	movwf       temp_status			;Store WREG and STATUS
    bcf         INTCON3,INT1IF		;Clears the interrupt flag
	swapf       PORTB,W             ;Puts PORTB7:4 into W3:0
    andlw       0x0F                ;W: 0000XXXX
    movwf       Key_Pressed
    incf        Key_Pressed,f
    dcfsnz      Key_Pressed,f       ;Decrement working reg, skip next line if 0
    goto        Pressed1			;Goes here if 1 on keypad was pressed
    dcfsnz      Key_Pressed,f
    goto        Pressed2			;Goes here if 2 on keypad was pressed
    dcfsnz      Key_Pressed,f
    goto        Pressed3			;Goes here if 3 on keypad was pressed
    return 

ISR_Timer0
    bcf         INTCON,TMR0IF		;Clears interrupt flag
    btfsc       Machine_state,2
    call        Step				;Goes here if the motor is running
    btfss       RunStateBits,2
    call        RDelayDec			;Goes here if delaying for photoresistor
    call        Time				;Decrements the TimeH:L pair
    return

;; START ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;The code that runs when the user requests the machine to start
Start
    movlw       b'01001111'
    movwf       TIMEL
    movlw       b'00001111'
    movwf       TIMEH
    bcf         RunStateBits,0		;Initializes minute timer
    rtc_read	0x00
    movf        0x75,w
    andlw       b'01111111'
    movwf       StartTime			;Obtains start time
    movlw       b'00000000'
    movwf       FlashlightCounter	;Clears Flashlight Counter

    call        Clear_LCD
    call        Line1
    Display     StartMsg			;Displays in operation message

    clrf        Turn_counter		;Clears Turn Counter
	
;Rotates the motor forward and gets data
;Loops nine times for each slot
Forward
    movlw       b'00001010'
    incf        Turn_counter
    cpfslt      Turn_counter		;Checks if it has turned 9 turns, if it has
    goto        Back				;start going backwards
    call        MotorForward        ;Moves the motor forward
    call        MotorForCorrect
    call        MotorBackCorrect	;Corrects for slipping
    call        GetIRData           ;Checks for presence of flashlight
    movf        Turn_counter,w
    addlw       0x1A
    movwf       FSR1L
    movlw       0x0
    movwf       INDF1
    clrf        FSR1L
    btfss       RunStateBits,1
    goto        Forward
    call        GetPRData			;Gets data from photoresistors
    call        GetStatus			;Sets status of flashlight being exmained
    goto        Forward

;Turns the motor back to the starting position
Back
    dcfsnz      Turn_counter
    goto        MotorZero
    call        MotorBackward
    goto        Back

MotorZero
    call        MotorZeroCorrect	;Corrects the motor position more finely
    clrf        LATC				;Clear motor output
    rtc_read	0x00
    movf        0x75,w
    andlw       b'01111111'
    movwf       StopTime			;Obtain ending time

    BCDToBin    StopTime
    movwf       StopTime
    BCDToBin    StartTime
    subwf       StopTime,w
    btfsc       STATUS,N
    addlw       b'00111100'
    btfsc       STATUS,Z
    addlw       b'00111100'
    btfsc       RunStateBits,0
    addlw       b'00111100'			;Finds the elapsed operation time and stores
    movwf       StopTime			;the result in StopTime
	
    clrf        Machine_state
    bsf         Machine_state,3		;Set state to displaying log
    call        Clear_LCD
    call        Line1
    Display     DoneLine1
    call        Line2
    Display     DoneLine2			;Displays the done operation message
    clrf        DataDisplay
    call        LogOperation		;Stores the log in permanent memory

    goto        MainMenu			;Returns to infinite loop / end of code

;; Keypad Interrupt Routines ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Pressed1 is run if the user pressed 1 on the keypad
Pressed1
    btfss       Machine_state,0
    goto        CheckMachineState	;If not in idle state, goto second check
    clrf        Machine_state
    bsf         Machine_state, 1	;If machine is idle, set to run state
    return

CheckMachineState
    btfss       Machine_state,1
    goto        DispMainMenu		;If not running, goto back to main menu
    return

;Pressed2 is run if the user pressed 1 on the keypad
Pressed2
    btfsc       Machine_state,0
    goto        SetLogs				;If idle => start displaying permanent logs
    btfsc       Machine_state,3
    goto        SetResults			;If at end of operation => display log
    btfsc       Machine_state,4
    goto        NextLog				;If viewing permanent logs => Show next stored log
    btfsc       Machine_state,6
    goto        DisplayResults		;If viewing single permanent logs => display log data
    return

;Pressed3 is run if the user pressed 1 on the keypad
Pressed3
    btfsc       Machine_state,4
    goto        DisplayLogData		;If in permeant logs => display data
    return
	
;; Motor Routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Step is run when Timer0 overflows and machine state is set accordingly
Step
    dcfsnz      Step_Counter
    call        Stop_Motor			;If motor ran enough => stops motor

    btfsc       Motor_Step,0
    goto        Step1
    btfsc       Motor_Step,1
    goto        Step2
    btfsc       Motor_Step,2
    goto        Step3
    goto        Step4				;Sets the next appropriate motor step output
Step1
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00100001'
    movwf       LATC                ;Set motor to first step
    bcf         Motor_Step,0
    btfsc       Motor_Step,7
    goto        SetStep4			;Sets appropriate next step depending on
    goto        SetStep2			;whether going forwards or backwards
Step2
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00000101'
    movwf       LATC                ;Set motor to second step
    bcf         Motor_Step,1
    btfsc       Motor_Step,7
    goto        SetStep1			;Sets appropriate next step depending on
    goto        SetStep3			;whether going forwards or backwards
Step3
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00000110'
    movwf       LATC                ;Set motor to thrid step
    bcf         Motor_Step,2
    btfsc       Motor_Step,7
    goto        SetStep2			;Sets appropriate next step depending on
    goto        SetStep4			;whether going forwards or backwards
Step4
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00100010'
    movwf       LATC                ;Set motor to fourth step
    bcf         Motor_Step,3
    btfsc       Motor_Step,7
    goto        SetStep3			;Sets appropriate next step depending on
    goto        SetStep1			;whether going forwards or backwards
SetStep1
    bsf         Motor_Step,0
    goto        Step_Done
SetStep2
    bsf         Motor_Step,1
    goto        Step_Done
SetStep3
    bsf         Motor_Step,2
    goto        Step_Done
SetStep4
    bsf         Motor_Step,3
    goto        Step_Done			;Sets the appropriate step as required
Step_Done
    return

MotorForward
    movlw       d'50'
    movwf       Step_Counter		;Sets the appropriate number of steps
    bsf         Motor_Step,7        ;Sets the motor to forward direction
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor			;Waits for motor to finish turning

MotorBackward
    movlw       d'50'
    movwf       Step_Counter		;Sets the appropriate number of steps
    bcf         Motor_Step,7        ;Sets the motor to backward direction
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor			;Waits for motor to finish turning

MotorForCorrect
    movlw       d'16'
    movwf       Step_Counter		;Sets the appropriate number of steps
    bsf         Motor_Step,7        ;Sets the motor to forward direction
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor			;Waits for motor to finish turning
MotorBackCorrect
    movlw       d'7'
    cpfseq      Turn_counter		;If 7th turn => needs a bigger correction
    goto        BackCorrect15
    goto        BackCorrect17
BackCorrect15
    movlw       d'15'
    movwf       Step_Counter		;Sets the appropriate number of steps
    goto        BackCorrectEnd
BackCorrect17
    movlw       d'18'
    movwf       Step_Counter		;Sets the appropriate number of steps
    goto        BackCorrectEnd
BackCorrectEnd
    bcf         Motor_Step,7        ;Sets the motor to backward direction
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor			;Waits for motor to finish turning

MotorZeroCorrect
    movlw       d'6'
    movwf       Step_Counter		;Sets the appropriate number of steps
    bcf         Motor_Step,7        ;Sets the motor to backward direction
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor			;Waits for motor to finish turning

WaitMotor
    btfss       Motor_Step,6        ;Wait for Motor
    goto        WaitMotor
    bcf         Machine_state,2		;Clears the machine state so that motor is no longer running
    return

Stop_Motor
    clrf        LATC
    bsf         Motor_Step,6		;Stops the motor
    return

;; Logs ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Prepares the machine to display the permanent logs
SetLogs
    clrf        Machine_state
    bsf         Machine_state,4		;Set the required machine state
    clrf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       EEOffset			;Read where the data is stored
    clrf        EEADR
    incf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       NumTests			;Read number of stored logs
    movf        NumTests,w
    cpfseq      Zero				;If no logs, display end of logs message
    goto        DateDisplay			;else display stored logs
    goto        EndofLogs

;Displays the end of logs message if no more permanent logs
EndofLogs
    clrf        Machine_state
    bsf         Machine_state,5
    call        Clear_LCD
    call        Line1
    Display     NoLogMsg
    call        Line2
    Display     FinalRetMsg
    return

;Shows the next stored log
NextLog
    dcfsnz      NumTests
    goto        EndofLogs			;If no more logs, display end of logs message
    movf        EEOffset,w
    bcf         STATUS,Z
    addlw       b'11110000'
    btfsc       STATUS,Z
    addlw       b'11110000'
    movwf       EEOffset			;Find where the next log is stored
    call        DateDisplay			;Display that log
    return

;Display the data of the log the user wants to view
DisplayLogData
    movlw       d'3'
    movwf       DataDisplay
    clrf        Machine_state
    bsf         Machine_state,6		;set required state
    goto        DisplayResults

;Displays the return message
LogRetDisplay
    Display     LogRetMsg
    return

;Displays the results at the end of the operation
SetResults
    clrf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       EEOffset
    goto        DisplayResults

;Display the required information
DisplayResults
    movlw       d'2'
    cpfsgt      DataDisplay
    goto        DateDisplay			;Display the date first
    movlw       d'3'
    cpfsgt      DataDisplay
    goto        TimeDisplay			;Display the time of operation
    movlw       d'4'
    cpfsgt      DataDisplay
    goto        NumberDisplay		;Display the number of flashlights
    goto        FlashlightDisplay	;Display the state of each flashlight

DateDisplay
    call        Clear_LCD
    call        Line1
    Display     DateMsg
    movf        EEOffset,w
    addlw       0x1
    movwf       EEADR
    call        EEPROM_Read
    movf        EEDATA,w			;Reads day of operation
    movwf       BCDL
    call        BCDLDISPLAY			;Displays day
    movlw       "/"
    call        WR_DATA
    movf        EEOffset,w
    addlw       0x2
    movwf       EEADR
    call        EEPROM_Read
    movf        EEDATA,w			;Reads month of operation
    movwf       BCDL
    call        BCDLDISPLAY			;Displays month
    movlw       "/"
    call        WR_DATA
    movf        EEOffset,w
    movwf       EEADR
    call        EEPROM_Read
    movf        EEDATA,w			;Reads year of operation
    movwf       BCDL
    call        BCDLDISPLAY			;Displays year
    call        Line2
    btfss       Machine_state,3
    goto        LogRetDisplay
    Display     RetMsg    
    incf        DataDisplay
    incf        DataDisplay
    incf        DataDisplay			;Displays return message on second line
    return

TimeDisplay
    call        Clear_LCD
    call        Line1
    Display     TimeMsg
    movf        EEOffset,w
    addlw       0x3
    movwf       EEADR
    call        EEPROM_Read			;Reads the time taken for the operation
    BinToBCD    EEDATA,Zero
    call        BCDLDISPLAY			;Displays the time
    movlw       "s"
    call        WR_DATA
    call        Line2
    Display     RetMsg				;Displays the return message
    incf        DataDisplay
    return

NumberDisplay
    call        Clear_LCD
    call        Line1
    Display     NumberMsg
    movf        EEOffset,w
    addlw       0x4
    movwf       EEADR
    call        EEPROM_Read			;Reads the number of flashlights tested
    BinToBCD    EEDATA,Zero
    movf        BCDL,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY			;Displays the number
    call        Line2
    Display     RetMsg				;Displays the return message
    incf        DataDisplay
    return

FlashlightDisplay
    movlw       d'14'
    cpfslt      DataDisplay
    return
    call        Clear_LCD
    call        Line1
    Display     FlashlightMsg
    movf        DataDisplay,w
    addlw       b'11111100'
    movwf       BCDDISPLAY
    BinToBCD    BCDDISPLAY,Zero		
    movf        BCDL,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY			;Displays the slot number
    movlw       " "
    call        WR_DATA
    movf        EEOffset,w
    addwf       DataDisplay,w
    movwf       EEADR
    call        EEPROM_Read			;Reads state of flashlight
    movf        EEDATA,w
    movwf       Flashlight1
    btfsc       Flashlight1,7		;Displays empty or state of flashlight
    goto        StatusDisplay
    goto        EmptyDisplay

;Displays the state of the flashlight (3,2,1-LED Fail or Pass)
StatusDisplay
    bcf         Flashlight1,7
    incf        Flashlight1,f
    dcfsnz      Flashlight1,f
    goto        DisplayA
    dcfsnz      Flashlight1,f
    goto        DisplayB
    dcfsnz      Flashlight1,f
    goto        DisplayC
    goto        DisplayD
   
DisplayA
    Display     LED					;Display 3-LED Fail
    goto        RetDisplay
DisplayB
    Display     LEDD				;Display 2-LED Fail
    goto        RetDisplay
DisplayC
    Display     LEDDD				;Display 1-LED Fail
    goto        RetDisplay
DisplayD
    Display     LEDDDD				;Display Pass
    goto        RetDisplay
EmptyDisplay
    Display     EmptyMsg			;Display Empty
    goto        RetDisplay
RetDisplay
    incf        DataDisplay
    call        Line2
    movlw       d'13'
    cpfsgt      DataDisplay
    goto        RetDisplay1
    Display     FinalRetMsg			;Displays the return message
    return
RetDisplay1
    Display     RetMsg				;Displays the return message
    return

;; General Subroutines ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
GetIRData
    bcf         RunStateBits,2
    movlw       b'00111100'
    movwf       ResistorDelay		;Set the resistor delay to 1 second
RDelay
    btfss       RunStateBits,2
    goto        RDelay				;Wait for resistor delay
    bcf         RunStateBits,1
    movlw       b'00000001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2					;Get A2D value from IR Sensor
    movlw       b'00001010'
    cpfsgt      ADRESH
    bsf         RunStateBits,1		;Set to indicate there is no light
    return

GetPRData
    incf        FlashlightCounter	;Increase flashlight counter
    movlw       b'00000101'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor1			;Gets A2D data for Resistor 1

    movlw       b'00001001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor2			;Gets A2D data for Resistor 2

    movlw       b'00010001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor3			;Gets A2D data for Resistor 3
   
    movlw       b'00010101'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor4			;Gets A2D data for Resistor 4

    movlw       b'00011001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor5			;Gets A2D data for Resistor 5

    movlw       b'00011101'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor6			;Gets A2D data for Resistor 6
    return

;The sensing logic that determines the state of the flashlight
GetStatus
    clrf        NumZeros
    clrf        ZeroBits
    movlw       d'8'
    cpfsgt      Resistor1
    bsf         ZeroBits,0
    cpfsgt      Resistor2
    bsf         ZeroBits,1
    cpfsgt      Resistor3
    bsf         ZeroBits,2
    cpfsgt      Resistor4
    bsf         ZeroBits,3
    cpfsgt      Resistor5
    bsf         ZeroBits,4
    cpfsgt      Resistor6           ;See any of the Resistor values are less
    bsf         ZeroBits,5          ;than 0.20V

    btfsc       ZeroBits,0
    incf        NumZeros
    btfsc       ZeroBits,1
    incf        NumZeros
    btfsc       ZeroBits,2
    incf        NumZeros
    btfsc       ZeroBits,3
    incf        NumZeros
    btfsc       ZeroBits,4
    incf        NumZeros
    btfsc       ZeroBits,5
    incf        NumZeros            ;Find the number of low signals

    movlw       d'5'
    cpfslt      NumZeros
    goto        Set0LED             ;If there are 5-6 zeros => 3-LED Fail

    movf        ZeroBits,w
    andlw       b'00000111'
    movwf       ZeroTest
    movlw       b'00000111'
    cpfseq      ZeroTest            ;Test if any three zeros are adjacent
    goto        Skip1               ;If no, test next set of three
    goto        SumPR               ;If yes, find sum of resistor values
Skip1
    movf        ZeroBits,w
    andlw       b'00001110'
    movwf       ZeroTest
    movlw       b'00001110'
    cpfseq      ZeroTest            ;Test if any three zeros are adjacent
    goto        Skip2               ;If no, test next set of three
    goto        SumPR               ;If yes, find sum of resistor values
Skip2
    movf        ZeroBits,w
    andlw       b'00011100'
    movwf       ZeroTest
    movlw       b'00011100'
    cpfseq      ZeroTest            ;Test if any three zeros are adjacent
    goto        Skip3               ;If no, test next set of three
    goto        SumPR               ;If yes, find sum of resistor values
Skip3
    movf        ZeroBits,w
    andlw       b'00111000'
    movwf       ZeroTest
    movlw       b'00111000'
    cpfseq      ZeroTest            ;Test if any three zeros are adjacent
    goto        Skip4               ;If no, test next set of three
    goto        SumPR               ;If yes, find sum of resistor values
Skip4
    movf        ZeroBits,w
    andlw       b'00110001'
    movwf       ZeroTest
    movlw       b'00110001'
    cpfseq      ZeroTest            ;Test if any three zeros are adjacent
    goto        Skip5               ;If no, test next set of three
    goto        SumPR               ;If yes, find sum of resistor values
Skip5
    movf        ZeroBits,w
    andlw       b'00100011'
    movwf       ZeroTest
    movlw       b'00100011'
    cpfseq      ZeroTest            ;Test if any three zeros are adjacent
    goto        Skip6               ;If no, goto next test
    goto        SumPR               ;If yes, find sum of resistor values

Skip6
    btfsc       ZeroBits,0          ;Check each individual resistor
    goto        TestPR1             ;If resistor value is low, we need to check
    btfsc       ZeroBits,1          ;the adajacent resistor values
    goto        TestPR2
    btfsc       ZeroBits,2
    goto        TestPR3
    btfsc       ZeroBits,3
    goto        TestPR4
    btfsc       ZeroBits,4
    goto        TestPR5
    btfsc       ZeroBits,5
    goto        TestPR6
    goto        Set3LED             ;If no lows, then set Pass

SumPR
    clrf        PRSumH
    clrf        PRSumL
    movf        Resistor1,w
    bcf         STATUS,C
    addwf       PRSumL,f
    btfsc       STATUS,C
    incf        PRSumH
    movf        Resistor2,w
    bcf         STATUS,C
    addwf       PRSumL,f
    btfsc       STATUS,C
    incf        PRSumH
    movf        Resistor3,w
    bcf         STATUS,C
    addwf       PRSumL,f
    btfsc       STATUS,C
    incf        PRSumH
    movf        Resistor4,w
    bcf         STATUS,C
    addwf       PRSumL,f
    btfsc       STATUS,C
    incf        PRSumH
    movf        Resistor5,w
    bcf         STATUS,C
    addwf       PRSumL,f
    btfsc       STATUS,C
    incf        PRSumH
    movf        Resistor6,w
    bcf         STATUS,C
    addwf       PRSumL,f
    btfsc       STATUS,C            ;Find the sum of all resitors and store the
    incf        PRSumH              ;value in PRSumH:L

    movlw       d'2'
    cpfslt      PRSumH
    goto        Set2LED             ;If PRSum>=512, Set 1-LED fail
    movlw       d'0'
    cpfsgt      PRSumH
    goto        SumPRL
    movlw       d'44'               ;If PRSum>=300, Set 1-LED fail
    cpfslt      PRSumL              ;else if 75<sum<300, set 2-LED fail
    goto        Set1LED
    goto        Set2LED

SumPRL
    movlw       d'75'               ;If PRSum < 75, Set 3-LED fail
    cpfslt      PRSumL
    goto        Set1LED
    goto        Set0LED

;Test each individual photoresistor. Find the values of the two resistors on
;either side, if both resistor values are greater than 100, then set Pass
;else set 1-LED fail
TestPR1
    movlw       d'100'
    cpfsgt      Resistor6
    goto        Set2LED
    cpfsgt      Resistor2
    goto        Set2LED
    goto        Set3LED
TestPR2
    movlw       d'100'
    cpfsgt      Resistor1
    goto        Set2LED
    cpfsgt      Resistor3
    goto        Set2LED
    goto        Set3LED
    return
TestPR3
    movlw       d'100'
    cpfsgt      Resistor2
    goto        Set2LED
    cpfsgt      Resistor4
    goto        Set2LED
    goto        Set3LED
    return
TestPR4
    movlw       d'100'
    cpfsgt      Resistor3
    goto        Set2LED
    cpfsgt      Resistor5
    goto        Set2LED
    goto        Set3LED
    return
TestPR5
    movlw       d'100'
    cpfsgt      Resistor4
    goto        Set2LED
    cpfsgt      Resistor6
    goto        Set2LED
    goto        Set3LED
    return
TestPR6
    movlw       d'100'
    cpfsgt      Resistor5
    goto        Set2LED
    cpfsgt      Resistor1
    goto        Set2LED
    goto        Set3LED             ;End of individual PR testing
    return

Set0LED
    movf        Turn_counter,w
    addlw       0x1A
    movwf       FSR1L
    movlw       b'10000000'
    movwf       INDF1
    clrf        FSR1L				;Set 3-LED Fail to appropriate Flashlight
    return
Set1LED
    movf        Turn_counter,w
    addlw       0x1A
    movwf       FSR1L
    movlw       b'10000001'
    movwf       INDF1
    clrf        FSR1L				;Set 2-LED Fail to appropriate Flashlight
    return
Set2LED
    movf        Turn_counter,w
    addlw       0x1A
    movwf       FSR1L
    movlw       b'10000010'
    movwf       INDF1
    clrf        FSR1L				;Set 1-LED Fail to appropriate Flashlight
    return
Set3LED
    movf        Turn_counter,w
    addlw       0x1A
    movwf       FSR1L
    movlw       b'10000011'
    movwf       INDF1
    clrf        FSR1L				;Set Pass to appropriate Flashlight
    return

RDelayDec
    dcfsnz      ResistorDelay
    bsf         RunStateBits,2		;Decrements the resistor delay, exits if done
    return

;Takes the BCD value (00-99) in BCDL and displays it to the LCD
BCDLDISPLAY
    swapf       BCDL,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    movf        BCDL,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    return

;Run everything during timer0 interrupt
Time
    decfsz      TIMEL,f
    return
    decfsz      TIMEH,f
    return							;Decrement the minute timer
    
    movlw       b'01001111'
    movwf       TIMEL
    movlw       b'00001110'
    movwf       TIMEH				;Set to 1 minute again

    call        Line1
    btfsc       Machine_state,0
    call        show_RTC			;If on main menu, update time

    btfsc       Machine_state,1
    bsf         RunStateBits,0		;If in operation, mark that one minute has elapsed
    return

;Displays and returns machine to the main menu
DispMainMenu
    clrf        Machine_state
    bsf         Machine_state,0
    call        Clear_LCD
    call        Line1
    call        show_RTC
    call        Line2
    Display     Line1Start
    return

;Increases the number of number of logs in permanent logs
IncfNumTests
    incf        EEDATA
    call        EEPROM_Write
    return

;Log the operation data to permanent storage
LogOperation
    clrf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    bcf         STATUS,Z
    addlw       d'16'
    btfsc       STATUS,Z
    addlw       d'16'
    movwf       EEOffset			;Get location of data storage
    incf        EEADR
    call        EEPROM_Read
    movlw       d'14'
    cpfsgt      EEDATA
    call        IncfNumTests
;Stores Year of operation
    rtc_read    0x06
    movf        0x75,w
    movwf       EEDATA
    movf        EEOffset,w
    movwf       EEADR
    call        EEPROM_Write
;Store Month of Operation
    rtc_read    0x05
    movf        0x75,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x1
    movwf       EEADR
    call        EEPROM_Write
;Store Day of Operation
    rtc_read    0x04
    movf        0x75,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x2
    movwf       EEADR
    call        EEPROM_Write
;Store time of Operation
    movf        StopTime,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x3
    movwf       EEADR
    call        EEPROM_Write
;Store number of flashlights tested
    movf        FlashlightCounter,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x4
    movwf       EEADR
    call        EEPROM_Write
;Store FlashlightData
    movf        Flashlight1,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x5
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight2,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x6
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight3,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x7
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight4,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x8
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight5,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x9
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight6,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xA
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight7,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xB
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight8,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xC
    movwf       EEADR
    call        EEPROM_Write
    movf        Flashlight9,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xD
    movwf       EEADR
    call        EEPROM_Write
;move new offset into EEPROM
    clrf        EEADR
    movf        EEOffset,w
    movwf       EEDATA
    call        EEPROM_Write
    return
    
;Reads values from RTC and writes to LCD
;source: http://www.pml4all.org/
show_RTC
    ;Get hour
    rtc_read	0x02		;Read Address 0x02 from DS1307---hour
	movf        0x77,w
	call        WR_DATA
	movf        0x78,w
	call        WR_DATA
	movlw       ":"
	call        WR_DATA

    ;Get minute
	rtc_read	0x01		;Read Address 0x01 from DS1307---min
	movf        0x77,w
	call        WR_DATA
	movf        0x78,w
	call        WR_DATA
	movlw		" "
	call        WR_DATA

    ;Get month
	rtc_read	0x05		;Read Address 0x05 from DS1307---month
	movf        0x77,w
	call        WR_DATA
	movf        0x78,w
	call        WR_DATA
	movlw       "/"
	call        WR_DATA
    
    ;Get day
	rtc_read	0x04		;Read Address 0x04 from DS1307---day
	movf	0x77,w
	call	WR_DATA
	movf	0x78,w
	call	WR_DATA
    movlw	"/"
	call	WR_DATA

	;Get year
	movlw	"2"				;First line shows 20**/**/**
	call	WR_DATA
	movlw	"0"
	call	WR_DATA
	rtc_read	0x06		;Read Address 0x06 from DS1307---year
	movf	0x77,w
	call	WR_DATA
	movf	0x78,w
	call	WR_DATA
	return

;Reads the value in EEPROM pointed to by the appropriate registers
;source: PIC18F4620 datasheet
EEPROM_Read
    BCF EECON1, EEPGD ; Point to DATA memory
    BCF EECON1, CFGS ; Access EEPROM
    BSF EECON1, RD ; EEPROM Read
	return

;Writes the value in EEDATA to the location pointed to by EEADR
;source: PIC18F4620 datasheet
EEPROM_Write
    BCF EECON1, EEPGD ; Point to DATA memory
    BCF EECON1, CFGS ; Access EEPROM
    BSF EECON1, WREN ; Enable writes
    MOVLW 55h ;
    MOVWF EECON2 ; Write 55h
    MOVLW 0AAh ;
    MOVWF EECON2 ; Write 0AAh
    BSF EECON1, WR ; Set WR bit to begin write
    BTFSC EECON1, WR ; Wait for write to complete
    BRA $-2
    BCF EECON1, WREN ; Disable writes on write complete (EEIF set)
	return
end