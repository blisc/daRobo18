    list p=18f4620              ; list directive to define processor
    #include <p18f4620.inc>     ; processor specific variable definitions
    #include <lcd.inc>			;Import LCD control functions from lcd.asm
    #include <rtc_macros.inc>

;;Default Internal oscialltor is 1MHz; To select Internal set OCS to INTIO67
;;Can be changed via OSCCON register and PPLLEN bit <OSCTUNE[6]>


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
	temp_w
	temp_status
    Machine_state
    ;bit 0 - main menu
    ;bit 1 - machine is runing
    ;bit 2 - motor
    ;bit 3 - displaying a single log
    ;bit 4 - choosing log
    ;bit 5 - end of logs
    ;bit 6 - displayings single log in EEPROM
    Motor_Step
    Turn_counter
    FlashlightCounter
    Step_Counter
    Zero
    BCDL
    BCDH
    BCDU
    COUNT
    BCDDISPLAY
    TIMEH
    TIMEL
    StartTime           ;0xF
    StopTime
    RunStateBits                
    ;bit 0: minutre overflow
    ;bit 1: presense of flashlight
    ;bit 2: photoresistor wait
    ;bit 3: error!!! in sensor logic
    DataDisplay
    ;0-14: what the log is displaying
    Key_Pressed
    ResistorDelay
    Resistor1
    Resistor2
    Resistor3
    Resistor4
    Resistor5
    Resistor6
    NumHigh
    NumMed              ;0x1C
    ;00 - No LED, 01 - 1, 10 - 2, 11 - 3
    Flashlight1
    Flashlight2
    Flashlight3
    Flashlight4
    Flashlight5
    Flashlight6
    Flashlight7
    Flashlight8
    Flashlight9
    EEOffset
    NumTests
    NumZeros
    ZeroBits    ;high is RX is zero
    ZeroTest
    PRSumH
    PRSumL
    ;State1
    ;State2
    ;State3
    ;State4
    ;State5
    ;State6
    ;Value1
    ;Value2
endc

;; ENTRY VECTORS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    org         0x0000
	goto        Mainline

	org         0x08				;high priority ISR
    call        ISR_Key
    movf        temp_status,w
	movwf       STATUS
    movf        temp_w,w
	retfie

	org         0x18				;low priority ISR
    movwf       temp_w
	movf        STATUS,w
	movwf       temp_status
    call        ISR_Timer0
    movf        temp_status,w
	movwf       STATUS
    movf        temp_w,w
	retfie

;; TABLES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
Display macro label
    local       Again
	movlw		upper label
    movwf		TBLPTRU
	movlw		high label
	movwf		TBLPTRH
	movlw		low label
	movwf		TBLPTRL
                                    ;Initializes table pointer
	tblrd*
	movf		TABLAT, W           ;reads table value
;Writes to LCD
Again
    call        WR_DATA
	tblrd+*
	movf		TABLAT, W
	bnz         Again
endm

BCD_Display macro num
    local       Overflow1
    local       Overflow2
    local       Write

;Initializes table pointer
    movlw		upper Number
    movwf		TBLPTRU
	movlw		high Number
	movwf		TBLPTRH

    ;Offset Table Pointer by BCD
    movlw		low Number           
    bcf         STATUS, OV
    addwf       num,w
    movwf		TBLPTRL
    btfsc       STATUS, OV
    call        Overflow1
    goto        Write

    ;Add if overflow occurs
Overflow1
    bcf         STATUS, OV
    incf        TBLPTRH
    btfsc       STATUS, OV
    call        Overflow2
    return

Overflow2
    incf        TBLPTRU
    return

Write
    tblrd*
	movf		TABLAT, W           ;reads table value
;Writes to LCD
    call        WR_DATA
endm

;Bin to BCD convertor from online forum
BinToBCD macro BINL, BINH
    local       ConvertBit
    clrf        BCDL
    clrf        BCDH
    clrf        BCDU
    movlw       d'16'
    movwf       COUNT
    bcf         STATUS,C                ;ADDED
ConvertBit
    rlcf        BINL,F      ;C
    rlcf        BINH,F
    movf        BCDL,W
    addwfc      BCDL,W      ;C,DC
    daw                     ;C
    movwf       BCDL
    movf        BCDH,W
    addwfc      BCDH,W
    daw
    movwf       BCDH
    rlcf        BCDU,F
    decfsz      COUNT
    bra         ConvertBit
endm

;Code taken from online
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
    call        Init
    call        LCD_Init
    call        RTC_init
    call        ISR_init
    call        EEPROM_Init
    banksel     Line1Start
    call        DispMainMenu

MainMenu
    btfss       Machine_state,1
    goto        MainMenu
    goto        Start

;; INITIALIZATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init
    movlw       B'01110000'
    movwf		OSCCON              ;Set internal oscillator to 32MHz = 8MHz per
    bsf         OSCTUNE, 6          ;clock cycle. Enables scaler

    movlw       b'11111111'
    movwf       TRISB               ;set PORTB to input - Keypad
    movlw       b'00011000'         ;Set PORTC 0:2 and 5:7 as output (0:2 and
    movwf       TRISC               ;5 for motor) RC3:4 are inputs for RTC
	clrf		TRISD               ;set PORTD to output for LCD
    movlw       b'00111111'
    movwf       TRISA               ;RA0:1 input for sensors, rest output
    movlw       b'00000111'
    movwf       TRISE

    clrf        LATC
    clrf        LATD
    clrf        LATA

    movlw       b'00010111'
    movwf       ADCON1              ;Sets AN0:12 to analog, Uses Vref
    movlw       b'00100110'         ;Sets 2Tad and 16Tosc and left justified
    movwf       ADCON2

    ;Initialize Timer for RTC display
    ;Ticks once every 60.014592 seconds.
    movlw       b'01001111'
    movwf       TIMEL
    movlw       b'00001110'
    movwf       TIMEH

    movlw       b'00000000'
    movwf       Zero
    return

RTC_init
;RTC SET-UP stuFFs
    call 	   i2c_common_setup
    ;rtc_resetAll
   ;call	   set_rtc_time
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

;Sets Timer0 to 16bit and configs Timer0, prescales 1:8. At 16 bit, the timer
;overflows every 122Hz. At 1:2, it overflows every 61.035Hz
    movlw       b'00000000'
    movwf       T0CON
    bsf         INTCON,GIEL
    bsf         T0CON,TMR0ON
    
	return

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
	movwf       temp_status
    bcf         INTCON3,INT1IF
    bsf         RunStateBits,3
	swapf       PORTB,W             ;Puts PORTB7:4 into W3:0
    andlw       0x0F                ;W: 0000XXXX
    movwf       Key_Pressed
    incf        Key_Pressed,f
    dcfsnz      Key_Pressed,f             ;decrement working reg, skip next line if 0
    goto        Pressed1
    dcfsnz      Key_Pressed,f
    goto        Pressed2
    dcfsnz      Key_Pressed,f
    goto        Pressed3
    return 

ISR_Timer0
    bcf         INTCON,TMR0IF
    btfsc       Machine_state,2
    call        Step
    btfss       RunStateBits,2
    call        RDelayDec
    call        Time
    return

;; START ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Start Routine
Start
    ;Initialize Timer
    movlw       b'01001111'
    movwf       TIMEL
    movlw       b'00001111'
    movwf       TIMEH
    bcf         RunStateBits,0
    rtc_read	0x00
    movf        0x75,w
    andlw       b'01111111'
    movwf       StartTime
    movlw       b'00000000'
    movwf       FlashlightCounter

    call        Clear_LCD
    call        Line1
    Display     StartMsg

    clrf        Turn_counter

Forward
    btfsc       Machine_state,0
    goto        Back
    movlw       b'00001010'
    incf        Turn_counter
    cpfslt      Turn_counter
    goto        Back
    call        MotorForward
    call        MotorForCorrect
    call        MotorBackCorrect
    call        GetIRData           ;Checks for presense of flashlight
    movf        Turn_counter,w
    addlw       0x1C
    movwf       FSR1L
    movlw       0x0
    movwf       INDF1
    clrf        FSR1L
    btfss       RunStateBits,1
    goto        Forward
    call        GetPRData
    call        GetStatus
    goto        Forward

Back
    dcfsnz      Turn_counter
    goto        MotorZero
    call        MotorBackward
    goto        Back

MotorZero
    call        MotorZeroCorrect
    clrf        LATC
    rtc_read	0x00
    movf        0x75,w
    andlw       b'01111111'
    movwf       StopTime

    BCDToBin    StopTime
    movwf       StopTime
    BCDToBin    StartTime
    subwf       StopTime,w
    btfsc       STATUS,N
    addlw       b'00111100'
    btfsc       STATUS,Z
    addlw       b'00111100'
    btfsc       RunStateBits,0
    addlw       b'00111100'
    movwf       StopTime

;; Operation is done, need to display results
    clrf        Machine_state
    bsf         Machine_state,3
    call        Clear_LCD
    call        Line1
    Display     DoneLine1
    call        Line2
    Display     DoneLine2
    clrf        DataDisplay
    call        LogOperation

    goto        MainMenu

;; Machine State Routines ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;check MachineState to see if go to start or ret
Pressed1
    btfss       Machine_state,0
    goto        CheckMachineState
    clrf        Machine_state
    bsf         Machine_state, 1
    return

CheckMachineState
    btfss       Machine_state,1
    goto        DispMainMenu
    return

Pressed2
    btfsc       Machine_state,0
    goto        SetLogs
    btfsc       Machine_state,3
    goto        SetResults
    btfsc       Machine_state,4
    goto        NextLog
    btfsc       Machine_state,6
    goto        DisplayResults
    return

Pressed3
    btfsc       Machine_state,4
    goto        DisplayLogData
    return
;Dispay Logs
;Logs
;    clrf        Machine_state
;    bsf         Machine_state, 2
;    call        Clear_LCD
;    call        Line1
;    Display     LogMsg
;    call        Line2
;    Display     RetMsg
;    return

;; Motor Routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Motor_Step: 0001-Step1, 0010-Step2, 0100-Step3, 1000-Step4
Step
    dcfsnz      Step_Counter
    call        Stop_Motor

    btfsc       Motor_Step,0
    goto        Step1
    btfsc       Motor_Step,1
    goto        Step2
    btfsc       Motor_Step,2
    goto        Step3
    goto        Step4
Step1
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00100001'
    movwf       LATC               ;Set moto to first squence
    bcf         Motor_Step,0
    btfsc       Motor_Step,7
    goto        SetStep4
    goto        SetStep2
Step2
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00000101'
    movwf       LATC               ;Set moto to second squence
    bcf         Motor_Step,1
    btfsc       Motor_Step,7
    goto        SetStep1
    goto        SetStep3
Step3
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00000110'
    movwf       LATC               ;Set moto to thrid squence
    bcf         Motor_Step,2
    btfsc       Motor_Step,7
    goto        SetStep2
    goto        SetStep4
Step4
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00100010'
    movwf       LATC               ;Set moto to fourth squence
    bcf         Motor_Step,3
    btfsc       Motor_Step,7
    goto        SetStep3
    goto        SetStep1
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
    goto        Step_Done
Step_Done
    return

MotorForward
    movlw       d'50'               ;Step_counter 40 => 40*1.8/2 = 36
    movwf       Step_Counter
    bsf         Motor_Step,7        ;Set forward
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor

MotorBackward
    movlw       d'50'               ;Step_counter 40 => 40*1.8/2 = 36
    movwf       Step_Counter
    bcf         Motor_Step,7        ;Set backward
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor

MotorForCorrect
    movlw       d'16'               ;Step_counter 40 => 40*1.8/2 = 36
    movwf       Step_Counter
    bsf         Motor_Step,7        ;Set forward
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor
MotorBackCorrect
    movlw       d'7'
    cpfseq      Turn_counter
    goto        BackCorrect15
    goto        BackCorrect17
BackCorrect15
    movlw       d'15'               ;Step_counter 40 => 40*1.8/2 = 36
    movwf       Step_Counter
    goto        BackCorrectEnd
BackCorrect17
    movlw       d'18'
    movwf       Step_Counter
    goto        BackCorrectEnd
BackCorrectEnd
    bcf         Motor_Step,7        ;Set backward
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor

MotorZeroCorrect
    movlw       d'6'               ;Step_counter 40 => 40*1.8/2 = 36
    movwf       Step_Counter
    bcf         Motor_Step,7        ;Set backward
    bcf         Motor_Step,6
    bsf         Machine_state,2
    goto        WaitMotor

WaitMotor
    btfss       Motor_Step,6        ;Wait for Motor
    goto        WaitMotor
    bcf         Machine_state,2
    return

Stop_Motor
    clrf        LATC
    bsf         Motor_Step,6
    return

;; Logs ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
SetLogs
    clrf        Machine_state
    bsf         Machine_state,4
    clrf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       EEOffset
    clrf        EEADR
    incf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       NumTests
    movf        NumTests,w
    cpfseq      Zero
    goto        DateDisplay
    goto        EndofLogs

EndofLogs
    clrf        Machine_state
    bsf         Machine_state,5
    call        Clear_LCD
    call        Line1
    Display     NoLogMsg
    call        Line2
    Display     FinalRetMsg
    return

NextLog
    dcfsnz      NumTests
    goto        EndofLogs
    movf        EEOffset,w
    bcf         STATUS,Z
    addlw       b'11110000'
    btfsc       STATUS,Z
    addlw       b'11110000'
    movwf       EEOffset
    call        DateDisplay
    return

DisplayLogData
    movlw       d'3'
    movwf       DataDisplay
    clrf        Machine_state
    bsf         Machine_state,6
    goto        DisplayResults

LogRetDisplay
    Display     LogRetMsg
    return

SetResults
    clrf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       EEOffset
    goto        DisplayResults

DisplayResults
    movlw       d'2'
    cpfsgt      DataDisplay
    goto        DateDisplay
    movlw       d'3'
    cpfsgt      DataDisplay
    goto        TimeDisplay
    movlw       d'4'
    cpfsgt      DataDisplay
    goto        NumberDisplay
    goto        FlashlightDisplay

DateDisplay
    call        Clear_LCD
    call        Line1
    Display     DateMsg
    movf        EEOffset,w
    addlw       0x1
    movwf       EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       BCDL
    call        BCDLDISPLAY
    movlw       "/"
    call        WR_DATA
    movf        EEOffset,w
    addlw       0x2
    movwf       EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       BCDL
    call        BCDLDISPLAY
    movlw       "/"
    call        WR_DATA
    movf        EEOffset,w
    movwf       EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       BCDL
    call        BCDLDISPLAY
    call        Line2
    btfss       Machine_state,3
    goto        LogRetDisplay
    Display     RetMsg    
    incf        DataDisplay
    incf        DataDisplay
    incf        DataDisplay
    return

TimeDisplay
    call        Clear_LCD
    call        Line1
    Display     TimeMsg
    movf        EEOffset,w
    addlw       0x3
    movwf       EEADR
    call        EEPROM_Read
    BinToBCD    EEDATA,Zero
    call        BCDLDISPLAY
    movlw       "s"
    call        WR_DATA
    call        Line2
    Display     RetMsg
    incf        DataDisplay
    return

NumberDisplay
    call        Clear_LCD
    call        Line1
    Display     NumberMsg
    movf        EEOffset,w
    addlw       0x4
    movwf       EEADR
    call        EEPROM_Read
    BinToBCD    EEDATA,Zero
    movf        BCDL,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        Line2
    Display     RetMsg
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
    BCD_Display BCDDISPLAY
    movlw       " "
    call        WR_DATA
    movf        EEOffset,w
    addwf       DataDisplay,w
    movwf       EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    movwf       Flashlight1
    btfsc       Flashlight1,7
    goto        StatusDisplay
    goto        EmptyDisplay

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
    Display     LED
    goto        RetDisplay
DisplayB
    Display     LEDD
    goto        RetDisplay
DisplayC
    Display     LEDDD
    goto        RetDisplay
DisplayD
    Display     LEDDDD
    goto        RetDisplay
EmptyDisplay
    Display     EmptyMsg
    goto        RetDisplay
RetDisplay
    incf        DataDisplay
    call        Line2
    movlw       d'13'
    cpfsgt      DataDisplay
    goto        RetDisplay1
    Display     FinalRetMsg
    return
RetDisplay1
    Display     RetMsg
    return

;; General Subroutines;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
GetIRData
    bcf         RunStateBits,2
    movlw       b'00111100'
    movwf       ResistorDelay
RDelay
    btfss       RunStateBits,2
    goto        RDelay
    bcf         RunStateBits,1
    movlw       b'00000001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movlw       b'00001010'
    cpfsgt      ADRESH
    bsf         RunStateBits,1
    return

GetPRData
    incf        FlashlightCounter
    movlw       b'00000101'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor1

    movlw       b'00001001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor2

    movlw       b'00010001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor3
   
    movlw       b'00010101'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor4

    movlw       b'00011001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor5

    movlw       b'00011101'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor6
    return

GetStatus
    clrf        NumZeros
    clrf        ZeroBits
    call        SensorError
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
    cpfsgt      Resistor6
    bsf         ZeroBits,5

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
    incf        NumZeros

    movlw       d'5'
    cpfslt      NumZeros
    goto        Set0LED

    movf        ZeroBits,w
    andlw       b'00000111'
    movwf       ZeroTest
    movlw       b'00000111'
    cpfseq      ZeroTest
    goto        Skip1
    goto        SumPR
Skip1
    movf        ZeroBits,w
    andlw       b'00001110'
    movwf       ZeroTest
    movlw       b'00001110'
    cpfseq      ZeroTest
    goto        Skip2
    goto        SumPR
Skip2
    movf        ZeroBits,w
    andlw       b'00011100'
    movwf       ZeroTest
    movlw       b'00011100'
    cpfseq      ZeroTest
    goto        Skip3
    goto        SumPR
Skip3
    movf        ZeroBits,w
    andlw       b'00111000'
    movwf       ZeroTest
    movlw       b'00111000'
    cpfseq      ZeroTest
    goto        Skip4
    goto        SumPR
Skip4
    movf        ZeroBits,w
    andlw       b'00110001'
    movwf       ZeroTest
    movlw       b'00110001'
    cpfseq      ZeroTest
    goto        Skip5
    goto        SumPR
Skip5
    movf        ZeroBits,w
    andlw       b'00100011'
    movwf       ZeroTest
    movlw       b'00100011'
    cpfseq      ZeroTest
    goto        Skip6
    goto        SumPR

Skip6
    btfsc       ZeroBits,0
    goto        TestPR1
    btfsc       ZeroBits,1
    goto        TestPR2
    btfsc       ZeroBits,2
    goto        TestPR3
    btfsc       ZeroBits,3
    goto        TestPR4
    btfsc       ZeroBits,4
    goto        TestPR5
    btfsc       ZeroBits,5
    goto        TestPR6
    goto        Set3LED

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
    btfsc       STATUS,C
    incf        PRSumH

    movf        PRSumH,w
    movwf       NumHigh
    movf        PRSumL,w
    movwf       NumMed
    BinToBCD    NumMed,NumHigh
    swapf       BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       d'2'
    cpfslt      PRSumH
    goto        Set2LED
    movlw       d'0'
    cpfsgt      PRSumH
    goto        Set1LED
    movlw       d'44'
    cpfsgt      PRSumL
    goto        Set1LED
    goto        Set2LED
    return

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
    goto        Set3LED
    return

Set0LED
;If no high nor med, then no LED
    movf        Turn_counter,w
    addlw       0x1C
    movwf       FSR1L
    movlw       b'10000000'
    movwf       INDF1
    clrf        FSR1L
    return

GetHighStatus
    movff       NumHigh,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    call        BCDLDISPLAY
    movlw       "H"
    call        WR_DATA
    dcfsnz      NumHigh,f
    goto        Set1LED
    dcfsnz      NumHigh,f
    goto        Set2LED
    dcfsnz      NumHigh,f
    goto        Set3LED
    return
    ;goto        SensorError

GetMedStatus
    movff       NumMed,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    call        BCDLDISPLAY
    movlw       "M"
    call        WR_DATA
    decf        NumMed,f
    dcfsnz      NumMed,f
    goto        Set1LED
    decf        NumMed,f
    dcfsnz      NumMed,f
    goto        Set2LED
    decf        NumMed,f
    dcfsnz      NumMed,f
    goto        Set3LED
    return
    ;goto        SensorError

Set1LED
    movf        Turn_counter,w
    addlw       0x1C
    movwf       FSR1L
    movlw       b'10000001'
    movwf       INDF1
    clrf        FSR1L
    return
Set2LED
    movf        Turn_counter,w
    addlw       0x1C
    movwf       FSR1L
    movlw       b'10000010'
    movwf       INDF1
    clrf        FSR1L
    return
Set3LED
    movf        Turn_counter,w
    addlw       0x1C
    movwf       FSR1L
    movlw       b'10000011'
    movwf       INDF1
    clrf        FSR1L
    return

SensorError
    call        Clear_LCD
    call        Line1

    movff       Resistor1,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    movff       Resistor2,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA
    
    movff       Resistor3,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    movff       Resistor4,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    call        Line2

    movff       Resistor5,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    movff       Resistor6,ResistorDelay
    BinToBCD    ResistorDelay,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    bcf         RunStateBits,3
;SensorErrorWait
;    btfss       RunStateBits,3
;    goto        SensorErrorWait
;    call        Clear_LCD
;    call        Line1
;    Display     StartMsg
    return

RDelayDec
    dcfsnz      ResistorDelay
    bsf         RunStateBits,2
    return

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

Time
    decfsz      TIMEL,f
    return
    decfsz      TIMEH,f
    return
    
    movlw       b'01001111'
    movwf       TIMEL
    movlw       b'00001110'
    movwf       TIMEH

    call        Line1
    btfsc       Machine_state,0
    call        show_RTC

    btfsc       Machine_state,1
    bsf         RunStateBits,0
    return

DispMainMenu
    clrf        Machine_state
    bsf         Machine_state,0
    call        Clear_LCD
    call        Line1
    call        show_RTC
    call        Line2
    Display     Line1Start
    return

IncfNumTests
    incf        EEDATA
    call        EEPROM_Write
    return

LogOperation
    clrf        EEADR
    call        EEPROM_Read
    movf        EEDATA,w
    bcf         STATUS,Z
    addlw       d'16'
    btfsc       STATUS,Z
    addlw       d'16'
    movwf       EEOffset
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
    

;;From PML4ALL Writes RTC DATA to LCD
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

set_rtc_time

		rtc_resetAll	;reset rtc

		rtc_set	0x00,	B'10000000'

		;set time
		rtc_set	0x06,	B'00010100'		; Year
		rtc_set	0x05,	B'00000010'		; Month
		rtc_set	0x04,	B'00100011'		; Date
		rtc_set	0x03,	B'00000111'		; Day
		rtc_set	0x02,	B'00010110'		; Hours
		rtc_set	0x01,	B'00101000'		; Minutes
		rtc_set	0x00,	B'00000000'		; Seconds
		return
;;TAKEN FROM DATA SHEET
EEPROM_Read
    BCF EECON1, EEPGD ; Point to DATA memory
    BCF EECON1, CFGS ; Access EEPROM
    BSF EECON1, RD ; EEPROM Read
return

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

EEPROM_Refresh
    CLRF EEADR ; Start at address 0
    CLRF EEADRH ;
    BCF EECON1, CFGS ; Set for memory
    BCF EECON1, EEPGD ; Set for Data EEPROM
    BSF EECON1, WREN ; Enable writes
EEPROM_Refresh_Loop ; Loop to refresh array
    BSF EECON1, RD ; Read current address
    MOVLW 55h ;
    MOVWF EECON2 ; Write 55h
    MOVLW 0AAh ;
    MOVWF EECON2 ; Write 0AAh
    BSF EECON1, WR ; Set WR bit to begin write
    BTFSC EECON1, WR ; Wait for write to complete
    BRA $-2
    INCFSZ EEADR, F ; Increment address
    BRA EEPROM_Refresh_Loop ; Not zero, do it again
    INCFSZ EEADRH, F ; Increment the high address
    BRA EEPROM_Refresh_Loop ; Not zero, do it again
    BCF EECON1, WREN ; Disable writes
return
end

;    bcf         Resistor1,0
;    rrncf       Resistor1,f
;    bcf         Resistor2,0
;    rrncf       Resistor2,f
;    bcf         Resistor3,0
;    rrncf       Resistor3,f
;    bcf         Resistor4,0
;    rrncf       Resistor4,f
;    bcf         Resistor5,0
;    rrncf       Resistor5,f
;    bcf         Resistor6,0
;    rrncf       Resistor6,f

;    clrf        State1
;    movf        Resistor1,w
;    addwf       Resistor2,w
;    movwf       NumMed
;    movlw       d'60'
;    cpfslt      NumMed
;    incf        State1
;    movlw       d'20'
;    cpfslt      NumMed
;    incf        State1

;    clrf        State2
;    movf        Resistor3,w
;    addwf       Resistor4,w
;    movwf       NumMed
;    movlw       d'60'
;    cpfslt      NumMed
;    incf        State2
;    movlw       d'20'
;    cpfslt      NumMed
;    incf        State2

;    clrf        State3
;    movf        Resistor5,w
;    addwf       Resistor6,w
;    movwf       NumMed
;    movlw       d'60'
;    cpfslt      NumMed
;    incf        State3
;    movlw       d'20'
;    cpfslt      NumMed
;    incf        State3

;    clrf        State4
;    movf        Resistor6,w
;    addwf       Resistor1,w
;    movwf       NumMed
;    movlw       d'60'
;    cpfslt      NumMed
;    incf        State4
;    movlw       d'20'
;    cpfslt      NumMed
;    incf        State4

;    clrf        State5
;    movf        Resistor2,w
;    addwf       Resistor3,w
;    movwf       NumMed
;    movlw       d'60'
;    cpfslt      NumMed
;    incf        State5
;    movlw       d'20'
;    cpfslt      NumMed
;    incf        State5

;    clrf        State6
;    movf        Resistor4,w
;    addwf       Resistor5,w
;    movwf       NumMed
;    movlw       d'60'
;    cpfslt      NumMed
;    incf        State6
;    movlw       d'20'
;    cpfslt      NumMed
;    incf        State6

;    clrf        Value1
;    clrf        Value2

;    movf        State1,w
;    addwf       Value1,f
;    movf        State2,w
;    addwf       Value1,f
;    movf        State3,w
;    addwf       Value1,f

;    movf        State4,w
;    addwf       Value2,f
;    movf        State5,w
;    addwf       Value2,f
;    movf        State6,w
;    addwf       Value2,f

;    movf        Value2,w
;    cpfseq      Value1
;    goto        SensorError

;    movlw       d'6'
;    cpfslt      Value1
;    goto        Set3LED
;    movlw       d'4'
;    cpfslt      Value1
;    goto        Set2LED
;    movlw       d'2'
;    cpfslt      Value1
;    goto        Set1LED
;    goto        SetEmpty
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;    clrf        NumHigh
;    clrf        NumMed
;    call        SensorError     ;Sensor Checking

;    movlw       d'160'
;Check number of highs
;    cpfslt      Resistor1
;    incf        NumHigh
;    cpfslt      Resistor2
;    incf        NumHigh
;    cpfslt      Resistor3
;    incf        NumHigh
;    cpfslt      Resistor4
;    incf        NumHigh
;    cpfslt      Resistor5
;    incf        NumHigh
;    cpfslt      Resistor6
;    incf        NumHigh

;    movlw       0x0
;    cpfseq      NumHigh
;    goto        GetHighStatus

;if no high, then check medium
;    movlw       d'40'
;    cpfslt      Resistor1
;    incf        NumMed
;    cpfslt      Resistor2
;    incf        NumMed
;    cpfslt      Resistor3
;    incf        NumMed
;    cpfslt      Resistor4
;    incf        NumMed
;    cpfslt      Resistor5
;    incf        NumMed
;    cpfslt      Resistor6
;    incf        NumMed

;    movlw       0x0
;    cpfseq      NumMed
;    goto        GetMedStatus
;    goto        Set0LED