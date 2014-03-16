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
    ;bit 2 - running motor
    ;bit 3 - displaying logs at end of operation
    Motor_Step
    Turn_counter
    FlashlightCounter
    Step_Counter
    Zero
    Binl
    Binh
    BCDL
    BCDH
    BCDU
    COUNT
    BCDDISPLAY
    TIMEH
    TIMEL
    StartTime
    StopTime
    RunStateBits                
    ;bit 0: minutre overflow
    ;bit 1: presense of flashlight
    ;bit 2: photoresistor wait
    ;bit 3: error!!! in sensor logic
    Key_Pressed
    ResistorDelay
    Resistor1
    Resistor2
    Resistor3
    Resistor4
    Resistor5
    Resistor6
    NumHigh
    NumMed
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
	db		"1: Start 2: Logs", 0
Line2Start
	db		"3: Motor", 0
StartMsg
	db		"U Pressed Start", 0
MotorMsg
	db		"Run Dat Motor", 0
LogMsg
	db		"Here be the logs", 0
RetMsg
	db		"1: Return", 0
Number
    db      "0123456789", 0
Done
    db      "Done!!!", 0
DoneTime
    db      "Time: ",0

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

    clrf        EEADRH
    bsf         RunStateBits,2
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

;; INTERRUPT SERVICES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ISR_Key
    movwf       temp_w
	movf        STATUS,w
	movwf       temp_status
    bcf         INTCON3,INT1IF
	swapf       PORTB,W             ;Puts PORTB7:4 into W3:0
    andlw       0x0F                ;W: 0000XXXX
    movwf       Key_Pressed
    incf        Key_Pressed,f
    decfsz      Key_Pressed,f             ;decrement working reg, skip next line if 0
    goto        Check2
    goto        SetStart
;Checks if 2 is pressed
Check2
    bsf         RunStateBits,3
    ;Checks Machine State
    btfss       Machine_state, 3
    return
    goto        DispMainMenu

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

    movlw       b'00000000'
    movwf       Turn_counter
    clrf        Turn_counter

Forward
    btfsc       Machine_state,0
    goto        Back
    movlw       b'00001010'
    incf        Turn_counter
    cpfslt      Turn_counter
    goto        Back
    call        MotorForward
    call        GetIRData           ;Checks for presense of flashlight
    movf        Turn_counter,w
    movwf       FSR2L
    movlw       0x0
    movwf       NumMed,0
    clrf        FSR2L
    btfss       RunStateBits,1
    goto        Forward
    call        GetPRData
    call        GetStatus
    goto        Forward

Back
    dcfsnz      Turn_counter
    goto        MDone
    call        MotorBackward
    goto        Back

MDone
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
    BinToBCD    StopTime,Zero

;; Operation is done, need to display results
    clrf        Machine_state
    bsf         Machine_state,3
    call        Clear_LCD
    call        Line1
    Display     Done
    call        Line2
    Display     DoneTime
    call        BCDLDISPLAY
    call        LogOperation

    goto        MainMenu

;; Machine State Routines ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;check MachineState to see if go to start or ret
SetStart
    btfss       Machine_state,0
    return
    clrf        Machine_state
    bsf         Machine_state, 1
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
    iorlw       b'00000001'
    movwf       LATC               ;Set moto to first squence
    bcf         Motor_Step,0
    btfsc       Motor_Step,7
    goto        SetStep4
    goto        SetStep2
Step2
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00000100'
    movwf       LATC               ;Set moto to second squence
    bcf         Motor_Step,1
    btfsc       Motor_Step,7
    goto        SetStep1
    goto        SetStep3
Step3
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00000010'
    movwf       LATC               ;Set moto to thrid squence
    bcf         Motor_Step,2
    btfsc       Motor_Step,7
    goto        SetStep2
    goto        SetStep4
Step4
    movf        PORTC,w
    andlw       b'00011000'
    iorlw       b'00100000'
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
    bsf         Machine_state,2
    movlw       d'34'               ;Step_counter 40 => 40*1.8/2 = 36
    movwf       Step_Counter
    bsf         Motor_Step,7        ;Set forward
    bcf         Motor_Step,6
    goto        WaitMotor

MotorBackward
    bsf         Machine_state,2
    movlw       d'34'               ;Step_counter 40 => 40*1.8/2 = 36
    movwf       Step_Counter
    bcf         Motor_Step,7        ;Set backward
    bcf         Motor_Step,6
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

;; General Subroutines ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

    movlw       b'000111101'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
    movf        ADRESH,w
    movwf       Resistor6
    return

GetStatus
    clrf        NumHigh
    clrf        NumMed
    clrf        NumLow
    movlw       d'180'

;Check number of highs
    cpfslt      Resistor1
    incf        NumHigh
    cpfslt      Resistor2
    incf        NumHigh
    cpfslt      Resistor3
    incf        NumHigh
    cpfslt      Resistor4
    incf        NumHigh
    cpfslt      Resistor5
    incf        NumHigh
    cpfslt      Resistor6
    incf        NumHigh

    movlw       0x0
    cpfseq      NumHigh
    goto        GetHighStatus

;if no high, then check medium
    movlw       d'60'
    cpfslt      Resistor1
    incf        NumMed
    cpfslt      Resistor2
    incf        NumMed
    cpfslt      Resistor3
    incf        NumMed
    cpfslt      Resistor4
    incf        NumMed
    cpfslt      Resistor5
    incf        NumMed
    cpfslt      Resistor6
    incf        NumMed

    movlw       0x0
    cpfseq      NumMed
    goto        GetMedStatus

;If no high nor med, then no LED
    movf        Turn_counter,w
    movwf       FSR2L
    movlw       b'10000000'
    movwf       NumMed,0
    clrf        FSR2L
    return

GetHighStatus
    dcfsnz      NumHigh
    goto        Set1LED
    dcfsnz      NumHigh
    goto        Set2LED
    dcfsnz      NumHigh
    goto        Set3LED
    goto        SensorError

GetMedStatus
    dcfsnz      NumMed
    nop
    dcfsnz      NumMed
    goto        Set1LED
    dcfsnz      NumMed
    nop
    dcfsnz      NumMed
    goto        Set2LED
    dcfsnz      NumMed
    nop
    dcfsnz      NumMed
    goto        Set3LED
    goto        SensorError

Set1LED
    movf        Turn_counter,w
    movwf       FSR2L
    movlw       b'10000001'
    movwf       NumMed,0
    clrf        FSR2L
    return
Set2LED
    movf        Turn_counter,w
    movwf       FSR2L
    movlw       b'10000010'
    movwf       NumMed,0
    clrf        FSR2L
    return
Set3LED
    movf        Turn_counter,w
    movwf       FSR2L
    movlw       b'10000011'
    movwf       NumMed,0
    clrf        FSR2L
    return

SensorError
    call        Line1

    BinToBCD    Resistor1,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    BinToBCD    Resistor2,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    BinToBCD    Resistor3,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    BinToBCD    Resistor4,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    call        Line2

    BinToBCD    Resistor5,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    BinToBCD    Resistor6,Zero
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    call        BCDLDISPLAY
    movlw       " "
    call        WR_DATA

    bcf         RunStateBits,3
SensorErrorWait
    btfss       RunStateBits,3
    goto        SensorErrorWait
    Display     StartMsg
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
    call        Line1
    call        show_RTC
    call        Line2
    Display     Line1Start
    return

LogOperation
    clrf        EEADR
    call        EEPROM_READ
    movf        EEDATA,w
    bcf         STATUS,3
    addlw       d'16'
    btfss       STATUS,3
    addlw       d'16'
    movwf       EEOffset
;Stores Year of operation
    rtc_read    0x06
    movf        0x75,w
    movwf       EEDATA
    movf        EEOffset,w
    movwf       EEADR
    call        EEPROM_WRITE
;Store Month of Operation
    rtc_read    0x05
    movf        0x75,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x1
    movwf       EEADR
    call        EEPROM_WRITE
;Store Day of Operation
    rtc_read    0x04
    movf        0x75,w
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x2
    movwf       EEADR
    call        EEPROM_WRITE
    return
;Store time of Operation
    movf        StopTime
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x3
    movwf       EEADR
    call        EEPROM_WRITE
;Store number of flashlights tested
    movf        FlashlightCounter
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x4
    movwf       EEADR
    call        EEPROM_WRITE
;Store FlashlightData
    movf        Flashlight1
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x5
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight2
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x6
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight3
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x7
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight4
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x8
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight5
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0x9
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight6
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xA
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight7
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xB
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight8
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xC
    movwf       EEADR
    call        EEPROM_WRITE
    movf        Flashlight9
    movwf       EEDATA
    movf        EEOffset,w
    addlw       0xD
    movwf       EEADR
    call        EEPROM_WRITE
;move new offset into EEPROM
    clrf        EEADR
    movf        EEOffset,w
    movwf       EEDATA
    call        EEPROM_WRITE
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
		rtc_set	0x02,	B'00100000'		; Hours
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