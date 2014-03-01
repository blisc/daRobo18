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
    Run_state
    Motor_Step
    Turn_counter
    Flashlight_counter
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
    TempDelay1
    TempDelay2
    TempDelay3
    StartTime
    StopTime
    TimeOV
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

    call        show_RTC
    banksel     Line1Start
    call        Line2
    Display     Line1Start

Stop
    btfss       Machine_state,1
    goto        Stop                ;END OF CODE
    goto        Start
    goto        DoneOp


;; INITIALIZATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init
    movlw       B'01110000'
    movwf		OSCCON              ;Set internal oscillator to 32MHz = 8MHz per
    bsf         OSCTUNE, 6          ;clock cycle. Enables scaler

    movlw       b'11111111'
    movwf       TRISB               ;set PORTB to input - Keypad
	clrf		TRISC               ;Set PORTC 0:2 and 5:7 as output (0:2 and 5 for motor)
    bsf         TRISC,3
    bsf         TRISC,4             ;RC3:4 are inputs for RTC
	clrf		TRISD               ;set PORTD to LCD
    movlw       b'00000011'
    movwf       TRISA               ;RA0:1 input for sensors, rest output

    clrf        LATC
    clrf        LATD
    clrf        LATA

    movlw       b'00001110'
    movwf       ADCON1              ;Sets AN0:1 (RA0:1) to analog, Uses Vss and Vdd
    movlw       b'10100110'         ;Sets 2Tad and 16Tosc
    movwf       ADCON2

    clrf        Machine_state
    bsf         Machine_state, 0    ;Set machine state to idle
    clrf        Run_state
    bsf         Run_state,0         ;Set run state to not running

    ;Initialize Timer for RTC display
    movlw       b'10010100'
    movwf       TIMEL
    movlw       b'00000100'
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
;overflows every 122Hz. At 1:8, it overflows every 15.261Hz
    movlw       b'00000010'
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
    movwf       H'30'
    incf        H'30',f
    decfsz      H'30',f             ;decrement working reg, skip next line if 0
    goto        Check2
    goto        CheckMachineState
;Checks if 2 is pressed
Check2
    ;Checks Machine State
    btfss       Machine_state, 0
    return
    decfsz      H'30', f
    goto        Check3              ;If not 2, wait until button released
    goto        Logs                ;If 2, display Logs
;Checks if 3 is pressed
Check3
    decfsz      H'30', f
    goto        Check4
    goto        Motor
Check4
    decfsz      H'30', f
    return
    goto        A2D

ISR_Timer0
    bcf         INTCON,TMR0IF
    btfsc       Run_state,1
    call        Step
    call        Time
    return

;; START ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Start Routine
Start
    ;Initialize Timer
    movlw       b'10010100'
    movwf       TIMEL
    movlw       b'00000100'
    movwf       TIMEH
    bcf         TimeOV,0
    rtc_read	0x00
    movf        0x75,w
    andlw       b'01111111'
    movwf       StartTime

    call        Clear_LCD
    call        Line1
    Display     StartMsg
    call        Line2
    Display     RetMsg

    movlw       b'00001001'
    movwf       Turn_counter

    call        Line1
Forward
    dcfsnz      Turn_counter
    goto        Back
    call        MotorForward
    ;;;;;;;;;;  GET DATA  ;;;;;;;;;;;;
    call        RanDelay
    ;movlw       '1'
    ;call        WR_DATA
    goto        Forward

Back
    movlw       b'00001011'
    incf        Turn_counter
    cpfslt      Turn_counter
    goto        MDone
    call        MotorBackward
    ;movlw       '2'
    ;call        WR_DATA
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
    btfsc       TimeOV,0
    addlw       b'00111100'
    movwf       StopTime
    BinToBCD    StopTime,Zero
    goto        DoneOp

DoneOp
    call        Clear_LCD
    call        Line1
    Display     Done
    call        Line2
    Display     DoneTime
    swapf       BCDL,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    movf        BCDL,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    clrf        Machine_state
    bsf         Machine_state,0
    goto        Stop

;; SUBROUTINES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;check MachineState to see if go to start or ret
CheckMachineState
    btfss       Machine_state, 0
    goto        MenuRet
    clrf        Machine_state
    bsf         Machine_state, 1
    return

;Dispay Logs
Logs
    clrf        Machine_state
    bsf         Machine_state, 2
    call        Clear_LCD
    call        Line1
    Display     LogMsg
    call        Line2
    Display     RetMsg
    return

Motor
    clrf        Machine_state
    bsf         Machine_state, 3
    clrf        Run_state
    bsf         Run_state,1
    call        Clear_LCD
    call        Line1
    Display     MotorMsg
    call        Line2
    Display     RetMsg
    bsf         Motor_Step,0
    bcf         Motor_Step,7
    movlw       b'11111111'
    movwf       Step_Counter
    return

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

;Return to main menu if 1 is pressed
MenuRet
    clrf        Machine_state
    bsf         Machine_state, 0
    clrf        Run_state
    bsf         Run_state,0
    call        Stop_Motor
    call        Clear_LCD
    call        Line1
    call        show_RTC
    call        Line2
    Display     Line1Start
    return

MotorForward
    clrf        Run_state
    bsf         Run_state,1
    movlw       d'24'               ;Step_counter 24 = 36 degrees
    movwf       Step_Counter
    bcf         Motor_Step,7        ;Set forward
    bcf         Motor_Step,6
    goto        WaitMotor

MotorBackward
    clrf        Run_state
    bsf         Run_state,1
    movlw       d'24'               ;Step_counter 24 = 36 degrees
    movwf       Step_Counter
    bsf         Motor_Step,7        ;Set backward
    bcf         Motor_Step,6
    goto        WaitMotor

WaitMotor
    btfss       Motor_Step,6        ;Wait for Motor
    goto        WaitMotor
    clrf        Run_state
    bsf         Run_state,5
    return

Stop_Motor
    clrf        LATC
    bsf         Motor_Step,6
    return

RanDelay
    movlw       b'01111111'
    movwf       TempDelay3
    movlw       b'11111111'
RanDelayLoop3
    movwf       TempDelay2
RanDelayLoop2
    movwf       TempDelay1
RanDelayLoop1
    decfsz      TempDelay1
    goto        RanDelayLoop1
    decfsz      TempDelay2
    goto        RanDelayLoop2
    decfsz      TempDelay3
    goto        RanDelayLoop3
    return


;A2D module
A2D
    clrf        Machine_state
    bsf         Machine_state, 4
    call        Clear_LCD
    movlw       b'00000001'
    movwf       ADCON0
    bsf         ADCON0,1
    btfsc       ADCON0,1
    goto        $-2
;Writes to LCD
    BinToBCD    ADRESL,ADRESH
    swapf       BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
    movf        BCDH,w
    andlw       0x0F
    movwf       BCDDISPLAY
    BCD_Display BCDDISPLAY
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
    
    movlw       b'10010100'
    movwf       TIMEL
    movlw       b'00000100'
    movwf       TIMEH

    call        Line1
    btfsc       Machine_state,0
    call        show_RTC

    btfsc       Machine_state,1
    bsf         TimeOV,0
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
end