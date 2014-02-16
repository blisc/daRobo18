    list p=18f4620              ; list directive to define processor
    #include <p18f4620.inc>     ; processor specific variable definitions
    #include <lcd.inc>			;Import LCD control functions from lcd.asm

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
    Motor_Step
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
    call        ISR_Timer
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

;; MAIN ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Mainline
    call        Init
    call        ISR_init
    call        LCD_Init
    Display     Line1Start
    call        Line2
    Display     Line2Start

Stop
    nop
    goto        Stop                ;END OF CODE


;; INITIALIZATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init
    movlw       B'01110000'
    movwf		OSCCON              ;Set internal oscillator to 8MHz
    bsf         OSCTUNE, 6          ;Enables scaler

    movlw       b'11111111'
    movwf       TRISB               ;set PORTB to input - Keypad
	clrf		TRISC               ;Set PORTC to output - 3:0 motor
	clrf		TRISD               ;set PORTD to LCD
    movlw       b'00000011'
    movwf       TRISA               ;RA0:1 input for sensors, resst output

    clrf        LATC
    clrf        LATD
    clrf        LATA

    movlw       b'00001101'
    movwf       ADCON1              ;Sets AN0:1 (RA0:1) to analog, Uses Vss and Vdd
    movlw       b'00001101'         ;Sets 2Tad and 16Tosc
    movwf       ADCON2

    clrf        Machine_state
    bsf         Machine_state, 0    ;Set machine state to idle
    return

;Enables RB1 pin, Sets RB1 to high and TMR0 to low priority
ISR_init
    bsf         RCON,IPEN
	bsf         INTCON,GIEH
    bcf         INTCON,GIEL

    bcf         INTCON3,INT1IF
    bsf         INTCON3,INT1IE
    bsf         INTCON3,INT1IP

    bcf         INTCON,TMR0IF
    bsf         INTCON,TMR0IE
    bcf         INTCON2,TMR0IP

;Sets Timer0 to 16bit and configs Timer0, prescales 1:128 for a period of 1s at 8MHz
;Switched to 1:64 for step for every half second
    movlw       b'00000010'
    movwf       T0CON
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

ISR_Timer
    bcf         INTCON,TMR0IF
    call        Step
    return

;; SUBROUTINES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;check MachineState to see if go to start or ret
CheckMachineState
    btfss       Machine_state, 0
    goto        MenuRet
    clrf        Machine_state
    bsf         Machine_state, 1
    call        Clear_LCD
    call        Line1
    Display     StartMsg
    call        Line2
    Display     RetMsg
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
    call        Clear_LCD
    call        Line1
    Display     MotorMsg
    call        Line2
    Display     RetMsg
    bsf         INTCON,GIEL
    bsf         Motor_Step,0
    call        Step
    bsf         T0CON,TMR0ON
    return

;Motor_Step: 0001-Step1, 0010-Step2, 0100-Step3, 1000-Step4
Step
    btfsc       Motor_Step,0
    goto        Step1
    btfsc       Motor_Step,1
    goto        Step2
    btfsc       Motor_Step,2
    goto        Step3
    btfsc       Motor_Step,3
    goto        Step4
Step1
    movlw       b'10000001'
    movwf       LATC               ;Set moto to first squence
    clrf        Motor_Step
    bsf         Motor_Step,1
    goto        Step_Done
Step2
    movlw       b'01000100'
    movwf       LATC               ;Set moto to second squence
    clrf        Motor_Step
    bsf         Motor_Step,2
    goto        Step_Done
Step3
    movlw       b'00100010'
    movwf       LATC               ;Set moto to thrid squence
    clrf        Motor_Step
    bsf         Motor_Step,3
    goto        Step_Done
Step4
    movlw       b'00011000'
    movwf       LATC               ;Set moto to fourth squence
    clrf        Motor_Step
    bsf         Motor_Step,0
    goto        Step_Done
Step_Done
    return

;Return to main menu if 1 is pressed
MenuRet
    clrf        Machine_state
    bsf         Machine_state, 0
    bcf         INTCON,GIEL
    bcf         T0CON,TMR0ON
    clrf        LATC
    call        Clear_LCD
    call        Line1
    Display     Line1Start
    call        Line2
    Display     Line2Start
    return

;A2D module
A2D
    clrf        Machine_state
    bsf         Machine_state, 4
    call        Clear_LCD
    movlw       b'00000001'
    movwf       ADCON0
    btfsc       ADCON0,1
    goto        $-2
;Writes to LCD
    movf        ADRESH,w
    call        WR_DATA
    movf        ADRESL,w
    call        WR_DATA
    end