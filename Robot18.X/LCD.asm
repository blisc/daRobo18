#include <p18f4620.inc>

;allocates memory for below variables in Data Memory
	udata
lcd_d1	res	1
lcd_d2	res	1
com		res	1
dat		res	1

;constans cause Emami be lazy
    #define	RS 	PORTD,2
    #define	E 	PORTD,3

;DAT Delay Macro
LCD_DELAY macro
	movlw   0x23
	movwf   lcd_d1
	decfsz  lcd_d1,f
	goto    $-2
	endm

    ;org and deiaply functions
	code
    global  LCD_Init,WR_INS,WR_DATA,Clear_LCD,Line1,Line2

;Initializes LCD for 4-bit input
;Sends 0011 three tims to reset LCD, then sets LCD to 4-bit input with 2
;lines with 5*7 dots
;LCD_Init
;    movlw   b'00110011'
;    call    WR_INS
;    movlw   b'00110010'
;    call    WR_INS
;    movlw   b'00101000'     ;4 bits, 2 lines, 5x7 dots 
;    call    WR_INS

LCD_Init
	bsf     E                ;E default high

	;Wait for LCD POR to finish (~15ms)
	call    lcdLongDelay
	call    lcdLongDelay
	call    lcdLongDelay

	;Ensure 8-bit mode first (no way to immediately guarantee 4-bit mode)
	; -> Send b'0011' 3 times
	movlw   b'00110011'
	call	WR_INS
	call     lcdLongDelay
	call     lcdLongDelay
	movlw	b'00110010'
	call	WR_INS
	call     lcdLongDelay
	call    lcdLongDelay

	; 4 bits, 2 lines, 5x7 dots
	movlw	b'00101000'
	call	WR_INS
	call    lcdLongDelay
	call    lcdLongDelay

	; display on/off
	movlw	b'00001110'
	call	WR_INS
	call    lcdLongDelay
	call    lcdLongDelay

	; Entry mode
	movlw	b'00000110'
	call	WR_INS
	call    lcdLongDelay
	call    lcdLongDelay

Clear_LCD
	; Clear ram
	movlw	b'00000001'
	call	WR_INS
	call    lcdLongDelay
	call    lcdLongDelay
	return

;WR_INS [taken from PML4ALL lab exerceises LCD]
;Write 8-bit intructions from working reg to LCD as 4-bit intructions
WR_INS
    bcf		RS				;clear RS
	movwf	com				;W --> com
	andlw	0xF0			;mask 4 bits MSB w = XXXX0000
	movwf	PORTD			;Send 4 bits MSB w = 76543210 ;7:4 Most Significant
                            ;Bit, 3:0 LestSignificantBit
	bsf		E				;
	;call	lcdLongDelay	;__    __
    nop
	bcf		E				;  |__|
    call    lcdLongDelay

	swapf	com,w
	andlw	0xF0			;1111 0010
	movwf	PORTD			;send 4 bits LSB
	bsf		E				;
	;call	lcdLongDelay	;__    __
    nop
	bcf		E				;  |__|
    call    lcdLongDelay
	return

;Same as WR_INS but data
WR_DATA
	bsf		RS
	movwf	dat
	movf	dat,w
	andlw	0xF0
	addlw	4
	movwf	PORTD
	bsf		E				;
	;call	lcdLongDelay	;__    __
    nop
	bcf		E				;  |__|
    call    lcdLongDelay

	swapf	dat,w
	andlw	0xF0
	addlw	4
	movwf	PORTD
	bsf		E				;
	;call	lcdLongDelay	;__    __
    nop
	bcf		E				;  |__|
    call    lcdLongDelay
	return

Line2
    movlw   b'11000000'
    call    WR_INS
    return

Line1
    movlw   b'10000000'
    call    WR_INS
    return

;delays..C&P
lcdLongDelay
    movlw   d'80'
    movwf   lcd_d2
LLD_LOOP
    LCD_DELAY
    decfsz  lcd_d2,f
    goto    LLD_LOOP
    return

    end