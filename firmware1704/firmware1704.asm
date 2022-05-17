;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashTwenty: Single-Chip DCD Interface
;;;
;


;;; Connections ;;;

;;;                                                           ;;;
;                         .--------.                            ;
;                 Supply -|01 \/ 14|- Ground                    ;
;      !ENBL --->    RA5 -|02    13|- RA0    ---> Next !ENBL    ;
;        PH3 --->    RA4 -|03    12|- RA1    <--- PH0           ;
;        PH2 --->    RA3 -|04    11|- RA2    <--- PH1           ;
;         WR --->    RC5 -|05    10|- RC0    ---> MMC SCK       ;
;         RD <---    RC4 -|06    09|- RC1    <--- MMC MISO      ;
;    MMC !CS <---    RC3 -|07    08|- RC2    ---> MMC MOSI      ;
;                         '--------'                            ;
;;;                                                           ;;;


;;; Assembler Directives ;;;

	list		P=PIC16F1704, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P16F1704.inc
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PPS1WAY_OFF & _ZCDDIS_ON & _PLLEN_ON & _STVREN_ON & _LVP_OFF
			;_WRT_OFF	Write protection off
			;_PPS1WAY_OFF	PPS can change more than once
			;_ZCDDIS_ON	Zero crossing detector disabled
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro			;Double-NOP
	bra	$+1
	endm


;;; Constants ;;;

EN_PORT	equ	PORTA
EN_PIN	equ	RA5
NE_PORT	equ	PORTA
NE_PIN	equ	RA0
PH_PORT	equ	PORTA
PH_IOC	equ	IOCAF
PH0_PIN	equ	RA1
PH1_PIN	equ	RA2
PH2_PIN	equ	RA3
PH3_PIN	equ	RA4
WR_PORT	equ	PORTC
WR_IOC	equ	IOCCF
WR_PIN	equ	RC5
CS_PORT	equ	PORTC
CS_PIN	equ	RC3

ERRCODE	equ	0x80	;Code returned in "stat" field when there's an error

;IWMFLAG:
IWMBYTE	equ	7	;Cleared when a byte has been received from IWM
IWMOVER	equ	6	;Set on transition to state 2 or 3 while IWM receiving
IWMTEMP	equ	5	;Used as temporary storage by ISR
IWMCNT1	equ	1	;Counter for how many times the ISR was called
IWMCNT0	equ	0	; "

;M_FLAGS:
M_FAIL	equ	7	;Set when there's been a failure on the MMC interface
M_CMDLR	equ	6	;Set when R3 or R7 is expected (5 bytes), not R1
M_CMDRB	equ	5	;Set when an R1b is expected (busy signal after reply)
M_RDCMD	equ	4	;Set when state machine should do a read command
M_ONGWR	equ	3	;Set when state machine shouldn't stop tran on a write
M_BKADR	equ	2	;Set when block (rather than byte) addressing is in use
M_CDVER	equ	1	;Set when dealing with a V2.0+ card, clear for 1.0


;;; Variable Storage ;;;

	cblock	0x20	;Bank 0 registers
	
	DEVNUM	;Selected device number
	DEVMAX	;Maximum device number
	IWMFLAG	;IWM receiver flags
	IWMFSAP	;IWM post-processing FSA pointer
	IWMLSBS	;LSB shift register for received or transmitted IWM bytes
	IWMREXP	;Size in groups of expected response to command
	CHKSUM	;Checksum
	CMDNUM	;Command number
	CMDCNT	;Command block count
	CMDHIGH	;Command high address byte
	CMDMED	;Command middle address byte
	CMDLOW	;Command low address byte
	RESPERR	;Command response status byte
	
	endc

	cblock	0x70	;Bank-common registers
	
	IWMSR	;Shift register for incoming data from IWM
	M_FLAGS	;MMC flags
	M_CMDN	;The MMC command to be sent, later the R1 response byte	
	M_ADR3	;First (high) byte of the address, first byte of R3/R7 response
	M_ADR2	;Second byte of the address, second byte of R3/R7 response
	M_ADR1	;Third byte of the address, third byte of R3/R7 response
	M_ADR0	;Fourth (low) byte of the address, last byte of R3/R7 response
	M_CRCH	;CRC16 high byte
	M_CRCL	;CRC16 low byte
	M_STATE	;State of card state machine
	X5	;Various purposes
	X4	;Various purposes
	X3	;Various purposes
	X2	;Various purposes
	X1	;Various purposes
	X0	;Various purposes
	
	endc


;;; Vectors ;;;

	org	0x0		;Reset vector
	movlp	high Init
	goto	Init

	org	0x4		;Interrupt vector
	;fall through


;;; Interrupt Handler ;;;

Interrupt
	btfsc	INTCON,INTF	;If we're here because of a rising edge on
	bra	IntDisable	; !ENBL, go take care of that
	lslf	IWMSR,W		;Make space for three new bits in the IWM shift
	lslf	WREG,W		; register
	lslf	WREG,W		; "
	movlb	30		;Grab the bits off the two-bit shift register
	iorwf	CLCDATA,W	; and put them into the IWM shift register (we
	movwf	IWMSR		; know the LSB of CLCDATA will be low because
	movlb	0		; !HSHK is low whenever we're receiving)
	incf	IWMFLAG,F	;Increment the counter of times the interrupt
	btfss	IWMFLAG,1	; handler was called; if this is the first of
	bra	IntFirst	; two, skip ahead
	movlb	7		;Clear any triggered edge on WR so we can spin
	bcf	WR_IOC,WR_PIN	; and wait for one later
	movlb	0		; "
	btfsc	WR_PORT,WR_PIN	;Grab the current state of WR and put that into
	bsf	IWMSR,0		; bit 0 of the IWM shift register
	btfss	INTCON,IOCIF	;Spin until we get an edge on WR or a rising
	bra	$-1		; edge on PH1 (a transition to state 2 or 3)
	nop			;Delay
	clrf	IWMFLAG		;Signal we have a byte and reset call counter
	movlw	high LutInv|0x80;Point to inversion LUT for translation ahead
	movwf	FSR0H		; "
	btfsc	WR_PORT,WR_PIN	;Grab the current state of WR (first bit of a
	bsf	IWMFLAG,IWMTEMP	; new cell) and put that into our temp bit
	clrf	TMR2		;Re-calibrate Timer2
	movf	IWMSR,W		;Use the inversion LUT to change the high/low
	movwf	FSR0L		; levels into transitions or non-transitions
	movf	INDF0,W		; and turn it into an IWM byte and write that
	movwf	INDF1		; to the IWM queue
	btfss	PH_PORT,PH0_PIN	;If PH0 is clear, clear the MSB of the byte we
	bcf	INDF1,7		; just write to indicate a suspend is on
	clrf	IWMSR		;Clear the IWM shift register and put the first
	btfsc	IWMFLAG,IWMTEMP	; bit of the new cell that we saved in the temp
	bsf	IWMSR,1		; bit into bit 1 of it
	movlw	-33		;Set Timer0 to interrupt again after 33 cycles
	movwf	TMR0		; and clear its interrupt flag
	bcf	INTCON,TMR0IF	; "
	DNOP			;Delay to get the read at a better position
	btfsc	WR_PORT,WR_PIN	;Grab the current state of WR and put that into
	bsf	IWMSR,0		; bit 0 of the IWM shift register
	movlb	7		;If we were released from the loop by a
	btfsc	PH_IOC,PH1_PIN	; transition to state 2 or 3, we're done
	bra	IntDataOver	; receiving data and must set flags accordingly
	movlb	0		;Otherwise, post process this byte
	bra	IntPostProc	; "

IntFirst
	btfsc	WR_PORT,WR_PIN	;Grab the current state of WR and put that into
	bsf	IWMSR,0		; bit 0 of the IWM shift register
	movlw	-27		;Set Timer0 to interrupt again after 27 cycles
	movwf	TMR0		; "
	bcf	INTCON,TMR0IF	;Clear Timer0 interrupt flag
	retfie

IntDataOver
	movlb	0		;Clear the IWM byte-received bit and set the
	clrf	IWMFLAG		; IWM data over flag to communicate the
	bsf	IWMFLAG,IWMOVER	; transition to state 2 or 3
	bcf	INTCON,TMR0IE	;Disable the Timer0 interrupt
	;fall through

IntPostProc
	movf	IWMFSAP,W
	brw
IPP
IPPJunk	movlw	IPPSync - IPP	;The first byte 'received' is junk, proceed to
	movwf	IWMFSAP		; expect a sync byte
	retfie
IPPSync	movlw	0xAA		;If the byte received was a sync byte (0xAA),
	xorwf	INDF1,W		; advance to expect the group count, otherwise
	movlw	IPPTCnt - IPP	; remain in this state
	btfsc	STATUS,Z	; "
	movwf	IWMFSAP		; "
	retfie
IPPTCnt	movlw	IPPRCnt - IPP	;The byte received is the transmit group count,
	movwf	IWMFSAP		; throw it away as we don't use it
	retfie
IPPRCnt	movf	INDF1,W		;The byte received is the expected response
	andlw	B'01111111'	; size count, save it in case it's needed
	movwf	IWMREXP		; "
	movlw	IPPLsbs - IPP	;Next byte will be the LSBs of the following
	movwf	IWMFSAP		; seven bytes
	retfie
IPPLsbs	movf	INDF1,W		;The byte received is the LSBs of the following
	movwf	IWMLSBS		; seven bytes, keep it to shift out later
	movlw	IPPByt1 - IPP	;Next byte will be first data byte in group
	movwf	IWMFSAP		; "
	retfie
IPPByt1	lsrf	IWMLSBS,F	;Shift out the first LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPByt2 - IPP	;Next byte will be second data byte in group
	movwf	IWMFSAP		; "
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPByt2	lsrf	IWMLSBS,F	;Shift out the next LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPByt3 - IPP	;Next byte will be third data byte in group
	movwf	IWMFSAP		; "
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPByt3	lsrf	IWMLSBS,F	;Shift out the next LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPByt4 - IPP	;Next byte will be fourth data byte in group
	movwf	IWMFSAP		; "
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPByt4	lsrf	IWMLSBS,F	;Shift out the next LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPByt5 - IPP	;Next byte will be fifth data byte in group
	movwf	IWMFSAP		; "
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPByt5	lsrf	IWMLSBS,F	;Shift out the next LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPByt6 - IPP	;Next byte will be sixth data byte in group
	movwf	IWMFSAP		; "
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPByt6	lsrf	IWMLSBS,F	;Shift out the next LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPByt7 - IPP	;Next byte will be seventh data byte in group
	btfss	STATUS,C	; but if we're in a suspend at this point, make
	movlw	IPPSus7 - IPP	; sure the byte after the seventh is treated
	movwf	IWMFSAP		; as a restart sync byte
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPByt7	lsrf	IWMLSBS,F	;Shift out the last LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPLsbs - IPP	;Next byte will be LSBs of the next group
	btfss	STATUS,C	; unless we're in a suspend, in which case it
	movlw	IPPRSyn - IPP	; will be a restart sync byte
	movwf	IWMFSAP		; "
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPSus7	lsrf	IWMLSBS,F	;Shift out the last LSB and rotate it into the
	rlf	INDF1,F		; received byte
	movlw	IPPRSyn - IPP	;A suspend was detected at the end of the sixth
	movwf	IWMFSAP		; byte, so next byte is a restart sync byte
	movlb	31		;Increment and wrap pointer
	incf	FSR1L_SHAD,F	; "
	bcf	FSR1L_SHAD,7	; "
	retfie
IPPRSyn	movlw	0xAA		;If the byte received was a sync byte (0xAA),
	xorwf	INDF1,W		; advance to expect the LSBs of the next group,
	movlw	IPPLsbs - IPP	; otherwise remain in this state
	btfsc	STATUS,Z	; "
	movwf	IWMFSAP		; "
	retfie

IntDisable
	movlw	B'00110010'	;Tristate RD
	tris	7		; "
	movlb	0		;Deassert !ENBL for the next external device
	bsf	NE_PORT,NE_PIN	; "
	bcf	INTCON,INTF	;Clear the rising !ENBL interrupt
	retfie


;;; Peripheral Initialization ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	SSPCON1		;SSP SPI master mode, clock set by baud rate
	movlw	B'00101010'	; generator to 400 kHz, clock idles low, data
	movwf	SSPCON1		; lines change on falling edge, data sampled on
	movlw	B'01000000'	; rising edge (CKP=0, CKE=1, SMP=0)
	movwf	SSP1STAT
	movlw	19
	movwf	SSP1ADD

	banksel	BAUDCON		;USART in synchronous mode, data clocked on
	movlw	B'01011000'	; rising edge, baud rate 500 kHz for IWM, and
	movwf	BAUDCON		; transmit all zeroes so RD line starts high
	clrf	SPBRGH
	movlw	15
	movwf	SPBRGL
	movlw	B'10110110'
	movwf	TXSTA
	movlw	B'10000000'
	movwf	RCSTA
	clrf	TXREG

	banksel	CLC1CON
	movlw	0x15 ;DT	;       ____
	movwf	CLC1SEL0	; DT --|    \           ____
	movlw	0x14 ;CK 	;      |     O---------|    \__
	movwf	CLC1SEL1	; CK --|____/   !PH2 --|____/  |  ____
	movlw	0x01 ;CLCIN1/PH1;                              `--\   \___ RD
	movwf	CLC1SEL2	;                       ____   ,--/___/
	movlw	0x02 ;CLCIN2/PH2;                PH1 --|    \__|
	movwf	CLC1SEL3	;                PH2 --|____/
	movlw	B'00000101'
	movwf	CLC1GLS0
	movlw	B'01000000'
	movwf	CLC1GLS1
	movlw	B'00100000'
	movwf	CLC1GLS2
	movlw	B'10000000'
	movwf	CLC1GLS3
	clrf	CLC1POL
	movlw	B'10000000'
	movwf	CLC1CON
	movlw	0x1A ;T2_match	;            0__                 0__
	movwf	CLC2SEL0	;             __|__               __|__
	movlw	0x03 ;CLCIN3/WR	;            |  S  |             |  S  |
	movwf	CLC2SEL1	;       WR --|D   Q|-------------|D   Q|
	movlw	B'00000010'	;            |     |             |     |
	movwf	CLC2GLS0	; T2_match --|>    |  T2_match --|>    |
	movlw	B'00001000'	;            |__R__|             |__R__|
	movwf	CLC2GLS1	;             __|                 __|
	clrf	CLC2GLS2	;            0                   0
	clrf	CLC2GLS3	;              CLC2                CLC3
	clrf	CLC2POL
	movlw	B'10000100'
	movwf	CLC2CON
	movlw	0x1A ;T2_match
	movwf	CLC3SEL0
	movlw	0x05 ;LC2_out
	movwf	CLC3SEL1
	movlw	B'00000010'
	movwf	CLC3GLS0
	movlw	B'00001000'
	movwf	CLC3GLS1
	clrf	CLC3GLS2
	clrf	CLC3GLS3
	clrf	CLC3POL
	movlw	B'10000100'
	movwf	CLC3CON

	banksel	T2CON		;Timer2 1:1 with instruction clock and matches
	movlw	15		; every 16 cycles
	movwf	PR2
	movlw	B'00000100'
	movwf	T2CON

	banksel	OPTION_REG	;Timer0 1:1 with instruction clock, INT reacts
	movlw	B'11011111'	; to rising edges
	movwf	OPTION_REG

	banksel	CLCIN0PPS	;CLCIN1=RA2=PH1, CLCIN2=RA3=PH2, CLCIN3=RC5=WR,
	movlw	B'00000010'	;MISO=RC1, INT=RA5=!ENBL
	movwf	CLCIN1PPS
	movlw	B'00000011'
	movwf	CLCIN2PPS
	movlw	B'00010101'
	movwf	CLCIN3PPS
	movlw	B'00010001'
	movwf	SSPDATPPS
	movlw	B'00000101'
	movwf	INTPPS

	banksel	RC4PPS		;RC4=LC1OUT, RC2=MOSI, RC0=SCK
	movlw	B'00000100'
	movwf	RC4PPS
	movlw	B'00010010'
	movwf	RC2PPS
	movlw	B'00010000'
	movwf	RC0PPS

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA
	clrf	ANSELC

	banksel	LATA		;Next !ENBL and !CS high
	movlw	B'00111111'
	movwf	LATA
	movwf	LATC

	banksel	TRISA		;Next !ENBL, !CS, SCK, MOSI outputs, RD output
	movlw	B'00111110'	; but tristated for now, MISO, WR, PH3-0, !ENBL
	movwf	TRISA		; inputs
	movlw	B'00110010'
	movwf	TRISC

	banksel	INLVLA		;TTL instead of Schmitt trigger for input ports
	clrf	INLVLA
	clrf	INLVLC

	movlw	12		;Delay approximately 2 ms at an instruction
	movwf	X0		; clock of 2 MHz until the PLL kicks in and the
PllWait	DELAY	110		; instruction clock gears up to 8 MHz
	decfsz	X0,F
	bra	PllWait

	movlw	B'00010000'	;INT interrupts on
	movwf	INTCON

	movlp	8
	goto	SysInit


;;; Error Handler ;;;

ErrorHandler
	movwf	X0		;Save the error code that brought us here
	bcf	INTCON,GIE	;Make sure interrupts are off
	movlw	B'00110010'	;Tristate RD (this is probably not necessary,
	tris	7		; but.. belt and suspenders)
	banksel	RC4PPS		;Return RD to be driven by its latch so it's
	clrf	RC4PPS		; always high (doesn't matter really)
	banksel	CLCIN0PPS	;Reconfigure CLC1 to pass through !ENBL to the 
	movlw	B'00000101'	; next device in the chain so we don't have
	movwf	CLCIN0PPS	; to do this in software
	banksel	CLC1CON		; "
	clrf	CLC1SEL0	; "
	movlw	B'00000010'	; "
	movwf	CLC1GLS0	; "
	clrf	CLC1GLS1	; "
	clrf	CLC1GLS2	; "
	clrf	CLC1GLS3	; "
	movlw	B'10000001'	; "
	movwf	CLC1CON		; "
	banksel	RA0PPS		; "
	movlw	B'00000100'	; "
	movwf	RA0PPS		; "
	movlb	0		; "
ErrHan0	movlw	10		;Delay five seconds between bursts of blinking
	movwf	X1		; "
ErrHan1	call	OneHalfSecond	; "
	decfsz	X1,F		; "
	bra	ErrHan1		; "
	movf	X0,W		;Move the error code into the shift register
	movwf	X1		; and shift the MSB into carry while setting
	lslf	X1,F		; the LSB as an end sentinel
	bsf	X1,0		; "
ErrHan2	bcf	CS_PORT,CS_PIN	;Lower !CS for 0.5s for a 1 or 1.5s for a 0,
	call	OneHalfSecond	; in either case making the bit cell 2s
	btfsc	STATUS,C	; "
	bsf	CS_PORT,CS_PIN	; "
	call	OneHalfSecond	; "
	call	OneHalfSecond	; "
	bsf	CS_PORT,CS_PIN	; "
	call	OneHalfSecond	; "
	lslf	X1,F		;Shift the next bit into carry; if the bit we
	btfsc	STATUS,Z	; just blinked out was the last, go back to
	bra	ErrHan0		; delay and start over, else blink out the next
	bra	ErrHan2		; bit

OneHalfSecond
	movlw	20		;Delay 3947587 cycles (including call), which
	movwf	X3		; is a little less than a half second
	clrf	X2		; "
OneHal0	DELAY	0		; "
	decfsz	X2,F		; "
	bra	OneHal0		; "
	decfsz	X3,F		; "
	bra	OneHal0		; "
	return			; "


;;; Mainline ;;;

Unselected
	movlw	B'00110010'	;Tristate RD (the interrupt handler will have
	tris	7		; done this already, but.. belt and suspenders)
	movlb	3		;Deassert !HSHK
	clrf	TXREG		; "
	movlb	0		;Deassert next !ENBL (the interrupt handler
	movlw	B'00111111'	; will have deasserted next !ENBL already,
	movwf	PORTA		; but.. belt and suspenders)
	clrf	DEVNUM		;Reset selected device number to zero
	btfsc	EN_PORT,EN_PIN	;Wait for Mac to drive !ENBL low
	bra	$-1		; "
	movlw	B'00100010'	;Drive RD
	tris	7		; "
	;fall through

IdleLoop
	movf	PORTA,W
	brw
	nop
	bra	IdleLoop	;HOFF asserted, but there's nothing to holdoff
	nop			; from, so wait for a state change
	bra	IdleLoop	;Data mode, but we haven't done a handshake, so
	nop			; wait for a state change
	bra	IdleLoop	;Idle state, we can do that
	nop			; "
	movlp	8		;HOST asserted, Mac wants to talk, so let's go
	goto	Receiver	; make that happen
	bra	IdleLoop	;RESET asserted, we're already idling, so how
	nop			; much more reset can we get
	bra	IdleLoop	;Mac wants us to drive RD low, the CLC will
	nop			; take care of that, so wait for state change
	bra	IdleLoop	;Mac wants us to drive RD high, the CLC will
	nop			; take care of that, so wait for state change
	bra	IdleLoop	;Mac wants us to drive RD high, the CLC will
	nop			; take care of that, so wait for state change
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	NextDevice	;Mac wants to select the next device
	nop			; "
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again
	bra	Unselected	;Mac has unselected us, do necessary things
	nop			; and wait to be selected again

NextDevice
	movf	DEVNUM,W	;If we've already selected the maximum device
	xorwf	DEVMAX,W	; number, select the daisy chained device
	btfsc	STATUS,Z	; downstream from us
	bra	NoMoreDevices	; "
	incf	DEVNUM,F	;Select the next emulated device number
	btfsc	PH_PORT,PH3_PIN	;Wait until the PH3 pin pulse has ended
	bra	$-1		; "
	bra	IdleLoop	;Return to idle loop

NoMoreDevices
	movlw	B'00110010'	;Tristate RD
	tris	7		; "
	bcf	NE_PORT,NE_PIN	;Drive the next !ENBL low
	btfss	EN_PORT,EN_PIN	;Wait for !ENBL to go high
	bra	$-1		; "
	bra	Unselected	;We've been unselected, take care of that


;;; Lookup Tables ;;;

	org	0x200

;The TashTwenty logo in the form of a 32x32 bitmap icon with mask
LutIcon
	#include	icon.inc


	org	0x300

;LUT for translating IWM inversions into IWM bytes
LutInv
	#include	inv.inc


	org	0x400

;LUT for flipping bytes left to right for use with USART as IWM transmitter
LutFlip
	#include	flip.inc


	org	0x500

;LUT for high byte of CCITT CRC16
LutCrc7
	#include	crc7.inc


	org	0x600

;LUT for high byte of CCITT CRC16
LutCrc16H
	#include	crc16hi.inc


	org	0x700

;LUT for low byte of CCITT CRC16
LutCrc16L
	#include	crc16lo.inc


	org	0x800

;;; System Initialization ;;;

GoError
	movlp	0		;Something went wrong, so pass !ENBL through
	goto	ErrorHandler	; and try to let the user know what happened

SysInit
	call	MmcInit		;Try to initialize the card
	btfsc	M_FLAGS,M_FAIL	;If card initialization failed, pass the error
	bra	GoError		; code to the error handler
	movlb	4		;Since initialization succeeded, crank the
	movlw	B'00100001'	; speed of the SPI interface up to 2 MHz
	movwf	SSPCON1		; "
	movlb	0		; "
	movlw	4		;Try to read the MBR up to four times which is,
	movwf	X2		; frankly, excessive
SysIni0	bsf	CS_PORT,CS_PIN	;Deassert !CS
	movlw	0x51		;Set up a read command for the card (R1-type
	movwf	M_CMDN		; response)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	clrf	M_ADR3		;Point the card address to the master boot
	clrf	M_ADR2		; record
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	CS_PORT,CS_PIN	;Assert !CS
	call	MmcCmd		;Send the read command
	movf	M_CMDN,W	;Treat any error flag as a failed command
	andlw	B'11111110'	; "
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	movlw	10		;If the operation failed, give up and pass a
	btfsc	M_FLAGS,M_FAIL	; code to the error handler
	bra	GoError		; "
	call	MmcGetData	;Get the card ready to read out data
	movlw	11		;If the operation failed, give up and pass a
	btfsc	M_FLAGS,M_FAIL	; code to the error handler
	bra	GoError		; "
	clrf	M_CRCH		;Clear the CRC registers for our read of the
	clrf	M_CRCL		; MBR
	movlw	0x22		;Point the push pointer off into space and read
	movwf	FSR1H		; 446 bytes to get us past the bootstrap area;
	movlw	0		; we want to update the CRC registers with this
	call	MmcReadData	; data but we don't use it for anything
	movlw	190		; "
	call	MmcReadData	; "
	movlw	0x21		;Read the remaining 66 bytes of the MBR (the
	movwf	FSR0H		; partition table and the 0x55 0xAA signature,
	movwf	FSR1H		; hopefully) into memory
	movlw	0x80		; "
	movwf	FSR0L		; "
	movwf	FSR1L		; "
	movlw	66		; "
	call	MmcReadData	; "
	call	MmcCheckCrc	;Check the CRC of the data read against what
	btfsc	STATUS,Z	; we calculated; if they don't match, try
	bra	SysIni1		; again, if we already tried four times, give
	decfsz	X2,F		; up and pass a code to the error handler
	bra	SysIni0		; "
	movlw	12		; "
	bra	GoError		; "
SysIni1	bsf	CS_PORT,CS_PIN	;Deassert !CS
	moviw	-2[FSR1]	;Check that the MBR signature matches, if it
	xorlw	0x55		; doesn't, this isn't an MBR and we can't do
	movlw	13		; anything, so pass a code to the error handler
	btfss	STATUS,Z	; "
	bra	GoError		; "
	moviw	-1[FSR1]	; "
	xorlw	0xAA		; "
	movlw	13		; "
	btfss	STATUS,Z	; "
	bra	GoError		; "
	movlw	4		;Iterate through the four primary MBR
	movwf	X3		; partitions
SysIni2	movlw	4		;Try loading the block four times which is,
	movwf	X2		; frankly, excessive
SysIni3	bsf	CS_PORT,CS_PIN	;Deassert !CS
	movlw	1		;Set the first byte of the partition entry as
	movwf	INDF0		; a flag for later use
	moviw	4[FSR0]		;Interpret the type field of this partition as
	movwf	M_ADR0		; a block number and set up to read that block
	btfsc	STATUS,Z	;If it's zero, skip doing this
	bra	SysIni4		; "
	xorlw	0xAF		;If it's 0xAF (HFS partition), skip doing this
	btfsc	STATUS,Z	; "
	bra	SysIni4		; "
	movlw	0x51		;Set up a read command for the card (R1-type
	movwf	M_CMDN		; response)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	clrf	M_ADR3		;The rest of the address is all zeroes
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	call	MmcConvAddr	;Convert address to bytes if necessary
	bcf	CS_PORT,CS_PIN	;Assert !CS
	call	MmcCmd		;Send the read command
	movf	M_CMDN,W	;Treat any error flag as a failed command
	andlw	B'11111110'	; "
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	movlw	14		;If the operation failed, give up and pass a
	btfsc	M_FLAGS,M_FAIL	; code to the error handler
	bra	GoError		; "
	call	MmcGetData	;Get the card ready to read out data
	movlw	15		;If the operation failed, give up and pass a
	btfsc	M_FLAGS,M_FAIL	; code to the error handler
	bra	GoError		; "
	clrf	M_CRCH		;Clear the CRC registers for our read of the
	clrf	M_CRCL		; icon block
	movlw	0x22		;Point the push pointer off into space and read
	movwf	FSR1H		; 256 bytes to read what we hope is the word
	movlw	0		; 'ICON' repeated 64 times and check this by
	call	MmcReadData	; means of checking that the CRC is 0xBEF6
	movf	M_CRCH,W	;If the CRC doesn't match, clear that flag we
	xorlw	0xBE		; set earlier
	btfss	STATUS,Z	; "
	clrf	INDF0		; "
	movf	M_CRCL,W	; "
	xorlw	0xF6		; "
	btfss	STATUS,Z	; "
	clrf	INDF0		; "
	movlw	0		;Read the rest of the block and throw it away
	call	MmcReadData	; "
	call	MmcCheckCrc	;Check the CRC of the data read against what
	btfsc	STATUS,Z	; we calculated; if they don't match, try
	bra	SysIni4		; again, if we already tried four times, give
	decfsz	X2,F		; up and pass a code to the error handler
	bra	SysIni3		; "
	movlw	16		; "
	bra	GoError		; "
SysIni4	bsf	CS_PORT,CS_PIN	;Deassert !CS
	movlw	0		;If the CRC of the icon block didn't match,
	btfss	INDF0,0		; clear the partition type to zero so we don't
	movwi	4[FSR0]		; present it as a drive
	addfsr	FSR0,16		;Move on to the next partition until we've
	decfsz	X3,F		; done all four
	bra	SysIni2		; "
	movlw	0x21		;Point both pointers at the partition table in
	movwf	FSR0H		; memory
	movwf	FSR1H		; "
	movlw	0x80		; "
	movwf	FSR0L		; "
	movwf	FSR1L		; "
	movlw	4		;Iterate through all four partition entries
	movwf	X0		; "
	movlw	-1		;Start off with zero devices and increment from
	movwf	DEVMAX		; there
SysIni5	moviw	4[FSR0]		;If this partition's type is zero or was made
	btfsc	STATUS,Z	; zero because it pointed to an invalid icon,
	bra	SysIni6		; skip over it
	moviw	15[FSR0]	;If this partition's size is greater than
	btfss	STATUS,Z	; 0x7FFFFF blocks, skip over it
	bra	SysIni6		; "
	moviw	14[FSR0]	; "
	andlw	B'10000000'	; "
	btfss	STATUS,Z	; "
	bra	SysIni6		; "
	moviw	8[FSR0]		;Copy the starting block number and block
	movwi	3[FSR1]		; count from the MBR partition entry into a 
	moviw	9[FSR0]		; smaller partition entry for our use
	movwi	2[FSR1]		; "
	moviw	10[FSR0]	; "
	movwi	1[FSR1]		; "
	moviw	11[FSR0]	; "
	movwi	0[FSR1]		; "
	moviw	12[FSR0]	; "
	movwi	7[FSR1]		; "
	moviw	13[FSR0]	; "
	movwi	6[FSR1]		; "
	moviw	14[FSR0]	; "
	movwi	5[FSR1]		; "
	moviw	4[FSR0]		; "
	movwi	4[FSR1]		; "
	incf	DEVMAX,F	;Increment the number of valid devices
	addfsr	FSR1,8		;Move push pointer ahead to next entry
SysIni6	addfsr	FSR0,16		;Move pop pointer ahead to next entry
	decfsz	X0,F		;Decrement entry count and go to process the
	bra	SysIni5		; next if applicable
	incf	DEVMAX,W	;If there are no valid partitions, that's an
	movlw	17		; error too
	btfsc	STATUS,Z	; "
	bra	GoError		; "
	bsf	INTCON,GIE	;Setup is all done, enable interrupts and
	movlp	0		; wait for some drive activity to respond to
	goto	Unselected	; "


;;; Receiver ;;;

Receiver
	movlw	4		;Spend about 262.144 ms waiting for the state
	movwf	X1		; machine to try and get the card back to
Receiv0	clrf	TMR1H		; waiting for a command or a write token, which
	clrf	TMR1L		; is about how long we have before Mac tires of
	movlw	MSSWrEtTok - MSS; waiting for us to acknowledge a command (we
	btfss	M_FLAGS,M_ONGWR	; actually have ~350 ms, but be cautious);
	movlw	MSSWaitCmd - MSS; hopefully this will be enough, spec says that
	call	MmcWaitState	; writes may take up to 200 ms
	decfsz	X1,F		; "
	bra	Receiv0		; "
	movlb	3		;Assert !HSHK to signal to Mac that we're
	btfss	TXSTA,TRMT	; ready to receive
	bra	$-1		; "
	movlw	0x80		; "
	movwf	TXREG		; "
	movlb	7		;Set the IOC peripheral up as required by the
	movlw	B'00000100'	; interrupt handler, catching rising edges on
	movwf	IOCAP		; PH1 (signalling the end of a transmission)
	clrf	IOCAN		; and both rising and falling edges on WR (to
	movlw	B'00100000'	; catch the start of a new byte)
	movwf	IOCCP		; "
	movwf	IOCCN		; "
	clrf	IOCAF		; "
	clrf	IOCCF		; "
	movlb	0		; "
	movlw	0x21		;Move pointers to the beginning of the queue
	movwf	FSR0H		; in linear memory from 0x2100-0x217F
	movwf	FSR1H		; "
	clrf	FSR0L		; "
	clrf	FSR1L		; "
	clrf	IWMFSAP		;Point IWM FSA pointer at starting state
	clrf	IWMFLAG		;Clear IWM flags before we start interrupts
	bsf	INTCON,TMR0IE	;Enable interrupt on Timer0, starting receive
	;fall through

WaitForCmd
	movlw	0x05		;Check if we've gotten five bytes yet, which is
	subwf	FSR1L,W		; most of the header of a command, enough to
	btfsc	STATUS,C	; tell what it is; if we have, skip ahead to
	goto	ProcessCmd	; process it
	btfss	IWMFLAG,IWMOVER	;If the transmission hasn't ended yet, loop
	bra	WaitForCmd	; until it does or it reaches five bytes
	call	ReturnToIdle	;If the transmission ended before it reached
	movlp	0		; five bytes, wait until Mac returns to the
	goto	IdleLoop	; idle state and go back to the idle loop

ProcessCmd
	clrf	CHKSUM		;Start checksum off at zero
	clrf	RESPERR		;Response error is zero by default
	moviw	FSR0++		;Store the first five bytes of the command
	movwf	CMDNUM		; outside the queue so they don't get
	subwf	CHKSUM,F	; overwritten, updating the checksum as we go
	moviw	FSR0++		; "
	movwf	CMDCNT		; "
	subwf	CHKSUM,F	; "
	moviw	FSR0++		; "
	movwf	CMDHIGH		; "
	subwf	CHKSUM,F	; "
	moviw	FSR0++		; "
	movwf	CMDMED		; "
	subwf	CHKSUM,F	; "
	moviw	FSR0++		; "
	movwf	CMDLOW		; "
	subwf	CHKSUM,F	; "
	decf	CMDNUM,W	;If command byte is 1 or 65 (write or continued
	btfsc	STATUS,Z	; write) we can't afford to wait for the end of
	goto	CmdWrite	; the command, so start processing immediately;
	addlw	-64		; fall through to wait for other commands to
	btfsc	STATUS,Z	; finish before responding to them
	goto	CmdWriteCont	; "
	;fall through

WaitToFinish
	btfsc	IWMFLAG,IWMOVER	;Check if the last byte's been received and
	bra	WaitFn0		; skip ahead if it has
	movf	FSR0L,W		;If we haven't yet received a byte, loop until
	xorwf	FSR1L,W		; we have
	btfsc	STATUS,Z	; "
	bra	WaitToFinish	; "
	moviw	FSR0++		;Update the checksum with the byte and loop
	bcf	FSR0L,7		; again - besides writes, which don't use this
	subwf	CHKSUM,F	; loop, all the commands we care about have
	bra	WaitToFinish	; the important info in the first five bytes
WaitFn0	movf	FSR0L,W		;If we've updated the checksum with all the
	xorwf	FSR1L,W		; bytes in this command, skip ahead
	btfsc	STATUS,Z	; "
	goto	ReceiveDone	; "
	moviw	FSR0++		;Update the checksum with this byte and loop
	bcf	FSR0L,7		; to process the next, if there is one
	subwf	CHKSUM,F	; "
	bra	WaitFn0		; "

ReceiveDone
	call	ReturnToIdle	;Return to the idle state (or the idle loop)
	movf	CHKSUM,W	;If checksum didn't match the data (that is,
	btfss	STATUS,Z	; make it sum to zero), we send a NAK so
	goto	BadChecksum	; hopefully Mac will send the command again
	movf	CMDNUM,W	;If the command number is 0, this is a read
	btfsc	STATUS,Z	; command
	goto	CmdRead		; "
	addlw	-3		;If the command number is 3, this is a status
	btfsc	STATUS,Z	; command; if it's anything else, we don't know
	goto	CmdStatus	; how to handle it, so fake a response
	clrf	CMDCNT		; "
	;fall through

CmdPlaceholder
	call	RequestToSend	;Assert !HSHK and wait to be in state 1; if
	btfss	STATUS,Z	; we ended up in an unexpected state, bail out
	bra	CmdPla3		; to idle loop
	movlw	0x80		;Initialize the state used by the WriteByte
	movwf	IWMLSBS		; subprogram
	clrf	CHKSUM		; "
	movlw	0xAA		;Send the sync byte
	call	WriteIwmByte	; "
	movf	CMDNUM,W	;Form a response code from the command number
	andlw	B'00111111'	; by setting the MSB (and clearing byte 6,
	iorlw	B'10000000'	; which seems to indicate a continued command)
	call	WriteByte	; and send that byte
	movf	CMDCNT,W	;Write byte count, command error byte (in case
	call	WriteByte	; this routine is being used for an error
	movf	RESPERR,W	; response), and pad bytes
	call	WriteByte	; "
	movlw	0		; "
	call	WriteByte	; "
	movlw	0		; "
	call	WriteByte	; "
	movlw	0		; "
	call	WriteByte	; "
	decf	IWMREXP,F	;If the expected response length was only one
	btfsc	STATUS,Z	; group, skip ahead to send the checksum and
	bra	CmdPla2		; finish
CmdPla0	movlw	7		;Pad out the response with groups of zeroes
	movwf	X0		; until it reaches the size the command called
CmdPla1	movlw	0		; for
	call	WriteByte	; "
	decfsz	X0,F		; "
	bra	CmdPla1		; "
	decfsz	IWMREXP,F	; "
	bra	CmdPla0		; "
CmdPla2	movf	CHKSUM,W	;Write the checksum to finish out the last
	call	WriteByte	; group
	call	ReturnToIdle	;Deassert !HSHK and wait to be in state 2
CmdPla3	movlp	0		;Return to the idle loop
	goto	IdleLoop	; "

BadChecksum
	call	RequestToSend	;Assert !HSHK and wait to be in state 1; if
	btfss	STATUS,Z	; we ended up in an unexpected state, bail out
	bra	BadChk1		; to idle loop
	movlw	0x80		;Initialize the state used by the WriteByte
	movwf	IWMLSBS		; subprogram
	clrf	CHKSUM		; "
	movlw	0xAA		;Send the sync byte
	call	WriteIwmByte	; "
	movlw	0x7F		;Send the response code 0x7F, which is used to
	call	WriteByte	; mean NAK, a prompt for Mac to resend
	movlw	5		;Pad out the rest of the group
	movwf	X0		; "
BadChk0	movlw	0		; "
	call	WriteByte	; "
	decfsz	X0,F		; "
	bra	BadChk0		; "
	movf	CHKSUM,W	;Write the checksum to finish out the group
	call	WriteByte	; "
	call	ReturnToIdle	;Deassert !HSHK and wait to be in state 2
BadChk1	movlp	0		;Return to the idle loop
	goto	IdleLoop	; "

CmdRead
	movlw	ERRCODE		;Ready an error code in case we need to jump
	movwf	RESPERR		; into the placeholder response routine
	call	SetupCmdAddr	;Set up the card address for this command
	btfsc	STATUS,C	;If the read is out of bounds, send an error
	goto	CmdPlaceholder	; response
CmdRea0	bsf	M_FLAGS,M_RDCMD	;Set the state machine to do a read for us
	bcf	M_FLAGS,M_ONGWR	;Cancel any ongoing write
	movlw	4		;Spend about 262.144 ms waiting for the state
	movwf	X1		; machine to prepare the card to read out the
CmdRea5	clrf	TMR1H		; data, which is about how long we have before
	clrf	TMR1L		; Mac tires of waiting for a response from us
	movlw	MSSRdData - MSS	; (we actually have ~350 ms, but be cautious)
	call	MmcWaitState	; "
	decfsz	X1,F		; "
	bra	CmdRea5		; "
	btfsc	PIR1,TMR1IF	;If there was any failure, respond with a
	goto	CmdPlaceholder	; placeholder with an error code
	clrf	M_CRCH		;Clear the CRC registers for our read of the
	clrf	M_CRCL		; block
	movlw	0x21		;Point both pointers to the 128-byte queue and
	movwf	FSR0H		; read the first two bytes of the sector out of
	movwf	FSR1H		; the card - we need to do this so we stay two
	clrf	FSR0L		; bytes ahead while writing the sector to the
	clrf	FSR1L		; USART so we know immediately if the CRC has
	movlw	2		; been matched or not and can mangle the
	call	MmcReadData	; checksum if it doesn't match
	movlw	0x80		;Initialize the state used by the WriteByte
	movwf	IWMLSBS		; subprogram
	clrf	CHKSUM		; "
	call	RequestToSend	;Assert !HSHK and wait to be in state 1; if
	btfss	STATUS,Z	; we ended up in an unexpected state, skip
	clrf	IWMLSBS		; writes so card stays in a good state
	movlw	0xAA		;Send the sync byte
	call	WriteIwmByte	; "
	movlw	0x80		;Write the response code for a read command
	call	WriteByte	; "
	movf	CMDCNT,W	;Write out the count of remaining blocks to
	call	WriteByte	; send
	movlw	24		;Pad out the rest of the group and send 20
	movwf	X0		; empty tag bytes before sector data
CmdRea1	movlw	0		; "
	call	WriteByte	; "
	decfsz	X0,F		; "
	bra	CmdRea1		; "
	movlw	2		;Set up to copy 512 bytes from card to Mac
	movwf	X3		; "
	clrf	X2		; "
CmdRea2	moviw	FSR0++		;Pick up the next byte to write from the queue
	call	WriteByte	; and write it
	movlw	1		;Read a byte out of the card
	call	MmcReadData	; "
	bcf	FSR0L,7		;Wrap both pointers in the queue
	bcf	FSR1L,7		; "
	decfsz	X2,F		;Decrement the counter and loop if bytes remain
	bra	CmdRea2		; "
	decfsz	X3,F		; "
	bra	CmdRea2		; "
	movlw	MSSWaitCmd - MSS;Finished with read, so set state back to
	movwf	M_STATE		; WaitCmd
	movf	M_CRCH,W	;Feeding the CRC through the CRC registers
	iorwf	M_CRCL,W	; should leave it at zero if the CRC is right;
	btfss	STATUS,Z	; if it's not, send the wrong checksum so Mac
	bra	CmdRea4		; knows something is wrong
	movf	CHKSUM,W	;Finish the block off with the (right) checksum
	call	WriteByte	; "
	call	MmcIncAddr	;Increment the block address for next time
	call	ReturnToIdle	;Return to the idle state
	movf	IWMLSBS,F	;If we got into an unexpected state somewhere,
	btfsc	STATUS,Z	; don't try to send the next block, just hurry
	bra	CmdRea3		; back to the idle loop
	decfsz	CMDCNT,F	;Decrement the block count and go to send the
	bra	CmdRea0		; next block if any are left
CmdRea3	bsf	CS_PORT,CS_PIN	;Deassert !CS
	movlp	0		;Return to the idle loop
	goto	IdleLoop	; "
CmdRea4	comf	CHKSUM,W	;CRC failed, so write what we know is the wrong
	call	WriteByte	; checksum to finish the transmission
	call	ReturnToIdle	;Return to the idle state quickly
	bra	CmdRea3

CmdStatus
	movlw	0x21		;Point to the partition information block for
	movwf	FSR0H		; the selected device
	swapf	DEVNUM,W	; "
	lsrf	WREG,W		; "
	iorlw	B'10000000'	; "
	movwf	FSR0L		; "
	moviw	4[FSR0]		;If the partition type is 0xAF, we're using a
	xorlw	0xAF		; built-in icon, not one loaded from the card,
	btfsc	STATUS,Z	; so skip setting up the read
	bra	CmdSta0		; "
	bsf	M_FLAGS,M_RDCMD	;Set the state machine to do a read for us
	bcf	M_FLAGS,M_ONGWR	;Cancel any ongoing write
	clrf	M_ADR3		;Point to the address of the icon to be loaded
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	moviw	4[FSR0]		; "
	movwf	M_ADR0		; "
	call	MmcConvAddr	;Convert address to bytes if necessary
	movlw	4		;Spend about 262.144 ms waiting for the state
	movwf	X1		; machine to prepare the card to read out the
CmdSta8	clrf	TMR1H		; icon, which is about how long we have before
	clrf	TMR1L		; Mac tires of waiting for a response from us
	movlw	MSSRdData - MSS	; (we actually have ~350 ms, but be cautious)
	call	MmcWaitState	; "
	decfsz	X1,F		; "
	bra	CmdSta8		; "
	btfsc	PIR1,TMR1IF	;If it timed out, use the built in icon instead
	bra	CmdSta0		; "
	movlw	0x22		;Point the push pointer off into space and
	movwf	FSR1H		; read and throw away the first 256 bytes of
	movlw	0		; the icon sector
	call	MmcReadData	; "
CmdSta0	movlw	0x80		;Initialize the state used by the WriteByte
	movwf	IWMLSBS		; subprogram
	clrf	CHKSUM		; "
	call	RequestToSend	;Assert !HSHK and wait to be in state 1; if
	btfss	STATUS,Z	; we ended up in an unexpected state, skip
	clrf	IWMLSBS		; writes so card stays in a good state
	movlw	0xAA		;Send the sync byte
	call	WriteIwmByte	; "
	movlw	0x83		;Write the response code for a status command
	call	WriteByte	; "
	movlw	B'00000101'	;Write out nine bytes that happen to all be
	movwf	X0		; 0x01 or 0x00 - pad byte, status byte, three
	bcf	STATUS,C	; pad bytes, device type word (0x0001), device
CmdSta1	movlw	0		; manufacturer word (0x0001)
	rlf	WREG,W		; "
	call	WriteByte	; "
	movf	X0,W		; "
	btfsc	STATUS,Z	; "
	bra	CmdSta2		; "
	lslf	X0,F		; "
	bra	CmdSta1		; "
CmdSta2	movlw	0xE6		;Write the device characteristics: mount, read,
	call	WriteByte	; write, icon included, disk in place
	moviw	5[FSR0]		;Write the device size which I BELIEVE is
	movwf	X2		; really the max sector number (so we have to
	moviw	6[FSR0]		; decrement from the partition table)
	movwf	X1		; TODO look into this and make sure
	moviw	7[FSR0]		; "
	movwf	X0		; "
	movlw	0xFF		; "
	addwf	X0,F		; "
	addwfc	X1,F		; "
	addwfc	X2,W		; "
	call	WriteByte	; "
	movf	X1,W		; "
	call	WriteByte	; "
	movf	X0,W		; "
	call	WriteByte	; "
	movlw	56		;Write the spare count word, the bad block
	movwf	X0		; count word, and the manufacturer reserved
CmdSta3	movlw	0		; area (52 bytes), all as zeroes
	call	WriteByte	; "
	decfsz	X0,F		; "
	bra	CmdSta3		; "
	clrf	X2		;Clear the did-we-use-the-built-in-icon flag
	moviw	4[FSR0]		;If the partition type is 0xAF or we had a fail
	xorlw	0xAF		; while reading from the card, we're using the
	btfss	STATUS,Z	; built-in icon, so skip ahead to do that
	btfsc	PIR1,TMR1IF	; "
	bra	CmdSta4		; "
	movlw	0		;Copy 256 bytes from the card to the Mac
	call	MmcCopyData	; "
	bra	CmdSta6		;Skip ahead to finish the status block
CmdSta4	movlw	high LutIcon|128;Copy the built-in icon from flash to the Mac
	movwf	FSR0H		; "
	clrf	FSR0L		; "
	clrf	X0		; "
CmdSta5	moviw	FSR0++		; "
	call	WriteByte	; "
	decfsz	X0,F		; "
	bra	CmdSta5		; "
	bsf	X2,0		;Set the did-we-use-the-built-in-icon flag
CmdSta6	movlw	10		;The credits
	call	WriteByte	; "
	movlw	'T'		; "
	call	WriteByte	; "
	movlw	'a'		; "
	call	WriteByte	; "
	movlw	's'		; "
	call	WriteByte	; "
	movlw	'h'		; "
	call	WriteByte	; "
	movlw	'T'		; "
	call	WriteByte	; "
	movlw	'w'		; "
	call	WriteByte	; "
	movlw	'e'		; "
	call	WriteByte	; "
	movlw	'n'		; "
	call	WriteByte	; "
	movlw	't'		; "
	call	WriteByte	; "
	movlw	'y'		; "
	call	WriteByte	; "
	movlw	5		;Write the spare count word, the bad block
	movwf	X0		; count word, and the manufacturer reserved
CmdSta7	movlw	0		; area (52 bytes), all as zeroes
	call	WriteByte	; "
	decfsz	X0,F		; "
	bra	CmdSta7		; "
	movf	CHKSUM,W	;Finish the block off with the checksum
	call	WriteByte	; "
	btfss	X2,0		;If we didn't use the built-in icon, finish up
	call	MmcCheckCrc	; with the card by ignoring the CRC and setting
	movlw	MSSWaitCmd - MSS; the state back to WaitCmd and deasserting !CS
	btfss	X2,0		; "
	movwf	M_STATE		; "
	btfss	X2,0		; "
	bsf	CS_PORT,CS_PIN	; "
	call	ReturnToIdle	;Return to the idle state
	movlp	0		;Return to the idle loop
	goto	IdleLoop	; "

CmdWrite
	movlw	ERRCODE		;Ready an error code in case we need to jump
	movwf	RESPERR		; into the placeholder response routine
	call	SetupCmdAddr	;Set up the card address for this command
	btfsc	STATUS,C	;If the write is out of bounds, send an error
	goto	WaitToFinish	; response after we've received the command
	movf	M_STATE,W	;If we're not in the await-command state, we
	xorlw	MSSWaitCmd - MSS; won't be able to do this write, so send an
	btfss	STATUS,Z	; error response
	goto	WaitToFinish	; "
	btfsc	M_FLAGS,M_ONGWR	;If we're starting a new write even though a
	bra	CmdWri6		; multiblock write is ongoing, handle it
	movlw	0x59		;Set up a multiblock write command for the card
	movwf	M_CMDN		; (R1-type reply)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	bcf	CS_PORT,CS_PIN	;Assert !CS
	call	MmcCmd		;Send the command
	movf	M_CMDN,W	;Treat any error flag as a failed command
	andlw	B'11111110'	; "
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	btfsc	M_FLAGS,M_FAIL	;If there was any failure, respond with a
	goto	WaitToFinish	; placeholder with an error code
	bsf	M_FLAGS,M_ONGWR	;Flag that a multiblock write is in progress
CmdWri0	movlb	4		;Clock a dummy byte while keeping MOSI high
	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlw	0xFC		;Clock the multiblock data token into the card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlb	0		; "
	clrf	M_CRCH		;Clear the CRC registers for our write of the
	clrf	M_CRCL		; block
	movlw	21		;Skip over the command pad byte plus the 20 tag
	movwf	X0		; bytes
CmdWri1	btfsc	IWMFLAG,IWMOVER	;It shouldn't happen, but we do need to be
	bra	CmdWri2		; ready in case transmission ends prematurely
	movf	FSR0L,W		;If we haven't yet received a byte, loop until
	xorwf	FSR1L,W		; we have
	btfsc	STATUS,Z	; "
	bra	CmdWri1		; "
	moviw	FSR0++		;Update the checksum with the byte and loop
	bcf	FSR0L,7		; again until we've done all the necessary
	subwf	CHKSUM,F	; byte-skipping
CmdWri2	decfsz	X0,F		; "
	bra	CmdWri1		; "
	movlw	2		;Copy 512 data bytes from Mac
	movwf	X1		; "
CmdWri3	btfsc	IWMFLAG,IWMOVER	;It shouldn't happen, but we do need to be
	bra	CmdWri4		; ready in case transmission ends prematurely
	movf	FSR0L,W		;If we haven't yet received a byte, loop until
	xorwf	FSR1L,W		; we have
	btfsc	STATUS,Z	; "
	bra	CmdWri3		; "
	moviw	FSR0++		;Grab the byte from the queue and wrap the
	bcf	FSR0L,7		; pointer
	subwf	CHKSUM,F	;Update the checksum
CmdWri4	movlb	4		;Clock the byte out to the card
	movwf	SSP1BUF		; "
	xorwf	M_CRCH,W	;Update the CRC with the byte while it's
	movwf	M_CRCH		; clocking out
	movlp	high LutCrc16H	; "
	callw			; "
	xorwf	M_CRCL,W	; "
	xorwf	M_CRCH,F	; "
	xorwf	M_CRCH,W	; "
	xorwf	M_CRCH,F	; "
	movlp	high LutCrc16L	; "
	callw			; "
	movwf	M_CRCL		; "
	movlp	8		; "
	btfss	SSP1STAT,BF	;Wait until the byte is done transmitting
	bra	$-1		; "
	movlb	0		; "
	decfsz	X0,F		;Loop again until we've copied 512 bytes
	bra	CmdWri3		; "
	decfsz	X1,F		; "
	bra	CmdWri3		; "
	;TODO wreck the CRC in case the transmission ended prematurely
	btfss	IWMFLAG,IWMOVER	;Wait for the transmission to be over
	bra	$-1		; "
	moviw	FSR0++		;Grab the last byte from the queue and wrap the
	bcf	FSR0L,7		; pointer
	subwf	CHKSUM,F	;Update the checksum
	btfss	STATUS,Z	;If the checksum is not zero, wreck the CRC so
	comf	M_CRCL,F	; the card doesn't store the bad data
	movlb	4		;Clock the high byte of the CRC out to the card
	movf	M_CRCH,W	; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	;Wait until the byte is done transmitting
	bra	$-1		; "
	movf	M_CRCL,W	;Clock the low byte of the CRC out to the card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	;Wait until the byte is done transmitting
	bra	$-1		; "
	movlw	0xFF		;Clock data response byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;Save data response to check later
	movwf	X2		; "
	movlw	0xFF		;Start clocking the first is-still-busy? byte
	movwf	SSP1BUF		; "
	movlb	0		; "
	bcf	PIR1,SSP1IF	;Clear SSP int flag so it knows when not busy
	movlw	MSSWrBusy - MSS	;Set the card state to WrBusy so state machine
	movwf	M_STATE		; knows
	decf	CMDCNT,W	;If this was the last block, clear the ongoing
	btfsc	STATUS,Z	; write flag so state machine knows not to stop
	bcf	M_FLAGS,M_ONGWR	; waiting on a write or stop tran token
	movf	X2,W		;Check if the write succeeded
	andlw	B'00011111'	; "
	xorlw	B'00000101'	; "
	btfsc	STATUS,Z	;If it did, clear the error flag so we send a
	clrf	RESPERR		; success response
	btfss	STATUS,Z	;If it didn't, clear the ongoing write flag so
	bcf	M_FLAGS,M_ONGWR	; the write ends here
CmdWri5	btfss	IWMFLAG,IWMOVER	;Wait until the Mac's transmission is over
	bra	$-1		; "
	movf	CHKSUM,W	;If the checksum didn't match, clear the
	btfss	STATUS,Z	; ongoing write flag because the write stops
	bcf	M_FLAGS,M_ONGWR	; here
	clrf	TMR1H		;Spend about 65.536 ms, which is about how long
	clrf	TMR1L		; we have before Mac timeouts waiting for us to
	movlw	MSSWrEtTok - MSS; return to idle, waiting for the state machine
	btfss	M_FLAGS,M_ONGWR	; to try and get the card back to waiting for
	movlw	MSSWaitCmd - MSS; a command or a write token (we actually have
	call	MmcWaitState	; ~90 ms, but let's be cautious)
	call	ReturnToIdle	;That's long enough, return us to idle
	movf	CHKSUM,W	;If the checksum was wrong, send a NAK, else
	btfss	STATUS,Z	; send either a failure or success depending
	goto	BadChecksum	; on RESPERR
	goto	CmdPlaceholder	; "
CmdWri6	movlw	1		;In the unlikely event Mac tried to start a new
	movwf	CHKSUM		; write while one was ongoing, pretend there
	bra	CmdWri5		; was a bad checksum so it tries again

CmdWriteCont
	movlw	ERRCODE		;Ready an error code in case we need to jump
	movwf	RESPERR		; into the placeholder response routine
	btfss	M_FLAGS,M_ONGWR	;If a write isn't ongoing as it should be,
	bra	CmdWrC0		; handle this
	movf	M_STATE,W	;If we're not in the await-token state, we
	xorlw	MSSWrEtTok - MSS; won't be able to do this write, handle this
	btfss	STATUS,Z	; "
	bra	CmdWrC0		; "
	bra	CmdWri0		;Else jump into the middle of CmdWrite
CmdWrC0	bcf	M_FLAGS,M_ONGWR	;If something went wrong, end the ongoing write
	goto	WaitToFinish	; and send an error response


;;; Subprograms ;;;

;Use the address and block count from the received command along with the
; partition start block to to populate the card address registers, also
; checking whether the address and block count stay within the partition.
; Returns with carry bit set on failure, clear on success.  Trashes X0 and X1.
SetupCmdAddr
	movf	FSR0H,W		;Save FSR0 because we're going to use it for
	movwf	X1		; looking up the partition information
	movf	FSR0L,W		; "
	movwf	X0		; "
	movlw	0x21		;Point to the partition information block for
	movwf	FSR0H		; the selected device
	swapf	DEVNUM,W	; "
	lsrf	WREG,W		; "
	iorlw	B'10000000'	; "
	movwf	FSR0L		; "
	decf	CMDCNT,W	;Reckon the maximum block number accessed by
	addwf	CMDLOW,W	; this command
	movwf	M_ADR0		; "
	movlw	0		; "
	addwfc	CMDMED,W	; "
	movwf	M_ADR1		; "
	movlw	0		; "
	addwfc	CMDHIGH,W	; "
	movwf	M_ADR2		; "
	moviw	7[FSR0]		;If the partition size is less than or equal to
	subwf	M_ADR0,W	; the maximum block number accessed, return
	moviw	6[FSR0]		; with carry set
	subwfb	M_ADR1,W	; "
	moviw	5[FSR0]		; "
	subwfb	M_ADR2,W	; "
	btfsc	STATUS,C	; "
	bra	SetCmA0		; "
	movf	INDF0,W		;Move the starting block of the partition into
	movwf	M_ADR3		; the card address registers
	moviw	1[FSR0]		; "
	movwf	M_ADR2		; "
	moviw	2[FSR0]		; "
	movwf	M_ADR1		; "
	moviw	3[FSR0]		; "
	addwf	CMDLOW,W	;Add the block address from the command to it
	movwf	M_ADR0		; "
	movf	CMDMED,W	; "
	addwfc	M_ADR1,F	; "
	movf	CMDHIGH,W	; "
	addwfc	M_ADR2,F	; "
	movlw	0		; "
	addwfc	M_ADR3,F	; "
	call	MmcConvAddr	;Convert address to bytes if necessary
	bcf	STATUS,C	;Make sure carry is clear before returning
	btfsc	M_FLAGS,M_FAIL	;But set carry if the address conversion failed
	bsf	STATUS,C	; "
SetCmA0	movf	X1,W		;Restore FSR0
	movwf	FSR0H		; "
	movf	X0,W		; "
	movwf	FSR0L		; "
	return

;Write the byte in W to the Mac, taking care of LSBs and holdoff.  Note that
; IWMLSBS should be set to 0x80 and CHKSUM should be set to 0 before calling
; this function for the first time in a transmission.
WriteByte
	movf	IWMLSBS,F	;If the command has been aborted, return
	btfsc	STATUS,Z	; without sending anything, hopefully to finish
	return			; up quickly
	subwf	CHKSUM,F	;Adjust checksum with the byte to be written
	bsf	STATUS,C	;Shift out the byte's LSB, shifting a 1 into
	rrf	WREG,W		; the MSB and catching the LSB in the LSBs
	rrf	IWMLSBS,F	; shift register
	call	WriteIwmByte	;Write the IWM byte out on the USART
	btfss	IWMLSBS,0	;If the LSB shift register isn't full yet,
	return			; we're done here
	bsf	STATUS,C	;Shift a 1 into the MSB of the LSB shift
	rrf	IWMLSBS,W	; register and write that out on the USART
	call	WriteIwmByte	; "
	movlw	0x80		;Prepare the LSB shift register for next group
	movwf	IWMLSBS		; "
WriteB0	lsrf	PORTA,W		;Take a quick look at the current state
	xorlw	B'00000001'	;If we're still in state 1 (data transfer), all
	btfsc	STATUS,Z	; is normal, return to the caller
	return			; "
WriteB1	lsrf	PORTA,W		;Take another quick look at the current state
	btfsc	STATUS,Z	;If we're in the holdoff state, spin until 
	bra	WriteB1		; we aren't
	xorlw	B'00000001'	;If the state we're in is state 1 (data
	btfsc	STATUS,Z	; transfer), we've exited holdoff and must send
	bra	WriteB2		; a sync byte before resuming
	movlb	3		;If we're in some other state, the command has
	clrf	TXREG		; been aborted or something is awry, but we may
	movlb	0		; be in the middle of something that has to
	clrf	IWMLSBS		; finish, so set RD and clear the LSBs SR as an
	return			; indicator that future calls exit immediately
WriteB2	movlw	0xAA		;Write a sync byte over the USART to signal
	call	WriteIwmByte	; that we're resuming transmission
	return

;Write the raw IWM byte in W to the USART.
WriteIwmByte
	movf	IWMLSBS,F	;If the command has been aborted, return
	btfsc	STATUS,Z	; without sending anything, hopefully to finish
	return			; up quickly
	movlp	high LutFlip	;Use the LUT to flip the byte left to right so
	callw			; it goes out correctly on the USART
	btfss	PIR1,TXIF	;Wait until the USART is ready to accept a byte
	bra	$-1		; "
	movlb	3		;Write the byte to the USART
	movwf	TXREG		; "
	movlb	0		; "
	movlp	8		;Restore PCLATH
	return

;Wait until Mac returns to the idle state (PH2-0 = 2), allowing it to make its
; way through states 0 (holdoff), 1 (data), and 3 (data request), but bailing
; out to the idle loop if it enters any other state, if PH3 is high, or if
; !ENBL is high.
ReturnToIdle
	btfss	PIR1,TXIF	;In case we're coming here from post-transmit,
	bra	$-1		; make sure we don't stomp a waiting byte
	movlb	3		;Raise !HSHK to confirm to Mac that we're
	clrf	TXREG		; ready to return to idle state
	movlb	31		;Prepare for a possible return to the idle loop
	decf	STKPTR,F	; by (possibly temporarily) swallowing the
	movlb	0		; return address and pointing PCLATH to the
	movlp	0		; first half of program memory
RetIdl0	lsrf	PORTA,W		;Switch based on current state
	brw			; "
	bra	RetIdl0		;In holdoff state (why?), keep checking
	bra	RetIdl0		;Still in data state, keep checking
	bra	RetIdl1		;In idle state, we're done here
	bra	RetIdl0		;Transitioning through negotiation state
	goto	IdleLoop	;Anything else is cause to bail out
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	IdleLoop	; "
	goto	Unselected	;!ENBL has been deasserted, bail WAY out
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
	goto	Unselected	; "
RetIdl1	movlb	31		;We've successfully transitioned to the idle
	incf	STKPTR,F	; state, so restore the stack pointer and
	movlb	0		; PCLATH and return to the caller
	movlp	8		; "
	return			; "

RequestToSend
	movlb	3		;Assert !HSHK to tell Mac we want to send some
	btfss	TXSTA,TRMT	; data
	bra	$-1		; "
	movlw	0x80		; "
	movwf	TXREG		; "
	movlb	0		; "
ReqSnd0	lsrf	PORTA,W		;Switch based on current state
	brw			; "
	bra	ReqSnd0		;In holdoff state (why?), keep checking
	bra	ReqSnd1		;In data state, we're done here
	bra	ReqSnd0		;In idle state, keep checking
	bra	ReqSnd0		;Transitioning through negotiation state
	return			;Anything else is cause to bail out with Z
	return			; clear, indicating to caller that they should
	return			; get back to the idle loop, stat
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
	return			; "
ReqSnd1	bsf	STATUS,Z	;We've successfully requested to send, set the
	return			; Z bit and return


;;; MMC Subprograms ;;;

;Initialize MMC card.  Sets M_FAIL on fail.  Trashes X0 through X3.
MmcInit
	clrf	M_FLAGS		;Make sure flags are all clear to begin with
	clrf	M_STATE		;Make sure state starts at awaiting command
	call	MmcIni0		;Call into the function below
	bsf	CS_PORT,CS_PIN	;Always deassert !CS
	movf	WREG,W		;If the init function returned a code other
	btfss	STATUS,Z	; than 0, set the fail flag
	bsf	M_FLAGS,M_FAIL	; "
	return			;Pass return code to caller
MmcIni0	movlb	4		;This is where all the SSP registers are
	movlw	10		;Send 80 clocks on SPI interface to ensure MMC
	movwf	X0		; card is started up and in native command mode
MmcIni1	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	decfsz	X0,F		; "
	bra	MmcIni1		; "
	movlb	0		;Assert !CS
	bcf	CS_PORT,CS_PIN	; "
	movlw	0x40		;Send command 0 (expect R1-type response)
	movwf	M_CMDN		; which, with !CS asserted, signals to the card
	clrf	M_ADR3		; that we want to enter SPI mode
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, unrecognized or
	retlw	1		; missing MMC card, fail the init operation
	movf	M_CMDN,W	;If this command returned any response other
	xorlw	0x01		; than 0x01 ('in idle state'), unrecognized MMC
	btfss	STATUS,Z	; card, fail the init operation
	retlw	2		; "
	bsf	M_FLAGS,M_CDVER	;Assume version 2.0+ to begin with
	clrf	X2		;Set retry counter to 0 (65536) for later use
	clrf	X3		; "
	movlw	0x48		;Send command 8 (expect R7-type response) to
	movwf	M_CMDN		; check if we're dealing with a V2.0+ card
	clrf	M_ADR3		; "
	clrf	M_ADR2		; "
	movlw	0x01		; "
	movwf	M_ADR1		; "
	movlw	0xAA		; "
	movwf	M_ADR0		; "
	bsf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	movf	M_CMDN,W	;If the command set any error flags or there
	andlw	B'11111110'	; was no response, switch assumptions and guess
	btfsc	STATUS,Z	; that we're dealing with a Version 1 card and
	btfsc	M_FLAGS,M_FAIL	; jump ahead to initialize it
	bcf	M_FLAGS,M_CDVER	; "
	btfss	M_FLAGS,M_CDVER	; "
	bra	MmcIni2		; "
	movf	M_ADR1,W	;If the command didn't error, but the lower 12
	andlw	B'00001111'	; bits of the R7 response are something besides
	xorlw	0x01		; 0x1AA, we're dealing with an unknown card, so
	btfss	STATUS,Z	; raise the fail flag and return to caller
	retlw	3		; "
	movf	M_ADR0,W	; "
	xorlw	0xAA		; "
	btfss	STATUS,Z	; "
	retlw	3		; "
MmcIni2	movlw	0x77		;Send command 55 (expect R1-type response),
	movwf	M_CMDN		; which is a prelude to an 'app' command
	clrf	M_ADR3		; "
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	movf	M_CMDN,W	;If we got a status with any error bits set,
	andlw	B'11111110'	; treat as a command failure
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	btfsc	M_FLAGS,M_FAIL	;If this command fails, this is an unknown card
	retlw	4		; so return the failure to caller
	movlw	0x69		;Send app command 41 (expect R1-type response)
	movwf	M_CMDN		; to initialize the card, setting the HCS
	clrf	M_ADR3		; (high-capacity support) bit if we're dealing
	btfsc	M_FLAGS,M_CDVER	; with a V2.0+ card to let the card know that
	bsf	M_ADR3,6	; we support cards bigger than 4 GB (up to 2 
	clrf	M_ADR2		; TB)
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	movf	M_CMDN,W	;If we got a status with any error bits set,
	andlw	B'11111110'	; treat as a command failure
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	btfsc	M_FLAGS,M_FAIL	;If this command fails, this is an unknown card
	retlw	5		; so return the failure to caller
	btfss	M_CMDN,0	;If it returned an 0x00 status, initialization
	bra	MmcIni3		; is finished
	DELAY	40		;If it returned an 0x01 status, delay for 120
	decfsz	X2,F		; cycles (15 us), decrement the retry counter,
	bra	MmcIni2		; and try again
	decfsz	X3,F		; "
	bra	MmcIni2		; "
	retlw	6		;If card still not ready, report failure
MmcIni3	movlw	0x7A		;Send command 58 (expect R3-type response) to
	movwf	M_CMDN		; read the operating condition register (OCR)
	clrf	M_ADR3		; "
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bsf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	movf	M_CMDN,W	;If we got a status with any error bits set,
	btfss	STATUS,Z	; treat as a command failure
	bsf	M_FLAGS,M_FAIL	; "
	btfsc	M_FLAGS,M_FAIL	;If this command fails, something is wrong, so
	retlw	7		; return the failure to caller
	bsf	M_FLAGS,M_BKADR	;If the card capacity status (CCS) bit of the
	btfsc	M_ADR3,6	; OCR is set, we're using block addressing, so
	bra	MmcIni4		; skip ahead
	bcf	M_FLAGS,M_BKADR	;We're dealing with byte, not block addressing
	movlw	0x50		;Send command 16 (expect R1-type response) to
	movwf	M_CMDN		; tell the card we want to deal in 512-byte
	clrf	M_ADR3		; sectors
	clrf	M_ADR2		; "
	movlw	0x02		; "
	movwf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	movf	M_CMDN,W	;If this command returned any response other
	btfss	STATUS,Z	; than 0x00, something is wrong, fail the init
	bsf	M_FLAGS,M_FAIL	; operation
	btfsc	M_FLAGS,M_FAIL	;If this command failed, something is wrong,
	retlw	8		; fail the init operation
MmcIni4	movlw	0x7B		;Send command 59 (expect R1-type response) to
	movwf	M_CMDN		; tell the card we want to make life hard on
	clrf	M_ADR3		; ourselves and have our CRCs checked by the
	clrf	M_ADR2		; card
	clrf	M_ADR1		; "
	movlw	0x01		; "
	movwf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	movf	M_CMDN,W	;If this command returned any response other
	btfss	STATUS,Z	; than 0x00, something is wrong, fail the init
	bsf	M_FLAGS,M_FAIL	; operation
	btfsc	M_FLAGS,M_FAIL	;If this command failed, something is wrong,
	retlw	9		; fail the init operation
	retlw	0		;Congratulations, card is initialized!

;Convert a block address to a byte address if byte addressing is in effect.
; Sets M_FAIL if the block address is above 0x7FFFFF (and thus can't fit as a
; byte address).
MmcConvAddr
	bcf	M_FLAGS,M_FAIL	;Assume no failure to start with
	btfsc	M_FLAGS,M_BKADR	;If block addressing is in effect, the address
	return			; does not need to be converted
	movf	M_ADR3,F	;Make sure that the top 9 bits of the block
	btfss	STATUS,Z	; address are clear; if they are not, set the
	bsf	M_FLAGS,M_FAIL	; fail flag
	btfsc	M_ADR2,7	; "
	bsf	M_FLAGS,M_FAIL	; "
	btfsc	M_FLAGS,M_FAIL	;If the fail flag is set, we're done
	return			; "
	lslf	M_ADR0,F	;Multiply the block address by 2 and then by
	rlf	M_ADR1,F	; 256
	rlf	M_ADR2,W	; "
	movwf	M_ADR3		; "
	movf	M_ADR1,W	; "
	movwf	M_ADR2		; "
	movf	M_ADR0,W	; "
	movwf	M_ADR1		; "
	clrf	M_ADR0		; "
	return

;Increment the address by one block according to the block/byte addressing
; mode.  No protection is provided against the address wrapping around.
MmcIncAddr
	movlw	0x01		;Add 1 to the address if we're in block mode,
	btfss	M_FLAGS,M_BKADR	; add 512 to the address if we're in byte mode,
	movlw	0		; in either case carrying the remainder through
	addwf	M_ADR0,F	; "
	movlw	0		; "
	btfss	M_FLAGS,M_BKADR	; "
	movlw	0x02		; "
	addwfc	M_ADR1,F	; "
	movlw	0		; "
	addwfc	M_ADR2,F	; "
	addwfc	M_ADR3,F	; "
	return

;Send the command contained in M_CMDN and M_ADR3-0 to MMC card.  Sets M_FAIL on
; fail.  Trashes X0 and X1.
MmcCmd
	bcf	M_FLAGS,M_FAIL	;Assume no failure to start with
	clrf	X0		;Start the CRC7 register out at 0
	movlp	high LutCrc7	;Point PCLATH to the CRC7 lookup table
	movlb	4		;Switch to the bank with the SSP registers
	movf	M_CMDN,W	;Clock out all six MMC buffer bytes as command,
	movwf	SSP1BUF		; calculating the CRC7 along the way
	xorwf	X0,W		; "
	callw			; "
	movwf	X0		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR3,W	; "
	movwf	SSP1BUF		; "
	xorwf	X0,W		; "
	callw			; "
	movwf	X0		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR2,W	; "
	movwf	SSP1BUF		; "
	xorwf	X0,W		; "
	callw			; "
	movwf	X0		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR1,W	; "
	movwf	SSP1BUF		; "
	xorwf	X0,W		; "
	callw			; "
	movwf	X0		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR0,W	; "
	movwf	SSP1BUF		; "
	xorwf	X0,W		; "
	callw			; "
	movlp	8		; "
	movwf	X0		; "
	bsf	X0,0		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	X0,W		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	;TODO for CMD12, it is necessary to clock and throw away a stuff byte?
	movlw	8		;Try to get status as many as eight times
	movwf	X0		; "
MmcCmd1	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	incf	SSP1BUF,W	;If we read back anything but 0xFF, skip ahead
	btfss	STATUS,Z	; "
	bra	MmcCmd2		; "
	decfsz	X0,F		;Decrement the attempt counter until we've
	bra	MmcCmd1		; tried eight times; if we haven't gotten a
	bsf	M_FLAGS,M_FAIL	; reply by the eighth attempt, signal failure
	movlb	0		; and return
	return			; "
MmcCmd2	decf	WREG,W		;Store the byte we received as R1-type status
	movwf	M_CMDN		; in the buffer
	btfss	M_FLAGS,M_CMDLR	;If we aren't expecting a long (R3/R7-type)
	bra	MmcCmd3		; reply, we're done
	movlw	0xFF		;Clock first extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_ADR3		; "
	movlw	0xFF		;Clock second extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_ADR2		; "
	movlw	0xFF		;Clock third extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_ADR1		; "
	movlw	0xFF		;Clock fourth extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_ADR0		; "
MmcCmd3	btfsc	M_FLAGS,M_CMDRB	;If we're expecting an R1b reply, wait for the
	call	MmcWaitBusy	; card not to be busy anymore before returning
	movlb	0		;Restore BSR to 0
	return

;Waits for the card not to be busy anymore.  Sets M_FAIL on fail.  Trashes X0
; and X1.  Expects BSR to be 4 and does not set or reset this.
MmcWaitBusy
	clrf	X0		;Check 65536 times to see if the card is busy
	clrf	X1		; "
MmcWai0	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;Check if MISO is still low, if it's not, the
	btfss	STATUS,Z	; card is no longer busy and we can return
	return			; "
	decfsz	X0,F		;If it's not done, try again
	bra	MmcWai0		; "
	decfsz	X1,F		; "
	bra	MmcWai0		; "
	bsf	M_FLAGS,M_FAIL	;If out of tries, fail the operation
	return

;Try to get the data token for a read from the card.  Sets M_FAIL on fail.
; Trashes X0 and X1.
MmcGetData
	bcf	M_FLAGS,M_FAIL	;Assume no failure to start with
	movlb	4		;Switch to the bank with the SSP registers
	clrf	X0		;Try 65536 times to get the data token
	clrf	X1		; "
MmcGet0	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;If we've received the data token, return
	xorlw	0xFE		; "
	btfsc	STATUS,Z	; "
	bra	MmcGet1		; "
	decfsz	X0,F		;If not, decrement the retry count and try
	bra	MmcGet0		; again
	decfsz	X1,F		; "
	bra	MmcGet0		; "
	bsf	M_FLAGS,M_FAIL	;If we didn't get the data token after 65536
MmcGet1	movlb	0		; tries, give up and fail the operation
	return			; "

;Read the number of bytes in W from the card into memory where FSR1 is pointed,
; updating CRC registers.  Trashes X0 and X1.
MmcReadData
	movwf	X0		;Store the count of bytes to read
	movlb	4		;Clock the first byte out of the card
	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	;Wait until the transfer is done
	bra	$-1		; "
	movf	SSP1BUF,W	;Store the received byte in the buffer and also
	movwf	X1		; keep it to update the CRC with
	movwi	FSR1++		; "
	decf	X0,F		;If for some reason the caller only wanted to
	btfsc	STATUS,Z	; read one byte, skip the next loop
	bra	MmcRea1		; "
MmcRea0	movlw	0xFF		;Clock the next byte out of the card
	movwf	SSP1BUF		; "
	movf	X1,W		;Update the CRC with the previous byte while
	xorwf	M_CRCH,W	; the next one is clocking out of the card
	movwf	M_CRCH		; "
	movlp	high LutCrc16H	; "
	callw			; "
	xorwf	M_CRCL,W	; "
	xorwf	M_CRCH,F	; "
	xorwf	M_CRCH,W	; "
	xorwf	M_CRCH,F	; "
	movlp	high LutCrc16L	; "
	callw			; "
	movwf	M_CRCL		; "
	btfss	SSP1STAT,BF	;Wait until the transfer is done
	bra	$-1		; "
	movf	SSP1BUF,W	;Store the received byte in the buffer and also
	movwf	X1		; keep it to update the CRC with
	movwi	FSR1++		; "
	decfsz	X0,F		;Decrement the byte counter and loop around if
	bra	MmcRea0		; there are bytes left to clock out
MmcRea1	movf	X1,W		;Update the CRC with the last byte
	xorwf	M_CRCH,W	; "
	movwf	M_CRCH		; "
	movlp	high LutCrc16H	; "
	callw			; "
	xorwf	M_CRCL,W	; "
	xorwf	M_CRCH,F	; "
	xorwf	M_CRCH,W	; "
	xorwf	M_CRCH,F	; "
	movlp	high LutCrc16L	; "
	callw			; "
	movwf	M_CRCL		; "
	movlb	0		;Restore BSR
	movlp	8		;Restore PCLATH
	return

;Read the number of bytes in W from the card into WriteByte, updating CRC
; registers.  Trashes X0 and X1.
MmcCopyData
	movwf	X0		;Store the count of bytes to read
	movlb	4		;Clock the first byte out of the card
	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	;Wait until the transfer is done
	bra	$-1		; "
	movf	SSP1BUF,W	;Store the received byte to update the CRC with
	movwf	X1		; and also call WriteByte
	movlb	0		; "
	call	WriteByte	; "
	movlb	4		; "
	decf	X0,F		;If for some reason the caller only wanted to
	btfsc	STATUS,Z	; read one byte, skip the next loop
	bra	MmcCop1		; "
MmcCop0	movlw	0xFF		;Clock the next byte out of the card
	movwf	SSP1BUF		; "
	movf	X1,W		;Update the CRC with the previous byte while
	xorwf	M_CRCH,W	; the next one is clocking out of the card
	movwf	M_CRCH		; "
	movlp	high LutCrc16H	; "
	callw			; "
	xorwf	M_CRCL,W	; "
	xorwf	M_CRCH,F	; "
	xorwf	M_CRCH,W	; "
	xorwf	M_CRCH,F	; "
	movlp	high LutCrc16L	; "
	callw			; "
	movwf	M_CRCL		; "
	btfss	SSP1STAT,BF	;Wait until the transfer is done
	bra	$-1		; "
	movf	SSP1BUF,W	;Store the received byte to update the CRC with
	movwf	X1		; and also call WriteByte
	movlb	0		; "
	movlp	8		; "
	call	WriteByte	; "
	movlb	4		; "
	decfsz	X0,F		;Decrement the byte counter and loop around if
	bra	MmcCop0		; there are bytes left to clock out
MmcCop1	movf	X1,W		;Update the CRC with the last byte
	xorwf	M_CRCH,W	; "
	movwf	M_CRCH		; "
	movlp	high LutCrc16H	; "
	callw			; "
	xorwf	M_CRCL,W	; "
	xorwf	M_CRCH,F	; "
	xorwf	M_CRCH,W	; "
	xorwf	M_CRCH,F	; "
	movlp	high LutCrc16L	; "
	callw			; "
	movwf	M_CRCL		; "
	movlb	0		;Restore BSR
	movlp	8		;Restore PCLATH
	return

;Read the CRC out of the card and check it against the calculated CRC.  Sets Z
; if they match, clears it if not.
MmcCheckCrc
	movlb	4		;Clock the high CRC byte out of the card
	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	xorwf	M_CRCH,W	;If it doesn't match what we calculated, we've
	btfss	STATUS,Z	; failed but still need to clock out the low
	bra	MmcChe0		; byte so skip ahead
	movlw	0xFF		;Clock the low CRC byte out of the card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	xorwf	M_CRCL,W	;The high byte matched, so whether the whole
	movlb	0		; thing matches or not hinges on the low byte,
	return			; so compare them and return with result in Z
MmcChe0	movlw	0xFF		;Clock the low CRC byte out of the card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	bcf	STATUS,Z	;Clear Z because high byte didn't match and
	movlb	0		; return
	return			; "

;Try to advance to the state requested in W, timing out if Timer1 overflows.
; Trashes X0.
MmcWaitState
	movwf	X0		;Save the state caller wants to get to
	movlw	B'00110001'	;Turn on Timer1
	movwf	T1CON		; "
	bcf	PIR1,TMR1IF	;Clear the Timer1 interrupt flag if it was set
	movf	M_STATE,W	;If we're already in the state caller wants to
	xorwf	X0,W		; be in, skip ahead
	btfsc	STATUS,Z	; "
	bra	MmcWaS1		; "
MmcWaS0	call	MmcStepState	;Step the state
	movf	M_STATE,W	;If we're now in the state caller wants to be
	xorwf	X0,W		; in, skip ahead
	btfsc	STATUS,Z	; "
	bra	MmcWaS1		; "
	btfsc	PIR1,TMR1IF	;If Timer1 overflowed, skip ahead
	bra	MmcWaS2		; "
	bra	MmcWaS0		;Loop until something happens
MmcWaS1	bcf	T1CON,TMR1ON	;Stop Timer1 and clear its interrupt flag so
	bcf	PIR1,TMR1IF	; caller knows we're in the state requested and
	return			; did not time out
MmcWaS2	bcf	T1CON,TMR1ON	;Stop Timer1 and leave its interrupt flag set
	return			; so caller knows a timeout occurred

;State machine to ensure that the card stays in a good state.
MmcStepState
	btfss	PIR1,SSP1IF	;If the SSP is busy, we can't do anything, so
	return			; return
	movlb	4		;Switch depending on the current state
	movf	M_STATE,W	; "
	call	MSSCall		; "
	movwf	M_STATE		;Save the returned value in W as the new state
	movlb	0		; and return
	movlp	8		; "
	return			; "
MSSCall	brw
MSS
MSSWaitCmd
	btfss	M_FLAGS,M_RDCMD	;If we're not set to do a read command, cycle
	retlw	MSSWaitCmd - MSS; back to this state doing nothing
	movlb	0		;Assert !CS
	bcf	CS_PORT,CS_PIN	; "
	movlb	4		; "
	movlw	0x51		;Clock out the command byte for a read
	movwf	SSP1BUF		; "
	movlw	0xE8		;Set up the CRC register with the CRC7 of the
	movwf	M_CRCL		; read command byte
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSRdCmd1 - MSS	;Transition to the next read command state
MSSRdCmd1
	movf	M_ADR3,W	;Clock out the high byte of the address
	movwf	SSP1BUF		; "
	movlp	high LutCrc7	;Update the CRC for the command
	xorwf	M_CRCL,W	; "
	callw			; "
	movwf	M_CRCL		; "
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSRdCmd2 - MSS	;Transition to the next read command state
MSSRdCmd2
	movf	M_ADR2,W	;Clock out the next byte of the address
	movwf	SSP1BUF		; "
	movlp	high LutCrc7	;Update the CRC for the command
	xorwf	M_CRCL,W	; "
	callw			; "
	movwf	M_CRCL		; "
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSRdCmd3 - MSS	;Transition to the next read command state
MSSRdCmd3
	movf	M_ADR1,W	;Clock out the next byte of the address
	movwf	SSP1BUF		; "
	movlp	high LutCrc7	;Update the CRC for the command
	xorwf	M_CRCL,W	; "
	callw			; "
	movwf	M_CRCL		; "
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSRdCmd4 - MSS	;Transition to the next read command state
MSSRdCmd4
	movf	M_ADR0,W	;Clock out the low byte of the address
	movwf	SSP1BUF		; "
	movlp	high LutCrc7	;Update the CRC for the command
	xorwf	M_CRCL,W	; "
	callw			; "
	movwf	M_CRCL		; "
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSRdCmd5 - MSS	;Transition to the next read command state
MSSRdCmd5
	movf	M_CRCL,W	;Clock out the CRC for the command
	movwf	SSP1BUF		; "
	bcf	M_FLAGS,M_RDCMD	;Read command complete, M_ADR* are free
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSRdCmd6 - MSS	;Transition to the next read command state
MSSRdCmd6
	movlw	0xFF		;Clock first is-still-busy? byte out of the
	movwf	SSP1BUF		; card
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSRdCmd7 - MSS	; "
MSSRdCmd7
	btfss	SSP1BUF,7	;If MSB is set, this isn't an R1 response byte,
	bra	$+6		; so clock out another byte and try again
	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	movlb	0		; "
	bcf	PIR1,SSP1IF	; "
	retlw	MSSRdCmd7 - MSS	; "
	movf	SSP1BUF,W	;If MSB is clear but an error bit is set, the
	andlw	B'11111110'	; command was not processed, so go back to the
	btfsc	STATUS,Z	; WaitCmd state after deasserting !CS
	bra	$+4		; "
	movlb	0		; "
	bsf	CS_PORT,CS_PIN	; "
	retlw	MSSWaitCmd - MSS; "
	movlw	0xFF		;If all error bits are clear in the response
	movwf	SSP1BUF		; byte, clock out the first is-read-token? byte
	movlb	0		; "
	bcf	PIR1,SSP1IF	; "
	retlw	MSSRdTok - MSS	; "
MSSRdTok
	movf	SSP1BUF,W	;Check if the byte read from the card is the
	xorlw	0xFE		; read token
	btfsc	STATUS,Z	;If the byte read was not the read token, start
	bra	$+6		; the next byte clocking out and remain in this
	movlw	0xFF		; state for next call
	movwf	SSP1BUF		; "
	movlb	0		; "
	bcf	PIR1,SSP1IF	; "
	retlw	MSSRdTok - MSS	; "
	movlw	0xFD		;If byte read was the read token, transition
	movwf	M_CRCH		; to RdData using CRC register as an up counter
	movwf	M_CRCL		; set to -515 (512 bytes + 2 CRC bytes + 1),
	retlw	MSSRdData - MSS	; assuming we have to throw away the read data
MSSRdData
	incf	M_CRCL,F	;Step the up counter
	btfsc	STATUS,Z	; "
	incf	M_CRCH,F	; "
	btfss	STATUS,Z	;If the up counter overflowed, deassert !CS
	bra	$+4		; and reset the state to WaitCmd without
	movlb	0		; starting a byte clocking
	bsf	CS_PORT,CS_PIN	; "
	retlw	MSSWaitCmd - MSS; "
	movlw	0xFF		;Start the next byte to read clocking out and
	movwf	SSP1BUF		; stay in this state for next time
	movlb	0		; "
	bcf	PIR1,SSP1IF	; "
	retlw	MSSRdData - MSS	; "
MSSWrEtTok
	btfsc	M_FLAGS,M_ONGWR	;If there's an ongoing write, don't proceed in
	retlw	MSSWrEtTok - MSS; sending an end tran token
	movlw	0xFD		;Start an end tran token clocking
	movwf	SSP1BUF		; "
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSEtJnk1 - MSS	;Transition to EtJnk1
MSSEtJnk1
	movlw	0xFF		;Byte we got in when we clocked out the end
	movwf	SSP1BUF		; tran token is junk, so is the next one
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSEtJnk2 - MSS	;Transition to EtJnk2
MSSEtJnk2
	movlw	0xFF		;Byte we got in from previous state is junk,
	movwf	SSP1BUF		; the one we're clocking now is legit
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSEtBusy - MSS	;Transition to EtBusy
MSSEtBusy
	movf	SSP1BUF,W	;If the byte we read out of the card is not all
	btfsc	STATUS,Z	; zeroes, deassert !CS and transition to
	bra	$+4		; WaitCmd
	movlb	0		; "
	bsf	CS_PORT,CS_PIN	; "
	retlw	MSSWaitCmd - MSS; "
	movlw	0xFF		;If the byte we read out of the card is all
	movwf	SSP1BUF		; zeroes, clock the next one out while keeping
	movlb	0		; MOSI high and stay in this state
	bcf	PIR1,SSP1IF	; "
	retlw	MSSEtBusy - MSS	; "
MSSWrBusy
	movf	SSP1BUF,W	;If the byte clocked in is 0x00, clock another
	btfss	STATUS,Z	; byte out and return to this state
	bra	$+6		; "
	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	movlb	0		; "
	bcf	PIR1,SSP1IF	; "
	retlw	MSSWrBusy - MSS	; "
	btfsc	M_FLAGS,M_ONGWR	;If there's an ongoing write, transition to let
	retlw	MSSWrEtTok - MSS; the caller send a data token if it wants to
	movlw	0xFD		;Else, start an end tran token clocking
	movwf	SSP1BUF		; "
	movlb	0		;Clear the SSP interrupt flag because it's in
	bcf	PIR1,SSP1IF	; use now
	retlw	MSSEtJnk1 - MSS	;Transition to EtJnk1


;;; End of Program ;;;

	end
