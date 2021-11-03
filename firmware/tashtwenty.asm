;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashTwenty: Single-Chip DCD Interface
;;;
;


;;; License ;;;

;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <https://www.gnu.org/licenses/>.


;;; Connections ;;;

;;;                                                          ;;;
;                         .--------.                           ;
;                 Supply -|01 \/ 14|- Ground                   ;
;      !ENBL --->    RA5 -|02    13|- RA0    ---> Always 0     ;
;        PH3 --->    RA4 -|03    12|- RA1    <--- PH0          ;
;        PH2 --->    RA3 -|04    11|- RA2    <--- PH1          ;
;         WR --->    RC5 -|05    10|- RC0    ---> MMC SCK      ;
;         RD <---    RC4 -|06    09|- RC1    <--- MMC MISO     ;
;    MMC !CS <---    RC3 -|07    08|- RC2    ---> MMC MOSI     ;
;                         '--------'                           ;
;;;                                                          ;;;


;;; Assembler Directives ;;;

	list		P=PIC16F1825, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P16F1825.inc
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_CPD_OFF	Data memory protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _LVP_OFF
			;_WRT_OFF	Write protection off
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

TXNOTRC	equ	7	;Set if a transmit is ongoing, clear if a receive is

M_FAIL	equ	7	;Set when there's been a failure on the MMC interface
M_CMDLR	equ	6	;Set when R3 or R7 is expected (5 bytes), not R1


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS	;You've got to have flags
	GROUPS	;Count of groups to transmit, count of groups received
	M_FLAGS	;MMC flags
	M_BUF1	;The command to be sent, later the R1 response byte	
	M_BUF2	;First (high) byte of the address, first byte of R3/R7 response
	M_BUF3	;Second byte of the address, second byte of R3/R7 response
	M_BUF4	;Third byte of the address, third byte of R3/R7 response
	M_BUF5	;Fourth (low) byte of the address, last byte of R3/R7 response
	M_BUF6	;CRC byte of the command to be sent
	TEMP	;Various purposes
	TEMP2	;Various purposes
	TEMP3	;Various purposes
	D3
	D2
	D1
	D0
	
	endc

	cblock	0x20	;Bank 0 registers (receive shorthand)
	
	RC_SYNC	;Sync byte
	RC_CMDG	;Command groups + 0x80
	RC_RSPG	;Response groups + 0x80
	RC_CMDN	;Command number
	RC_BLKS	;Block count
	RC_ADRH	;Block address high
	RC_ADRM	;Block address middle
	RC_ADRL	;Block address low
	
	endc

	cblock 0x20	;Bank 0 registers (transmit shorthand)
	
	TX_CMDN	;Command number
	TX_BLKS	;Block count
	TX_STAT	;Status
	TX_PAD1	;Pad byte
	TX_PAD2	;Pad byte
	TX_PAD3	;Pad byte
	TX_CSUM	;Checksum, if reply is only one group long
	
	endc


;;; Vectors ;;;

	org	0x0		;Reset vector
	bra	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

;BSR must always be at 0 when interrupts are on

Interrupt
	lslf	PORTA,W		;32 entries in the jump table, each has four
	brw			; instructions; jump to the one matching state
	
IntEn0	bra	IntSuspend	;Mac wants to suspend communication and is
	nop			; waiting for us to drive RD high to deassert
	nop			; !HSHK; handle it
	nop			; "
IntEn1	movlb	7		;This is the communication state, so we should
	clrf	IOCAF		; never get here unless it's a blip on one of
	retfie			; the state lines - if it is, clear and return
	nop			; and just hope timing isn't too badly wrecked
IntEn2	bsf	PORTC,RC4	;This state (idle) is two states away from the
	bra	IntAbort	; communication state, which is strange, but
	nop			; assume that it means mac wants to abort the
	nop			; operation in progress
IntEn3	btfss	FLAGS,TXNOTRC	;In a receive, this means mac is done and is
	bra	IntDoneReceive	; waiting !HSHK to be deasserted; normally
	bra	IntAbort	; transmit will handle this state in mainline,
	nop			; so if we're here it's bad, treat as an abort
IntEn4	bsf	PORTC,RC4	;This state (reset) is two states away from the
	bra	IntAbort	; communication state, which is strange, but
	nop			; assume that it means mac wants to abort the
	nop			; operation in progress
IntEn5	bcf	PORTC,RC4	;Now this is really weird; why would mac go to
	bra	Interrupt	; this state (drive RD low) in the middle of
	nop			; communication? Whatever, drive RD low and
	nop			; immediately circle back until state changes
IntEn6	bsf	PORTC,RC4	;Now this is really weird; why would mac go to
	bra	Interrupt	; this state (drive RD high) in the middle of
	nop			; communication? Whatever, drive RD high and
	nop			; immediately circle back until state changes
IntEn7	bsf	PORTC,RC4	;Now this is really weird; why would mac go to
	bra	Interrupt	; this state (drive RD high) in the middle of
	nop			; communication? Whatever, drive RD high and
	nop			; immediately circle back until state changes
IntEnNx	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to talk to the next device in mid-
	nop			; communication? Whatever, drive RD high and
	nop			; force an unexplained return to the idle state
IntNEn	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state
	bsf	PORTC,RC4	;Now this is really REALLY weird; why would mac
	bra	IntAbort	; decide to deassert !ENBL mid-communication?
	nop			; Whatever, drive RD high and force an
	nop			; unexplained return to the idle state

IntSuspend
	movlp	high DecodeRecv	;If we were interrupted doing a receive, decode
	btfss	FLAGS,TXNOTRC	; the data we've received so far
	call	DecodeRecv	; "
	bsf	PORTC,RC4	;Signal that we acknowledge the suspension
IntSTr1	movf	PORTA,W		;Check the current state
	btfsc	STATUS,Z	;If it's 0 (holdoff), mac is still asking us to
	bra	IntSTr1		; hold off, so loop again
	xorlw	B'00000100'	;If it's 2 (idle), mac wants to abort the
	btfsc	STATUS,Z	; command in progress
	bra	IntAbort	; "
	xorlw	B'00000110'	;If it's 1 (communication), mac is probably
	btfsc	STATUS,Z	; moving through it to state 3 (request send)
	bra	IntSTr1		; so loop until it gets there
	xorlw	B'00000100'	;If it's anything else but 3 (request send),
	btfss	STATUS,Z	; something weird's going on so abort
	bra	IntAbort	; "
	bcf	PORTC,RC4	;Signal that we're ready to resume
IntSTr2	movf	PORTA,W		;Check the current state
	xorlw	B'00000110'	;If it's still 3 (request send), mac hasn't
	btfsc	STATUS,Z	; acknowledged that we're ready yet, so loop
	bra	IntSTr2		; again
	xorlw	B'00000100'	;If it's anything else but 1 (communication),
	btfss	STATUS,Z	; something weird's going on, so abort
	bra	IntAbort	; "
	movlb	7		;Clear the interrupt that got us here and any
	clrf	IOCAF		; we received after that
	movlb	31		;Don't return to where we left off in the
	movlw	high Transmitter; transmitter or receiver loop, return to the
	btfss	FLAGS,TXNOTRC	; beginning of it; note that both transmitter
	movlw	high Receiver	; and receiver will take care of setting BSR
	movwf	TOSH		; and enabling interrupts - we use return here
	movlw	low Transmitter	; instead of retfie so that the FSRs don't get
	btfss	FLAGS,TXNOTRC	; overwritten
	movlw	low Receiver	; "
	movwf	TOSL		; "
	return

IntDoneReceive
	bsf	PORTC,RC4	;Deassert !HSHK to acknowledge receive is done
IntDRc0	movf	PORTA,W		;Check the current state
	xorlw	B'00000110'	;If it's still 3, mac hasn't acknowledged our
	btfsc	STATUS,Z	; acknowledgment of the end of received data
	bra	IntDRc0		; yet, so loop again
	xorlw	B'00000010'	;If it's anything else but 2 (idle), something
	btfss	STATUS,Z	; weird's going on, so abort
	bra	IntAbort	; "
	movlp	high DecodeRecv	;Decode the data we've received since the start
	call	DecodeRecv	; or since the last interruption
	movlb	7		;Clear the interrupt that got us here and any
	clrf	IOCAF		; we received after that
	movlb	31		;Swallow the interrupt return address and
	decf	STKPTR,F	; instead return to who called receiver
	movlb	0		; without reenabling interrupts
	return			; "

IntAbort
	movlb	7		;Clear the interrupt that got us here and any
	clrf	IOCAF		; we received after that
	movlb	31		;Swallow the return address where the PC was
	decf	STKPTR,F	; when the interrupt happened
	movlw	high IdleJump	;Overwrite the return address from which
	movwf	TOSH		; receiver or transmitter was called with the
	movlw	low IdleJump	; address of the idle jump table so that a
	movwf	TOSL		; return from interrupt dumps us back there
	movlb	0		; to handle the current state
	return			;Return to idle without reenabling interrupts


;;; Init ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	IOCAP		;When interrupts are on, any change on !ENBL or
	movlw	B'00111110'	; the phase pins triggers one
	movwf	IOCAP
	movwf	IOCAN

	banksel	SSPCON1		;SSP SPI master mode, clock set by baud rate
	movlw	B'00101010'	; generator to 400 kHz, clock idles low, data
	movwf	SSPCON1		; lines change on falling edge, data sampled on
	movlw	B'01000000'	; rising edge (CKP=0, CKE=1, SMP=0)
	movwf	SSP1STAT
	movlw	19
	movwf	SSP1ADD

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA
	clrf	ANSELC

	banksel	LATA		;RA0 always off, RD and !CS high to start
	movlw	B'00111110'
	movwf	LATA
	movlw	B'00111111'
	movwf	LATC

	banksel	TRISA		;State pins inputs, RA0 output, WR input, RD
	movlw	B'00111110'	; tristated to start, MISO input, !CS, MOSI,
	movwf	TRISA		; and SCK output
	movlw	B'00110010'
	movwf	TRISC

	movlw	0x20		;Start pointers at top of linear memory
	movwf	FSR0H
	movwf	FSR1H
	clrf	FSR0L
	clrf	FSR1L

	movlw	B'00001000'	;Interrupt on PORTA pin change enabled, but
	movwf	INTCON		; interrupt subsystem off for now

	movlb	0		;Start BSR at 0

	movlw	11		;Delay approximately 1 ms to allow the MMC card
	movwf	TEMP		; to come up - do not call MmcInit before this
DelayMs	DELAY	242		; has been done
	decfsz	TEMP,F		; "
	bra	DelayMs		; "
	call	MmcInit		;Initialize!
	movlb	4		;Now that we're initialized, crank the speed
	movlw	B'00100001'	; of the SPI interface up to 2 MHz
	movwf	SSPCON1		; "
	movlb	0		; "


;;; Mainline Code ;;;

WaitNoEnbl
	btfss	PORTA,RA5	;Spin until !ENBL goes high
	bra	$-1		; "

WaitEnbl
	movlw	B'00110010'	;Tristate RD
	tris	7		; "
	bsf	PORTC,RC4	;Latch a 1 on RD for when we drive it
	btfsc	PORTA,RA5	;Spin until !ENBL goes low
	bra	$-1		; "
	movlw	B'00100010'	;Drive the RD pin, high at first
	tris	7		; "

IdleJump
	movf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
IdleEn0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	IdleJump	; off from, so drive RD high to deassert !HSHK
IdleEn1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	IdleJump	; drive RD high because what else can we do
IdleEn2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	IdleJump	; idling, so drive RD high to deassert !HSHK
IdleEn3	bsf	PORTC,RC4	;Mac wants to send a command, drive RD high to
	bra	GetCommand	; deassert !HSHK and prepare to receive
IdleEn4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	IdleJump	; drive RD high to deassert !HSHK 
IdleEn5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	IdleJump	; "
IdleEn6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	IdleJump	; high
IdleEn7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	IdleJump	; high
IdleENx	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
	bsf	PORTC,RC4	;Mac wants to talk to next device, none exists,
	bra	WaitNoEnbl	; so drive RD high and await !ENBL deasserted
IdleNEn	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	bra	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again

GetCommand
	movlp	high Receive	;Initiate receive
	call	Receive		; "
	movlp	0		; "
	call	CheckChecksum	;Calculate the checksum of the received block
	btfss	STATUS,Z	;If the checksum was bad, send a NAK
	bra	NakCommand	; "
	movf	RC_CMDN,W	;If the command number is 0x00, do a read
	btfsc	STATUS,Z	; "
	bra	CmdRead		; "
	addlw	-1		;If the command number is 0x01, do a write
	btfsc	STATUS,Z	; "
	bra	CmdWrite	; "
	addlw	-2		;If the command number is 0x03, get controller
	btfsc	STATUS,Z	; status (identify device)
	bra	CmdStatus	; "
	addlw	-62		;If the command number is 0x41, do a continued
	btfsc	STATUS,Z	; write
	bra	CmdWriteCont	; "
	bra	CmdUnknown	;If none of the above, fake a response

CmdUnknown
	movf	RC_CMDN,W	;Save the unknown command's number since it's
	movwf	TEMP2		; about to be overwritten
	call	ClearResponse	;Clear the buffer according to expectations
	movf	TEMP2,W		;Our reply is going to be the command number
	movwf	TX_CMDN		; with its MSB set, by convention, but data
	bsf	TX_CMDN,7	; will be all zeroes
	call	CalcChecksum	;Calculate the checksum of this placeholder
	movlp	high Transmit	;Initiate transmission
	call	Transmit	; "
	movlp	0		; "
	goto	IdleJump	;Done, hope the host doesn't mind

NakCommand
	movlw	1		;A NAK is a blank one-group command buffer with
	movwf	GROUPS		; a command number of 0x7F
	movlw	0x7F		; "
	movwf	TX_CMDN		; "
	clrf	TX_BLKS		; "
	clrf	TX_STAT		; "
	clrf	TX_PAD1		; "
	clrf	TX_PAD2		; "
	clrf	TX_PAD3		; "
	movlw	0x81		;We know what the checksum will be, so just
	movwf	TX_CSUM		; hardcode it
	movlp	high Transmit	;Initiate transmission
	call	Transmit	; "
	movlp	0		; "
	goto	IdleJump	;Done

CmdRead
	clrf	M_BUF5		;Shift the block address left by 9 (multiply it
	lslf	RC_ADRL,W	; by 512) to transform it into the byte address
	movwf	M_BUF4		; used by the MMC card
	rlf	RC_ADRM,W	; "
	movwf	M_BUF3		; "
	rlf	RC_ADRH,W	; "
	movwf	M_BUF2		; "
	movf	RC_BLKS,W	;Move the block count from receiver position to
	movwf	TX_BLKS		; transmitter position
	movlw	0x80		;Command number is the read command with the
	movwf	TX_CMDN		; MSB set
	clrf	TX_STAT		;A zero status means all is well
	clrf	TX_PAD1		;Clear the padding bytes
	clrf	TX_PAD2		; "
	clrf	TX_PAD3		; "
	;TODO clear 20 tag bytes too?
CmdRea0	movlw	0x02		;Increment the read address by 512 - we do this
	addwf	M_BUF4,F	; on the first loop iteration because the first
	movlw	0		; sector on the MMC card is the controller
	addwfc	M_BUF3,F	; status block, conveniently
	addwfc	M_BUF2,F	; "
	movlw	0x51		;Set up a read command for the MMC card
	movwf	M_BUF1		; "
	bcf	PORTC,RC3	;Assert !CS
	btfss	M_FLAGS,M_FAIL	;If there haven't been any previous failures,
	call	MmcCmd		; send the MMC command
	btfsc	M_FLAGS,M_FAIL	;If the operation failed, set the MSB of the
	bsf	TX_STAT,7	; status returned by the drive to indicate so
	movlw	0x20		;Point FSR0 past the six header bytes and the
	movwf	FSR0H		; 20 'tag' bytes to where the real data starts
	movlw	0x1A		; "
	movwf	FSR0L		; "
	btfss	M_FLAGS,M_FAIL	;If the operation didn't fail, read data from
	call	MmcReadData	; the MMC card
	bsf	PORTC,RC3	;Deassert !CS
	btfsc	M_FLAGS,M_FAIL	;If the operation failed, set the MSB of the
	bsf	TX_STAT,7	; status returned by the drive to indicate so
	movlw	0x20		;Point FSR0 at the top of linear memory, where
	movwf	FSR0H		; we want transmission to start from
	clrf	FSR0L		; "
	movlw	77		;Set the group count to 77 (512 bytes of data,
	btfsc	M_FLAGS,M_FAIL	; 20 'tag' bytes, 6 byte header, checksum byte)
	movlw	1		; unless there was a failure, in which case set
	movwf	GROUPS		; it to 1 to carry the bad status back to mac
	call	CalcChecksum	;Calculate the checksum on our response
	movlp	high Transmit	;Initiate transmission
	call	Transmit	; "
	movlp	0		; "
	decfsz	TX_BLKS,F	;Decrement the block count; if it hits zero,
	bra	CmdRea0		; that's all the reading we were requested to
	goto	IdleJump	; do, so call it done

CmdWrite
	clrf	M_BUF5		;Shift the block address left by 9 (multiply it
	lslf	RC_ADRL,W	; by 512) to transform it into the byte address
	movwf	M_BUF4		; used by the MMC card
	rlf	RC_ADRM,W	; "
	movwf	M_BUF3		; "
	rlf	RC_ADRH,W	; "
	movwf	M_BUF2		; "
	movlw	0x02		;Increment the read address by 512 - we do this
	addwf	M_BUF4,F	; at the start of a write because the first
	movlw	0		; sector on the MMC card is the controller
	addwfc	M_BUF3,F	; status block
	addwfc	M_BUF2,F	; "
CmdWriteCont
	movlw	0x58		;Set up a write command for the MMC card, maybe
	movwf	M_BUF1		; with an address set up by a previous command
	bcf	PORTC,RC3	;Assert !CS
	btfss	M_FLAGS,M_FAIL	;If there haven't been any previous failures,
	call	MmcCmd		; send the MMC command
	movlw	0x20		;Point FSR0 past the sync, length, header, and
	movwf	FSR0H		; tag bytes, and to the data we'll be writing
	movlw	0x1D		; "
	movwf	FSR0L		; "
	btfss	M_FLAGS,M_FAIL	;If the operation didn't fail, write data to
	call	MmcWriteData	; the MMC card
	bsf	PORTC,RC3	;Deassert !CS
	movf	RC_BLKS,W	;Move the block count from receiver position to
	movwf	TX_BLKS		; transmitter position
	movlw	0x81		;Command number is the write command with the
	movwf	TX_CMDN		; MSB set
	clrf	TX_STAT		;A zero status means all is well, so if there
	btfsc	M_FLAGS,M_FAIL	; have been failures talking to the MMC card,
	bsf	TX_STAT,7	; set MSB to indicate so
	clrf	TX_PAD1		;Clear the padding bytes
	clrf	TX_PAD2		; "
	clrf	TX_PAD3		; "
	movlw	0x20		;Point FSR0 at the top of linear memory, where
	movwf	FSR0H		; we want transmission to start from
	clrf	FSR0L		; "
	movlw	1		;Success or failure, we're transmitting one
	movwf	GROUPS		; group in response
	call	CalcChecksum	;Calculate the checksum on our response
	movlp	high Transmit	;Initiate transmission
	call	Transmit	; "
	movlp	0		; "
	movlw	0x02		;Increment the read address by 512 in case more
	addwf	M_BUF4,F	; blocks are to come (with command number 0x41)
	movlw	0		; because its handler is this one, just jumped
	addwfc	M_BUF3,F	; into a bit late
	addwfc	M_BUF2,F	; "
	goto	IdleJump	;Done

CmdStatus
	clrf	M_BUF5		;Set the address to read the first sector of
	clrf	M_BUF4		; the MMC card
	clrf	M_BUF3		; "
	clrf	M_BUF2		; "
	movlw	0x51		;Set up a read command for the MMC card
	movwf	M_BUF1		; "
	movlw	0x83		;Command number is the status command with the
	movwf	TX_CMDN		; MSB set
	clrf	TX_BLKS		;Block count is a pad byte in status command
	clrf	TX_STAT		;A zero status means all is well
	clrf	TX_PAD1		;Clear the padding bytes
	clrf	TX_PAD2		; "
	clrf	TX_PAD3		; "
	bcf	PORTC,RC3	;Assert !CS
	btfss	M_FLAGS,M_FAIL	;If there haven't been any previous failures,
	call	MmcCmd		; send the MMC command
	btfsc	M_FLAGS,M_FAIL	;If the operation failed, set the MSB of the
	bsf	TX_STAT,7	; status returned by the drive to indicate so
	movlw	0x20		;Point FSR0 past the six header bytes to where
	movwf	FSR0H		; the real data starts
	movlw	0x06		; "
	movwf	FSR0L		; "
	btfss	M_FLAGS,M_FAIL	;If the operation didn't fail, read data from
	call	MmcReadData	; the MMC card
	bsf	PORTC,RC3	;Deassert !CS
	btfsc	M_FLAGS,M_FAIL	;If the operation failed, set the MSB of the
	bsf	TX_STAT,7	; status returned by the drive to indicate so
	movlw	0x20		;Point FSR0 at the top of linear memory, where
	movwf	FSR0H		; we want transmission to start from
	clrf	FSR0L		; "
	movlw	49		;Set the group count to 49 (336 bytes of data,
	btfsc	M_FLAGS,M_FAIL	; 6 byte header, checksum byte) unless there
	movlw	1		; was a failure, in which case set it to 1 to
	movwf	GROUPS		; carry the bad status back to mac
	call	CalcChecksum	;Calculate the checksum on our response
	movlp	high Transmit	;Initiate transmission
	call	Transmit	; "
	movlp	0		; "
	goto	IdleJump	;Done


;;; Subprograms ;;;

;Calculate the checksum of the block about to be transmitted, pointed to by
; FSR0 and containing GROUPS * 7 bytes.  Trashes TEMP, TEMP2, and FSR1.
CalcChecksum
	movf	FSR0H,W		;Copy FSR0 into FSR1
	movwf	FSR1H		; "
	movf	FSR0L,W		; "
	movwf	FSR1L		; "
	movf	GROUPS,W	;Copy the group count into TEMP
	movwf	TEMP		; "
	clrf	TEMP2		;Zero the running sum (mod 256)
	bra	CalcCh1		;Jump the first add so we sum one less than the
CalcCh0	moviw	FSR1++		; total bytes in the block and can write the
	addwf	TEMP2,F		; checksum to the very last byte position
CalcCh1	moviw	FSR1++		;Add the bytes in this group to the running sum
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	decfsz	TEMP,F		;Loop until we've summed the last group
	bra	CalcCh0		; "
	comf	TEMP2,W		;Two's complement the sum so the whole block
	addlw	1		; adds up to zero and write it to the block's
	movwi	FSR1++		; last byte
	return

;Calculate the checksum of the block just received, starting at the third byte
; of linear memory and containing GROUPS * 7 bytes.  Trashes TEMP, TEMP2, and
; FSR1.
CheckChecksum
	movlw	0x20		;Point FSR1 to the third byte of linear memory
	movwf	FSR1H		; "
	movlw	0x03		; "
	movwf	FSR1L		; "
	movf	GROUPS,W	;Copy the group count into TEMP
	movwf	TEMP		; "
	clrf	TEMP2		;Zero the running sum (mod 256)
CheckC0	moviw	FSR1++		;Add the next seven bytes to the running sum
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	moviw	FSR1++		; "
	addwf	TEMP2,F		; "
	decfsz	TEMP,F		;Loop until we've summed the last group
	bra	CheckC0		; "
	movf	TEMP2,F		;Set Z if block sum is 0 and checksum is good
	return

;Clear linear memory for the response to a command according to the expected
; response length according to mac, pointing FSR0 to the beginning of it and
; setting GROUPS to the number of groups to be sent.  Trashes TEMP.
ClearResponse
	movlw	0x20		;Point FSR0 to the top of linear memory
	movwf	FSR0H		; "
	clrf	FSR0L		; "
	movf	RC_RSPG,W	;Snuff the top bit of the number of groups in
	andlw	B'01111111'	; the expected response and copy this into
	movwf	GROUPS		; the group count as well as a temporary
	movwf	TEMP		; counter
	movlw	0		;Zero is what we're setting the buffer to
ClearR0	movwi	FSR0++		;Write seven zeroes to the buffer
	movwi	FSR0++		; "
	movwi	FSR0++		; "
	movwi	FSR0++		; "
	movwi	FSR0++		; "
	movwi	FSR0++		; "
	movwi	FSR0++		; "
	decfsz	TEMP,F		;Loop until we've zeroed the last group
	bra	ClearR0		; "
	movlw	0x20		;Point FSR0 to the top of linear memory again
	movwf	FSR0H		; "
	clrf	FSR0L		; "
	return


;;; MMC Subprograms ;;;

;Initialize MMC card.  Sets M_FAIL on fail.  Trashes TEMP.
MmcInit
	movlb	4		;This is where all the SSP registers are
	movlw	10		;Send 80 clocks on SPI interface to ensure MMC
	movwf	TEMP		; card is started up and in native command mode
MmcIni0	movlw	0xFF		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	decfsz	TEMP,F		; "
	bra	MmcIni0		; "
	movlb	0		;Assert !CS
	bcf	PORTC,RC3	; "
	movlw	0x40		;Send command 0 (expect R1-type response)
	movwf	M_BUF1		; which, with !CS asserted, signals to the card
	clrf	M_BUF2		; that we want to enter SPI mode
	clrf	M_BUF3		; "
	clrf	M_BUF4		; "
	clrf	M_BUF5		; "
	movlw	0x95		; "
	movwf	M_BUF6		; "
	bcf	M_FLAGS,M_CMDLR	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, unrecognized or
	return			; missing MMC card, fail the init operation
	movf	M_BUF1,W	;If this command returned any response other
	xorlw	0x01		; than 0x01 ('in idle state'), unrecognized MMC
	btfss	STATUS,Z	; card, fail the init operation
	bsf	M_FLAGS,M_FAIL	; "
	btfss	STATUS,Z	; "
	return			; "
MmcIni1	movlw	0x41		;Send command 1 (expect R1-type response),
	movwf	M_BUF1		; which tells the card to initialize
	clrf	M_BUF2		; "
	clrf	M_BUF3		; "
	clrf	M_BUF4		; "
	clrf	M_BUF5		; "
	clrf	M_BUF6		; "
	bcf	M_FLAGS,M_CMDLR	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, unrecognized MMC card,
	return			; fail the init operation
	movf	M_BUF1,W	;If it returned an 0x00 status, initialization
	btfsc	STATUS,Z	; is finished
	bra	MmcIni2		; "
	decf	M_BUF1,W	;If it returned an 0x01 status, initialization
	btfsc	STATUS,Z	; is still proceeding, so try again
	bra	MmcIni1		; "
	bsf	M_FLAGS,M_FAIL	;If it returned anything else, something is
	return			; awry, fail the init operation
MmcIni2	movlw	0x50		;Send command 16 (expect R1-type response) to
	movwf	M_BUF1		; tell the card we want to deal in 512-byte
	clrf	M_BUF2		; sectors
	clrf	M_BUF3		; "
	movlw	0x02		; "
	movwf	M_BUF4		; "
	clrf	M_BUF5		; "
	clrf	M_BUF6		; "
	bcf	M_FLAGS,M_CMDLR	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, something is wrong,
	return			; fail the init operation
	movf	M_BUF1,W	;If this command returned any response other
	btfss	STATUS,Z	; than 0x00, something is wrong, fail the init
	bsf	M_FLAGS,M_FAIL	; operation
	bsf	PORTC,RC3	;Deassert !CS
	return			;Regardless, we're done here

;Read 512 bytes of data from MMC card into buffer pointed to by FSR0.  Sets
; M_FAIL on fail. Trashes TEMP and FSR0.
MmcReadData
	movlb	4		;Switch to the bank with the SSP registers
	clrf	TEMP		;Try 256 times to get the data token
MmcRea1	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;If we've received the data token, skip ahead
	xorlw	0xFE		; "
	btfsc	STATUS,Z	; "
	bra	MmcRea2		; "
	DELAY	0		;If not, delay 768 cycles, decrement the retry
	decfsz	TEMP,F		; count, and try again
	bra	MmcRea1		; "
	bsf	M_FLAGS,M_FAIL	;If we didn't get the data token after 256
	movlb	0		; tries, give up and fail the operation
	return			; "
MmcRea2	clrf	TEMP		;Read 256 pairs of bytes
MmcRea3	movlw	0xFF		;Clock the next data byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;Move the byte we clocked out to the buffer and
	movwi	FSR0++		; increment the pointer
	movlw	0xFF		;Clock the next data byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;Move the byte we clocked out to the buffer and
	movwi	FSR0++		; increment the pointer
	decfsz	TEMP,F		;Decrement the counter and go back to read the
	bra	MmcRea3		; next byte pair, unless we're done
	movlw	0xFF		;Clock the first CRC byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlw	0xFF		;Clock the second CRC byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlb	0		;Return to the default bank
	return

;Write 512 bytes of data to MMC card from buffer pointed to by FSR0.  Sets
; M_FAIL on fail. Trashes TEMP and FSR0.
MmcWriteData
	movlb	4		;Switch to the bank with the SSP registers
	movlw	0xFF		;Clock a dummy byte while keeping MOSI high
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlw	0xFE		;Clock the data token into the MMC card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	clrf	TEMP		;Send 256 pairs of bytes
MmcWri0	moviw	FSR0++		;Clock the next data byte into the MMC card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	moviw	FSR0++		;Clock the next data byte into the MMC card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	decfsz	TEMP,F		;Decrement the counter and go back to write the
	bra	MmcWri0		; next byte pair, unless we're done
	clrf	SSP1BUF		;Clock a blank CRC byte into the MMC card
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	clrf	SSP1BUF		;Clock a blank CRC byte into the MMC card
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlw	0xFF		;Clock data response byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;If the status byte indicates that the write
	andlw	B'00011111'	; was rejected for any reason, fail the write
	xorlw	B'00000101'	; operation
	btfss	STATUS,Z	; "
	bra	MmcWri2		; "
	clrf	TEMP		;Check 65536 times to see if the write is done
	clrf	TEMP2		; "
MmcWri1	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;Check if MISO is still low, if it's not, the
	btfss	STATUS,Z	; write is done and we can return
	bra	MmcWri3		; "
	decfsz	TEMP,F		;If it's not done, try again
	bra	MmcWri1		; "
	decfsz	TEMP2,F		; "
	bra	MmcWri1		; "
MmcWri2	bsf	M_FLAGS,M_FAIL	;If out of tries, fail the write operation
MmcWri3	movlb	0		;Return to the default bank
	return

;Send the command contained in M_BUF1-6 to MMC card.  Sets M_FAIL on fail.
; Trashes TEMP.
MmcCmd
	bcf	M_FLAGS,M_FAIL	;Start out optimistic and say this didn't fail
	movlb	4		;Switch to the bank with the SSP registers
	movf	M_BUF1,W	;Clock out all six MMC buffer bytes as command
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_BUF2,W	; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_BUF3,W	; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_BUF4,W	; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_BUF5,W	; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_BUF6,W	; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlw	8		;Try to get status as many as eight times
	movwf	TEMP		; "
MmcCmd1	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	incf	SSP1BUF,W	;If we read back anything but 0xFF, skip ahead
	btfss	STATUS,Z	; "
	bra	MmcCmd2		; "
	decfsz	TEMP,F		;Decrement the attempt counter until we've
	bra	MmcCmd1		; tried eight times; if we haven't gotten a
	bsf	M_FLAGS,M_FAIL	; reply by the eighth attempt, signal failure
	movlb	0		; and return
	return			; "
MmcCmd2	decf	WREG,W		;Store the byte we received as R1-type status
	movwf	M_BUF1		; in the buffer
	btfss	M_FLAGS,M_CMDLR	;If we aren't expecting a long (R3/R7-type)
	bra	MmcCmd3		; reply, we're done
	movlw	0xFF		;Clock first extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_BUF2		; "
	movlw	0xFF		;Clock second extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_BUF3		; "
	movlw	0xFF		;Clock third extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_BUF4		; "
	movlw	0xFF		;Clock fourth extended reply byte out of the
	movwf	SSP1BUF		; MMC card while keeping MOSI high and store it
	btfss	SSP1STAT,BF	; in the buffer
	bra	$-1		; "
	movf	SSP1BUF,W	; "
	movwf	M_BUF5		; "
MmcCmd3	;TODO microchip's code sends another 8 clocks here but I can't find
	; anything in the spec that says we have to do that
	movlb	0
	return


;;; Interrupt Subprograms ;;;

;In receive mode, interrupt handler calls this code to decode 7-to-8 bytes
DecodeRecv
	movf	FSR1H,W		;If FSR1 == 0x2000, this is the first received
	andlw	B'00011111'	; stream and must be handled differently so the
	iorwf	FSR1L,W		; first 3 bytes from the IWM are preserved
	btfss	STATUS,Z	; "
	bra	Decode1		; "
	moviw	FSR0--		;If this was the first received stream, reckon
	moviw	FSR0--		; the group count as (FSR0-3)//8 and position
	moviw	FSR0--		; both pointers right after the initial 3 bytes
	lsrf	FSR0H,F		; received from the IWM (the sync byte, the
	rrf	FSR0L,F		; length of the message plus 0x81, and the
	lsrf	FSR0H,F		; expected length of the response plus 0x81)
	rrf	FSR0L,F		; "
	lsrf	FSR0H,F		; "
	rrf	FSR0L,W		; "
	movwf	TEMP2		; "
	movlw	0x20		; "
	movwf	FSR0H		; "
	movwf	FSR1H		; "
	movlw	0x03		; "
	movwf	FSR0L		; "
	movwf	FSR1L		; "
	bra	Decode2		; "
Decode1	moviw	FSR0--		;If this was not the first received stream,
	movf	FSR1L,W		; reckon the group count as (FSR0-FSR1-1)//8
	subwf	FSR0L,W		; and position FSR0 one byte ahead of FSR1 so
	movwf	TEMP2		; that the repeated sync byte is overwritten
	movf	FSR1H,W		; "
	subwfb	FSR0H,W		; "
	lsrf	WREG,W		; "
	rrf	TEMP2,F		; "
	lsrf	WREG,W		; "
	rrf	TEMP2,F		; "
	lsrf	WREG,W		; "
	rrf	TEMP2,F		; "
	movf	FSR1H,W		; "
	movwf	FSR0H		; "
	movf	FSR1L,W		; "
	movwf	FSR0L		; "
	moviw	FSR0++		; "
Decode2	movf	TEMP2,W		;Add the count of groups received in this
	addwf	GROUPS,F	; stream to the total groups received count
	movf	TEMP2,F		;If the group count is 0, don't do the next
	btfsc	STATUS,Z	; loop at all
	bra	Decode4		; "
Decode3	moviw	FSR0++		;Pick up the byte with the LSBs of the next 
	movwf	TEMP		; seven
	lsrf	TEMP,F		;Rotate the LSB of the first byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	lsrf	TEMP,F		;Rotate the LSB of the second byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	lsrf	TEMP,F		;Rotate the LSB of the third byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	lsrf	TEMP,F		;Rotate the LSB of the fourth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	lsrf	TEMP,F		;Rotate the LSB of the fifth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	lsrf	TEMP,F		;Rotate the LSB of the sixth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	lsrf	TEMP,F		;Rotate the LSB of the last byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	decfsz	TEMP2,F		;Decrement the group count and loop if it's not
	bra	Decode3		; yet zero
Decode4	movf	FSR1H,W		;FSR1 now points to one past the end of final
	movwf	FSR0H		; data, copy it to FSR0 so future received data
	movf	FSR1L,W		; is written there too
	movwf	FSR0L		; "
	return


;;; Transmitter Code ;;;

	org	0x17FE

XmitIdleJump
	movlb	31		;We're in a state we don't know how to handle,
	movlw	high IdleJump	; so warp into the idle jump table
	movwf	TOSH		; "
	movlw	low IdleJump	; "
	movwf	TOSL		; "
	movlb	0		; "
	return			; "

;Transmits GROUPS 7-byte groups of non-encoded data starting at FSR0.  Returns
; to the caller only if all is well.  Trashes FSR0, TEMP, TEMP2, and GROUPS.
Transmit
	bsf	FLAGS,TXNOTRC	;Set flags for a transmit
	movf	PORTA,W		;Check the current state
	xorlw	B'00000100'	;If it's 2, then mac is ready for us to request
	btfss	STATUS,Z	; to send; if it's anything else, we can't
	bra	XmitIdleJump	; handle it so return to the idle jump table
	bcf	PORTC,RC4	;Assert !HSHK to request to send
Transm0	movf	PORTA,W		;Check the current state
	xorlw	B'00000100'	;If we're still in state 2, keep checking
	btfsc	STATUS,Z	; "
	bra	Transm0		; "
	xorlw	B'00000010'	;If we're in state 3, most likely mac is about
	btfsc	STATUS,Z	; to change to state 1, so keep checking
	bra	Transm0		; "
	xorlw	B'00000100'	;If we're in state 1, get ready to send; if
	btfss	STATUS,Z	; we're in any other state, we can't handle it
	bra	XmitIdleJump	; here so return to the idle jump table
	movlb	7		;Clear any IOC flags that have been set up to
	clrf	IOCAF		; now
	;fall through

Transmitter
	movlb	0		;Point BSR to 0 so we can access PORTC
	bsf	INTCON,GIE	;Enable interrupts
	movlw	128		;Set up fractional delay counter (see below)
	movwf	TEMP		; "
	movlw	30		;First send 30 groups of eight ones and two
	movwf	TEMP2		; zeroes to make sure IWM is synced with us
XSyncLp	bcf	PORTC,RC4	; 0 IWM listens to falling edges only
	movlw	87		;+1 Reckon the delay to slow this from a 2 us
	addwf	TEMP,F		;+2  data cell to a 96/47 us data cell as 16
	btfsc	STATUS,C	;+3  cycles plus 16/47 of a cycle, approximated
	bra	$+1		;+4  here as 87/256 of a cycle
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	nop			;-4
	decfsz	TEMP2,F		;-3
	bra	XSyncLp		;-2 -1
	nop			;-1
XSync	bcf	PORTC,RC4	; 0 Start of 0xAA sync byte
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	nop			;-2
XDataLp	bsf	INTCON,GIE	;-1 Reenable interrupts if they were disabled
	bcf	PORTC,RC4	; 0 Start 7-to-8-encoded group; MSB always set
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	6[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	5[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	4[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	3[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	2[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	1[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	DNOP			;+6 +7
	DNOP			;-8 -7
	DNOP			;-6 -5
	DNOP			;-4 -3
	moviw	0[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	nop			;+6
	bcf	INTCON,GIE	;+7 Disable interrupts while we increment the
	movlw	7		;-8  read pointer by 7 and decrement the group
	addwf	FSR0L,F		;-7  count by 1 so the changes are all or
	movlw	0		;-6  nothing; if group count has not yet hit
	addwfc	FSR0H,F		;-5  zero, loop around to transmit the next
	decfsz	GROUPS,F	;-4  group of seven bytes (sent as eight)
	goto	XDataLp		;-3 -2  "
XFiniLp	movf	PORTA,W		;Check the current state
	xorlw	B'00000010'	;If it's still 1 (communication), mac hasn't
	btfsc	STATUS,Z	; realized we're done transmitting yet, so loop
	bra	XFiniLp		; "
	xorlw	B'00000100'	;If it's 3 (request send), mac is probably
	btfsc	STATUS,Z	; transitioning through on its way to 2, so
	bra	XFiniLp		; loop
	xorlw	B'00000010'	;If it's anything but 2 (idle), give it to the
	btfss	STATUS,Z	; interrupt handler to deal with because
	bsf	INTCON,GIE	; something weird's going on
	return


;;; Receiver Code ;;;

RecvIdleJump
	movlb	31		;We're in a state we don't know how to handle,
	movlw	high IdleJump	; so warp into the idle jump table
	movwf	TOSH		; "
	movlw	low IdleJump	; "
	movwf	TOSL		; "
	movlb	0		; "
	return			; "

;Receives data from the mac.  Returns to the caller only if all is well.
; Overwrites linear memory with received data in the following way: first byte
; is sync byte, second byte is 0x80 + number of groups in command according to
; mac, third byte is 0x80 + number of groups in anticipated response, followed
; by a string of GROUPS seven-byte decoded groups.  Trashes FSR0, FSR1, TEMP,
; and TEMP2.
Receive
	movlw	0x20		;Reset both pointers to the top of linear
	movwf	FSR0H		; memory
	movwf	FSR1H		; "
	clrf	FSR0L		; "
	clrf	FSR1L		; "
	bcf	FLAGS,TXNOTRC	;Set flags for a receive
	clrf	GROUPS		;Clear counter of total received groups
Receiv0	movf	PORTA,W		;Check the current state
	xorlw	B'00000100'	;If it's 2, then mac hasn't said it's ready to
	btfsc	STATUS,Z	; send yet, keep waiting
	bra	Receiv0		; "
	xorlw	B'00000010'	;If it's 3, mac has said it's ready to send,
	btfss	STATUS,Z	; if it's anything else, we can't handle it
	bra	RecvIdleJump	; here so return to the idle jump table
	bcf	PORTC,RC4	;Assert !HSHK to say we're ready to receive
Receiv1	movf	PORTA,W		;If we're still in state 3, keep checking
	xorlw	B'00000110'	; "
	btfsc	STATUS,Z	; "
	bra	Receiv1		; "
	xorlw	B'00000100'	;If we're in state 1, get ready to receive; if
	btfss	STATUS,Z	; we're in any other state, we can't handle it
	bra	RecvIdleJump	; here so return to the idle jump table
	movlb	7		;Clear any IOC flags that have been set up to
	clrf	IOCAF		; now
	;fall through

Receiver
	movlb	0		;Point BSR to 0 so we can access PORTC
	bsf	INTCON,GIE	;Enable interrupts
StZero	btfss	PORTC,RC5	;Signal starts at zero, wait until transition
	bra	$-1		; to one, this is MSB (always 1) of first byte
	movlw	B'10000000'	;003 cycles, 0.18 bit times
	movwf	INDF0		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToZero6		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToZero6		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToZero6		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToZero6		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToZero6		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToZero6		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToZero6		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToZero6		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToZero5		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToZero5		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToZero5		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToZero5		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToZero5		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToZero5		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToZero5		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToZero5		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToZero4		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToZero4		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToZero4		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToZero4		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToZero4		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToZero4		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToZero4		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToZero4		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToZero4		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToZero3		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToZero3		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToZero3		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToZero3		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToZero3		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToZero3		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToZero3		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToZero3		;074 cycles, 4.53 bit times
	btfss	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	ToZero2		;076 cycles, 4.65 bit times
	btfss	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	ToZero2		;078 cycles, 4.77 bit times
	btfss	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	ToZero2		;080 cycles, 4.90 bit times
	btfss	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	ToZero2		;082 cycles, 5.02 bit times
	btfss	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	ToZero2		;084 cycles, 5.14 bit times
	btfss	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	ToZero2		;086 cycles, 5.26 bit times
	btfss	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	ToZero2		;088 cycles, 5.39 bit times
	btfss	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	ToZero2		;090 cycles, 5.51 bit times
	btfss	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	ToZero1		;092 cycles, 5.63 bit times
	btfss	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	ToZero1		;094 cycles, 5.75 bit times
	btfss	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	ToZero1		;096 cycles, 5.88 bit times
	btfss	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	ToZero1		;098 cycles, 6.00 bit times
	btfss	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	ToZero1		;100 cycles, 6.12 bit times
	btfss	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	ToZero1		;102 cycles, 6.24 bit times
	btfss	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	ToZero1		;104 cycles, 6.36 bit times
	btfss	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	ToZero1		;106 cycles, 6.49 bit times
	btfss	PORTC,RC5	;107 cycles, 6.55 bit times
	goto	ToZero0		;108 cycles, 6.61 bit times
	btfss	PORTC,RC5	;109 cycles, 6.67 bit times
	goto	ToZero0		;110 cycles, 6.73 bit times
	btfss	PORTC,RC5	;111 cycles, 6.79 bit times
	goto	ToZero0		;112 cycles, 6.85 bit times
	btfss	PORTC,RC5	;113 cycles, 6.92 bit times
	goto	ToZero0		;114 cycles, 6.98 bit times
	btfss	PORTC,RC5	;115 cycles, 7.04 bit times
	goto	ToZero0		;116 cycles, 7.10 bit times
	btfss	PORTC,RC5	;117 cycles, 7.16 bit times
	goto	ToZero0		;118 cycles, 7.22 bit times
	btfss	PORTC,RC5	;119 cycles, 7.28 bit times
	goto	ToZero0		;120 cycles, 7.34 bit times
	btfss	PORTC,RC5	;121 cycles, 7.40 bit times
	goto	ToZero0		;122 cycles, 7.47 bit times
	moviw	FSR0++		;123 cycles, 7.53 bit times
	goto	StOne		;124 cycles, 7.59 bit times

ToOne6	movlw	B'01000000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToZero5		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToZero5		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToZero5		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToZero5		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToZero5		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToZero5		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToZero5		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToZero5		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToZero4		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToZero4		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToZero4		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToZero4		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToZero4		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToZero4		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToZero4		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToZero4		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToZero3		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToZero3		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToZero3		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToZero3		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToZero3		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToZero3		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToZero3		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToZero3		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToZero3		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToZero2		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToZero2		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToZero2		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToZero2		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToZero2		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToZero2		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToZero2		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToZero2		;074 cycles, 4.53 bit times
	btfss	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	ToZero1		;076 cycles, 4.65 bit times
	btfss	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	ToZero1		;078 cycles, 4.77 bit times
	btfss	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	ToZero1		;080 cycles, 4.90 bit times
	btfss	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	ToZero1		;082 cycles, 5.02 bit times
	btfss	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	ToZero1		;084 cycles, 5.14 bit times
	btfss	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	ToZero1		;086 cycles, 5.26 bit times
	btfss	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	ToZero1		;088 cycles, 5.39 bit times
	btfss	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	ToZero1		;090 cycles, 5.51 bit times
	btfss	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	ToZero0		;092 cycles, 5.63 bit times
	btfss	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	ToZero0		;094 cycles, 5.75 bit times
	btfss	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	ToZero0		;096 cycles, 5.88 bit times
	btfss	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	ToZero0		;098 cycles, 6.00 bit times
	btfss	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	ToZero0		;100 cycles, 6.12 bit times
	btfss	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	ToZero0		;102 cycles, 6.24 bit times
	btfss	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	ToZero0		;104 cycles, 6.36 bit times
	btfss	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	ToZero0		;106 cycles, 6.49 bit times
	moviw	FSR0++		;107 cycles, 6.55 bit times
	goto	StOne		;108 cycles, 6.61 bit times

ToOne5	movlw	B'00100000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToZero4		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToZero4		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToZero4		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToZero4		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToZero4		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToZero4		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToZero4		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToZero4		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToZero3		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToZero3		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToZero3		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToZero3		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToZero3		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToZero3		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToZero3		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToZero3		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToZero2		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToZero2		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToZero2		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToZero2		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToZero2		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToZero2		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToZero2		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToZero2		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToZero2		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToZero1		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToZero1		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToZero1		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToZero1		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToZero1		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToZero1		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToZero1		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToZero1		;074 cycles, 4.53 bit times
	btfss	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	ToZero0		;076 cycles, 4.65 bit times
	btfss	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	ToZero0		;078 cycles, 4.77 bit times
	btfss	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	ToZero0		;080 cycles, 4.90 bit times
	btfss	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	ToZero0		;082 cycles, 5.02 bit times
	btfss	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	ToZero0		;084 cycles, 5.14 bit times
	btfss	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	ToZero0		;086 cycles, 5.26 bit times
	btfss	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	ToZero0		;088 cycles, 5.39 bit times
	btfss	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	ToZero0		;090 cycles, 5.51 bit times
	moviw	FSR0++		;091 cycles, 5.57 bit times
	goto	StOne		;092 cycles, 5.63 bit times

ToOne4	movlw	B'00010000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToZero3		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToZero3		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToZero3		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToZero3		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToZero3		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToZero3		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToZero3		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToZero3		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToZero2		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToZero2		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToZero2		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToZero2		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToZero2		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToZero2		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToZero2		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToZero2		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToZero1		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToZero1		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToZero1		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToZero1		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToZero1		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToZero1		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToZero1		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToZero1		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToZero1		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToZero0		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToZero0		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToZero0		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToZero0		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToZero0		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToZero0		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToZero0		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToZero0		;074 cycles, 4.53 bit times
	moviw	FSR0++		;075 cycles, 4.59 bit times
	goto	StOne		;076 cycles, 4.65 bit times

ToOne3	movlw	B'00001000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToZero2		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToZero2		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToZero2		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToZero2		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToZero2		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToZero2		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToZero2		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToZero2		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToZero1		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToZero1		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToZero1		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToZero1		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToZero1		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToZero1		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToZero1		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToZero1		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToZero0		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToZero0		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToZero0		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToZero0		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToZero0		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToZero0		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToZero0		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToZero0		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToZero0		;058 cycles, 3.55 bit times
	moviw	FSR0++		;059 cycles, 3.61 bit times
	goto	StOne		;060 cycles, 3.67 bit times

ToOne2	movlw	B'00000100'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToZero1		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToZero1		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToZero1		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToZero1		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToZero1		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToZero1		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToZero1		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToZero1		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToZero0		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToZero0		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToZero0		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToZero0		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToZero0		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToZero0		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToZero0		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToZero0		;040 cycles, 2.45 bit times
	moviw	FSR0++		;041 cycles, 2.51 bit times
	goto	StOne		;042 cycles, 2.57 bit times

ToOne1	movlw	B'00000010'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToZero0		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToZero0		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToZero0		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToZero0		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToZero0		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToZero0		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToZero0		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToZero0		;024 cycles, 1.47 bit times
	moviw	FSR0++		;025 cycles, 1.53 bit times
	goto	StOne		;026 cycles, 1.59 bit times

ToOne0	movlw	B'00000001'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	moviw	FSR0++		;005 cycles, 0.31 bit times
	;fall through
	
StOne	btfsc	PORTC,RC5	;Signal starts at one, wait until transition
	bra	$-1		; to zero, this is MSB (always 1) of first byte
	movlw	B'10000000'	;003 cycles, 0.18 bit times
	movwf	INDF0		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToOne6		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToOne6		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToOne6		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToOne6		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToOne6		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToOne6		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToOne6		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToOne6		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToOne5		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToOne5		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToOne5		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToOne5		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToOne5		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToOne5		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToOne5		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToOne5		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToOne4		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToOne4		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToOne4		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToOne4		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToOne4		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToOne4		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToOne4		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToOne4		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToOne4		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToOne3		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToOne3		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToOne3		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToOne3		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToOne3		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToOne3		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToOne3		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToOne3		;074 cycles, 4.53 bit times
	btfsc	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	ToOne2		;076 cycles, 4.65 bit times
	btfsc	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	ToOne2		;078 cycles, 4.77 bit times
	btfsc	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	ToOne2		;080 cycles, 4.90 bit times
	btfsc	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	ToOne2		;082 cycles, 5.02 bit times
	btfsc	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	ToOne2		;084 cycles, 5.14 bit times
	btfsc	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	ToOne2		;086 cycles, 5.26 bit times
	btfsc	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	ToOne2		;088 cycles, 5.39 bit times
	btfsc	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	ToOne2		;090 cycles, 5.51 bit times
	btfsc	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	ToOne1		;092 cycles, 5.63 bit times
	btfsc	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	ToOne1		;094 cycles, 5.75 bit times
	btfsc	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	ToOne1		;096 cycles, 5.88 bit times
	btfsc	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	ToOne1		;098 cycles, 6.00 bit times
	btfsc	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	ToOne1		;100 cycles, 6.12 bit times
	btfsc	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	ToOne1		;102 cycles, 6.24 bit times
	btfsc	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	ToOne1		;104 cycles, 6.36 bit times
	btfsc	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	ToOne1		;106 cycles, 6.49 bit times
	btfsc	PORTC,RC5	;107 cycles, 6.55 bit times
	goto	ToOne0		;108 cycles, 6.61 bit times
	btfsc	PORTC,RC5	;109 cycles, 6.67 bit times
	goto	ToOne0		;110 cycles, 6.73 bit times
	btfsc	PORTC,RC5	;111 cycles, 6.79 bit times
	goto	ToOne0		;112 cycles, 6.85 bit times
	btfsc	PORTC,RC5	;113 cycles, 6.92 bit times
	goto	ToOne0		;114 cycles, 6.98 bit times
	btfsc	PORTC,RC5	;115 cycles, 7.04 bit times
	goto	ToOne0		;116 cycles, 7.10 bit times
	btfsc	PORTC,RC5	;117 cycles, 7.16 bit times
	goto	ToOne0		;118 cycles, 7.22 bit times
	btfsc	PORTC,RC5	;119 cycles, 7.28 bit times
	goto	ToOne0		;120 cycles, 7.34 bit times
	btfsc	PORTC,RC5	;121 cycles, 7.40 bit times
	goto	ToOne0		;122 cycles, 7.47 bit times
	moviw	FSR0++		;123 cycles, 7.53 bit times
	goto	StZero		;124 cycles, 7.59 bit times

ToZero6	movlw	B'01000000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToOne5		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToOne5		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToOne5		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToOne5		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToOne5		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToOne5		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToOne5		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToOne5		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToOne4		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToOne4		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToOne4		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToOne4		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToOne4		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToOne4		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToOne4		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToOne4		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToOne3		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToOne3		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToOne3		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToOne3		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToOne3		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToOne3		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToOne3		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToOne3		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToOne3		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToOne2		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToOne2		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToOne2		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToOne2		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToOne2		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToOne2		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToOne2		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToOne2		;074 cycles, 4.53 bit times
	btfsc	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	ToOne1		;076 cycles, 4.65 bit times
	btfsc	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	ToOne1		;078 cycles, 4.77 bit times
	btfsc	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	ToOne1		;080 cycles, 4.90 bit times
	btfsc	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	ToOne1		;082 cycles, 5.02 bit times
	btfsc	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	ToOne1		;084 cycles, 5.14 bit times
	btfsc	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	ToOne1		;086 cycles, 5.26 bit times
	btfsc	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	ToOne1		;088 cycles, 5.39 bit times
	btfsc	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	ToOne1		;090 cycles, 5.51 bit times
	btfsc	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	ToOne0		;092 cycles, 5.63 bit times
	btfsc	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	ToOne0		;094 cycles, 5.75 bit times
	btfsc	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	ToOne0		;096 cycles, 5.88 bit times
	btfsc	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	ToOne0		;098 cycles, 6.00 bit times
	btfsc	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	ToOne0		;100 cycles, 6.12 bit times
	btfsc	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	ToOne0		;102 cycles, 6.24 bit times
	btfsc	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	ToOne0		;104 cycles, 6.36 bit times
	btfsc	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	ToOne0		;106 cycles, 6.49 bit times
	moviw	FSR0++		;107 cycles, 6.55 bit times
	goto	StZero		;108 cycles, 6.61 bit times

ToZero5	movlw	B'00100000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToOne4		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToOne4		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToOne4		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToOne4		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToOne4		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToOne4		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToOne4		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToOne4		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToOne3		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToOne3		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToOne3		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToOne3		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToOne3		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToOne3		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToOne3		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToOne3		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToOne2		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToOne2		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToOne2		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToOne2		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToOne2		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToOne2		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToOne2		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToOne2		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToOne2		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToOne1		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToOne1		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToOne1		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToOne1		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToOne1		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToOne1		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToOne1		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToOne1		;074 cycles, 4.53 bit times
	btfsc	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	ToOne0		;076 cycles, 4.65 bit times
	btfsc	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	ToOne0		;078 cycles, 4.77 bit times
	btfsc	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	ToOne0		;080 cycles, 4.90 bit times
	btfsc	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	ToOne0		;082 cycles, 5.02 bit times
	btfsc	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	ToOne0		;084 cycles, 5.14 bit times
	btfsc	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	ToOne0		;086 cycles, 5.26 bit times
	btfsc	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	ToOne0		;088 cycles, 5.39 bit times
	btfsc	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	ToOne0		;090 cycles, 5.51 bit times
	moviw	FSR0++		;091 cycles, 5.57 bit times
	goto	StZero		;092 cycles, 5.63 bit times

ToZero4	movlw	B'00010000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToOne3		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToOne3		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToOne3		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToOne3		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToOne3		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToOne3		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToOne3		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToOne3		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToOne2		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToOne2		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToOne2		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToOne2		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToOne2		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToOne2		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToOne2		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToOne2		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToOne1		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToOne1		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToOne1		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToOne1		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToOne1		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToOne1		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToOne1		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToOne1		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToOne1		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	ToOne0		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	ToOne0		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	ToOne0		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	ToOne0		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	ToOne0		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	ToOne0		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	ToOne0		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	ToOne0		;074 cycles, 4.53 bit times
	moviw	FSR0++		;075 cycles, 4.59 bit times
	goto	StZero		;076 cycles, 4.65 bit times

ToZero3	movlw	B'00001000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToOne2		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToOne2		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToOne2		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToOne2		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToOne2		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToOne2		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToOne2		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToOne2		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToOne1		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToOne1		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToOne1		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToOne1		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToOne1		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToOne1		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToOne1		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToOne1		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	ToOne0		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	ToOne0		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	ToOne0		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	ToOne0		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	ToOne0		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	ToOne0		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	ToOne0		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	ToOne0		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	ToOne0		;058 cycles, 3.55 bit times
	moviw	FSR0++		;059 cycles, 3.61 bit times
	goto	StZero		;060 cycles, 3.67 bit times

ToZero2	movlw	B'00000100'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToOne1		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToOne1		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToOne1		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToOne1		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToOne1		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToOne1		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToOne1		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToOne1		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	ToOne0		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	ToOne0		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	ToOne0		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	ToOne0		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	ToOne0		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	ToOne0		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	ToOne0		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	ToOne0		;040 cycles, 2.45 bit times
	moviw	FSR0++		;041 cycles, 2.51 bit times
	goto	StZero		;042 cycles, 2.57 bit times

ToZero1	movlw	B'00000010'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	DNOP			;005-006 cycles, 0.31-0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	ToOne0		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	ToOne0		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	ToOne0		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	ToOne0		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	ToOne0		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	ToOne0		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	ToOne0		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	ToOne0		;024 cycles, 1.47 bit times
	moviw	FSR0++		;025 cycles, 1.53 bit times
	goto	StZero		;026 cycles, 1.59 bit times

ToZero0	movlw	B'00000001'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	moviw	FSR0++		;005 cycles, 0.31 bit times
	goto	StZero		;006 cycles, 0.37 bit times


;;; End of Program ;;;
	end
