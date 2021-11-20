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

ABORTED	equ	7	;Set if a transmit or receive operation was aborted
DEV3ON	equ	4	;Set if we're emulating a fourth device
DEV2ON	equ	3	;Set if we're emulating a third device
DEV1ON	equ	2	;Set if we're emulating a second device
DEVSEL1	equ	1	;MSB of which device is selected
DEVSEL0	equ	0	;LSB of which device is selected

M_FAIL	equ	7	;Set when there's been a failure on the MMC interface
M_CMDLR	equ	6	;Set when R3 or R7 is expected (5 bytes), not R1
M_CMDRB	equ	5	;Set when an R1b is expected (busy signal after reply)
M_MBRD	equ	4	;Set when a multiblock read is in progress
M_MBWR	equ	3	;Set when a multiblock write is in progress


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS	;You've got to have flags
	GROUPS	;Count of groups to transmit, count of groups received
	CRC16H	;High byte of the CRC16 register
	CRC16L	;Low byte of the CRC16 register
	M_FLAGS	;MMC flags
	M_CMDN	;The MMC command to be sent, later the R1 response byte	
	M_ADR3	;First (high) byte of the address, first byte of R3/R7 response
	M_ADR2	;Second byte of the address, second byte of R3/R7 response
	M_ADR1	;Third byte of the address, third byte of R3/R7 response
	M_ADR0	;Fourth (low) byte of the address, last byte of R3/R7 response
	M_CRC	;CRC byte of the command to be sent
	TEMP	;Various purposes
	TEMP2	;Various purposes
	TEMP3	;Various purposes
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
	nop			;We expect RA0 to be high
	nop			; "
	
IntEn0	movlb	7		;This is the suspend state, so we should never
	clrf	IOCAF		; get here unless it's a blip on one of the
	retfie			; state lines - if it is, clear and return and
	nop			; just hope timing isn't too badly wrecked
IntEn1	movlb	7		;This is the communication state, so we should
	clrf	IOCAF		; never get here unless it's a blip on one of
	retfie			; the state lines - if it is, clear and return
	nop			; and just hope timing isn't too badly wrecked
IntEn2	bsf	PORTC,RC4	;Being in state 2 (idle) without first going to
	bra	IntAbort	; state 3 means mac wants to abort the
	nop			; operation in progress
	nop			; "
IntEn3	bra	IntDoneReceive	;This means mac is done and is waiting for
	nop			; !HSHK to be deasserted
	nop			; "
	nop			; "
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

IntDoneReceive
	movlp	high RecvDecode	;Decode the data we've received
	call	RecvDecode	; "
	bsf	PORTC,RC4	;Deassert !HSHK to acknowledge receive is done
IntDRc0	decf	PORTA,W		;Check the current state
	xorlw	B'00000110'	;If it's still 3, mac hasn't acknowledged our
	btfsc	STATUS,Z	; acknowledgment of the end of received data
	bra	IntDRc0		; yet, so loop again
	xorlw	B'00000010'	;If it's anything else but 2 (idle), something
	btfss	STATUS,Z	; weird's going on, so abort
	bra	IntAbort	; "
	movlb	7		;Clear the interrupt that got us here and any
	clrf	IOCAF		; we received after that
	movlb	31		;Swallow the interrupt return address and
	decf	STKPTR,F	; instead return to who called receiver
	movlb	0		; without reenabling interrupts
	movlp	0		;Reset PCLATH to where we're returning
	return			; "

IntAbort
	movlb	7		;Clear the interrupt that got us here and any
	clrf	IOCAF		; we received after that
	movlb	31		;Swallow the return address where the PC was
	decf	STKPTR,F	; when the interrupt happened
	movlb	0		; "
	bsf	FLAGS,ABORTED	;Raise the aborted flag
	movlp	0		;Reset PCLATH to where we're returning
	return			;Return to caller without reenabling interrupts


;;; Init ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	IOCAP		;When interrupts are on, any change on !ENBL,
	movlw	B'00111100'	; PH3, PH2, or PH1 triggers one; this is what
	movwf	IOCAP		; gets us out of the endless receive loop
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

	banksel	LATA		;Next !ENBL off, RD and !CS high to start
	movlw	B'00111111'
	movwf	LATA
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

	bsf	FLAGS,DEV1ON
	bsf	FLAGS,DEV2ON
	bsf	FLAGS,DEV3ON

	goto	Dev0Jump


;;; Mainline Code ;;;

GetCommand
	bsf	PORTC,RC4	;Deassert !HSHK and prepare to receive
	movlp	high Receive	;Initiate receive
	call	Receive		; "
	btfsc	FLAGS,ABORTED	;If it was aborted, return to idle loop
	return			; "
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
	;TODO log unknown commands and expected response lengths somehow
	call	ClearResponse	;Clear the buffer according to expectations
	movf	TEMP2,W		;Our reply is going to be the command number
	movwf	TX_CMDN		; with its MSB set, by convention, but data
	bsf	TX_CMDN,7	; will be all zeroes
	call	CalcChecksum	;Calculate the checksum of this placeholder
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	return			;Done, hope the host doesn't mind

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
	movlw	0x20		;Point FSR0 at the top of linear memory, where
	movwf	FSR0H		; we want transmission to start from
	clrf	FSR0L		; "
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	return			;Done

CmdRead
	call	MmcStopOngoing	;Stop any multiblock read or write
	clrf	M_ADR0		;Shift the block address left by 9 (multiply it
	lslf	RC_ADRL,W	; by 512) to transform it into the byte address
	movwf	M_ADR1		; used by the MMC card
	rlf	RC_ADRM,W	; "
	movwf	M_ADR2		; "
	rlf	RC_ADRH,W	; "
	andlw	B'00000111'
	movwf	M_ADR3		; "
	btfsc	FLAGS,DEVSEL1
	bsf	M_ADR3,4
	btfsc	FLAGS,DEVSEL0
	bsf	M_ADR3,3
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
	addwf	M_ADR1,F	; on the first loop iteration because the first
	movlw	0		; sector on the MMC card is the controller
	addwfc	M_ADR2,F	; status block, conveniently
	addwfc	M_ADR3,F	; "
	movlw	0x51		;Set up a read command for the MMC card (R1-
	movwf	M_CMDN		; type response)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
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
	btfsc	FLAGS,ABORTED	;If the command was aborted, return to idle
	return			; loop
	decfsz	TX_BLKS,F	;Decrement the block count; if it hits zero,
	bra	CmdRea0		; that's all the reading we were requested to
	return			; do, so call it done

CmdWrite
	call	MmcStopOngoing	;Stop any multiblock read or write
	clrf	M_ADR0		;Shift the block address left by 9 (multiply it
	lslf	RC_ADRL,W	; by 512) to transform it into the byte address
	movwf	M_ADR1		; used by the MMC card
	rlf	RC_ADRM,W	; "
	movwf	M_ADR2		; "
	rlf	RC_ADRH,W	; "
	andlw	B'00000111'
	movwf	M_ADR3		; "
	btfsc	FLAGS,DEVSEL1
	bsf	M_ADR3,4
	btfsc	FLAGS,DEVSEL0
	bsf	M_ADR3,3
	movlw	0x02		;Increment the read address by 512 - we do this
	addwf	M_ADR1,F	; at the start of a write because the first
	movlw	0		; sector on the MMC card is the controller
	addwfc	M_ADR2,F	; status block
	addwfc	M_ADR3,F	; "
	movlw	0x59		;Set up a write command for the MMC card (R1-
	movwf	M_CMDN		; type reply)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	bsf	M_FLAGS,M_MBWR	;Set the flag for an ongoing multiblock write
	bcf	PORTC,RC3	;Assert !CS
	btfss	M_FLAGS,M_FAIL	;If there haven't been any previous failures,
	call	MmcCmd		; send the MMC command
CmdWriteCont
	btfss	M_FLAGS,M_MBWR	;If for some reason we got here without there
	bra	NakCommand	; being a multiblock write going on, NAK
	movlw	0x20		;Point FSR0 past the sync, length, header, and
	movwf	FSR0H		; tag bytes, and to the data we'll be writing
	movlw	0x1D		; "
	movwf	FSR0L		; "
	btfss	M_FLAGS,M_FAIL	;If the operation didn't fail, write data to
	call	MmcWriteData	; the MMC card
	decf	RC_BLKS,W	;If the blocks-remaining counter is at one,
	btfsc	STATUS,Z	; stop the multiblock write here before sending
	call	MmcStopOngoing	; our response to the mac
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
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	return			;Done

CmdStatus
	call	MmcStopOngoing	;Stop any multiblock read or write
	clrf	M_ADR0		;Set the address to read the first sector of
	clrf	M_ADR1		; the MMC card
	clrf	M_ADR2		; "
	clrf	M_ADR3		; "
	btfsc	FLAGS,DEVSEL1
	bsf	M_ADR3,4
	btfsc	FLAGS,DEVSEL0
	bsf	M_ADR3,3
	movlw	0x51		;Set up a read command for the MMC card (R1-
	movwf	M_CMDN		; type reply)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
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
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	return			;Done


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
	clrf	M_FLAGS		;Make sure flags are all clear to begin with
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
	movwf	M_CMDN		; which, with !CS asserted, signals to the card
	clrf	M_ADR3		; that we want to enter SPI mode
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, unrecognized or
	return			; missing MMC card, fail the init operation
	movf	M_CMDN,W	;If this command returned any response other
	xorlw	0x01		; than 0x01 ('in idle state'), unrecognized MMC
	btfss	STATUS,Z	; card, fail the init operation
	bsf	M_FLAGS,M_FAIL	; "
	btfss	STATUS,Z	; "
	return			; "
MmcIni1	movlw	0x41		;Send command 1 (expect R1-type response),
	movwf	M_CMDN		; which tells the card to initialize
	clrf	M_ADR3		; "
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, unrecognized MMC card,
	return			; fail the init operation
	movf	M_CMDN,W	;If it returned an 0x00 status, initialization
	btfsc	STATUS,Z	; is finished
	bra	MmcIni2		; "
	decf	M_CMDN,W	;If it returned an 0x01 status, initialization
	btfsc	STATUS,Z	; is still proceeding, so try again
	bra	MmcIni1		; "
	bsf	M_FLAGS,M_FAIL	;If it returned anything else, something is
	return			; awry, fail the init operation
MmcIni2	movlw	0x50		;Send command 16 (expect R1-type response) to
	movwf	M_CMDN		; tell the card we want to deal in 512-byte
	clrf	M_ADR3		; sectors
	clrf	M_ADR2		; "
	movlw	0x02		; "
	movwf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, something is wrong,
	return			; fail the init operation
	movf	M_CMDN,W	;If this command returned any response other
	btfss	STATUS,Z	; than 0x00, something is wrong, fail the init
	bsf	M_FLAGS,M_FAIL	; operation
	movlw	0x7B		;Send command 59 (expect R1-type response) to
	movwf	M_CMDN		; tell the card we want to make life hard on
	clrf	M_ADR3		; ourselves and have our CRCs checked by the
	clrf	M_ADR2		; card
	clrf	M_ADR1		; "
	movlw	0x01		; "
	movwf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	btfsc	M_FLAGS,M_FAIL	;If this command failed, something is wrong,
	return			; fail the init operation
	movf	M_CMDN,W	;If this command returned any response other
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
; M_FAIL on fail. Trashes TEMP, TEMP2, and FSR0.
MmcWriteData
	movlb	4		;Switch to the bank with the SSP registers
	clrf	CRC16H		;Start CRC16 registers at zero
	clrf	CRC16L		; "
	movlw	0xFF		;Clock a dummy byte while keeping MOSI high
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlw	0xFE		;Clock the data token into the MMC card (note
	btfsc	M_FLAGS,M_MBWR	; that CMD25, multi-block write, requires a
	movlw	0xFC		; different token than CMD24, single-block
	movwf	SSP1BUF		; write, so we differentiate based on the flag
	btfss	SSP1STAT,BF	; for ongoing multi-block write)
	bra	$-1		; "
	clrf	TEMP		;Send 256 pairs of bytes
MmcWri0	moviw	FSR0++		;Clock the next data byte into the MMC card
	movwf	SSP1BUF		; "
	xorwf	CRC16H,W	;Update the CRC while the byte is transmitting
	movwf	CRC16H		; "
	movlp	high LutCrc16H	; "
	callw			; "
	xorwf	CRC16L,W	; "
	xorwf	CRC16H,F	; "
	xorwf	CRC16H,W	; "
	xorwf	CRC16H,F	; "
	movlp	high LutCrc16L	; "
	callw			; "
	movwf	CRC16L		; "
	btfss	SSP1STAT,BF	;Wait until the byte is done transmitting
	bra	$-1		; "
	moviw	FSR0++		;Clock the next data byte into the MMC card
	movwf	SSP1BUF		; "
	xorwf	CRC16H,W	;Update the CRC while the byte is transmitting
	movwf	CRC16H		; "
	movlp	high LutCrc16H	; "
	callw			; "
	xorwf	CRC16L,W	; "
	xorwf	CRC16H,F	; "
	xorwf	CRC16H,W	; "
	xorwf	CRC16H,F	; "
	movlp	high LutCrc16L	; "
	callw			; "
	movwf	CRC16L		; "
	btfss	SSP1STAT,BF	;Wait until the byte is done transmitting
	bra	$-1		; "
	decfsz	TEMP,F		;Decrement the counter and go back to write the
	bra	MmcWri0		; next byte pair, unless we're done
	movlp	0		;Restore PCLATH from all the CRC calculating
	movf	CRC16H,W	;Clock the high CRC byte into the MMC card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	CRC16L,W	;Clock the low CRC byte into the MMC card
	movwf	SSP1BUF		; "
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
	bsf	M_FLAGS,M_FAIL	; "
	btfss	M_FLAGS,M_FAIL	;If we didn't fail, wait for the card not to be
	call	MmcWaitBusy	; busy anymore
	movlb	0		;Return to the default bank
	return

;Stop a multiblock read or write, if one is ongoing, and deassert !CS.  Sets
; M_FAIL on fail.  Trashes TEMP and TEMP2.
MmcStopOngoing
	btfsc	M_FLAGS,M_FAIL	;If there's a fail on the MMC card, return
	return			; immediately
	btfsc	M_FLAGS,M_MBWR	;If there's an ongoing write operation, make it
	bra	MmcSto0		; stop
	btfss	M_FLAGS,M_MBRD	;If there's not an ongoing read operation, this
	return			; function was called needlessly, return
	movlw	0x4C		;Send a CMD12 to stop the ongoing read, R1b
	movwf	M_CMDN		; reply type
	clrf	M_ADR3		; "
	clrf	M_ADR2		; "
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	M_FLAGS,M_CMDLR	; "
	bsf	M_FLAGS,M_CMDRB	; "
	call	MmcCmd		; "
	bcf	M_FLAGS,M_MBRD	;Clear the ongoing multiblock read flag
	bsf	PORTC,RC3	;Deassert !CS
	return
MmcSto0	movlb	4		;Switch to the bank with the SSP registers
	movlw	0xFD		;Clock the end-of-data token into the MMC card
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movlw	0xFF		;Clock a dummy byte while keeping MOSI high
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	call	MmcWaitBusy	;Wait for the card to not be busy anymore
	movlb	0		;Return to the default bank
	bcf	M_FLAGS,M_MBWR	;Clear the ongoing multiblock write flag
	bsf	PORTC,RC3	;Deassert !CS
	return

;Send the command contained in M_BUF1-6 to MMC card.  Sets M_FAIL on fail.
; Trashes TEMP.
MmcCmd
	bcf	M_FLAGS,M_FAIL	;Start out optimistic and say this didn't fail
	clrf	M_CRC		;Start the CRC7 register out at 0
	movlp	high LutCrc7	;Point PCLATH to the CRC7 lookup table
	movlb	4		;Switch to the bank with the SSP registers
	movf	M_CMDN,W	;Clock out all six MMC buffer bytes as command,
	movwf	SSP1BUF		; calculating the CRC7 along the way
	xorwf	M_CRC,W		; "
	callw			; "
	movwf	M_CRC		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR3,W	; "
	movwf	SSP1BUF		; "
	xorwf	M_CRC,W		; "
	callw			; "
	movwf	M_CRC		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR2,W	; "
	movwf	SSP1BUF		; "
	xorwf	M_CRC,W		; "
	callw			; "
	movwf	M_CRC		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR1,W	; "
	movwf	SSP1BUF		; "
	xorwf	M_CRC,W		; "
	callw			; "
	movwf	M_CRC		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR0,W	; "
	movwf	SSP1BUF		; "
	xorwf	M_CRC,W		; "
	callw			; "
	movlp	0		; "
	movwf	M_CRC		; "
	bsf	M_CRC,0		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_CRC,W		; "
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

;Waits for the card not to be busy anymore.  Sets M_FAIL on fail.  Trashes TEMP
; and TEMP2.  Expects BSR to be 4 and does not set or reset this.
MmcWaitBusy
	clrf	TEMP		;Check 65536 times to see if the card is busy
	clrf	TEMP2		; "
MmcWai0	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;Check if MISO is still low, if it's not, the
	btfss	STATUS,Z	; card is no longer busy and we can return
	return			; "
	decfsz	TEMP,F		;If it's not done, try again
	bra	MmcWai0		; "
	decfsz	TEMP2,F		; "
	bra	MmcWai0		; "
	bsf	M_FLAGS,M_FAIL	;If out of tries, fail the operation
	return


;;; Idle Handler ;;;

	org	0x7F0

SelectNext
	movlw	B'00110010'	;Tristate RD
	tris	7		; "
	bcf	PORTA,RA0	;Assert !ENBL for next device in chain
	btfss	PORTA,RA5	;Spin until our !ENBL goes high
	bra	$-1		; "
	bsf	PORTA,RA0	;Deassert !ENBL for next device in chain

WaitEnbl
	movlw	B'00110010'	;Tristate RD
	tris	7		; "
	bsf	PORTC,RC4	;Latch a 1 on RD for when we drive it
	bcf	FLAGS,DEVSEL1	;Select device 0 in the flag bits
	bcf	FLAGS,DEVSEL0	; "
	btfsc	PORTA,RA5	;Spin until !ENBL goes low
	bra	$-1		; "
	movlw	B'00100010'	;Drive the RD pin, high at first
	tris	7		; "

Dev0Jump
	decf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
Dev0Ph0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev0Jump	; off from, so drive RD high to deassert !HSHK
Dev0Ph1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev0Jump	; drive RD high because what else can we do
Dev0Ph2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev0Jump	; idling, so drive RD high to deassert !HSHK
Dev0Ph3	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev0Jump	; way to escape the idle loop
Dev0Ph4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev0Jump	; drive RD high to deassert !HSHK 
Dev0Ph5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev0Jump	; "
Dev0Ph6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev0Jump	; high
Dev0Ph7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev0Jump	; high
Dev0Ph8	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0Ph9	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0PhA	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0PhB	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0PhC	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0PhD	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0PhE	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0PhF	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev1	; "
Dev0NEn	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again

EnterDev1
	btfss	FLAGS,DEV1ON	;If there is no device 1, select the next
	goto	SelectNext	; device and wait for !ENBL to be deasserted
	bcf	FLAGS,DEVSEL1	;Select device 1 in the flag bits
	bsf	FLAGS,DEVSEL0	; "

Dev1SelJump
	decf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
Dv1SPh0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev1Jump	; off from, so drive RD high to deassert !HSHK
Dv1SPh1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev1Jump	; drive RD high because what else can we do
Dv1SPh2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev1Jump	; idling, so drive RD high to deassert !HSHK
Dv1SPh3	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev1Jump	; way to escape the idle loop
Dv1SPh4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev1Jump	; drive RD high to deassert !HSHK 
Dv1SPh5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev1Jump	; "
Dv1SPh6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev1Jump	; high
Dv1SPh7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev1Jump	; high
Dv1SPh8	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev1SelJump	; off from, so drive RD high to deassert !HSHK
Dv1SPh9	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev1SelJump	; drive RD high because what else can we do
Dv1SPhA	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev1SelJump	; idling, so drive RD high to deassert !HSHK
Dv1SPhB	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev1SelJump	; way to escape the idle loop
Dv1SPhC	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev1SelJump	; drive RD high to deassert !HSHK 
Dv1SPhD	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev1SelJump	; "
Dv1SPhE	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev1SelJump	; high
Dv1SPhF	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev1SelJump	; high
Dv1SNEn	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again

Dev1Jump
	decf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
Dev1Ph0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev1Jump	; off from, so drive RD high to deassert !HSHK
Dev1Ph1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev1Jump	; drive RD high because what else can we do
Dev1Ph2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev1Jump	; idling, so drive RD high to deassert !HSHK
Dev1Ph3	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev1Jump	; way to escape the idle loop
Dev1Ph4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev1Jump	; drive RD high to deassert !HSHK 
Dev1Ph5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev1Jump	; "
Dev1Ph6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev1Jump	; high
Dev1Ph7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev1Jump	; high
Dev1Ph8	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1Ph9	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1PhA	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1PhB	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1PhC	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1PhD	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1PhE	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1PhF	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev2	; "
Dev1NEn	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again

EnterDev2
	btfss	FLAGS,DEV2ON	;If there is no device 2, select the next
	goto	SelectNext	; device and wait for !ENBL to be deasserted
	bsf	FLAGS,DEVSEL1	;Select device 2 in the flag bits
	bcf	FLAGS,DEVSEL0	; "

Dev2SelJump
	decf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
Dv2SPh0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev2Jump	; off from, so drive RD high to deassert !HSHK
Dv2SPh1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev2Jump	; drive RD high because what else can we do
Dv2SPh2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev2Jump	; idling, so drive RD high to deassert !HSHK
Dv2SPh3	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev2Jump	; way to escape the idle loop
Dv2SPh4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev2Jump	; drive RD high to deassert !HSHK 
Dv2SPh5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev2Jump	; "
Dv2SPh6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev2Jump	; high
Dv2SPh7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev2Jump	; high
Dv2SPh8	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev2SelJump	; off from, so drive RD high to deassert !HSHK
Dv2SPh9	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev2SelJump	; drive RD high because what else can we do
Dv2SPhA	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev2SelJump	; idling, so drive RD high to deassert !HSHK
Dv2SPhB	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev2SelJump	; way to escape the idle loop
Dv2SPhC	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev2SelJump	; drive RD high to deassert !HSHK 
Dv2SPhD	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev2SelJump	; "
Dv2SPhE	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev2SelJump	; high
Dv2SPhF	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev2SelJump	; high
Dv2SNEn	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again

Dev2Jump
	decf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
Dev2Ph0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev2Jump	; off from, so drive RD high to deassert !HSHK
Dev2Ph1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev2Jump	; drive RD high because what else can we do
Dev2Ph2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev2Jump	; idling, so drive RD high to deassert !HSHK
Dev2Ph3	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev2Jump	; way to escape the idle loop
Dev2Ph4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev2Jump	; drive RD high to deassert !HSHK 
Dev2Ph5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev2Jump	; "
Dev2Ph6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev2Jump	; high
Dev2Ph7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev2Jump	; high
Dev2Ph8	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2Ph9	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2PhA	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2PhB	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2PhC	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2PhD	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2PhE	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2PhF	bsf	PORTC,RC4	;Mac wants to talk to next device
	bra	EnterDev3	; "
Dev2NEn	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again

EnterDev3
	btfss	FLAGS,DEV3ON	;If there is no device 3, select the next
	goto	SelectNext	; device and wait for !ENBL to be deasserted
	bsf	FLAGS,DEVSEL1	;Select device 3 in the flag bits
	bsf	FLAGS,DEVSEL0	; "

Dev3SelJump
	decf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
Dv3SPh0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev3Jump	; off from, so drive RD high to deassert !HSHK
Dv3SPh1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev3Jump	; drive RD high because what else can we do
Dv3SPh2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev3Jump	; idling, so drive RD high to deassert !HSHK
Dv3SPh3	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev3Jump	; way to escape the idle loop
Dv3SPh4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev3Jump	; drive RD high to deassert !HSHK 
Dv3SPh5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev3Jump	; "
Dv3SPh6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev3Jump	; high
Dv3SPh7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev3Jump	; high
Dv3SPh8	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev3SelJump	; off from, so drive RD high to deassert !HSHK
Dv3SPh9	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev3SelJump	; drive RD high because what else can we do
Dv3SPhA	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev3SelJump	; idling, so drive RD high to deassert !HSHK
Dv3SPhB	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev3SelJump	; way to escape the idle loop
Dv3SPhC	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev3SelJump	; drive RD high to deassert !HSHK 
Dv3SPhD	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev3SelJump	; "
Dv3SPhE	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev3SelJump	; high
Dv3SPhF	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev3SelJump	; high
Dv3SNEn	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again

Dev3Jump
	decf	PORTA,W		;32 entries in the jump table, each has two
	brw			; instructions; jump to the one matching state
Dev3Ph0	bsf	PORTC,RC4	;HOFF is asserted but there's nothing to hold
	bra	Dev3Jump	; off from, so drive RD high to deassert !HSHK
Dev3Ph1	bsf	PORTC,RC4	;Mac wants to exchange data for some reason, so
	bra	Dev3Jump	; drive RD high because what else can we do
Dev3Ph2	bsf	PORTC,RC4	;Mac wants us to idle, cool, we're already
	bra	Dev3Jump	; idling, so drive RD high to deassert !HSHK
Dev3Ph3	call	GetCommand	;Mac wants to send a command, this is the only
	bra	Dev3Jump	; way to escape the idle loop
Dev3Ph4	bsf	PORTC,RC4	;Mac wants us to reset, this is sorta reset, so
	bra	Dev3Jump	; drive RD high to deassert !HSHK 
Dev3Ph5	bcf	PORTC,RC4	;Mac wants us to drive RD low, so drive RD low
	bra	Dev3Jump	; "
Dev3Ph6	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev3Jump	; high
Dev3Ph7	bsf	PORTC,RC4	;Mac wants us to drive RD high, so drive RD
	bra	Dev3Jump	; high
Dev3Ph8	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3Ph9	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3PhA	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3PhB	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3PhC	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3PhD	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3PhE	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3PhF	bsf	PORTC,RC4	;Mac wants to talk to next device, we have none
	goto	SelectNext	; so select the next and await !ENBL deasserted
Dev3NEn	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again
	goto	WaitEnbl	;!ENBL has been deasserted, so get off the bus
	nop			; and wait for it to be asserted again


;;; Transmitter Code ;;;

	org	0x1000

;Transmits GROUPS 7-byte groups of non-encoded data starting at FSR0.  Sets
; ABORTED if a phase change caused the command to be aborted, in which case
; caller should return to idle state as soon as possible.  Trashes FSR0, TEMP,
; TEMP2, and GROUPS.
Transmit
	movlp	high Transmit	;Point PCLATH to this page so gotos work
	movlb	0		;Point BSR to 0 so we can access PORTA/PORTC
	bcf	FLAGS,ABORTED	;Clear aborted flag to start
	decf	PORTA,W		;Check the current state
	xorlw	B'00000100'	;If it's 2, then mac is ready for us to request
	btfss	STATUS,Z	; to send; if it's anything else, signal an
	goto	XAbort		; abort to get us to the idle jump table ASAP
	bcf	PORTC,RC4	;Assert !HSHK to request to send
Transm0	decf	PORTA,W		;Check the current state
	xorlw	B'00000100'	;If we're still in state 2, keep checking
	btfsc	STATUS,Z	; "
	bra	Transm0		; "
	xorlw	B'00000010'	;If we're in state 3, most likely mac is about
	btfsc	STATUS,Z	; to change to state 1, so keep checking
	bra	Transm0		; "
	xorlw	B'00000100'	;If we're in state 1, get ready to send; if any
	btfss	STATUS,Z	; other state, we can't handle it, so signal an
	goto	XAbort		; abort to get us to the idle jump table ASAP
	;fall through

Transmitter
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
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	DNOP			;-6 -5
	nop			;-4
	decfsz	TEMP2,F		;-3 If we haven't yet transmitted the last of
	bra	XSyncLp		;-2(-1)  the autosync groups, loop for another
	nop			;-1
XSync	bcf	PORTC,RC4	; 0 Start of 0xAA sync byte
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	nop			; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the byte; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the byte; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
XDataLp	bcf	PORTC,RC4	; 0 Start 7-to-8-encoded group; MSB always set
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,7		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,6		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,5		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,4		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,3		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,2		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,1		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	DNOP			;-2 -1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	6[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	5[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	4[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	3[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	2[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	1[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	decf	PORTA,W		;+6 Check the current state
	andlw	B'00111100'	;+7 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-8  continue transmitting the group; else,
	goto	XAbort		;-7  abort and get us back to idle loop ASAP
	decf	PORTA,W		;-6 Check the current state
	andlw	B'00111100'	;-5 If it is 0 (holdoff) or 1 (communication),
	btfss	STATUS,Z	;-4  continue transmitting the group; else,
	goto	XAbort		;-3  abort and get us back to idle loop ASAP
	moviw	0[FSR0]		;-2
	btfsc	WREG,0		;-1
	bcf	PORTC,RC4	; 0
	movlw	87		;+1
	addwf	TEMP,F		;+2
	btfsc	STATUS,C	;+3
	bra	$+1		;+4
	bsf	PORTC,RC4	;+5
	movlw	7		;+6 Increment the pointer by 7 to transmit
	addwf	FSR0L,F		;+7  the next group
	movlw	0		;-8  "
	addwfc	FSR0H,F		;-7  "
	decf	PORTA,W		;-6 Check the current state
	btfsc	STATUS,Z	;-5 If it's 0 (holdoff), then effect a suspend
	bra	XSuspend	;-4  now that we're done transmitting a group
	decfsz	GROUPS,F	;-3 We finished a group, decrement group count;
	goto	XDataLp		;-2(-1)  if there are more, loop again
	;fall through

XFinished
	decf	PORTA,W		;Check the current state
	xorlw	B'00000010'	;If it's still 1 (communication), mac hasn't
	btfsc	STATUS,Z	; realized we're done transmitting yet, so loop
	bra	XFinished	; "
	xorlw	B'00000100'	;If it's 3 (request send), mac is probably
	btfsc	STATUS,Z	; transitioning through on its way to 2, so
	bra	XFinished	; loop
	xorlw	B'00000010'	;If it's anything but 2 (idle), set the flag
	btfss	STATUS,Z	; for an aborted command to let the caller know
	bsf	FLAGS,ABORTED	; that something weird's going on
	movlp	0		;Reset PCLATH to where we're returning
	return

XSuspend
	decf	GROUPS,F	;We finished a group, so decrement group count
XSuspe0	decf	PORTA,W		;Check the current state
	btfsc	STATUS,Z	;If it's 0 (holdoff), mac is still asking us to
	bra	XSuspe0		; hold off, so loop again
	xorlw	B'00000010'	;If it's anything else but 1 (communication),
	btfss	STATUS,Z	; either mac wants to abort the command or
	bra	XAbort		; something weird's going on, so abort
	movf	GROUPS,F	;If mac asked us to suspend during the last
	btfsc	STATUS,Z	; group, we resume by being finished and
	bra	XFinished	; waiting to go to state 2 (idle)
	goto	XSync		;Resume transmission after interrupted group

XAbort
	bsf	PORTC,RC4	;Deassert !HSHK if it was asserted
	bsf	FLAGS,ABORTED	;Raise the aborted flag
	movlp	0		;Reset PCLATH to where we're returning
	return


;;; Receiver Code ;;;

	org	0x1800

RecvAbort
	bsf	PORTC,RC4	;Deassert !HSHK if it was asserted
	bsf	FLAGS,ABORTED	;Raise the aborted flag
	movlp	0		;Reset PCLATH to where we're returning
	return

;Receives data from the mac.  Sets ABORTED if a phase change caused the command
; to be aborted, in which case caller should return to idle state as soon as
; possible.  Overwrites linear memory with received data in the following way:
; first byte is sync byte, second byte is 0x80 + number of groups in command
; according to mac, third byte is 0x80 + number of groups in anticipated
; response, followed by a string of GROUPS seven-byte decoded groups.  Trashes
; FSR0, FSR1, TEMP, and TEMP2.
Receive
	movlb	0		;Point BSR to 0 so we can access PORTA/PORTC
	bcf	FLAGS,ABORTED	;Clear aborted flag to start
	movlw	0x20		;Reset both pointers to the top of linear
	movwf	FSR0H		; memory
	movwf	FSR1H		; "
	clrf	FSR0L		; "
	clrf	FSR1L		; "
	clrf	GROUPS		;Clear counter of total received groups
Receiv0	decf	PORTA,W		;Check the current state
	xorlw	B'00000100'	;If it's 2, then mac hasn't said it's ready to
	btfsc	STATUS,Z	; send yet, keep waiting
	bra	Receiv0		; "
	xorlw	B'00000010'	;If it's 3, mac has said it's ready to send,
	btfss	STATUS,Z	; if it's anything else, we can't handle it
	bra	RecvAbort	; here so return to the idle jump table
	bcf	PORTC,RC4	;Assert !HSHK to say we're ready to receive
Receiv1	decf	PORTA,W		;If we're still in state 3, keep checking
	xorlw	B'00000110'	; "
	btfsc	STATUS,Z	; "
	bra	Receiv1		; "
	xorlw	B'00000100'	;If we're in state 1, get ready to receive; if
	btfss	STATUS,Z	; we're in any other state, we can't handle it
	bra	RecvAbort	; here so return to the idle jump table
	movlb	7		;Clear any IOC flags that have been set up to
	clrf	IOCAF		; now
	;fall through

Receiver
	movlp	high Receiver	;Point PCLATH to this page so gotos work
	movlb	0		;Point BSR to 0 so we can access PORTC
	bsf	INTCON,GIE	;Enable interrupts so we have a way to exit
RcSt0	btfss	PORTC,RC5	;Signal starts at zero, wait until transition
	bra	$-1		; to one, this is MSB (always 1) of first byte
	movlw	B'10000000'	;003 cycles, 0.18 bit times
	movwf	INDF0		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo0_6		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo0_6		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo0_6		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo0_6		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo0_6		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo0_6		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo0_6		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo0_6		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo0_5		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo0_5		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo0_5		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo0_5		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo0_5		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo0_5		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo0_5		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo0_5		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo0_4		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo0_4		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo0_4		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo0_4		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo0_4		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo0_4		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo0_4		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo0_4		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo0_4		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo0_3		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo0_3		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo0_3		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo0_3		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo0_3		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo0_3		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo0_3		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo0_3		;074 cycles, 4.53 bit times
	btfss	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	RcTo0_2		;076 cycles, 4.65 bit times
	btfss	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	RcTo0_2		;078 cycles, 4.77 bit times
	btfss	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	RcTo0_2		;080 cycles, 4.90 bit times
	btfss	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	RcTo0_2		;082 cycles, 5.02 bit times
	btfss	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	RcTo0_2		;084 cycles, 5.14 bit times
	btfss	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	RcTo0_2		;086 cycles, 5.26 bit times
	btfss	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	RcTo0_2		;088 cycles, 5.39 bit times
	btfss	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	RcTo0_2		;090 cycles, 5.51 bit times
	btfss	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	RcTo0_1		;092 cycles, 5.63 bit times
	btfss	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	RcTo0_1		;094 cycles, 5.75 bit times
	btfss	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	RcTo0_1		;096 cycles, 5.88 bit times
	btfss	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	RcTo0_1		;098 cycles, 6.00 bit times
	btfss	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	RcTo0_1		;100 cycles, 6.12 bit times
	btfss	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	RcTo0_1		;102 cycles, 6.24 bit times
	btfss	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	RcTo0_1		;104 cycles, 6.36 bit times
	btfss	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	RcTo0_1		;106 cycles, 6.49 bit times
	btfss	PORTC,RC5	;107 cycles, 6.55 bit times
	goto	RcTo0_0		;108 cycles, 6.61 bit times
	btfss	PORTC,RC5	;109 cycles, 6.67 bit times
	goto	RcTo0_0		;110 cycles, 6.73 bit times
	btfss	PORTC,RC5	;111 cycles, 6.79 bit times
	goto	RcTo0_0		;112 cycles, 6.85 bit times
	btfss	PORTC,RC5	;113 cycles, 6.92 bit times
	goto	RcTo0_0		;114 cycles, 6.98 bit times
	btfss	PORTC,RC5	;115 cycles, 7.04 bit times
	goto	RcTo0_0		;116 cycles, 7.10 bit times
	btfss	PORTC,RC5	;117 cycles, 7.16 bit times
	goto	RcTo0_0		;118 cycles, 7.22 bit times
	btfss	PORTC,RC5	;119 cycles, 7.28 bit times
	goto	RcTo0_0		;120 cycles, 7.34 bit times
	btfss	PORTC,RC5	;121 cycles, 7.40 bit times
	goto	RcTo0_0		;122 cycles, 7.47 bit times
	btfss	PORTA,RA1	;123 cycles, 7.53 bit times
	bcf	INDF0,7		;124 cycles, 7.59 bit times
	moviw	FSR0++		;125 cycles, 7.65 bit times
	goto	RcSt1		;126 cycles, 7.71 bit times

RcTo1_6	movlw	B'01000000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo0_5		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo0_5		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo0_5		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo0_5		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo0_5		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo0_5		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo0_5		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo0_5		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo0_4		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo0_4		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo0_4		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo0_4		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo0_4		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo0_4		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo0_4		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo0_4		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo0_3		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo0_3		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo0_3		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo0_3		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo0_3		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo0_3		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo0_3		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo0_3		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo0_3		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo0_2		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo0_2		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo0_2		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo0_2		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo0_2		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo0_2		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo0_2		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo0_2		;074 cycles, 4.53 bit times
	btfss	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	RcTo0_1		;076 cycles, 4.65 bit times
	btfss	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	RcTo0_1		;078 cycles, 4.77 bit times
	btfss	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	RcTo0_1		;080 cycles, 4.90 bit times
	btfss	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	RcTo0_1		;082 cycles, 5.02 bit times
	btfss	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	RcTo0_1		;084 cycles, 5.14 bit times
	btfss	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	RcTo0_1		;086 cycles, 5.26 bit times
	btfss	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	RcTo0_1		;088 cycles, 5.39 bit times
	btfss	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	RcTo0_1		;090 cycles, 5.51 bit times
	btfss	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	RcTo0_0		;092 cycles, 5.63 bit times
	btfss	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	RcTo0_0		;094 cycles, 5.75 bit times
	btfss	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	RcTo0_0		;096 cycles, 5.88 bit times
	btfss	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	RcTo0_0		;098 cycles, 6.00 bit times
	btfss	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	RcTo0_0		;100 cycles, 6.12 bit times
	btfss	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	RcTo0_0		;102 cycles, 6.24 bit times
	btfss	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	RcTo0_0		;104 cycles, 6.36 bit times
	btfss	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	RcTo0_0		;106 cycles, 6.49 bit times
	btfss	PORTA,RA1	;107 cycles, 6.55 bit times
	bcf	INDF0,7		;108 cycles, 6.61 bit times
	moviw	FSR0++		;109 cycles, 6.67 bit times
	goto	RcSt1		;110 cycles, 6.73 bit times

RcTo1_5	movlw	B'00100000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo0_4		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo0_4		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo0_4		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo0_4		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo0_4		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo0_4		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo0_4		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo0_4		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo0_3		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo0_3		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo0_3		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo0_3		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo0_3		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo0_3		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo0_3		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo0_3		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo0_2		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo0_2		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo0_2		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo0_2		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo0_2		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo0_2		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo0_2		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo0_2		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo0_2		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo0_1		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo0_1		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo0_1		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo0_1		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo0_1		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo0_1		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo0_1		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo0_1		;074 cycles, 4.53 bit times
	btfss	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	RcTo0_0		;076 cycles, 4.65 bit times
	btfss	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	RcTo0_0		;078 cycles, 4.77 bit times
	btfss	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	RcTo0_0		;080 cycles, 4.90 bit times
	btfss	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	RcTo0_0		;082 cycles, 5.02 bit times
	btfss	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	RcTo0_0		;084 cycles, 5.14 bit times
	btfss	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	RcTo0_0		;086 cycles, 5.26 bit times
	btfss	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	RcTo0_0		;088 cycles, 5.39 bit times
	btfss	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	RcTo0_0		;090 cycles, 5.51 bit times
	btfss	PORTA,RA1	;091 cycles, 5.57 bit times
	bcf	INDF0,7		;092 cycles, 5.63 bit times
	moviw	FSR0++		;093 cycles, 5.69 bit times
	goto	RcSt1		;094 cycles, 5.75 bit times

RcTo1_4	movlw	B'00010000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo0_3		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo0_3		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo0_3		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo0_3		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo0_3		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo0_3		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo0_3		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo0_3		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo0_2		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo0_2		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo0_2		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo0_2		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo0_2		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo0_2		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo0_2		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo0_2		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo0_1		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo0_1		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo0_1		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo0_1		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo0_1		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo0_1		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo0_1		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo0_1		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo0_1		;058 cycles, 3.55 bit times
	btfss	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo0_0		;060 cycles, 3.67 bit times
	btfss	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo0_0		;062 cycles, 3.79 bit times
	btfss	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo0_0		;064 cycles, 3.92 bit times
	btfss	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo0_0		;066 cycles, 4.04 bit times
	btfss	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo0_0		;068 cycles, 4.16 bit times
	btfss	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo0_0		;070 cycles, 4.28 bit times
	btfss	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo0_0		;072 cycles, 4.41 bit times
	btfss	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo0_0		;074 cycles, 4.53 bit times
	btfss	PORTA,RA1	;075 cycles, 4.59 bit times
	bcf	INDF0,7		;076 cycles, 4.65 bit times
	moviw	FSR0++		;077 cycles, 4.71 bit times
	goto	RcSt1		;078 cycles, 4.77 bit times

RcTo1_3	movlw	B'00001000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo0_2		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo0_2		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo0_2		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo0_2		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo0_2		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo0_2		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo0_2		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo0_2		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo0_1		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo0_1		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo0_1		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo0_1		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo0_1		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo0_1		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo0_1		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo0_1		;040 cycles, 2.45 bit times
	btfss	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo0_0		;042 cycles, 2.57 bit times
	btfss	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo0_0		;044 cycles, 2.69 bit times
	btfss	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo0_0		;046 cycles, 2.82 bit times
	btfss	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo0_0		;048 cycles, 2.94 bit times
	btfss	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo0_0		;050 cycles, 3.06 bit times
	btfss	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo0_0		;052 cycles, 3.18 bit times
	btfss	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo0_0		;054 cycles, 3.30 bit times
	btfss	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo0_0		;056 cycles, 3.43 bit times
	btfss	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo0_0		;058 cycles, 3.55 bit times
	btfss	PORTA,RA1	;059 cycles, 3.61 bit times
	bcf	INDF0,7		;060 cycles, 3.67 bit times
	moviw	FSR0++		;061 cycles, 3.73 bit times
	goto	RcSt1		;062 cycles, 3.79 bit times

RcTo1_2	movlw	B'00000100'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo0_1		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo0_1		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo0_1		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo0_1		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo0_1		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo0_1		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo0_1		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo0_1		;024 cycles, 1.47 bit times
	btfss	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo0_0		;026 cycles, 1.59 bit times
	btfss	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo0_0		;028 cycles, 1.71 bit times
	btfss	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo0_0		;030 cycles, 1.84 bit times
	btfss	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo0_0		;032 cycles, 1.96 bit times
	btfss	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo0_0		;034 cycles, 2.08 bit times
	btfss	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo0_0		;036 cycles, 2.20 bit times
	btfss	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo0_0		;038 cycles, 2.33 bit times
	btfss	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo0_0		;040 cycles, 2.45 bit times
	btfss	PORTA,RA1	;041 cycles, 2.51 bit times
	bcf	INDF0,7		;042 cycles, 2.57 bit times
	moviw	FSR0++		;043 cycles, 2.63 bit times
	goto	RcSt1		;044 cycles, 2.69 bit times

RcTo1_1	movlw	B'00000010'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfss	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo0_0		;010 cycles, 0.61 bit times
	btfss	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo0_0		;012 cycles, 0.73 bit times
	btfss	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo0_0		;014 cycles, 0.86 bit times
	btfss	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo0_0		;016 cycles, 0.98 bit times
	btfss	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo0_0		;018 cycles, 1.10 bit times
	btfss	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo0_0		;020 cycles, 1.22 bit times
	btfss	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo0_0		;022 cycles, 1.35 bit times
	btfss	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo0_0		;024 cycles, 1.47 bit times
	btfss	PORTA,RA1	;025 cycles, 1.53 bit times
	bcf	INDF0,7		;026 cycles, 1.59 bit times
	moviw	FSR0++		;027 cycles, 1.65 bit times
	goto	RcSt1		;028 cycles, 1.71 bit times

RcTo1_0	movlw	B'00000001'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	moviw	FSR0++		;007 cycles, 0.43 bit times
	;fall through
	
RcSt1	btfsc	PORTC,RC5	;Signal starts at one, wait until transition
	bra	$-1		; to zero, this is MSB (always 1) of first byte
	movlw	B'10000000'	;003 cycles, 0.18 bit times
	movwf	INDF0		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo1_6		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo1_6		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo1_6		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo1_6		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo1_6		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo1_6		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo1_6		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo1_6		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo1_5		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo1_5		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo1_5		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo1_5		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo1_5		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo1_5		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo1_5		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo1_5		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo1_4		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo1_4		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo1_4		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo1_4		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo1_4		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo1_4		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo1_4		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo1_4		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo1_4		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo1_3		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo1_3		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo1_3		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo1_3		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo1_3		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo1_3		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo1_3		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo1_3		;074 cycles, 4.53 bit times
	btfsc	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	RcTo1_2		;076 cycles, 4.65 bit times
	btfsc	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	RcTo1_2		;078 cycles, 4.77 bit times
	btfsc	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	RcTo1_2		;080 cycles, 4.90 bit times
	btfsc	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	RcTo1_2		;082 cycles, 5.02 bit times
	btfsc	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	RcTo1_2		;084 cycles, 5.14 bit times
	btfsc	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	RcTo1_2		;086 cycles, 5.26 bit times
	btfsc	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	RcTo1_2		;088 cycles, 5.39 bit times
	btfsc	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	RcTo1_2		;090 cycles, 5.51 bit times
	btfsc	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	RcTo1_1		;092 cycles, 5.63 bit times
	btfsc	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	RcTo1_1		;094 cycles, 5.75 bit times
	btfsc	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	RcTo1_1		;096 cycles, 5.88 bit times
	btfsc	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	RcTo1_1		;098 cycles, 6.00 bit times
	btfsc	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	RcTo1_1		;100 cycles, 6.12 bit times
	btfsc	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	RcTo1_1		;102 cycles, 6.24 bit times
	btfsc	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	RcTo1_1		;104 cycles, 6.36 bit times
	btfsc	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	RcTo1_1		;106 cycles, 6.49 bit times
	btfsc	PORTC,RC5	;107 cycles, 6.55 bit times
	goto	RcTo1_0		;108 cycles, 6.61 bit times
	btfsc	PORTC,RC5	;109 cycles, 6.67 bit times
	goto	RcTo1_0		;110 cycles, 6.73 bit times
	btfsc	PORTC,RC5	;111 cycles, 6.79 bit times
	goto	RcTo1_0		;112 cycles, 6.85 bit times
	btfsc	PORTC,RC5	;113 cycles, 6.92 bit times
	goto	RcTo1_0		;114 cycles, 6.98 bit times
	btfsc	PORTC,RC5	;115 cycles, 7.04 bit times
	goto	RcTo1_0		;116 cycles, 7.10 bit times
	btfsc	PORTC,RC5	;117 cycles, 7.16 bit times
	goto	RcTo1_0		;118 cycles, 7.22 bit times
	btfsc	PORTC,RC5	;119 cycles, 7.28 bit times
	goto	RcTo1_0		;120 cycles, 7.34 bit times
	btfsc	PORTC,RC5	;121 cycles, 7.40 bit times
	goto	RcTo1_0		;122 cycles, 7.47 bit times
	btfss	PORTA,RA1	;123 cycles, 7.53 bit times
	bcf	INDF0,7		;124 cycles, 7.59 bit times
	moviw	FSR0++		;125 cycles, 7.65 bit times
	goto	RcSt0		;126 cycles, 7.71 bit times

RcTo0_6	movlw	B'01000000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo1_5		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo1_5		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo1_5		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo1_5		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo1_5		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo1_5		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo1_5		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo1_5		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo1_4		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo1_4		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo1_4		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo1_4		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo1_4		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo1_4		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo1_4		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo1_4		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo1_3		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo1_3		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo1_3		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo1_3		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo1_3		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo1_3		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo1_3		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo1_3		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo1_3		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo1_2		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo1_2		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo1_2		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo1_2		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo1_2		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo1_2		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo1_2		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo1_2		;074 cycles, 4.53 bit times
	btfsc	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	RcTo1_1		;076 cycles, 4.65 bit times
	btfsc	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	RcTo1_1		;078 cycles, 4.77 bit times
	btfsc	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	RcTo1_1		;080 cycles, 4.90 bit times
	btfsc	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	RcTo1_1		;082 cycles, 5.02 bit times
	btfsc	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	RcTo1_1		;084 cycles, 5.14 bit times
	btfsc	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	RcTo1_1		;086 cycles, 5.26 bit times
	btfsc	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	RcTo1_1		;088 cycles, 5.39 bit times
	btfsc	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	RcTo1_1		;090 cycles, 5.51 bit times
	btfsc	PORTC,RC5	;091 cycles, 5.57 bit times
	goto	RcTo1_0		;092 cycles, 5.63 bit times
	btfsc	PORTC,RC5	;093 cycles, 5.69 bit times
	goto	RcTo1_0		;094 cycles, 5.75 bit times
	btfsc	PORTC,RC5	;095 cycles, 5.81 bit times
	goto	RcTo1_0		;096 cycles, 5.88 bit times
	btfsc	PORTC,RC5	;097 cycles, 5.94 bit times
	goto	RcTo1_0		;098 cycles, 6.00 bit times
	btfsc	PORTC,RC5	;099 cycles, 6.06 bit times
	goto	RcTo1_0		;100 cycles, 6.12 bit times
	btfsc	PORTC,RC5	;101 cycles, 6.18 bit times
	goto	RcTo1_0		;102 cycles, 6.24 bit times
	btfsc	PORTC,RC5	;103 cycles, 6.30 bit times
	goto	RcTo1_0		;104 cycles, 6.36 bit times
	btfsc	PORTC,RC5	;105 cycles, 6.43 bit times
	goto	RcTo1_0		;106 cycles, 6.49 bit times
	btfss	PORTA,RA1	;107 cycles, 6.55 bit times
	bcf	INDF0,7		;108 cycles, 6.61 bit times
	moviw	FSR0++		;109 cycles, 6.67 bit times
	goto	RcSt0		;110 cycles, 6.73 bit times

RcTo0_5	movlw	B'00100000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo1_4		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo1_4		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo1_4		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo1_4		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo1_4		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo1_4		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo1_4		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo1_4		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo1_3		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo1_3		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo1_3		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo1_3		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo1_3		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo1_3		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo1_3		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo1_3		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo1_2		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo1_2		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo1_2		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo1_2		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo1_2		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo1_2		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo1_2		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo1_2		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo1_2		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo1_1		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo1_1		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo1_1		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo1_1		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo1_1		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo1_1		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo1_1		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo1_1		;074 cycles, 4.53 bit times
	btfsc	PORTC,RC5	;075 cycles, 4.59 bit times
	goto	RcTo1_0		;076 cycles, 4.65 bit times
	btfsc	PORTC,RC5	;077 cycles, 4.71 bit times
	goto	RcTo1_0		;078 cycles, 4.77 bit times
	btfsc	PORTC,RC5	;079 cycles, 4.83 bit times
	goto	RcTo1_0		;080 cycles, 4.90 bit times
	btfsc	PORTC,RC5	;081 cycles, 4.96 bit times
	goto	RcTo1_0		;082 cycles, 5.02 bit times
	btfsc	PORTC,RC5	;083 cycles, 5.08 bit times
	goto	RcTo1_0		;084 cycles, 5.14 bit times
	btfsc	PORTC,RC5	;085 cycles, 5.20 bit times
	goto	RcTo1_0		;086 cycles, 5.26 bit times
	btfsc	PORTC,RC5	;087 cycles, 5.32 bit times
	goto	RcTo1_0		;088 cycles, 5.39 bit times
	btfsc	PORTC,RC5	;089 cycles, 5.45 bit times
	goto	RcTo1_0		;090 cycles, 5.51 bit times
	btfss	PORTA,RA1	;091 cycles, 5.57 bit times
	bcf	INDF0,7		;092 cycles, 5.63 bit times
	moviw	FSR0++		;093 cycles, 5.69 bit times
	goto	RcSt0		;094 cycles, 5.75 bit times

RcTo0_4	movlw	B'00010000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo1_3		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo1_3		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo1_3		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo1_3		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo1_3		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo1_3		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo1_3		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo1_3		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo1_2		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo1_2		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo1_2		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo1_2		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo1_2		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo1_2		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo1_2		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo1_2		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo1_1		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo1_1		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo1_1		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo1_1		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo1_1		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo1_1		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo1_1		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo1_1		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo1_1		;058 cycles, 3.55 bit times
	btfsc	PORTC,RC5	;059 cycles, 3.61 bit times
	goto	RcTo1_0		;060 cycles, 3.67 bit times
	btfsc	PORTC,RC5	;061 cycles, 3.73 bit times
	goto	RcTo1_0		;062 cycles, 3.79 bit times
	btfsc	PORTC,RC5	;063 cycles, 3.86 bit times
	goto	RcTo1_0		;064 cycles, 3.92 bit times
	btfsc	PORTC,RC5	;065 cycles, 3.98 bit times
	goto	RcTo1_0		;066 cycles, 4.04 bit times
	btfsc	PORTC,RC5	;067 cycles, 4.10 bit times
	goto	RcTo1_0		;068 cycles, 4.16 bit times
	btfsc	PORTC,RC5	;069 cycles, 4.22 bit times
	goto	RcTo1_0		;070 cycles, 4.28 bit times
	btfsc	PORTC,RC5	;071 cycles, 4.35 bit times
	goto	RcTo1_0		;072 cycles, 4.41 bit times
	btfsc	PORTC,RC5	;073 cycles, 4.47 bit times
	goto	RcTo1_0		;074 cycles, 4.53 bit times
	btfss	PORTA,RA1	;075 cycles, 4.59 bit times
	bcf	INDF0,7		;076 cycles, 4.65 bit times
	moviw	FSR0++		;077 cycles, 4.71 bit times
	goto	RcSt0		;078 cycles, 4.77 bit times

RcTo0_3	movlw	B'00001000'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo1_2		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo1_2		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo1_2		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo1_2		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo1_2		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo1_2		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo1_2		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo1_2		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo1_1		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo1_1		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo1_1		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo1_1		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo1_1		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo1_1		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo1_1		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo1_1		;040 cycles, 2.45 bit times
	btfsc	PORTC,RC5	;041 cycles, 2.51 bit times
	goto	RcTo1_0		;042 cycles, 2.57 bit times
	btfsc	PORTC,RC5	;043 cycles, 2.63 bit times
	goto	RcTo1_0		;044 cycles, 2.69 bit times
	btfsc	PORTC,RC5	;045 cycles, 2.75 bit times
	goto	RcTo1_0		;046 cycles, 2.82 bit times
	btfsc	PORTC,RC5	;047 cycles, 2.88 bit times
	goto	RcTo1_0		;048 cycles, 2.94 bit times
	btfsc	PORTC,RC5	;049 cycles, 3.00 bit times
	goto	RcTo1_0		;050 cycles, 3.06 bit times
	btfsc	PORTC,RC5	;051 cycles, 3.12 bit times
	goto	RcTo1_0		;052 cycles, 3.18 bit times
	btfsc	PORTC,RC5	;053 cycles, 3.24 bit times
	goto	RcTo1_0		;054 cycles, 3.30 bit times
	btfsc	PORTC,RC5	;055 cycles, 3.37 bit times
	goto	RcTo1_0		;056 cycles, 3.43 bit times
	btfsc	PORTC,RC5	;057 cycles, 3.49 bit times
	goto	RcTo1_0		;058 cycles, 3.55 bit times
	btfss	PORTA,RA1	;059 cycles, 3.61 bit times
	bcf	INDF0,7		;060 cycles, 3.67 bit times
	moviw	FSR0++		;061 cycles, 3.73 bit times
	goto	RcSt0		;062 cycles, 3.79 bit times

RcTo0_2	movlw	B'00000100'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo1_1		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo1_1		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo1_1		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo1_1		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo1_1		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo1_1		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo1_1		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo1_1		;024 cycles, 1.47 bit times
	btfsc	PORTC,RC5	;025 cycles, 1.53 bit times
	goto	RcTo1_0		;026 cycles, 1.59 bit times
	btfsc	PORTC,RC5	;027 cycles, 1.65 bit times
	goto	RcTo1_0		;028 cycles, 1.71 bit times
	btfsc	PORTC,RC5	;029 cycles, 1.77 bit times
	goto	RcTo1_0		;030 cycles, 1.84 bit times
	btfsc	PORTC,RC5	;031 cycles, 1.90 bit times
	goto	RcTo1_0		;032 cycles, 1.96 bit times
	btfsc	PORTC,RC5	;033 cycles, 2.02 bit times
	goto	RcTo1_0		;034 cycles, 2.08 bit times
	btfsc	PORTC,RC5	;035 cycles, 2.14 bit times
	goto	RcTo1_0		;036 cycles, 2.20 bit times
	btfsc	PORTC,RC5	;037 cycles, 2.26 bit times
	goto	RcTo1_0		;038 cycles, 2.33 bit times
	btfsc	PORTC,RC5	;039 cycles, 2.39 bit times
	goto	RcTo1_0		;040 cycles, 2.45 bit times
	btfss	PORTA,RA1	;041 cycles, 2.51 bit times
	bcf	INDF0,7		;042 cycles, 2.57 bit times
	moviw	FSR0++		;043 cycles, 2.63 bit times
	goto	RcSt0		;044 cycles, 2.69 bit times

RcTo0_1	movlw	B'00000010'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	DNOP			;007-008 cycles, 0.43-0.49 bit times
	btfsc	PORTC,RC5	;009 cycles, 0.55 bit times
	goto	RcTo1_0		;010 cycles, 0.61 bit times
	btfsc	PORTC,RC5	;011 cycles, 0.67 bit times
	goto	RcTo1_0		;012 cycles, 0.73 bit times
	btfsc	PORTC,RC5	;013 cycles, 0.80 bit times
	goto	RcTo1_0		;014 cycles, 0.86 bit times
	btfsc	PORTC,RC5	;015 cycles, 0.92 bit times
	goto	RcTo1_0		;016 cycles, 0.98 bit times
	btfsc	PORTC,RC5	;017 cycles, 1.04 bit times
	goto	RcTo1_0		;018 cycles, 1.10 bit times
	btfsc	PORTC,RC5	;019 cycles, 1.16 bit times
	goto	RcTo1_0		;020 cycles, 1.22 bit times
	btfsc	PORTC,RC5	;021 cycles, 1.29 bit times
	goto	RcTo1_0		;022 cycles, 1.35 bit times
	btfsc	PORTC,RC5	;023 cycles, 1.41 bit times
	goto	RcTo1_0		;024 cycles, 1.47 bit times
	btfss	PORTA,RA1	;025 cycles, 1.53 bit times
	bcf	INDF0,7		;026 cycles, 1.59 bit times
	moviw	FSR0++		;027 cycles, 1.65 bit times
	goto	RcSt0		;028 cycles, 1.71 bit times

RcTo0_0	movlw	B'00000001'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	moviw	FSR0++		;007 cycles, 0.43 bit times
	goto	RcSt0		;008 cycles, 0.49 bit times


;;; Received Data Decoder ;;;

RecvDecode
	movf	RC_CMDG,W	;Trust the group count that mac sent is
	andlw	B'01111111'	; accurate and copy it into the group count
	movwf	GROUPS		; variable that we return as well as a temp var
	movwf	TEMP2		; that we can trash
	movlw	0x20		;Start both pointers after the three initial
	movwf	FSR0H		; IWM bytes (the sync byte, the length of the
	movwf	FSR1H		; message in groups plus 0x80, the length of
	movlw	0x03		; the expected reply in groups plus 0x80)
	movwf	FSR0L		; "
	movwf	FSR1L		; "

RDLoop	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_7		; into suspend here, switch loops
	moviw	FSR0++		;Pick up the byte with the LSBs of the next 
	movwf	TEMP		; seven
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_6		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the first byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_5		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the second byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_4		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the third byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_3		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the fourth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_2		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the fifth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_1		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the sixth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_0		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the last byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	decfsz	TEMP2,F		;Decrement the group count and loop if it's not
	bra	RDLoop		; yet zero
	return

RDSus_7	moviw	FSR0++		;Pick up the byte with the LSBs of the next 
	movwf	TEMP		; seven
RDSus_6	lsrf	TEMP,F		;Rotate the LSB of the first byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
RDSus_5	lsrf	TEMP,F		;Rotate the LSB of the second byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
RDSus_4	lsrf	TEMP,F		;Rotate the LSB of the third byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
RDSus_3	lsrf	TEMP,F		;Rotate the LSB of the fourth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
RDSus_2	lsrf	TEMP,F		;Rotate the LSB of the fifth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
RDSus_1	lsrf	TEMP,F		;Rotate the LSB of the sixth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
RDSus_0	lsrf	TEMP,F		;Rotate the LSB of the last byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	moviw	FSR0++		;Advance the input pointer
	decf	TEMP2,F		;Decrement the group count
	btfsc	STATUS,Z	;If the group count happens to be zero, mac
	return			; suspended in the last group and we're done
RDSusLp	moviw	FSR0++		;Read the next byte, and if it's a sync byte,
	xorlw	0xAA		; continue processing data
	btfsc	STATUS,Z	; "
	bra	RDLoop		; "
	btfss	FSR0H,2		;As a safety measure, check if we've gone off
	bra	RDSusLp		; the edge of linear memory before we loop
	bsf	FLAGS,ABORTED	; to check the next byte; if we have, set the
	return			; aborted flag and return


;;; CRC Lookup Tables ;;;

	org	0x1D00

LutCrc7
	retlw	0x00
	retlw	0x12
	retlw	0x24
	retlw	0x36
	retlw	0x48
	retlw	0x5A
	retlw	0x6C
	retlw	0x7E
	retlw	0x90
	retlw	0x82
	retlw	0xB4
	retlw	0xA6
	retlw	0xD8
	retlw	0xCA
	retlw	0xFC
	retlw	0xEE
	retlw	0x32
	retlw	0x20
	retlw	0x16
	retlw	0x04
	retlw	0x7A
	retlw	0x68
	retlw	0x5E
	retlw	0x4C
	retlw	0xA2
	retlw	0xB0
	retlw	0x86
	retlw	0x94
	retlw	0xEA
	retlw	0xF8
	retlw	0xCE
	retlw	0xDC
	retlw	0x64
	retlw	0x76
	retlw	0x40
	retlw	0x52
	retlw	0x2C
	retlw	0x3E
	retlw	0x08
	retlw	0x1A
	retlw	0xF4
	retlw	0xE6
	retlw	0xD0
	retlw	0xC2
	retlw	0xBC
	retlw	0xAE
	retlw	0x98
	retlw	0x8A
	retlw	0x56
	retlw	0x44
	retlw	0x72
	retlw	0x60
	retlw	0x1E
	retlw	0x0C
	retlw	0x3A
	retlw	0x28
	retlw	0xC6
	retlw	0xD4
	retlw	0xE2
	retlw	0xF0
	retlw	0x8E
	retlw	0x9C
	retlw	0xAA
	retlw	0xB8
	retlw	0xC8
	retlw	0xDA
	retlw	0xEC
	retlw	0xFE
	retlw	0x80
	retlw	0x92
	retlw	0xA4
	retlw	0xB6
	retlw	0x58
	retlw	0x4A
	retlw	0x7C
	retlw	0x6E
	retlw	0x10
	retlw	0x02
	retlw	0x34
	retlw	0x26
	retlw	0xFA
	retlw	0xE8
	retlw	0xDE
	retlw	0xCC
	retlw	0xB2
	retlw	0xA0
	retlw	0x96
	retlw	0x84
	retlw	0x6A
	retlw	0x78
	retlw	0x4E
	retlw	0x5C
	retlw	0x22
	retlw	0x30
	retlw	0x06
	retlw	0x14
	retlw	0xAC
	retlw	0xBE
	retlw	0x88
	retlw	0x9A
	retlw	0xE4
	retlw	0xF6
	retlw	0xC0
	retlw	0xD2
	retlw	0x3C
	retlw	0x2E
	retlw	0x18
	retlw	0x0A
	retlw	0x74
	retlw	0x66
	retlw	0x50
	retlw	0x42
	retlw	0x9E
	retlw	0x8C
	retlw	0xBA
	retlw	0xA8
	retlw	0xD6
	retlw	0xC4
	retlw	0xF2
	retlw	0xE0
	retlw	0x0E
	retlw	0x1C
	retlw	0x2A
	retlw	0x38
	retlw	0x46
	retlw	0x54
	retlw	0x62
	retlw	0x70
	retlw	0x82
	retlw	0x90
	retlw	0xA6
	retlw	0xB4
	retlw	0xCA
	retlw	0xD8
	retlw	0xEE
	retlw	0xFC
	retlw	0x12
	retlw	0x00
	retlw	0x36
	retlw	0x24
	retlw	0x5A
	retlw	0x48
	retlw	0x7E
	retlw	0x6C
	retlw	0xB0
	retlw	0xA2
	retlw	0x94
	retlw	0x86
	retlw	0xF8
	retlw	0xEA
	retlw	0xDC
	retlw	0xCE
	retlw	0x20
	retlw	0x32
	retlw	0x04
	retlw	0x16
	retlw	0x68
	retlw	0x7A
	retlw	0x4C
	retlw	0x5E
	retlw	0xE6
	retlw	0xF4
	retlw	0xC2
	retlw	0xD0
	retlw	0xAE
	retlw	0xBC
	retlw	0x8A
	retlw	0x98
	retlw	0x76
	retlw	0x64
	retlw	0x52
	retlw	0x40
	retlw	0x3E
	retlw	0x2C
	retlw	0x1A
	retlw	0x08
	retlw	0xD4
	retlw	0xC6
	retlw	0xF0
	retlw	0xE2
	retlw	0x9C
	retlw	0x8E
	retlw	0xB8
	retlw	0xAA
	retlw	0x44
	retlw	0x56
	retlw	0x60
	retlw	0x72
	retlw	0x0C
	retlw	0x1E
	retlw	0x28
	retlw	0x3A
	retlw	0x4A
	retlw	0x58
	retlw	0x6E
	retlw	0x7C
	retlw	0x02
	retlw	0x10
	retlw	0x26
	retlw	0x34
	retlw	0xDA
	retlw	0xC8
	retlw	0xFE
	retlw	0xEC
	retlw	0x92
	retlw	0x80
	retlw	0xB6
	retlw	0xA4
	retlw	0x78
	retlw	0x6A
	retlw	0x5C
	retlw	0x4E
	retlw	0x30
	retlw	0x22
	retlw	0x14
	retlw	0x06
	retlw	0xE8
	retlw	0xFA
	retlw	0xCC
	retlw	0xDE
	retlw	0xA0
	retlw	0xB2
	retlw	0x84
	retlw	0x96
	retlw	0x2E
	retlw	0x3C
	retlw	0x0A
	retlw	0x18
	retlw	0x66
	retlw	0x74
	retlw	0x42
	retlw	0x50
	retlw	0xBE
	retlw	0xAC
	retlw	0x9A
	retlw	0x88
	retlw	0xF6
	retlw	0xE4
	retlw	0xD2
	retlw	0xC0
	retlw	0x1C
	retlw	0x0E
	retlw	0x38
	retlw	0x2A
	retlw	0x54
	retlw	0x46
	retlw	0x70
	retlw	0x62
	retlw	0x8C
	retlw	0x9E
	retlw	0xA8
	retlw	0xBA
	retlw	0xC4
	retlw	0xD6
	retlw	0xE0
	retlw	0xF2

	org	0x1E00

LutCrc16H
	retlw	0x00
	retlw	0x10
	retlw	0x20
	retlw	0x30
	retlw	0x40
	retlw	0x50
	retlw	0x60
	retlw	0x70
	retlw	0x81
	retlw	0x91
	retlw	0xA1
	retlw	0xB1
	retlw	0xC1
	retlw	0xD1
	retlw	0xE1
	retlw	0xF1
	retlw	0x12
	retlw	0x02
	retlw	0x32
	retlw	0x22
	retlw	0x52
	retlw	0x42
	retlw	0x72
	retlw	0x62
	retlw	0x93
	retlw	0x83
	retlw	0xB3
	retlw	0xA3
	retlw	0xD3
	retlw	0xC3
	retlw	0xF3
	retlw	0xE3
	retlw	0x24
	retlw	0x34
	retlw	0x04
	retlw	0x14
	retlw	0x64
	retlw	0x74
	retlw	0x44
	retlw	0x54
	retlw	0xA5
	retlw	0xB5
	retlw	0x85
	retlw	0x95
	retlw	0xE5
	retlw	0xF5
	retlw	0xC5
	retlw	0xD5
	retlw	0x36
	retlw	0x26
	retlw	0x16
	retlw	0x06
	retlw	0x76
	retlw	0x66
	retlw	0x56
	retlw	0x46
	retlw	0xB7
	retlw	0xA7
	retlw	0x97
	retlw	0x87
	retlw	0xF7
	retlw	0xE7
	retlw	0xD7
	retlw	0xC7
	retlw	0x48
	retlw	0x58
	retlw	0x68
	retlw	0x78
	retlw	0x08
	retlw	0x18
	retlw	0x28
	retlw	0x38
	retlw	0xC9
	retlw	0xD9
	retlw	0xE9
	retlw	0xF9
	retlw	0x89
	retlw	0x99
	retlw	0xA9
	retlw	0xB9
	retlw	0x5A
	retlw	0x4A
	retlw	0x7A
	retlw	0x6A
	retlw	0x1A
	retlw	0x0A
	retlw	0x3A
	retlw	0x2A
	retlw	0xDB
	retlw	0xCB
	retlw	0xFB
	retlw	0xEB
	retlw	0x9B
	retlw	0x8B
	retlw	0xBB
	retlw	0xAB
	retlw	0x6C
	retlw	0x7C
	retlw	0x4C
	retlw	0x5C
	retlw	0x2C
	retlw	0x3C
	retlw	0x0C
	retlw	0x1C
	retlw	0xED
	retlw	0xFD
	retlw	0xCD
	retlw	0xDD
	retlw	0xAD
	retlw	0xBD
	retlw	0x8D
	retlw	0x9D
	retlw	0x7E
	retlw	0x6E
	retlw	0x5E
	retlw	0x4E
	retlw	0x3E
	retlw	0x2E
	retlw	0x1E
	retlw	0x0E
	retlw	0xFF
	retlw	0xEF
	retlw	0xDF
	retlw	0xCF
	retlw	0xBF
	retlw	0xAF
	retlw	0x9F
	retlw	0x8F
	retlw	0x91
	retlw	0x81
	retlw	0xB1
	retlw	0xA1
	retlw	0xD1
	retlw	0xC1
	retlw	0xF1
	retlw	0xE1
	retlw	0x10
	retlw	0x00
	retlw	0x30
	retlw	0x20
	retlw	0x50
	retlw	0x40
	retlw	0x70
	retlw	0x60
	retlw	0x83
	retlw	0x93
	retlw	0xA3
	retlw	0xB3
	retlw	0xC3
	retlw	0xD3
	retlw	0xE3
	retlw	0xF3
	retlw	0x02
	retlw	0x12
	retlw	0x22
	retlw	0x32
	retlw	0x42
	retlw	0x52
	retlw	0x62
	retlw	0x72
	retlw	0xB5
	retlw	0xA5
	retlw	0x95
	retlw	0x85
	retlw	0xF5
	retlw	0xE5
	retlw	0xD5
	retlw	0xC5
	retlw	0x34
	retlw	0x24
	retlw	0x14
	retlw	0x04
	retlw	0x74
	retlw	0x64
	retlw	0x54
	retlw	0x44
	retlw	0xA7
	retlw	0xB7
	retlw	0x87
	retlw	0x97
	retlw	0xE7
	retlw	0xF7
	retlw	0xC7
	retlw	0xD7
	retlw	0x26
	retlw	0x36
	retlw	0x06
	retlw	0x16
	retlw	0x66
	retlw	0x76
	retlw	0x46
	retlw	0x56
	retlw	0xD9
	retlw	0xC9
	retlw	0xF9
	retlw	0xE9
	retlw	0x99
	retlw	0x89
	retlw	0xB9
	retlw	0xA9
	retlw	0x58
	retlw	0x48
	retlw	0x78
	retlw	0x68
	retlw	0x18
	retlw	0x08
	retlw	0x38
	retlw	0x28
	retlw	0xCB
	retlw	0xDB
	retlw	0xEB
	retlw	0xFB
	retlw	0x8B
	retlw	0x9B
	retlw	0xAB
	retlw	0xBB
	retlw	0x4A
	retlw	0x5A
	retlw	0x6A
	retlw	0x7A
	retlw	0x0A
	retlw	0x1A
	retlw	0x2A
	retlw	0x3A
	retlw	0xFD
	retlw	0xED
	retlw	0xDD
	retlw	0xCD
	retlw	0xBD
	retlw	0xAD
	retlw	0x9D
	retlw	0x8D
	retlw	0x7C
	retlw	0x6C
	retlw	0x5C
	retlw	0x4C
	retlw	0x3C
	retlw	0x2C
	retlw	0x1C
	retlw	0x0C
	retlw	0xEF
	retlw	0xFF
	retlw	0xCF
	retlw	0xDF
	retlw	0xAF
	retlw	0xBF
	retlw	0x8F
	retlw	0x9F
	retlw	0x6E
	retlw	0x7E
	retlw	0x4E
	retlw	0x5E
	retlw	0x2E
	retlw	0x3E
	retlw	0x0E
	retlw	0x1E

	org	0x1F00

LutCrc16L
	retlw	0x00
	retlw	0x21
	retlw	0x42
	retlw	0x63
	retlw	0x84
	retlw	0xA5
	retlw	0xC6
	retlw	0xE7
	retlw	0x08
	retlw	0x29
	retlw	0x4A
	retlw	0x6B
	retlw	0x8C
	retlw	0xAD
	retlw	0xCE
	retlw	0xEF
	retlw	0x31
	retlw	0x10
	retlw	0x73
	retlw	0x52
	retlw	0xB5
	retlw	0x94
	retlw	0xF7
	retlw	0xD6
	retlw	0x39
	retlw	0x18
	retlw	0x7B
	retlw	0x5A
	retlw	0xBD
	retlw	0x9C
	retlw	0xFF
	retlw	0xDE
	retlw	0x62
	retlw	0x43
	retlw	0x20
	retlw	0x01
	retlw	0xE6
	retlw	0xC7
	retlw	0xA4
	retlw	0x85
	retlw	0x6A
	retlw	0x4B
	retlw	0x28
	retlw	0x09
	retlw	0xEE
	retlw	0xCF
	retlw	0xAC
	retlw	0x8D
	retlw	0x53
	retlw	0x72
	retlw	0x11
	retlw	0x30
	retlw	0xD7
	retlw	0xF6
	retlw	0x95
	retlw	0xB4
	retlw	0x5B
	retlw	0x7A
	retlw	0x19
	retlw	0x38
	retlw	0xDF
	retlw	0xFE
	retlw	0x9D
	retlw	0xBC
	retlw	0xC4
	retlw	0xE5
	retlw	0x86
	retlw	0xA7
	retlw	0x40
	retlw	0x61
	retlw	0x02
	retlw	0x23
	retlw	0xCC
	retlw	0xED
	retlw	0x8E
	retlw	0xAF
	retlw	0x48
	retlw	0x69
	retlw	0x0A
	retlw	0x2B
	retlw	0xF5
	retlw	0xD4
	retlw	0xB7
	retlw	0x96
	retlw	0x71
	retlw	0x50
	retlw	0x33
	retlw	0x12
	retlw	0xFD
	retlw	0xDC
	retlw	0xBF
	retlw	0x9E
	retlw	0x79
	retlw	0x58
	retlw	0x3B
	retlw	0x1A
	retlw	0xA6
	retlw	0x87
	retlw	0xE4
	retlw	0xC5
	retlw	0x22
	retlw	0x03
	retlw	0x60
	retlw	0x41
	retlw	0xAE
	retlw	0x8F
	retlw	0xEC
	retlw	0xCD
	retlw	0x2A
	retlw	0x0B
	retlw	0x68
	retlw	0x49
	retlw	0x97
	retlw	0xB6
	retlw	0xD5
	retlw	0xF4
	retlw	0x13
	retlw	0x32
	retlw	0x51
	retlw	0x70
	retlw	0x9F
	retlw	0xBE
	retlw	0xDD
	retlw	0xFC
	retlw	0x1B
	retlw	0x3A
	retlw	0x59
	retlw	0x78
	retlw	0x88
	retlw	0xA9
	retlw	0xCA
	retlw	0xEB
	retlw	0x0C
	retlw	0x2D
	retlw	0x4E
	retlw	0x6F
	retlw	0x80
	retlw	0xA1
	retlw	0xC2
	retlw	0xE3
	retlw	0x04
	retlw	0x25
	retlw	0x46
	retlw	0x67
	retlw	0xB9
	retlw	0x98
	retlw	0xFB
	retlw	0xDA
	retlw	0x3D
	retlw	0x1C
	retlw	0x7F
	retlw	0x5E
	retlw	0xB1
	retlw	0x90
	retlw	0xF3
	retlw	0xD2
	retlw	0x35
	retlw	0x14
	retlw	0x77
	retlw	0x56
	retlw	0xEA
	retlw	0xCB
	retlw	0xA8
	retlw	0x89
	retlw	0x6E
	retlw	0x4F
	retlw	0x2C
	retlw	0x0D
	retlw	0xE2
	retlw	0xC3
	retlw	0xA0
	retlw	0x81
	retlw	0x66
	retlw	0x47
	retlw	0x24
	retlw	0x05
	retlw	0xDB
	retlw	0xFA
	retlw	0x99
	retlw	0xB8
	retlw	0x5F
	retlw	0x7E
	retlw	0x1D
	retlw	0x3C
	retlw	0xD3
	retlw	0xF2
	retlw	0x91
	retlw	0xB0
	retlw	0x57
	retlw	0x76
	retlw	0x15
	retlw	0x34
	retlw	0x4C
	retlw	0x6D
	retlw	0x0E
	retlw	0x2F
	retlw	0xC8
	retlw	0xE9
	retlw	0x8A
	retlw	0xAB
	retlw	0x44
	retlw	0x65
	retlw	0x06
	retlw	0x27
	retlw	0xC0
	retlw	0xE1
	retlw	0x82
	retlw	0xA3
	retlw	0x7D
	retlw	0x5C
	retlw	0x3F
	retlw	0x1E
	retlw	0xF9
	retlw	0xD8
	retlw	0xBB
	retlw	0x9A
	retlw	0x75
	retlw	0x54
	retlw	0x37
	retlw	0x16
	retlw	0xF1
	retlw	0xD0
	retlw	0xB3
	retlw	0x92
	retlw	0x2E
	retlw	0x0F
	retlw	0x6C
	retlw	0x4D
	retlw	0xAA
	retlw	0x8B
	retlw	0xE8
	retlw	0xC9
	retlw	0x26
	retlw	0x07
	retlw	0x64
	retlw	0x45
	retlw	0xA2
	retlw	0x83
	retlw	0xE0
	retlw	0xC1
	retlw	0x1F
	retlw	0x3E
	retlw	0x5D
	retlw	0x7C
	retlw	0x9B
	retlw	0xBA
	retlw	0xD9
	retlw	0xF8
	retlw	0x17
	retlw	0x36
	retlw	0x55
	retlw	0x74
	retlw	0x93
	retlw	0xB2
	retlw	0xD1
	retlw	0xF0


;;; End of Program ;;;
	end
