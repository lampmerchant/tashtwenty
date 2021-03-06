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

;FLAGS:
ABORTED	equ	7	;Set if a transmit or receive operation was aborted
RNFERR	equ	6	;Set if host tries to access a too-high block address
DEVSEL1	equ	1	;MSB of which device is selected
DEVSEL0	equ	0	;LSB of which device is selected

;M_FLAGS:
M_FAIL	equ	7	;Set when there's been a failure on the MMC interface
M_CMDLR	equ	6	;Set when R3 or R7 is expected (5 bytes), not R1
M_CMDRB	equ	5	;Set when an R1b is expected (busy signal after reply)
M_MBRD	equ	4	;Set when a multiblock read is in progress
M_MBWR	equ	3	;Set when a multiblock write is in progress
M_BKADR	equ	2	;Set when block (rather than byte) addressing is in use
M_CDVER	equ	1	;Set when dealing with a V2.0+ card, clear for 1.0

;DVxFLAG:
DV_EXST	equ	7	;Set when device exists

;Firmware version
FWVER_3	equ	0x20
FWVER_2	equ	0x21
FWVER_1	equ	0x11
FWVER_0	equ	0x28


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
	TEMP	;Various purposes
	TEMP2	;Various purposes
	D3
	D2
	D1
	D0
	
	endc

	cblock	0x20	;Bank 0 registers
	
	;Partition struct
	DV0FLAG	;Device 0 flags
	DV0LN_2	;Device 0 LBA length
	DV0LN_1	; "
	DV0LN_0	; "
	DV0ST_3	;Device 0 start LBA
	DV0ST_2	; "
	DV0ST_1	; "
	DV0ST_0	; "
	DV1FLAG	;Device 1 flags
	DV1LN_2	;Device 1 LBA length
	DV1LN_1	; "
	DV1LN_0	; "
	DV1ST_3	;Device 1 start LBA
	DV1ST_2	; "
	DV1ST_1	; "
	DV1ST_0	; "
	DV2FLAG	;Device 2 flags
	DV2LN_2	;Device 2 LBA length
	DV2LN_1	; "
	DV2LN_0	; "
	DV2ST_3	;Device 2 start LBA
	DV2ST_2	; "
	DV2ST_1	; "
	DV2ST_0	; "
	DV3FLAG	;Device 3 flags
	DV3LN_2	;Device 3 LBA length
	DV3LN_1	; "
	DV3LN_0	; "
	DV3ST_3	;Device 3 start LBA
	DV3ST_2	; "
	DV3ST_1	; "
	DV3ST_0	; "
	
	;Error (high nibble) and unknown command (low nibble) log
	LOG_00
	LOG_01
	LOG_02
	LOG_03
	LOG_04
	LOG_05
	LOG_06
	LOG_07
	LOG_08
	LOG_09
	LOG_0A
	LOG_0B
	LOG_0C
	LOG_0D
	LOG_0E
	LOG_0F
	LOG_10
	LOG_11
	LOG_12
	LOG_13
	LOG_14
	LOG_15
	LOG_16
	LOG_17
	LOG_18
	LOG_19
	LOG_1A
	LOG_1B
	LOG_1C
	LOG_1D
	LOG_1E
	LOG_1F
	
	;Card info
	CARDINF

	;TODO Unused 7 bytes
	UNUSED6
	UNUSED5
	UNUSED4
	UNUSED3
	UNUSED2
	UNUSED1
	UNUSED0
	
	endc

	cblock	0x68	;Top of data buffer (receive shorthand)
	
	RC_SYNC	;Sync byte
	RC_CMDG	;Command groups + 0x80
	RC_RSPG	;Response groups + 0x80
	RC_CMDN	;Command number
	RC_BLKS	;Block count
	RC_ADRH	;Block address high
	RC_ADRM	;Block address middle
	RC_ADRL	;Block address low
	
	endc

	cblock	0x68	;Top of data buffer (transmit shorthand)
	
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

	movlw	B'00001000'	;Interrupt on PORTA pin change enabled, but
	movwf	INTCON		; interrupt subsystem off for now

	movlw	0x20		;Zero all bank 0 registers
	movwf	FSR0H
	clrf	FSR0L
	movlw	72
	movwf	TEMP
	movlw	0
ClrLoop	movwi	FSR0++
	decfsz	TEMP,F
	bra	ClrLoop

	movlb	0		;Start BSR at 0

	movlw	11		;Delay approximately 1 ms to allow the MMC card
	movwf	TEMP		; to come up - do not call MmcInit before this
DelayMs	DELAY	242		; has been done
	decfsz	TEMP,F		; "
	bra	DelayMs		; "
	call	MmcInit		;Initialize!
	btfsc	M_FLAGS,M_FAIL	;If we couldn't initialize the card, give up
	goto	NoDevices	; and pass !ENBL through to next device
	movlb	4		;Now that we're initialized, crank the speed
	movlw	B'00100001'	; of the SPI interface up to 2 MHz
	movwf	SSPCON1		; "
	movlb	0		; "

	movlw	0x51		;Set up a read command for the MMC card (R1-
	movwf	M_CMDN		; type response)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	clrf	M_ADR3		;Point the card address to the master boot
	clrf	M_ADR2		; record
	clrf	M_ADR1		; "
	clrf	M_ADR0		; "
	bcf	PORTC,RC3	;Assert !CS
	call	MmcCmd		;Send the MMC command
	btfsc	M_FLAGS,M_FAIL	;If the operation failed, give up and just pass
	goto	NoDevices	; !ENBL through to the next device
	movlw	0x20		;Point FSR0 to the top of the data buffer
	movwf	FSR0H		; "
	movlw	0x48		; "
	movwf	FSR0L		; "
	call	MmcReadData	;Read data from the MMC card
	btfsc	M_FLAGS,M_FAIL	;If the operation failed, give up and just pass
	goto	NoDevices	; !ENBL through to the next device
	bsf	PORTC,RC3	;Deassert !CS
	
	movlw	0x22		;Point FSR0 to the MBR partition table
	movwf	FSR0H		; "
	movlw	0x06		; "
	movwf	FSR0L		; "
	movlw	0x20		;Point FSR1 to the device structs
	movwf	FSR1H		; "
	clrf	FSR1L		; "
	movlw	4		;Loop through all four MBR partition entries
	movwf	TEMP		; "
MbrLoop	moviw	4[FSR0]		;Check whether the partition type is 0xAF,
	xorlw	0xAF		; which is the type code for HFS; if it isn't,
	btfss	STATUS,Z	; then move ahead to the next one
	bra	MbrNext		; "
	moviw	15[FSR0]	;Check whether the partition size > 0xFFFFFF;
	btfss	STATUS,Z	; if it is, it's too big, move ahead to the
	bra	MbrNext		; next one
	moviw	11[FSR0]	;Copy the partition start LBA, converting the
	movwi	4[FSR1]		; little-endian number to big-endian as we go
	moviw	10[FSR0]	; "
	movwi	5[FSR1]		; "
	moviw	9[FSR0]		; "
	movwi	6[FSR1]		; "
	moviw	8[FSR0]		; "
	movwi	7[FSR1]		; "
	moviw	14[FSR0]	;Copy the partition LBA size, converting the
	movwi	1[FSR1]		; little-endian number to big-endian as we go
	moviw	13[FSR0]	; "
	movwi	2[FSR1]		; "
	moviw	12[FSR0]	; "
	movwi	3[FSR1]		; "
	movlw	1 << DV_EXST	;Set the device-exists flag for this partition
	movwi	0[FSR1]		; "
	addfsr	FSR1,8		;Advance device struct pointer to next entry
MbrNext	addfsr	FSR0,16		;Advance the MBR pointer to the next entry
	decfsz	TEMP,F		;If we have partition entries left to scan,
	bra	MbrLoop		; loop for the next one
	moviw	0[FSR0]		;After all that, check whether the 0xAA55 (LE)
	xorlw	0x55		; MBR signature is present; if it isn't, then
	btfss	STATUS,Z	; all we did above was for nothing because this
	goto	NoDevices	; isn't a valid MBR, so just pass !ENBL through
	moviw	1[FSR0]		; "
	xorlw	0xAA		; "
	btfss	STATUS,Z	; "
	goto	NoDevices	; "
	btfss	DV0FLAG,DV_EXST	;If we didn't detect any valid partitions, just
	goto	NoDevices	; pass !ENBL through; if we have at least one,
	goto	WaitEnbl	; enter the normal idle loop and wait for !ENBL


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
	btfsc	STATUS,Z	; write (write routine will tell this apart)
	bra	CmdWrite	; "
	bra	CmdUnknown	;If none of the above, fake a response

CmdUnknown
	movf	RC_CMDN,W	;Save the unknown command's number since it's
	movwf	TEMP2		; about to be overwritten
	call	ClearResponse	;Clear the buffer according to expectations
	movf	TEMP2,W		;Our reply is going to be the command number
	movwf	TX_CMDN		; with its MSB set, by convention, but data
	bsf	TX_CMDN,7	; will be all zeroes
	clrf	FSR0H		;Point FSR0 at error/unknown command log
	bsf	FSR0H,5		; "
	clrf	FSR0L		; "
	bsf	FSR0L,5		; "
	andlw	B'0011111'	;Mask off the low 5 bits of the command code
	addwf	FSR0L,F		; "
	movf	INDF0,W		;If the count (low nibble) is already at 15,
	andlw	B'00001111'	; keep it there; if it's below 15, increment it
	xorlw	B'00001111'	; "
	btfss	STATUS,Z	; "
	incf	INDF0,F		; "
	call	CalcChecksum	;Calculate the checksum of placeholder reply
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
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	return			;Done

ErrorSetup
	movlb	31		;Insert the latter half of this function into
	movf	TOSH,W		; the call stack, then return to the caller so
	movwf	TEMP		; the caller can retlw a status code that we
	movf	TOSL,W		; can act on
	movwf	TEMP2		; "
	movlw	high ErrSet0	; "
	movwf	TOSH		; "
	movlw	low ErrSet0	; "
	movwf	TOSL		; "
	incf	STKPTR,F	; "
	movf	TEMP,W		; "
	movwf	TOSH		; "
	movf	TEMP2,W		; "
	movwf	TOSL		; "
	movlb	0		; "
	return			; "
ErrSet0	movf	WREG,W		;If the caller returned status 0, just return
	btfsc	STATUS,Z	; to idle loop
	return			; "
	movwf	TX_PAD1		;Store error code in first transmitter pad byte
	clrf	FSR0H		;Point FSR0 at error/unknown command log
	bsf	FSR0H,5		; "
	clrf	FSR0L		; "
	bsf	FSR0L,5		; "
	andlw	B'0011111'	;Mask off the low 5 bits of the error code (we
	addwf	FSR0L,F		; need to keep to 32 or fewer errors)
	movf	INDF0,W		;If the error count (high nibble) is already at
	andlw	B'11110000'	; 15, keep it there; if it's below 15,
	xorlw	B'11110000'	; increment it
	movlw	0x10		; "
	btfsc	STATUS,Z	; "
	addwf	INDF0,F		; "
	bsf	PORTC,RC3	;Deassert !CS
	movlw	1		;Communicate failure to the host by setting the
	movwf	GROUPS		; MSB of the status byte and sending only the
	bsf	TX_STAT,7	; header group
	call	CalcChecksum	;Calculate the checksum
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	return			;Done

CmdRead
	call	ErrorSetup	;Set up error handling
	movf	RC_BLKS,W	;Move the block count from receiver position to
	movwf	TX_BLKS		; transmitter position
	movlw	0x80		;Command number is the read command with the
	movwf	TX_CMDN		; MSB set
	clrf	TX_STAT		;A zero status means all is well
	call	MmcStopOngoing	;Stop any multiblock read or write
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x01		; "
	call	TranslateAddr	;Translate block address to MMC block address
	btfsc	FLAGS,RNFERR	;If we couldn't translate this block address to
	retlw	0x02		; an MMC address, report the error
	call	MmcConvAddr	;Convert the address if necessary
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x03		; "
	clrf	TX_PAD1		;Clear the padding bytes (don't do this before
	clrf	TX_PAD2		; we convert the address since they overlap
	clrf	TX_PAD3		; with the address bytes)
	;TODO clear 20 tag bytes too?
	movlw	0x52		;Set up a read command for the MMC card (R1-
	movwf	M_CMDN		; type response)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	bcf	PORTC,RC3	;Assert !CS
	call	MmcCmd		;Send the MMC command
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x04		; "
	bsf	M_FLAGS,M_MBRD	;Set the flag for an ongoing multiblock read
CmdRea0	movlw	0x20		;Point FSR0 past the six header bytes and the
	movwf	FSR0H		; 20 'tag' bytes to where the real data starts
	movlw	0x62		; "
	movwf	FSR0L		; "
	call	MmcReadData	;Read data from the MMC card
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x05		; "
	decf	TX_BLKS,W	;If the blocks-remaining counter is at one,
	btfsc	STATUS,Z	; stop the multiblock read here before sending
	call	MmcStopOngoing	; our response to the mac
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x06		; "
	movlw	77		;Set the group count to 77 (512 bytes of data,
	movwf	GROUPS		; 20 'tag' bytes, 6 byte header, checksum byte)
	call	CalcChecksum	;Calculate the checksum on our response
	movlp	high Transmit	;Initiate transmission
	call	Transmit	; "
	btfsc	FLAGS,ABORTED	;If the command was aborted, return to idle
	retlw	0x00		; loop
	call	MmcIncAddr	;Increment address for next read
	decfsz	TX_BLKS,F	;Decrement the block count; if it hits zero,
	bra	CmdRea0		; that's all the reading we were requested to
	retlw	0x00		; do, so call it done

CmdWrite
	call	ErrorSetup	;Set up error handling
	movf	RC_BLKS,W	;Move the block count from receiver position to
	movwf	TX_BLKS		; transmitter position
	movlw	0x81		;Command number is the write command with the
	movwf	TX_CMDN		; MSB set
	clrf	TX_STAT		;A zero status means all is well
	movf	RC_CMDN,W	;If this is a continued write, we don't want to
	xorlw	0x41		; set up the write anew, so skip ahead
	btfsc	STATUS,Z	; "
	bra	CmdWriteCont	; "
	call	MmcStopOngoing	;Stop any multiblock read or write
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x07		; "
	call	TranslateAddr	;Translate block address to MMC block address
	btfsc	FLAGS,RNFERR	;If we couldn't translate this block address to
	retlw	0x08		; an MMC address, report the error
	call	MmcConvAddr	;Convert the address if necessary
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x09		; "
	movlw	0x59		;Set up a write command for the MMC card (R1-
	movwf	M_CMDN		; type reply)
	bcf	M_FLAGS,M_CMDLR	; "
	bcf	M_FLAGS,M_CMDRB	; "
	bcf	PORTC,RC3	;Assert !CS
	call	MmcCmd		;Send the MMC command
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x0A		; "
	bsf	M_FLAGS,M_MBWR	;Set the flag for an ongoing multiblock write
CmdWriteCont
	btfss	M_FLAGS,M_MBWR	;If for some reason we got here without there
	retlw	0x0B		; being a multiblock write going on, error
	movlw	0x20		;Point FSR0 past the sync, length, header, and
	movwf	FSR0H		; tag bytes, and to the data we'll be writing
	movlw	0x65		; "
	movwf	FSR0L		; "
	call	MmcWriteData	;Write data to the MMC card
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x0C		; "
	decf	RC_BLKS,W	;If the blocks-remaining counter is at one,
	btfsc	STATUS,Z	; stop the multiblock write here before sending
	call	MmcStopOngoing	; our response to the mac
	btfsc	M_FLAGS,M_FAIL	;If MMC operation failed, report the error
	retlw	0x0D		; "
	clrf	TX_PAD1		;Clear the padding bytes (don't do this before
	clrf	TX_PAD2		; we convert the address since they overlap
	clrf	TX_PAD3		; with the address bytes)
	movlw	1		;Acknowledgment is one group in length
	movwf	GROUPS		; "
	call	CalcChecksum	;Calculate the checksum on our response
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	retlw	0x00		;Done

CmdStatus
	call	ClearResponse	;Clear the buffer for our response
	movlw	0x83		;Command number is the status command with the
	movwf	TX_CMDN		; MSB set
	clrf	TX_STAT		;A zero status means all is well
	clrf	TX_BLKS		;Clear the block count as it's not relevant
	clrf	TX_PAD1		;Clear the padding bytes
	clrf	TX_PAD2		; "
	clrf	TX_PAD3		; "
	movlw	0x20		;Point FSR0 at the appropriate struct for the
	movwf	FSR0H		; selected device
	clrf	FSR0L		; "
	btfsc	FLAGS,DEVSEL1	; "
	addfsr	FSR0,16		; "
	btfsc	FLAGS,DEVSEL0	; "
	addfsr	FSR0,8		; "
	movlw	0x20		;Point FSR1 past the six header bytes to where
	movwf	FSR1H		; the real data starts
	movlw	0x4E		; "
	movwf	FSR1L		; "
	movlw	0x01		;Device manufacturer is 0x0100, not sure if
	movwi	2[FSR1]		; anybody cares
	movlw	0xF6		;Device is mountable, readable, writeable,
	movwi	4[FSR1]		; ejectable (?), icon_included, & disk_in_place
	moviw	3[FSR0]		;Copy the LBA length of the partition into the
	movwi	7[FSR1]		; block size of the drive
	moviw	2[FSR0]		; "
	movwi	6[FSR1]		; "
	moviw	1[FSR0]		; "
	movwi	5[FSR1]		; "
	addfsr	FSR1,7		;Decrement the block size of the drive because
	movlw	0xFF		; it appears that the block size of the drive
	addwf	INDF1,F		; is actually the maximum block on the drive
	addfsr	FSR1,-1		; TODO look into this further
	addwfc	INDF1,F		; "
	addfsr	FSR1,-1		; "
	addwfc	INDF1,F		; "
	addfsr	FSR1,7		;Point FSR1 to manufacturer info field
	moviw	0[FSR0]		;Copy the partition information for this device
	movwi	FSR1++		; into the first 8 bytes
	moviw	1[FSR0]		; "
	movwi	FSR1++		; "
	moviw	2[FSR0]		; "
	movwi	FSR1++		; "
	moviw	3[FSR0]		; "
	movwi	FSR1++		; "
	moviw	4[FSR0]		; "
	movwi	FSR1++		; "
	moviw	5[FSR0]		; "
	movwi	FSR1++		; "
	moviw	6[FSR0]		; "
	movwi	FSR1++		; "
	moviw	7[FSR0]		; "
	movwi	FSR1++		; "
	movlw	0x20		;Point FSR0 to remaining 40 bytes in bank 0
	movwf	FSR0L		; "
	movlw	40		;Copy 40 bytes to the manufacturer info field
	movwf	TEMP		; "
CmdSta0	moviw	FSR0++		; "
	movwi	FSR1++		; "
	decfsz	TEMP,F		; "
	bra	CmdSta0		; "
	movlw	FWVER_3		;Copy firmware version into last four bytes of
	movwi	FSR1++		; manufacturer info field and leave FSR1
	movlw	FWVER_2		; pointing to icon field
	movwi	FSR1++		; "
	movlw	FWVER_1		; "
	movwi	FSR1++		; "
	movlw	FWVER_0		; "
	movwi	FSR1++		; "
	movlw	high Icon | 0x80;Point FSR0 to the icon table
	movwf	FSR0H		; "
	movlw	low Icon	; "
	movwf	FSR0L		; "
	clrf	TEMP		;Copy 256 bytes to the icon field
CmdSta1	moviw	FSR0++		; "
	movwi	FSR1++		; "
	decfsz	TEMP,F		; "
	bra	CmdSta1		; "
	movlw	0x0A		;The credits
	movwi	FSR1++		; "
	movlw	'T'		; "
	movwi	FSR1++		; "
	movlw	'a'		; "
	movwi	FSR1++		; "
	movlw	's'		; "
	movwi	FSR1++		; "
	movlw	'h'		; "
	movwi	FSR1++		; "
	movlw	'T'		; "
	movwi	FSR1++		; "
	movlw	'w'		; "
	movwi	FSR1++		; "
	movlw	'e'		; "
	movwi	FSR1++		; "
	movlw	'n'		; "
	movwi	FSR1++		; "
	movlw	't'		; "
	movwi	FSR1++		; "
	movlw	'y'		; "
	movwi	FSR1++		; "
	call	CalcChecksum	;Calculate the checksum on our response
	movlp	high Transmit	;Initiate transmission; if it's aborted, we
	call	Transmit	; don't actually do anything differently
	return			;Done


;;; Subprograms ;;;

;Calculate the checksum of the block about to be transmitted, containing
; GROUPS * 7 bytes.  Trashes TEMP, TEMP2, and FSR0.
CalcChecksum
	movlw	0x20		;Point FSR0 at the top of the data buffer
	movwf	FSR0H		; "
	movlw	0x48		; "
	movwf	FSR0L		; "
	movf	GROUPS,W	;Copy the group count into TEMP
	movwf	TEMP		; "
	clrf	TEMP2		;Zero the running sum (mod 256)
	bra	CalcCh1		;Jump the first add so we sum one less than the
CalcCh0	moviw	FSR0++		; total bytes in the block and can write the
	addwf	TEMP2,F		; checksum to the very last byte position
CalcCh1	moviw	FSR0++		;Add the bytes in this group to the running sum
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	decfsz	TEMP,F		;Loop until we've summed the last group
	bra	CalcCh0		; "
	comf	TEMP2,W		;Two's complement the sum so the whole block
	addlw	1		; adds up to zero and write it to the block's
	movwi	FSR0++		; last byte
	return

;Calculate the checksum of the block just received, starting at the third byte
; of the data buffer and containing GROUPS * 7 bytes.  Trashes TEMP, TEMP2, and
; FSR0.
CheckChecksum
	movlw	0x20		;Point FSR0 to the third byte of the data
	movwf	FSR0H		; buffer
	movlw	0x4B		; "
	movwf	FSR0L		; "
	movf	GROUPS,W	;Copy the group count into TEMP
	movwf	TEMP		; "
	clrf	TEMP2		;Zero the running sum (mod 256)
CheckC0	moviw	FSR0++		;Add the next seven bytes to the running sum
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	moviw	FSR0++		; "
	addwf	TEMP2,F		; "
	decfsz	TEMP,F		;Loop until we've summed the last group
	bra	CheckC0		; "
	movf	TEMP2,F		;Set Z if block sum is 0 and checksum is good
	return

;Clear the data buffer for the response to a command according to the expected
; response length according to mac, setting GROUPS to the number of groups to
; be sent.  Trashes TEMP and FSR0.
ClearResponse
	movlw	0x20		;Point FSR0 at the top of the data buffer
	movwf	FSR0H		; "
	movlw	0x48		; "
	movwf	FSR0L		; "
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
	return

;Translate the address from a command into an address on the MMC card.  Sets
; RNFERR if (considering the block count) the command would address memory that
; is beyond the boundary of the drive.
TranslateAddr
	bcf	FLAGS,RNFERR	;Assume no error to start with
	movlw	0x20		;Point FSR0 at the appropriate struct for the
	movwf	FSR0H		; selected device
	clrf	FSR0L		; "
	btfsc	FLAGS,DEVSEL1	; "
	addfsr	FSR0,16		; "
	btfsc	FLAGS,DEVSEL0	; "
	addfsr	FSR0,8		; "
	decf	RC_BLKS,W	;Add the block count (less one because it's
	addwf	RC_ADRL,F	; one-relative) to the block address so we can
	movlw	0		; do a full boundary check
	addwfc	RC_ADRM,F	; "
	addwfc	RC_ADRH,F	; "
	moviw	3[FSR0]		;Compare the maximum block address that would
	subwf	RC_ADRL,W	; be accessed by this command to the LBA length
	moviw	2[FSR0]		; of the partition; if the maximum block
	subwfb	RC_ADRM,W	; address is greater than or equal to the LBA
	moviw	1[FSR0]		; length of the partition, this is an invalid
	subwfb	RC_ADRH,W	; access and must be rejected; set the record-
	btfsc	STATUS,C	; not-found flag
	bsf	FLAGS,RNFERR	; "
	decf	RC_BLKS,W	;Restore the block address as before
	subwf	RC_ADRL,F	; "
	movlw	0		; "
	subwfb	RC_ADRM,F	; "
	subwfb	RC_ADRH,F	; "
	btfsc	FLAGS,RNFERR	;If we had a record-not-found error, we're done
	return			; "
	moviw	7[FSR0]		;Add the block address to the start address for
	addwf	RC_ADRL,W	; the partition and store the result in the
	movwf	M_ADR0		; MMC address registers
	moviw	6[FSR0]		; "
	addwfc	RC_ADRM,W	; "
	movwf	M_ADR1		; "
	moviw	5[FSR0]		; "
	addwfc	RC_ADRH,W	; "
	movwf	M_ADR2		; "
	clrf	M_ADR3		; "
	moviw	4[FSR0]		; "
	addwfc	M_ADR3,F	; "
	return


;;; MMC Subprograms ;;;

;Initialize MMC card.  Sets M_FAIL on fail.  Trashes TEMP and TEMP2.
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
	bsf	M_FLAGS,M_CDVER	;Assume version 2.0+ to begin with
	clrf	TEMP		;Set retry counter to 0 (65536) for later use
	clrf	TEMP2		; "
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
	bra	MmcIni1		; "
	movf	M_ADR1,W	;If the command didn't error, but the lower 12
	andlw	B'00001111'	; bits of the R7 response are something besides
	xorlw	0x01		; 0x1AA, we're dealing with an unknown card, so
	btfss	STATUS,Z	; raise the fail flag and return to caller
	bsf	M_FLAGS,M_FAIL	; "
	movf	M_ADR0,W	; "
	xorlw	0xAA		; "
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	btfsc	M_FLAGS,M_FAIL	; "
	return			; "
MmcIni1	movlw	0x77		;Send command 55 (expect R1-type response),
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
	return			; so return the failure to caller
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
	return			; so return the failure to caller
	btfss	M_CMDN,0	;If it returned an 0x00 status, initialization
	bra	MmcIni2		; is finished
	DELAY	40		;If it returned an 0x01 status, delay for 120
	decfsz	TEMP,F		; cycles (15 us), decrement the retry counter,
	bra	MmcIni1		; and try again
	decfsz	TEMP2,F		; "
	bra	MmcIni1		; "
	bsf	M_FLAGS,M_FAIL	;If we've gone 65536 attempts and the card is
	return			; still not ready, report failure to caller
MmcIni2	movlw	0x7A		;Send command 58 (expect R3-type response) to
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
	return			; return the failure to caller
	bsf	M_FLAGS,M_BKADR	;If the card capacity status (CCS) bit of the
	btfsc	M_ADR3,6	; OCR is set, we're using block addressing, so
	bra	MmcIni4		; skip ahead
MmcIni3	bcf	M_FLAGS,M_BKADR	;We're dealing with byte, not block addressing
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
	return			; fail the init operation
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
	return			; fail the init operation
	bsf	PORTC,RC3	;Deassert !CS
	movf	M_FLAGS,W	;Set up the card info byte of the status block
	andlw	B'00000111'	; "
	movwf	CARDINF		; "
	return			;Congratulations, card is initialized!

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

;Read 512 bytes of data from MMC card into buffer pointed to by FSR0.  Sets
; M_FAIL on fail. Trashes TEMP, TEMP2, and FSR0.
MmcReadData
	bcf	M_FLAGS,M_FAIL	;Assume no failure to start with
	movlb	4		;Switch to the bank with the SSP registers
	clrf	TEMP		;Try 65536 times to get the data token
	clrf	TEMP2		; "
MmcRea1	movlw	0xFF		;Clock a byte out of the MMC card while keeping
	movwf	SSP1BUF		; MOSI high
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	SSP1BUF,W	;If we've received the data token, skip ahead
	xorlw	0xFE		; "
	btfsc	STATUS,Z	; "
	bra	MmcRea2		; "
	decfsz	TEMP,F		;If not, decrement the retry count and try
	bra	MmcRea1		; again
	decfsz	TEMP2,F		; "
	bra	MmcRea1		; "
	bsf	M_FLAGS,M_FAIL	;If we didn't get the data token after 65536
	movlb	0		; tries, give up and fail the operation
	return			; "
MmcRea2	clrf	TEMP		;Read 256 pairs of bytes
	clrf	CRC16H		;Start CRC16 registers at zero
	clrf	CRC16L		; "
MmcRea3	movlw	0xFF		;Clock the next data byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	movf	TEMP,F		;If this is the first byte, we don't have a
	btfsc	STATUS,Z	; previous byte to CRC, so skip ahead
	bra	MmcRea4		; "
	moviw	-1[FSR0]	;Update the CRC with the previous byte while
	xorwf	CRC16H,W	; the next one is being received
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
MmcRea4	btfss	SSP1STAT,BF	;Wait until the next byte is received
	bra	$-1		; "
	movf	SSP1BUF,W	;Move the byte we clocked out to the buffer and
	movwi	FSR0++		; increment the pointer
	movlw	0xFF		;Clock the next data byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	moviw	-1[FSR0]	;Update the CRC with the previous byte while
	xorwf	CRC16H,W	; the next one is being received
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
	btfss	SSP1STAT,BF	;Wait until the next byte is received
	bra	$-1		; "
	movf	SSP1BUF,W	;Move the byte we clocked out to the buffer and
	movwi	FSR0++		; increment the pointer
	decfsz	TEMP,F		;Decrement the counter and go back to read the
	bra	MmcRea3		; next byte pair, unless we're done
	movlw	0xFF		;Clock the high CRC byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	moviw	-1[FSR0]	;Update the CRC with the final data byte while
	xorwf	CRC16H,W	; the high CRC byte is being received
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
	movlp	0		;Restore PCLATH from all the CRC calculating
	btfss	SSP1STAT,BF	;Wait until the byte is received
	bra	$-1		; "
	movf	SSP1BUF,W	;If it doesn't match what we calculated, set
	xorwf	CRC16H,W	; the fail flag
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	movlw	0xFF		;Clock the low CRC byte out of the MMC card
	movwf	SSP1BUF		; while keeping MOSI high
	btfss	SSP1STAT,BF	;Wait until the byte is received
	bra	$-1		; "
	movf	SSP1BUF,W	;If it doesn't match what we calculated, set
	xorwf	CRC16L,W	; the fail flag
	btfss	STATUS,Z	; "
	bsf	M_FLAGS,M_FAIL	; "
	movlb	0		;Return to the default bank
	btfss	M_FLAGS,M_FAIL	;If the CRC check didn't fail, return now
	return			; "
	call	MmcStopOngoing	;If the CRC check did fail, stop any ongoing
	bsf	M_FLAGS,M_FAIL	; read (since the card doesn't know anything's
	return			; wrong) and then return failure

;Write 512 bytes of data to MMC card from buffer pointed to by FSR0.  Sets
; M_FAIL on fail. Trashes TEMP, TEMP2, and FSR0.
MmcWriteData
	bcf	M_FLAGS,M_FAIL	;Assume no failure to start with
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
	bcf	M_FLAGS,M_FAIL	;Assume no failure to start with
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
	bcf	M_FLAGS,M_FAIL	;Assume no failure to start with
	clrf	TEMP		;Start the CRC7 register out at 0
	movlp	high LutCrc7	;Point PCLATH to the CRC7 lookup table
	movlb	4		;Switch to the bank with the SSP registers
	movf	M_CMDN,W	;Clock out all six MMC buffer bytes as command,
	movwf	SSP1BUF		; calculating the CRC7 along the way
	xorwf	TEMP,W		; "
	callw			; "
	movwf	TEMP		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR3,W	; "
	movwf	SSP1BUF		; "
	xorwf	TEMP,W		; "
	callw			; "
	movwf	TEMP		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR2,W	; "
	movwf	SSP1BUF		; "
	xorwf	TEMP,W		; "
	callw			; "
	movwf	TEMP		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR1,W	; "
	movwf	SSP1BUF		; "
	xorwf	TEMP,W		; "
	callw			; "
	movwf	TEMP		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	M_ADR0,W	; "
	movwf	SSP1BUF		; "
	xorwf	TEMP,W		; "
	callw			; "
	movlp	0		; "
	movwf	TEMP		; "
	bsf	TEMP,0		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	movf	TEMP,W		; "
	movwf	SSP1BUF		; "
	btfss	SSP1STAT,BF	; "
	bra	$-1		; "
	;TODO for CMD12, it is necessary to clock and throw away a stuff byte?
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

	org	0x7F2

NoDevices
	btfsc	PORTA,RA5	;Spin until our !ENBL goes low
	bra	$-1		; "
	bcf	PORTA,RA0	;Assert !ENBL for next device in chain
	btfss	PORTA,RA5	;Spin until our !ENBL goes high
	bra	$-1		; "
	bsf	PORTA,RA0	;Deassert !ENBL for next device in chain
	bra	NoDevices

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
	btfss	DV1FLAG,DV_EXST	;If there is no device 1, select the next
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
	btfss	DV2FLAG,DV_EXST	;If there is no device 2, select the next
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
	btfss	DV3FLAG,DV_EXST	;If there is no device 3, select the next
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


;;; Icon ;;;

	org	0xF00

Icon
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x03
	retlw	0xFE
	retlw	0xE0
	retlw	0x00
	retlw	0x0E
	retlw	0x07
	retlw	0xB0
	retlw	0x00
	retlw	0x18
	retlw	0x00
	retlw	0x70
	retlw	0x00
	retlw	0x33
	retlw	0x40
	retlw	0xF0
	retlw	0x00
	retlw	0x2E
	retlw	0xDB
	retlw	0xE0
	retlw	0x00
	retlw	0x3C
	retlw	0x9B
	retlw	0xC0
	retlw	0x00
	retlw	0x1D
	retlw	0x9B
	retlw	0x00
	retlw	0x00
	retlw	0x09
	retlw	0x9B
	retlw	0x00
	retlw	0x00
	retlw	0x09
	retlw	0x9B
	retlw	0x00
	retlw	0x00
	retlw	0x08
	retlw	0x9B
	retlw	0x00
	retlw	0x00
	retlw	0x08
	retlw	0xBB
	retlw	0xE0
	retlw	0x00
	retlw	0x0C
	retlw	0x7B
	retlw	0xB0
	retlw	0x7F
	retlw	0xFC
	retlw	0x3A
	retlw	0x7E
	retlw	0x80
	retlw	0x06
	retlw	0x00
	retlw	0xF1
	retlw	0x80
	retlw	0x07
	retlw	0x83
	retlw	0xE1
	retlw	0x80
	retlw	0x03
	retlw	0xFF
	retlw	0xC1
	retlw	0x80
	retlw	0x00
	retlw	0xFF
	retlw	0x01
	retlw	0x80
	retlw	0x00
	retlw	0x00
	retlw	0x01
	retlw	0x8C
	retlw	0x00
	retlw	0x00
	retlw	0x01
	retlw	0x80
	retlw	0x00
	retlw	0x00
	retlw	0x01
	retlw	0x80
	retlw	0x00
	retlw	0x00
	retlw	0x01
	retlw	0x7F
	retlw	0xFF
	retlw	0xFF
	retlw	0xFE
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x00
	retlw	0x03
	retlw	0xFE
	retlw	0xE0
	retlw	0x00
	retlw	0x0F
	retlw	0xFF
	retlw	0xF0
	retlw	0x00
	retlw	0x1F
	retlw	0xFF
	retlw	0xF0
	retlw	0x00
	retlw	0x3F
	retlw	0xFF
	retlw	0xF0
	retlw	0x00
	retlw	0x3F
	retlw	0xFF
	retlw	0xE0
	retlw	0x00
	retlw	0x3F
	retlw	0xFF
	retlw	0xC0
	retlw	0x00
	retlw	0x1F
	retlw	0xFF
	retlw	0x00
	retlw	0x00
	retlw	0x0F
	retlw	0xFF
	retlw	0x00
	retlw	0x00
	retlw	0x0F
	retlw	0xFF
	retlw	0x00
	retlw	0x00
	retlw	0x0F
	retlw	0xFF
	retlw	0x00
	retlw	0x00
	retlw	0x0F
	retlw	0xFF
	retlw	0xE0
	retlw	0x00
	retlw	0x0F
	retlw	0xFF
	retlw	0xF0
	retlw	0x7F
	retlw	0xFF
	retlw	0xFF
	retlw	0xFE
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0xFF
	retlw	0x7F
	retlw	0xFF
	retlw	0xFF
	retlw	0xFE


;;; Transmitter Code ;;;

	org	0x1000

;Transmits GROUPS 7-byte groups of non-encoded data from the data buffer.  Sets
; ABORTED if a phase change caused the command to be aborted, in which case
; caller should return to idle state as soon as possible.  Trashes FSR0, TEMP,
; TEMP2, and GROUPS.
Transmit
	movlb	0		;Point BSR to 0 so we can access PORTA/PORTC
	movlw	0x20		;Point FSR0 at the top of the data buffer
	movwf	FSR0H		; "
	movlw	0x48		; "
	movwf	FSR0L		; "
	bcf	FLAGS,ABORTED	;Clear aborted flag to start
	decf	PORTA,W		;Check the current state
	xorlw	B'00000100'	;If it's 2, then mac is ready for us to request
	btfss	STATUS,Z	; to send; if it's anything else, signal an
	bra	XAbort		; abort to get us to the idle jump table ASAP
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
	bra	XAbort		; abort to get us to the idle jump table ASAP
	;fall through

Transmitter
	movlw	128		;Set up fractional delay counter (see below)
	movwf	TEMP		; "
	clrf	TEMP2		;Clear counter for sync byte loop
	movlp	high XFromBuffer;Select from-buffer as transmit loop
	bsf	PORTC,RC4	;Raise RD and then delay long enough to make
	DELAY	55		; sure that IWM is awaiting an MSB

XSyncByteLoop
	incf	TEMP2,F		;-2 Increment the sync bit counter; if it's
	btfsc	TEMP2,0		;-1  odd, falling edge on RD to send a 1 bit;
	bcf	PORTC,RC4	; 0  this way we send 0xAA as a sync byte
	movlw	87		;+1 (*1) Reckon the delay to slow this from a
	addwf	TEMP,F		;+2  2 us data cell to a 96/47 us data cell as
	btfsc	STATUS,C	;+3  16 cycles plus 16/47 of a cycle, which is
	bra	$+1		;+4  approximated here as 87/256 of a cycle,
	bsf	PORTC,RC4	;+5  and raise RD for the next falling edge
	decf	PORTA,W		;+6 (*2) Check the current state; if it is 0
	andlw	B'00111100'	;+7  (holdoff) or 1 (communication), continue
	btfss	STATUS,Z	;-8  transmitting; else, abort and get us back
	goto	XAbort		;-7  to idle loop ASAP
	movlw	0x40		;-6 If we've more sync bits to send, loop for
	btfss	TEMP2,3		;-5  the next, else set LSBs register for the
	bra	XSyncByteLoop	;-4(-3)  group we're about to send and jump
	movwf	TEMP2		;-3  into the appropriate send loop
	clrf	PCL		;-2 -1  "

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
	clrf	TEMP2		;Clear counter for sync byte loop
	goto	XSyncByteLoop	;Resume transmission after interrupted group

XAbort
	bsf	PORTC,RC4	;Deassert !HSHK if it was asserted
	bsf	FLAGS,ABORTED	;Raise the aborted flag
	movlp	0		;Reset PCLATH to where we're returning
	return

	org	0x1100

XFromBuffer
	bcf	PORTC,RC4	; 0 Start 7-to-8-encoded group; MSB always set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	INDF0,7		;-1 Falling edge on RD if bit 7 of this byte is
	bcf	PORTC,RC4	; 0  set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	INDF0,6		;-1 Falling edge on RD if bit 6 of this byte is
	bcf	PORTC,RC4	; 0  set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	INDF0,5		;-1 Falling edge on RD if bit 5 of this byte is
	bcf	PORTC,RC4	; 0  set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	INDF0,4		;-1 Falling edge on RD if bit 4 of this byte is
	bcf	PORTC,RC4	; 0  set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	INDF0,3		;-1 Falling edge on RD if bit 3 of this byte is
	bcf	PORTC,RC4	; 0  set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	INDF0,2		;-1 Falling edge on RD if bit 2 of this byte is
	bcf	PORTC,RC4	; 0  set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	INDF0,1		;-1 Falling edge on RD if bit 1 of this byte is
	bcf	PORTC,RC4	; 0  set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	btfsc	INDF0,0		;+6 Copy the LSB of this byte into the LSB
	bsf	TEMP2,7		;+7  register
	addfsr	FSR0,1		;-8 Increment pointer to transmit next byte
	nop			;-7
	nop			;-6
	nop			;-5
	lsrf	TEMP2,F		;-4 Shift the LSB register right by one; if a
	btfss	STATUS,C	;-3  one falls out, it's time to send the
	clrf	PCL		;-2(-1)  LSBs, otherwise loop again
	nop			;-1
	bcf	PORTC,RC4	; 0 Start of LSB IWM byte; MSB always set
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	TEMP2,6		;-1 Falling edge on RD if the 7th byte's LSB
	bcf	PORTC,RC4	; 0  was high
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	TEMP2,5		;-1 Falling edge on RD if the 6th byte's LSB
	bcf	PORTC,RC4	; 0  was high
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	TEMP2,4		;-1 Falling edge on RD if the 5th byte's LSB
	bcf	PORTC,RC4	; 0  was high
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	TEMP2,3		;-1 Falling edge on RD if the 4th byte's LSB
	bcf	PORTC,RC4	; 0  was high
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	TEMP2,2		;-1 Falling edge on RD if the 3rd byte's LSB
	bcf	PORTC,RC4	; 0  was high
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	TEMP2,1		;-1 Falling edge on RD if the 2nd byte's LSB
	bcf	PORTC,RC4	; 0  was high
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	decf	PORTA,W		;+6 (*2)
	andlw	B'00111100'	;+7  "
	btfss	STATUS,Z	;-8  "
	goto	XAbort		;-7  "
	decf	PORTA,W		;-6 (*2)
	andlw	B'00111100'	;-5  "
	btfss	STATUS,Z	;-4  "
	goto	XAbort		;-3  "
	nop			;-2
	btfsc	TEMP2,0		;-1 Falling edge on RD if the 1st byte's LSB
	bcf	PORTC,RC4	; 0  was high
	movlw	87		;+1 (*1)
	addwf	TEMP,F		;+2  "
	btfsc	STATUS,C	;+3  "
	bra	$+1		;+4  "
	bsf	PORTC,RC4	;+5  "
	movlw	0x40		;+6 Reset the LSBs register for the next
	movwf	TEMP2		;+7  group, if there is one
	decf	PORTA,W		;-8 Check the current state
	btfsc	STATUS,Z	;-7 If it's 0 (holdoff), then effect a suspend
	goto	XSuspend	;-6  now that we're done transmitting a group
	nop			;-5
	nop			;-4
	decfsz	GROUPS,F	;-3 We finished a group, decrement group count;
	clrf	PCL		;-2(-1)  if there are more, loop again,
	goto	XFinished	;-1  otherwise we're finished transmitting


;;; Receiver Code ;;;

	org	0x1800

RecvAbort
	bsf	PORTC,RC4	;Deassert !HSHK if it was asserted
	bsf	FLAGS,ABORTED	;Raise the aborted flag
	movlp	0		;Reset PCLATH to where we're returning
	return

;Receives data from the mac.  Sets ABORTED if a phase change caused the command
; to be aborted, in which case caller should return to idle state as soon as
; possible.  Overwrites data buffer with received data in the following way:
; first byte is sync byte, second byte is 0x80 + number of groups in command
; according to mac, third byte is 0x80 + number of groups in anticipated
; response, followed by a string of GROUPS seven-byte decoded groups.  Trashes
; FSR0, FSR1, TEMP, and TEMP2.
Receive
	movlb	0		;Point BSR to 0 so we can access PORTA/PORTC
	bcf	FLAGS,ABORTED	;Clear aborted flag to start
	movlw	0x20		;Point FSR0 at the top of the data buffer
	movwf	FSR0H		; "
	movlw	0x48		; "
	movwf	FSR0L		; "
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
	addfsr	FSR0,1		;125 cycles, 7.65 bit times
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
	addfsr	FSR0,1		;109 cycles, 6.67 bit times
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
	addfsr	FSR0,1		;093 cycles, 5.69 bit times
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
	addfsr	FSR0,1		;077 cycles, 4.71 bit times
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
	addfsr	FSR0,1		;061 cycles, 3.73 bit times
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
	addfsr	FSR0,1		;043 cycles, 2.63 bit times
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
	addfsr	FSR0,1		;027 cycles, 1.65 bit times
	goto	RcSt1		;028 cycles, 1.71 bit times

RcTo1_0	movlw	B'00000001'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	addfsr	FSR0,1		;007 cycles, 0.43 bit times
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
	addfsr	FSR0,1		;125 cycles, 7.65 bit times
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
	addfsr	FSR0,1		;109 cycles, 6.67 bit times
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
	addfsr	FSR0,1		;093 cycles, 5.69 bit times
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
	addfsr	FSR0,1		;077 cycles, 4.71 bit times
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
	addfsr	FSR0,1		;061 cycles, 3.73 bit times
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
	addfsr	FSR0,1		;043 cycles, 2.63 bit times
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
	addfsr	FSR0,1		;027 cycles, 1.65 bit times
	goto	RcSt0		;028 cycles, 1.71 bit times

RcTo0_0	movlw	B'00000001'	;003 cycles, 0.18 bit times
	iorwf	INDF0,F		;004 cycles, 0.24 bit times
	btfss	PORTA,RA1	;005 cycles, 0.31 bit times
	bcf	INDF0,7		;006 cycles, 0.37 bit times
	addfsr	FSR0,1		;007 cycles, 0.43 bit times
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
	movlw	0x4B		; the expected reply in groups plus 0x80)
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
	addfsr	FSR0,1		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_5		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the second byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_4		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the third byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_3		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the fourth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_2		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the fifth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_1		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the sixth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
	btfss	INDF0,7		;If the MSB of the next byte is clear, we went
	bra	RDSus_0		; into suspend here, switch loops
	lsrf	TEMP,F		;Rotate the LSB of the last byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
	decfsz	TEMP2,F		;Decrement the group count and loop if it's not
	bra	RDLoop		; yet zero
	return

RDSus_7	moviw	FSR0++		;Pick up the byte with the LSBs of the next 
	movwf	TEMP		; seven
RDSus_6	lsrf	TEMP,F		;Rotate the LSB of the first byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
RDSus_5	lsrf	TEMP,F		;Rotate the LSB of the second byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
RDSus_4	lsrf	TEMP,F		;Rotate the LSB of the third byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
RDSus_3	lsrf	TEMP,F		;Rotate the LSB of the fourth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
RDSus_2	lsrf	TEMP,F		;Rotate the LSB of the fifth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
RDSus_1	lsrf	TEMP,F		;Rotate the LSB of the sixth byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
RDSus_0	lsrf	TEMP,F		;Rotate the LSB of the last byte of the group
	rlf	INDF0,W		; into place
	movwi	FSR1++		;Write the restored byte to memory
	addfsr	FSR0,1		;Advance the input pointer
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
