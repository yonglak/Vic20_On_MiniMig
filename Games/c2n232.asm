;; Copyright © 2001 Marko Mäkelä
;;
;;     This program is free software; you can redistribute it and/or modify
;;     it under the terms of the GNU General Public License as published by
;;     the Free Software Foundation; either version 2 of the License, or
;;     (at your option) any later version.
;;
;;     This program is distributed in the hope that it will be useful,
;;     but WITHOUT ANY WARRANTY; without even the implied warranty of
;;     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;     GNU General Public License for more details.
;;
;;     You should have received a copy of the GNU General Public License
;;     along with this program; if not, write to the Free Software
;;     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

;;; This program code can be compiled with avra(1), the Atmel AVR assembler
;;; by Jon Anders Haugum <jonah@omegav.ntnu.no>, and probably with AVRASM
;;; by Atmel.

.device AT90S2313
	
	.equ SREG=$3F		; Status Register
	.equ SPL=$3D		; Stack Pointer Low
	.equ GIMSK=$3B		; General Interrupt MaSK register
	.equ GIFR=$3A		; General Interrupt Flag Register
	.equ TIMSK=$39		; Timer/Counter Interrupt MaSK register
	.equ TIFR=$38		; Timer/Counter Interrupt Flag register
	.equ MCUCR=$35		; MCU general Control Register
	.equ TCCR0=$33		; Timer/Counter 0 Control Register
	.equ TCNT0=$32		; Timer/Counter 0 (8-bit)
	.equ TCCR1A=$2F		; Timer/Counter 1 Control Register A
	.equ TCCR1B=$2E		; Timer/Counter 1 Control Register B
	.equ TCNT1H=$2D		; Timer/Counter 1 High Byte
	.equ TCNT1L=$2C		; Timer/Counter 1 Low Byte
	.equ OCR1AH=$2B		; Output Compare Register 1 High Byte
	.equ OCR1AL=$2A		; Output Compare Register 1 Low Byte
	.equ ICR1H=$25		; T/C 1 Input Capture Register High Byte
	.equ ICR1L=$24		; T/C 1 Input Capture Register Low Byte
	.equ WDTCR=$21		; Watchdog Timer Control Register
	.equ EEAR=$1E		; EEPROM Address Register
	.equ EEDR=$1D		; EEPROM Data Register
	.equ EECR=$1C		; EEPROM Control Register
	.equ PORTB=$18		; Data Register, Port B
	.equ DDRB=$17		; Data Direction Register, Port B
	.equ PINB=$16		; Input Pins, Port B
	.equ PORTD=$12		; Data Register, Port D
	.equ DDRD=$11		; Data Direction Register, Port D
	.equ PIND=$10		; Input Pins, Port D
	.equ UDR=$0C		; UART I/O Data Register
	.equ USR=$0B		; UART Status Register
	.equ UCR=$0A		; UART Control Register
	.equ UBRR=$09		; UART Baud Rate Register
	.equ ACSR=$08		; Analog Comparator Control and Status Register

	.equ	BAUDRATE=12	; 38400 bps at 8 MHz

	.def	PARAM0=r0	; first parameter
	.def	PARAM1=r1	; second parameter
	.def	PARAM2=r2	; third parameter
	.def	PARAM3=r3	; fourth parameter

	.def	PFLO=r2		; recv: least significant byte of pulse fall
	.def	PFHI=r4		; recv: most significant byte of pulse fall
	.def	PMIN=r5		; minimum pulse width (recv and send)
	.def	PDIFF=r6	; maximum-minimum (recv and send)
	.def	PCMP=r7		; recv: binary search variable
	.def	PSMAX=r8	; recv: binary search maximum
	.def	PSCNT=r8	; send: bit count
	.def	PDATA=r9	; recv/send: current data octet

	.def	MAXBUF=r14	; upper limit for receive buffer
	.def	RXCOUNT=r15	; number of octets to receive from UART

	;; r16 and ZH are received for interrupts
	;; ZL is for receiving UART communications

	.def	YL	=r28	; transmit pointer
	.def	YH	=r29	; constantly zero
	.def	ZL	=r30	; receive buffer pointer
	.def	ZH	=r31	; temporary register used in interrupts

	.dseg
rxbuf:	.byte 124		; UART raw data receive buffer
rxbufe:
stack:	.byte 4
stacke:
	.cseg
	rjmp reset
	rjmp error		; INT0 (external interrupt 0, serial ATN)
	rjmp error		; INT1 (external interrupt 1, serial CLK)
	rjmp icp1		; ICP1 (input capture 1, cassette WRITE)
	rjmp oc1		; OC1 (output compare 1, cassette READ)
	rjmp error		; OVF1 (timer 1 overflow)
	rjmp error		; OVF0 (timer 0 overflow)
	rjmp rxc		; RXC (UART receive complete)
	rjmp error		; DRE (UART data register empty)
	rjmp error		; TXC (UART transmit complete)
;	rjmp error		; ACI (analog comparator) [fall through]

	;; error: go to the RESET sequence
error:
	;; RESET handler
reset:	cli
	ldi r16, stacke-1
	out SPL, r16
	;; I/O port initialization
	clr r16
	out DDRD, r16
	ldi r16, $89
	out PORTB, r16		; SENSE=high; READ=high; CTS=high (stop rx)
	out DDRB, r16
	;; disable the analog comparator
	ldi r16, $80
	out ACSR, r16
	;; initialize the UART
	ldi r16, BAUDRATE
	out UBRR, r16
	;; enable idle mode sleep
	ldi r16, $20
	out MCUCR, r16
	clr ZL
	clr YH
	clr PMIN
	clr PDIFF
	clr RXCOUNT		; no data to receive
	ldi r16, (rxbufe-rxbuf)*3/4
  	mov MAXBUF, r16		; safety margin for RX buffer overflow

	;; idle mode
idle:	out TIMSK, YH		; disable ICP and OC interrupts
	sbi PORTB, 0		; SENSE=high
idle_common:
	ldi r16, $98
	out UCR, r16		; enable RX and TX; enable RXC interrupts
	sbis PORTB, 7
	rjmp idle_dc1_done	; jump if receiving was previously enabled
	ldi r16, $11		; DC1, ctrl-q, permission to send
	sbis USR, 5
	rjmp PC-1		; wait for UART Data Register Empty (DRE)
	out UDR, r16
	cbi PORTB, 7		; drop CTS (start receiving)
idle_dc1_done:
	sei			; enable interrupts
	;; idle loop
	sleep
	rjmp PC-1

	;; pulse width measurement mode
pulse:	ser r16
	out OCR1AH, YH		; set the output compare value
	out OCR1AL, r16		; (16 bits, 0x00ff)
	cbi PORTB, 0		; SENSE=low
	clr PARAM0		; select pulse width measurement mode for ICP
	ldi r16, $48
	out TIFR, r16		; clear pending ICP and OC interrupts
	out TIMSK, r16		; enable ICP and OC interrupts
	ldi r16, $cb		; activate ICP on rising edge, CK/64
	out TCCR1B, r16
	rjmp idle_common

	;; wait for parameters
.macro waitparams
	sei			; enable interrupts
	sleep			; wait for interrupt
	cli			; disable interrupts
	tst RXCOUNT
	brne PC-4		; wait until all parameters have been received
.endmacro

	;; save data
save:	waitparams		; enable interrupts and wait for the parameters
	out OCR1AH, YH		; set the output compare value
	out OCR1AL, PARAM2	; long pulse width
	cbi PORTB, 0		; SENSE=low
	ldi r16, $48
	out TIFR, r16		; clear pending ICP and OC interrupts
	out TIMSK, r16		; enable ICP and OC interrupts
	ldi r16, $cb		; activate ICP on rising edge, CK/64
	out TCCR1B, r16
	rjmp idle_common	; go to idle loop

	;; load data
load:	waitparams		; enable interrupts and wait for the parameters
	out OCR1AH, YH		; set the output compare value
	out OCR1AL, YH		; (16 bits)
	ldi ZL, rxbuf		; load the buffer pointers
	mov YL, ZL
	cbi PORTB, 0		; SENSE=low
	ldi r16, $40
	out TIFR, r16		; clear pending OC interrupts
	out TIMSK, r16		; enable OC interrupts
	rjmp idle_common	; go to idle loop

	;; calibrate the receiver (read an $f0 byte and then actual data)
recvcal:
	clr PMIN
	clr PDIFF
	;; fall through
	;; receive data from the cassette interface
recv:	waitparams		; enable interrupts and wait for the parameters
	ser r16
	out OCR1AH, r16		; set the output compare value
	out OCR1AL, r16		; (16 bits, 0xffff)
	ldi r16, $08
	out TIFR, r16		; clear pending ICP interrupts
	out TIMSK, r16		; enable ICP interrupts
	ldi r16, $09		; activate ICP on falling edge, CK/1
	out TCCR1B, r16
	sei			; enable interrupts
	cbi PORTB, 3		; drop READ to initiate transfer
	rjmp idle_common

	;; send data to the cassette interface
send:	waitparams
	ldi r16, $02		; activate ICP on falling edge, no CTC1, CK/8
	out TCCR1B, r16
	com RXCOUNT		; set RXCOUNT:7, a flag for rxc interrupt
	mov PSCNT, RXCOUNT	; negative bit count (start sending a new byte)
	ldi ZL, rxbuf		; load the buffer pointers
	mov YL, ZL
	rjmp idle_common

	;; UART receive complete
rxc:	sbrs RXCOUNT, 7
	rjmp rxc_nosend
	;; the custom send mode is active
	cbi UCR, 7		; disable RXC interrupts
	sbrs PSCNT, 7		; skip "sei" if no icp1 transfer is pending
	sei			; re-enable interrupts to reduce icp1 latency
  	in r16, UDR		; read the character
	in ZH, USR
	sbrc ZH, 4
	rjmp rxc_error		; framing error
	sbrc ZH, 3
	rjmp rxc_error		; overrun
	cli
	in ZH, TIMSK		; see if the sender has been stopped
	cpse ZH, YH
	rjmp rxc_enqueue_sei	; sender is going
	;; the sender has been stopped: restart it
	rcall rxc_enqueue_sei	; enqueue the character
	cli
	ldi ZH, $08
	out TIFR, ZH		; clear pending ICP interrupts
	out TIMSK, ZH		; enable ICP interrupts
	sbic PIND, 6
	reti			; return if cassette write=high
	rjmp icp1send0		; otherwise start a new transfer

rxc_enqueue_sei:
	sei
	;; enqueue a character
rxc_enqueue:
	mov ZH, YL		; dequeuing pointer
	sub ZH, ZL		; enqueuing pointer
	dec ZH
	sbrc ZH, 7
	subi ZH, rxbuf-rxbufe	; reduce the difference modulo the buffer
	cpse ZH, YH
	rjmp rxc_novf
	rjmp rxc_error		; buffer overflow
rxc_novf:
	cpse ZH, MAXBUF
	rjmp rxc_st		; buffer not 75% full; enqueue directly
	ldi ZH, $13		; DC3, ctrl-s, request to stop sending
	out UDR, ZH
	sbi PORTB, 7		; raise CTS (stop receiving)
	rjmp rxc_st		; enqueue the data

	;; not a custom send character
rxc_nosend:
  	in r16, UDR
	in ZH, USR
	andi ZH, $18		; check for overrun or framing error
	brne rxc_error
	cpi ZL, rxbuf
	brsh rxc_pulse
	;; received a command or a parameter
	cpse RXCOUNT, YH	; number of bytes to receive
	rjmp rxc_param
	rjmp rxc_cmd		; no data to receive -> get a command
rxc_param:
	dec RXCOUNT
rxc_st:	clr ZH			; store the character and exit from interrupt
	st Z+, r16
	ldi ZH, rxbufe
	ldi r16, $98
	cli			; disable interrupts
	out UCR, r16		; enable RXC interrupts
	cpse ZL, ZH		; wrap the buffer from top to bottom if needed
	reti
	ldi ZL, rxbuf
	reti

	;; data byte for the cassette load emulation
rxc_pulse:
	subi r16, $41
	brlo rxc_error		; not a pulse character (smaller than 'A')
	cpi r16, $04
	brsh rxc_error		; not a pulse character (greater than 'D')
	mov YH, ZL
	mov ZL, r16
	ld r16, Z
	mov ZL, YH
	clr YH
	cpse YL, ZL
	rjmp rxc_enqueue
	;; the buffer was empty
	in ZH, TCCR1B
	andi ZH, 7
	brne rxc_enqueue	; the timer was running, do not start it
	;; start the timer for pulse stream output
	ldi ZH, $80
	out TCCR1A, ZH		; clear OC1 on compare match
	ldi ZH, $0b
	out TCCR1B, ZH		; count pulses of CK/64, clear on compare match
	clr ZH
	out OCR1AH, ZH
	out OCR1AL, PARAM0	; time base for pause
	rjmp rxc_enqueue

	;; process a command
rxc_cmd:
	cpi r16, 7
	brlo rxc_cmdok
rxc_error:			; communication error: send a NUL character
	cli			; disable interrupts
	clr r16
	out UDR, r16		; send a NUL character
	clr PMIN		; clear the calibrated pulse widths
	clr PDIFF
	;; fall through with command = 0 (idle mode)
rxc_cmdok:
	sbi PORTB, 3		; raise READ
	clr PDATA
	clr YH
	out TIMSK, YH		; disable timer interrupts
	out TCCR0, YH		; timer 0 stopped
	out TCCR1A, YH		; timer 1 disconnected from OC1 output
	out TCCR1B, YH		; timer 1 stopped
	out TCNT1H, YH		; clear timer 1 counter
	out TCNT1L, YH		; (16 bits)
	ldi ZH, stacke-1
	out SPL, ZH		; restore the stack pointer

	;; echo the command and branch according to it
	mov ZH, r16
	ori ZH, $30
	out UDR, ZH

	clr ZL			; receive the parameters to the registers
	cpi r16, 4
	brsh cmd4_7
	;; 0 to 3
	cpi r16, 2
	brsh cmd2_3
	;; 0 or 1
	clr RXCOUNT		; no parameters
	sbrs r16, 0
	rjmp idle		; command 0
	rjmp pulse		; command 1
cmd2_3:	;; 2 or 3
	ldi ZH, 3
	mov RXCOUNT, ZH		; 3 parameters for save
	sbrs r16, 0
	rjmp save		; command 2
	inc RXCOUNT		; 4 parameters for load
	rjmp load		; command 3
cmd4_7:	;; 4 to 7
	ldi ZH, 2
	mov RXCOUNT, ZH		; 2 parameters
	cpi r16, 6
	brsh cmd6
	;; 4 or 5
	sbi PORTB, 0		; SENSE=high
	sbrs r16, 0
	rjmp send		; command 4
	rjmp recv		; command 5
cmd6:	;; 6
	rjmp recvcal

	;; input capture for custom receive function
icp1recv:
	sbrc PARAM3, 6
	rjmp icp1recvhi
	in PFLO, ICR1L		; got a falling edge: sample the time
	in PFHI, ICR1H
	sbi PORTB, 3		; raise READ
	ldi r16, $49
	out TCCR1B, r16		; activate ICP on rising edge, CK/1
	reti
	;; got the maximum pulse width in PARAM3
icp1recvmax:
	sub PARAM3, PMIN
	brlo rxc_error
	mov ZL, PARAM3		; ZL is always 0 in this function
	swap ZL			; divide by 16
	andi ZL, $0f
	breq rxc_error		; not enough difference between pulse widths
	inc PARAM3
	mov PDIFF, PARAM3
	rjmp icp1recvdone	; done calibrating
	;; got a rising edge (measure the pulse width)
icp1recvhi:
	in PARAM3, ICR1L
	in r16, ICR1H
	sub PARAM3, PFLO	; calculate the pulse width
	sbc r16, PFHI
	lsr r16
	ror PARAM3
	lsr r16
	ror PARAM3		; PARAM3 scaled to CK/4 sample rate
	tst PDIFF
	brne icp1recvd		; already calibrated -> receive it
	tst PMIN
	brne icp1recvmax
	;; got the minimum pulse width in PARAM3
	mov PMIN, PARAM3	; store the minimum pulse width
	rjmp icp1recvdone
icp1recvd:
	;; quantize a data pulse to a nibble (in ZL)
	ldi ZL, $f0
	dec PARAM3		; allow some jitter
	sub PARAM3, PMIN
	brlo icp1recvn
	ldi ZH, 16		; number of alternatives
	mov PCMP, PDIFF		; indirectly set the maximum value (PSMAX)
	clr PARAM2		; minimum value

	;; match the pulse in a binary search
icp1recvdl:
	dec PCMP
	mov PSMAX, PCMP		; alter the upper bound
icp1recvdb:
	lsr ZH
	breq icp1recvn		; finished the search
	add PCMP, PARAM2
	ror PCMP		; PCMP = (PSMAX + PARAM2) / 2
	cp PARAM3, PCMP
	brlo icp1recvdl
	;; at least that much: add the decision value
	add ZL, ZH
	inc PCMP
	mov PARAM2, PCMP	; alter the lower bound
	mov PCMP, PSMAX
	rjmp icp1recvdb

	;; received a nibble in ZL ($f0..$ff)
icp1recvn:
	tst PDATA
	brne icp1recvb
	mov PDATA, ZL
	rjmp icp1recvdone
icp1recvb:
	swap ZL
	and PDATA, ZL
	sbis USR, 5
	rjmp PC-1		; wait for UART Data Register Empty (DRE)
	out UDR, PDATA		; output the data
	clr PDATA

	sec
	sbc PARAM0, PDATA	; decrement the number of bytes to receive
	sbc PARAM1, PDATA
	brcs icp1finish		; exit if no more bytes to receive

icp1recvdone:
	clr ZL
	cbi PORTB, 3		; drop READ to initiate next transfer
	ldi r16, $09
	out TCCR1B, r16		; activate ICP on falling edge, CK/1
	reti

	;; finished sending all bytes
icp1finishsend:
	ldi ZL, $40		; send '@' as an "end of transfer" flag
	out UDR, ZL
	;; finished receiving or sending all bytes
icp1finish:
	sbi PORTB, 3		; raise READ
	clr PDATA
	clr RXCOUNT
	ldi ZL, stacke-1
	out SPL, ZL		; restore the stack pointer
	clr ZL
	clr YH
	out TIMSK, YH		; disable ICP and OC interrupts
	out TCCR1A, YH		; timer 1 disconnected from OC1 output
	out TCCR1B, YH		; timer 1 stopped
	out TCNT1H, YH		; clear timer 1 counter
	out TCNT1L, YH		; (16 bits)
	rjmp idle

	;; output compare: stop the timer
oc1:	in r16, TCCR1A
	sbrc r16, 7
	rjmp oc1_load		; 'load' operation: load a pulse
	;; stop the timer and exit the interrupt
oc1_stop:
	in r16, TCCR1B
	andi r16, $f8
	out TCCR1B, r16
	reti

	;; input capture (cassette write capture)
icp1:	in PARAM3, TCCR1B	; PARAM3 is otherwise unused during these ops
	sbrc PARAM3, 3
	rjmp icp1nosend
	;; input capture (custom send function)
	sbrc PSCNT, 7
	rjmp icp1send0		; set up next data byte (PSCNT was negative)
	sbi PORTB, 3		; raise READ (1+2+2+2=7 cycles after request)
	ldi YH, $40
	eor PARAM3, YH
	out TCCR1B, PARAM3	; trigger on the opposite edge of WRITE
	clr YH
	dec PSCNT		; decrement the bit count
	brmi icp1send8		; all done (all bits sent)
	sbrs PDATA, 0		; read the data bit
	cbi PORTB, 3		; lower READ (7+9 cycles after req)
	lsr PDATA		; shift the data register
	reti			; return from interrupt
	;; all bits sent: decrement and compare the byte counter
icp1send8:
	sec
	sbc PARAM0, YH		; decrement the number of bytes to send
	sbc PARAM1, YH
	brcs icp1finishsend	; all bytes sent: switch to idle mode
	reti

	;; prepare for sending a byte
icp1send0:
	cpse YL, ZL		; buffer empty?
	rjmp icp1send_nonempty
	out TIMSK, YH		; no more data -> disable input capture
	reti
icp1send_nonempty:
	ld PDATA, Y+		; load a data byte
	cpi YL, rxbufe
	brne icp1send_nowrap
	ldi YL, rxbuf		; wrap the buffer pointer
icp1send_nowrap:
	mov YH, YL		; dequeuing pointer
	sub YH, ZL		; enqueuing pointer
	dec YH
	sbrc YH, 7
	subi YH, rxbuf-rxbufe	; reduce the difference modulo the buffer
	cpi YH, (rxbufe-rxbuf)*3/4
	brlo icp1send_nodc1	; buffer more than 25% full
	sbis PORTB, 7
	rjmp icp1send_nodc1	; exit if receiving was previously enabled
	ldi YH, $11		; DC1, ctrl-q, permission to send
	out UDR, YH
	cbi PORTB, 7		; drop CTS (start receiving)
icp1send_nodc1:
	cbi PORTB, 3		; lower READ to initiate the transfer
	ldi YH, $42		; activate ICP on rising edge, no CTC1, CK/8
	out TCCR1B, YH
	ldi YH, 8
	mov PSCNT, YH		; initialize the bit counter
	clr YH
	reti

	;; input capture (cassette write; other than the custom send function)
icp1nosend:
	sbrs PARAM3, 7
	rjmp icp1recv
	out TCNT1H, YH		; clear timer 1 counter
	out TCNT1L, YH		; (16 bits)
	ldi r16, $cb		; activate ICP on rising edge, ICNC, CK/64
	out TCCR1B, r16
	in r16, ICR1L		; read the input
	tst PARAM0
	breq icp1pw		; pulse width measurement
	;; save mode (quantize the pulse widths)
	mov ZH, r16
	ldi r16, $41		; use signals 'A','B','C','D'
	tst ZH
	breq icp1pw		; pause detected
	inc r16
	cp ZH, PARAM0
	brlo icp1pw		; short pulse
	inc r16
	cp ZH, PARAM1
	brlo icp1pw		; medium pulse
	inc r16			; long pulse
	;; fall through
icp1pw:	sbis USR, 5		; pulse width measurement
	rjmp PC-1		; wait for UART Data Register Empty (DRE)
	out UDR, r16		; send out the pulse width
	reti

	;; output compare for load operation: get next pulse
oc1_load:
	sbrs r16, 6
	rjmp oc1_next
	ldi r16, $80
	out TCCR1A, r16
	reti
oc1_next:
	cpse YL, ZL		; buffer empty?
	rjmp oc1_nonempty
	rjmp oc1_stop		; no more data -> stop the timer
oc1_nonempty:
	ldi r16, $c0
	out TCCR1A, r16
	ld r16, Y+
	out OCR1AH, YH		; always zero
	out OCR1AL, r16		; write the timer
	cpi YL, rxbufe
	brne oc1_next_nowrap
	ldi YL, rxbuf		; wrap the buffer pointer
oc1_next_nowrap:
	mov ZH, YL		; dequeuing pointer
	sub ZH, ZL		; enqueuing pointer
	dec ZH
	sbrc ZH, 7
	subi ZH, rxbuf-rxbufe	; reduce the difference modulo the buffer
	cpi ZH, (rxbufe-rxbuf)*3/4
	brlo oc1_ret		; buffer more than 25% full
	sbis PORTB, 7
	reti			; exit if receiving was previously enabled
	ldi r16, $11		; DC1, ctrl-q, permission to send
	out UDR, r16
	cbi PORTB, 7		; drop CTS (start receiving)
oc1_ret:
	reti

; Local variables:
; compile-command: "avra c2n232.asm && uisp -dlpt=0x3bc -dprog=dapa --erase --upload if=c2n232.hex --verify"
; End:
