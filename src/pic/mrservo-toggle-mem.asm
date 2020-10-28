;/*************************************************************************
;Title:    Slow Motion Servo Turnout Motor Controller
;Authors:  Michael Petersen <railfan@drgw.net>
;          Nathan Holmes <maverick@drgw.net>
;File:     mrservo.asm
;License:  GNU General Public License v3
;
;LICENSE:
;    Copyright (C) 2014 Nathan Holmes and Michael Petersen
;
;    This program is free software; you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation; either version 3 of the License, or
;    any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;    
;    You should have received a copy of the GNU General Public License along 
;    with this program. If not, see http://www.gnu.org/licenses/
;    
;*************************************************************************/
;
;  See http://www.ndholmes.com/pmwiki.php/Electronics/ServoMotor
;   or http://www.iascaled.com/info/MRServo
;   for more information and schematic
;
; Version Notes
; v1.2 -  5 Aug 2011 - Added aux_output_mode to set the outputs based on direction rather
;                       than being used for power and direction relays
; v1.1 - 30 Jul 2011 - Added ability to shut down the PWM a short time after the servo reaches
;                       full throw.  This way, the servo doesn't burn out and you don't have to
;                       have perfect calibration on the throw.  Also slowed down the throw and
;                       adjusted the limit constants to something more realistic.
; v1.0 - 15 Jun 2011 - First public release

#include p10f320.inc
	
	list P=PIC10F320, ST=ON, MM=ON, R=DEC, X=ON
	__CONFIG   _CP_OFF & _WDTE_ON & _MCLRE_OFF & _BOREN_ON & _FOSC_INTOSC      

; PIN CONFIGURATION
;  PORTA.3 (Pin 8) - Logic level turnout direction input
;  PORTA.2 (Pin 3) - Directional Relay
;  PORTA.1 (Pin 4) - Frog Power Isolation Relay
;  PORTA.0 (Pin 5) - Servo Pulse Output

; START USER CONFIGURATION SECTION

; servo_upperlimit is the maximum travel in one direction, expressed as a pulse 
;  of 1+(servo_upperlimit/250) milliseconds, or +((servo_upperlimit-128)/250) percent throw

; servo_lowerlimit is the maximum travel in the other direction, expressed as a pulse of 
;  of 1+(servo_lowerlimit/250) milliseconds, or -((128-servo_lowerlimit)/250) percent throw

; servo_halfway is normally halfway between upper and lower, but you could change it manually
;  if you had some bizarre reason to set where the midpoint of throw was other than that

; servo_ramprate is a constant defining how fast the servo will move from one end to the other
;  Complete throw will be (in seconds) = (servo_upperlimit - servo_lowerlimit) / (50 * servo_ramprate)

; frog_power_off_delay is the amount of time the frog power relay will remain off after a throw completes
;  The delay is approximately (in seconds) = (frog_power_off_delay / 50)

servo_upperlimit      equ 170
servo_lowerlimit	  equ 80
servo_halfway         equ (servo_upperlimit + servo_lowerlimit)/2
servo_ramprate		  equ 2
frog_power_off_delay  equ 25 

; Set stall time to 255 to make servo always actively driven
servo_stalltime		  equ 50
; Set aux_output_mode to:
;   0 to set one output high for each direction
;   1 to use the interlocking relays for direction and power
aux_output_mode       equ 1

debounce              equ 250
 
; END USER CONFIGURATION SECTION

fake_eep_base_l       equ  240  
fake_eep_base_h       equ  0
fake_eep_len          equ  16  

; Variable definitions	
	cblock 0x10
temp1
temp2
temp3
temp4
flash_dir
power_off_timer
hz50_counter
stall_counter
servo_pulsewidth
cntl        ; bit3 = CNTL input, bit1 = throw in progress, bit0 = lockout state change
debounce_count
	endc

; Begin application
	org	0x00
	movlw 0x50    ; Set clock to 4MHz to match the 10F200 version
	movwf OSCCON  ; It's easier than recalculating everything

start
	clrwdt
	clrf	PORTA
	movlw	0x08
	movfw	TRISA

	movlw	0x80
	movwf OPTION_REG

	movlw	servo_halfway 
	movwf	servo_pulsewidth

	movlw	servo_stalltime
	movwf	stall_counter

   call get_flash_state    ; Get flash state
   movf flash_dir, w       ; Move direction into working
   bsf cntl, 3
   btfss STATUS, Z
   bcf cntl, 3

main_loop
	clrwdt

read_input
	btfsc	PORTA,3
	goto	pwm_math
cntl_low
	; PORTA3 low
	btfsc	cntl,0
	goto	pwm_math  ; Locked out
	; PORTA3 low and not locked out
	movf	cntl,W
	xorlw	0x08  ; Flip CNTL
	movwf	cntl
	bsf		cntl,0  ; Lock
	bsf		cntl,1  ; Throw in progress
	movlw	debounce
	movwf	debounce_count

pwm_math
	; Is cntl high or low?  Move pulsewidth in the right direction at the right ramp rate
	btfsc	cntl,3
	goto	increment_pwm

decrement_pwm
	movlw	(servo_lowerlimit + servo_ramprate)
    subwf   servo_pulsewidth, w
	btfss	STATUS, C   ; If (servo_lowerlimit + servo_ramprate) <= servo_pulsewidth, subtract
	goto	set_lowerlimit
	movlw 	servo_ramprate
	subwf	servo_pulsewidth, f
	goto	pwm_math_done

set_lowerlimit
	movlw	servo_lowerlimit
	movwf	servo_pulsewidth
	bcf		cntl,1  ; Throw done
	goto	pwm_math_done

increment_pwm
	movlw	servo_upperlimit - servo_ramprate
	movwf	temp1
	movf	servo_pulsewidth, W
	subwf	temp1, W
	btfss	STATUS,C
	goto	set_upperlimit
	movlw	servo_ramprate
	addwf	servo_pulsewidth, F
	goto	pwm_math_done
set_upperlimit
	movlw	servo_upperlimit
	movwf	servo_pulsewidth
	bcf		cntl,1  ; Throw done
pwm_math_done

#if 0 == aux_output_mode

direction_output_logic
	movlw	servo_halfway
	subwf	servo_pulsewidth, w
	btfss	STATUS, C
	goto	direction_output_off
direction_output_on
	bsf		PORTA, 2
	bcf		PORTA, 1
	goto	direction_output_logic_done
direction_output_off
	bcf		PORTA, 2
	bsf		PORTA, 1
direction_output_logic_done

#else

direction_relay_logic
	movlw	servo_halfway
	subwf	servo_pulsewidth, w
	btfss	STATUS, C
	goto	direction_relay_off
direction_relay_on
	bsf		PORTA, 2
	goto	direction_relay_logic_done
direction_relay_off
	bcf		PORTA, 2
direction_relay_logic_done

frog_relay_logic
	movlw	servo_upperlimit
	subwf	servo_pulsewidth, w
	btfsc	STATUS, Z
	goto	frog_relay_countdown
	
	movlw	servo_lowerlimit
	subwf	servo_pulsewidth, w
	btfsc	STATUS, Z
	goto	frog_relay_countdown
	
	movlw	frog_power_off_delay
	movwf	power_off_timer
	bcf		PORTA, 1
	goto frog_relay_logic_done

frog_relay_countdown
	movf	power_off_timer, w
	btfsc	STATUS, Z
	goto	frog_power_on
	decf	power_off_timer, f
	goto	frog_relay_logic_done

frog_power_on
	bsf		PORTA, 1

frog_relay_logic_done

#endif

#if 255 != servo_stalltime 
; Setup where servo shuts down after a given stall time

stall_timer_logic

	movlw	servo_upperlimit
	subwf	servo_pulsewidth, w
	btfsc	STATUS, Z
	goto	stall_countdown
	
	movlw	servo_lowerlimit
	subwf	servo_pulsewidth, w
	btfsc	STATUS, Z
	goto	stall_countdown

	; If we're here, the servo is in motion, keep resetting the stall counter
	movlw	servo_stalltime
	movwf	stall_counter
	goto	stall_timer_logic_done

stall_countdown
	movf	stall_counter, f
	btfss	STATUS, Z
	decf	stall_counter, f	

stall_timer_logic_done

#endif


start_delay
; 50 Hz code - roughly
	movlw	0x04
	movwf	temp3
delay_5ms
			;4998 cycles
	movlw	0x9B
	movwf	temp1
	movlw	0x04
	movwf	temp2
delay_5ms_inner
	decfsz	temp1, f
	goto	$+2
	decfsz	temp2, f
	goto	delay_5ms_inner

	clrwdt

	btfsc	cntl,1
	goto	debounce_done
	; Throw done
	decfsz	debounce_count, f
	goto	debounce_done
	; Debounce timeout expired after throw complete
	bcf		cntl,0  ; Unlock

debounce_done

	decfsz  temp3, f
	goto	delay_5ms

	incf hz50_counter, f

	movf	stall_counter, f
	btfsc	STATUS, Z
	goto	main_loop

; Send base (1 ms) servo pulse
	bsf		PORTA,0
; 1 ms code
			;998 cycles
	movlw	0xC7
	movwf	temp1
	movlw	0x01
	movwf	temp2
delay_1ms_inner
	decfsz	temp1, f
	goto	$+2
	decfsz	temp2, f
	goto	delay_1ms_inner

; Now send the 0-1 additional ms needed to move the servo
; Get the variable pulsewidth - each count is worth 1/250th of a ms, or 4 instructions
	movf	servo_pulsewidth, w

; Test if variable PWM is zero
; If you try to loop on it, you'll underflow and get a full pulse
;  and a twitchy servo going full range
	btfsc	STATUS, Z
	goto	servo_off
; In non-zero cases, move it into a loop counter and get going
	movwf   temp1
delay_pwm_loop
	nop
	decfsz	temp1, f
	goto delay_pwm_loop

servo_off
	bcf		PORTA,0

	goto main_loop


get_flash_state
   movlw    fake_eep_len
   movwf    temp2

   movlw 	fake_eep_base_h
   movwf    PMADRH
   movlw 	fake_eep_base_l
   movwf    temp1

read_flash_value
   movwf    PMADRL
	; Enable flash read mode
   bcf      PMCON1, CFGS
   bsf      PMCON1, RD
	nop  ; Required by read algorithm
	nop  ; Required by read algorithm
	
	movf     PMDATL, w          ; Get the data value
   btfss    STATUS, Z          ; If it's zero (all burned down), we don't want this one, keep looking
   goto readout                ; We've got a non-zero segment, read it
   decfsz   temp2, f           ; Decrement address length counter
	goto $+2                    ; If not zero, move to the next byte
   goto readout                ; We've hit the maximum length of segment, just read it

read_flash_inc_address
   incf     temp1, f           ; increment low address
   movf     temp1, w           ; load new low address into working and pop around again
   goto read_flash_value
   
readout
   movlw   8
   movwf   temp4
   movf    PMDATL, w
   movwf   temp3
readout_1
   rrf     temp3, f
   btfss   STATUS, C
   goto readout_foundit
   decfsz  temp4, f
   goto readout_foundit
   goto readout_1
      
readout_foundit
   movf    temp4, w
   movwf   flash_dir
   movlw   0x01
   andwf   flash_dir, f
   return

; Coming back, the temps are in the following states:
;  temp1 - low value of program address for active byte
;  temp2 - number of bytes remaining in block unused
;  temp4 - bit number to burn down


erase_block
   movlw 	fake_eep_base_h
   movwf    PMADRH
   movlw 	fake_eep_base_l
   movwf    PMADRL

	bcf PMCON1, CFGS
	bsf PMCON1, FREE
	bsf PMCON1, WREN
	movlw 0x55
	movwf PMCON2
	movlw 0xAA
	movwf PMCON2
	bsf PMCON1, WR
	nop
	nop
	bcf PMCON1, WREN
	return

write_flash_state
	call read_flash_value    ; Read our current state
	rlf  flash_dir, f
	rlf  flash_dir, f
	rlf  flash_dir, f ; Get flash_dir value in bit 3, not bit 0
	movf  ctrl, w
	andlw 0x08
	xorwf flash_dir, w
	btfsc STATUS, Z    ; If we're trying to set the same thing we read, bail
	return
	
	movf temp2, w
	btfsc STATUS, Z    ; If temp2 is zero, then we have no bytes remaining and need to erase the block
	call erase_block

	

	
	return
	
	
   

; General flash algorithm...
; Even number of 0 bits, throw one way
; Odd number of 0 bits, throw the other way
; Load base address
; Loop until byte isn't all 0
; Determine first 1
; Mask together, burn back to add a 0
; erase backwards


end


