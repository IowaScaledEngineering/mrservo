;/*************************************************************************
;Title:    Slow Motion Servo Turnout Motor Controller
;Authors:  Michael Petersen <railfan@drgw.net>
;          Nathan Holmes <maverick@drgw.net>
;File:     mrservo.asm
;License:  GNU General Public License v3
;
;LICENSE:
;    Copyright (C) 2012 Nathan Holmes and Michael Petersen
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

#include P10F200.INC
	
	list P=PIC10F200, ST=ON, MM=ON, R=DEC, X=ON
	__CONFIG   _CP_OFF & _WDT_ON & _MCLRE_OFF            

; PIN CONFIGURATION
;  GPIO.3 (Pin 8) - Logic level turnout direction input
;  GPIO.2 (Pin 3) - Directional Relay
;  GPIO.1 (Pin 4) - Frog Power Isolation Relay
;  GPIO.0 (Pin 5) - Servo Pulse Output

; START USER CONFIGURATION SECTION

; servo_upperlimit is the maximum travel in one direction, expressed as a pulse 
;  of 1+(servo_upperlimit/250) milliseconds, or +((servo_upperlimit-128)/250) percent throw

; servo_lowerlimit is the maximum travel in the other direction, expressed as a pulse of 
;  of 1+(servo_lowerlimit/250) milliseconds, or -((128-servo_lowerlimit)/250) percent throw

; servo_halfway is normally halfway between upper and lower, but you could change it manually
;  if you had some bizarre reason to set where the midpoint of throw was other than that

; servo_ramprate is a constant defining how fast the servo will move from one end to the other

; servo_delay works with servo_ramprate to slow down the throw.  Used when servo_ramprate is 1 and cannot
;  be reduced further.  The PWM will not be updated for the number of loops specified by servo_delay

; Complete throw will be (in seconds) = (servo_upperlimit - servo_lowerlimit) * (1 + servo_delay) / (50 * servo_ramprate)

; frog_power_off_delay is the amount of time the frog power relay will remain off after a throw completes
;  The delay is approximately (in seconds) = (frog_power_off_delay / 50)

; One nop in delay_pwm_loop:
; Full Throw = 255 / 0
; 30 Deg     = 170 / 80 (default)
; 20 Deg     = 153 / 97
; 15 Deg     = 147 / 103

; Two nop in delay_pwm_loop:
; 100 Deg     = 170 / 80 (default)

servo_upperlimit      equ 233
servo_lowerlimit	  equ 23
servo_halfway         equ (servo_upperlimit + servo_lowerlimit)/2
servo_ramprate		  equ 2
servo_delay           equ 0
frog_power_off_delay  equ 25 

; Set stall time to 255 to make servo always actively driven
servo_stalltime		  equ 50
; Set aux_output_mode to:
;   0 to set one output high for each direction
;   1 to use the interlocking relays for direction and power
aux_output_mode       equ 1

 
; END USER CONFIGURATION SECTION


; Variable definitions	
	cblock 0x10
temp1
temp2
temp3
power_off_timer
hz50_counter
stall_counter
servo_pulsewidth
delay_counter
	endc

; Begin application
	org	0x00
	movwf OSCCAL  ; Move oscillator calibration, since we need mostly precise timing

start
	clrwdt
	clrf	GPIO
	movlw	0x08
	tris	GPIO

	movlw	0x80
	option

	movlw	servo_halfway 
	movwf	servo_pulsewidth

	movlw	servo_stalltime
	movwf	stall_counter

	movlw	servo_delay
	movwf	delay_counter

main_loop
	clrwdt

pwm_math
	movf	delay_counter
	btfss	STATUS, Z
	goto decrement_delay_counter
	; Reset delay counter and check input
	movlw	servo_delay
	movwf	delay_counter
	goto check_input

decrement_delay_counter
	; Wait longer and skip the pwm math
	decf	delay_counter, f
	goto pwm_math_done

check_input
	; Is GPIO3 high or low?  Move pulsewidth in the right direction at the right ramp rate
	btfsc	GPIO,3
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
pwm_math_done

#if 0 == aux_output_mode

direction_output_logic
	movlw	servo_halfway
	subwf	servo_pulsewidth, w
	btfss	STATUS, C
	goto	direction_output_off
direction_output_on
	bsf		GPIO, 2
	bcf		GPIO, 1
	goto	direction_output_logic_done
direction_output_off
	bcf		GPIO, 2
	bsf		GPIO, 1
direction_output_logic_done

#else

direction_relay_logic
	movlw	servo_halfway
	subwf	servo_pulsewidth, w
	btfss	STATUS, C
	goto	direction_relay_off
direction_relay_on
	bsf		GPIO, 2
	goto	direction_relay_logic_done
direction_relay_off
	bcf		GPIO, 2
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
	bcf		GPIO, 1
	goto frog_relay_logic_done

frog_relay_countdown
	movf	power_off_timer, w
	btfsc	STATUS, Z
	goto	frog_power_on
	decf	power_off_timer, f
	goto	frog_relay_logic_done

frog_power_on
	bsf		GPIO, 1

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
	movf	stall_counter
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
	decfsz  temp3, f
	goto	delay_5ms

	incf hz50_counter, f

	movf	stall_counter
	btfsc	STATUS, Z
	goto	main_loop

; Send base (1 ms) servo pulse
	bsf		GPIO,0
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
	nop
	decfsz	temp1, f
	goto delay_pwm_loop

servo_off
	bcf		GPIO,0

	goto main_loop


end
