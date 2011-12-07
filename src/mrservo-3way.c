#include <system.h>
#include <pic16f628.h>

#pragma CLOCK_FREQ    4000000
#define UINT8 unsigned char

#define _BV(n)  (1<<(n))

#define SERVO_UPPERLIMIT 170
#define SERVO_LOWERLIMIT  80
#define SERVO_RAMPRATE     2
#define SERVO_STALLTIME   50 //  (in 1/50ths of seconds)

#define YELLOW 3
#define RED    2
#define GREEN  1

#define PORTA_SERVO1_OUT     0
#define PORTA_SERVO2_OUT     1
#define PORTA_LSIG_GRN       2
#define PORTA_LSIG_RED       3
#define PORTA_SWITCH1_IN     4
#define PORTA_SWITCH2_IN     5

#define PORTB_SIG_R_GRN      0
#define PORTB_SIG_R_RED      1
#define PORTB_SIG_C_GRN      2
#define PORTB_SIG_C_RED      3
#define PORTB_SIG_L_GRN      4
#define PORTB_SIG_L_RED      5
#define PORTB_USIG_GRN       6
#define PORTB_USIG_RED       7


UINT8 servo_pulsewidth1 = (SERVO_UPPERLIMIT + SERVO_LOWERLIMIT) / 2;
UINT8 servo_pulsewidth2 = (SERVO_UPPERLIMIT + SERVO_LOWERLIMIT) / 2;
UINT8 servo_upperlimit = SERVO_UPPERLIMIT;
UINT8 servo_lowerlimit = SERVO_LOWERLIMIT;
UINT8 servo_midpoint   = (SERVO_UPPERLIMIT + SERVO_LOWERLIMIT) / 2;
UINT8 servo_ramprate   = SERVO_RAMPRATE;
UINT8 servo_countdown_timer = SERVO_STALLTIME;

void output_pulse()
{
	UINT8 temp1=0, temp2=0;
	if (servo_countdown_timer == 0)
		return;
	servo_countdown_timer--;

	asm
	{
		; Send base (1 ms) servo pulse
			bsf		_porta, 0
		; 1 ms code
					;998 cycles
			movlw	0xC7
			movwf	_temp1
			movlw	0x01
			movwf	_temp2
		delay_1ms_inner:
			decfsz	_temp1, F
			goto	delay_1ms_end
			decfsz	_temp2, F
		delay_1ms_end:
			goto	delay_1ms_inner
		
		; Now send the 0-1 additional ms needed to move the servo
		; Get the variable pulsewidth - each count is worth 1/250th of a ms, or 4 instructions
			movf	_servo_pulsewidth1, W
		
		; Test if variable PWM is zero
		; If you try to loop on it, you'll underflow and get a full pulse
		;  and a twitchy servo going full range
			btfsc	_status, Z
			goto	servo_off
		; In non-zero cases, move it into a loop counter and get going
			movwf   _temp1
		delay_pwm_loop:
			nop
			decfsz	_temp1, F
			goto delay_pwm_loop
		
		servo_off:
			bcf		_porta, 0
	}
	clear_wdt();
	
	asm
	{
		; Send base (1 ms) servo pulse
			bsf		_porta, 1
		; 1 ms code
					;998 cycles
			movlw	0xC7
			movwf	_temp1
			movlw	0x01
			movwf	_temp2
		delay_1ms_inner2:
			decfsz	_temp1, F
			goto	delay_1ms_end2
			decfsz	_temp2, F
		delay_1ms_end2:
			goto	delay_1ms_inner2
		
		; Now send the 0-1 additional ms needed to move the servo
		; Get the variable pulsewidth - each count is worth 1/250th of a ms, or 4 instructions
			movf	_servo_pulsewidth2, W
		
		; Test if variable PWM is zero
		; If you try to loop on it, you'll underflow and get a full pulse
		;  and a twitchy servo going full range
			btfsc	_status, Z
			goto	servo_off2
		; In non-zero cases, move it into a loop counter and get going
			movwf   _temp1
		delay_pwm_loop2:
			nop
			decfsz	_temp1, F
			goto delay_pwm_loop2
		
		servo_off2:
			bcf		_porta, 1
	}
	clear_wdt();
}

void delay_roughly_17ms()
{
	delay_ms(4);
	clear_wdt();
	delay_ms(4);
	clear_wdt();
	delay_ms(4);
	clear_wdt();
	delay_ms(5);
	clear_wdt();
}


#define SWITCH_TURNOUT_L  0x10
#define SWITCH_TURNOUT_C  0x30
#define SWITCH_TURNOUT_R  0x20

#define INCREMENT 0
#define DECREMENT 1

void adjust_pwm(UINT8 switches)
{
	UINT8 servo1_dir, servo2_dir;
	
	switch(switches)
	{
		case SWITCH_TURNOUT_L:
			servo1_dir = DECREMENT;
			servo2_dir = INCREMENT;
			break;

		case SWITCH_TURNOUT_C:
			servo1_dir = servo2_dir = DECREMENT;
			break;

		case SWITCH_TURNOUT_R:
			servo2_dir = DECREMENT;
			servo1_dir = INCREMENT;
			break;

		default:
			// If we're not in a known state, just don't do anything
			return;
	}


	if (DECREMENT == servo1_dir)
	{
		// Decrement PWM
		if (servo_pulsewidth1 >= (servo_lowerlimit + servo_ramprate))
			servo_pulsewidth1 -= servo_ramprate;
		else
			servo_pulsewidth1 = servo_lowerlimit;

	}
	else
	{
		// Increment PWM
		if (servo_pulsewidth1 <= (servo_upperlimit - servo_ramprate))
			servo_pulsewidth1 += servo_ramprate;
		else
			servo_pulsewidth1 = servo_upperlimit;
	}

	if (DECREMENT == servo2_dir)
	{
		// Decrement PWM
		if (servo_pulsewidth2 >= (servo_lowerlimit + servo_ramprate))
			servo_pulsewidth2 -= servo_ramprate;
		else
			servo_pulsewidth2 = servo_lowerlimit;

	}
	else
	{
		// Increment PWM
		if (servo_pulsewidth2 <= (servo_upperlimit - servo_ramprate))
			servo_pulsewidth2 += servo_ramprate;
		else
			servo_pulsewidth2 = servo_upperlimit;
	}

	if ((servo_pulsewidth1 < servo_upperlimit && servo_pulsewidth1 > servo_lowerlimit)
		|| (servo_pulsewidth2 < servo_upperlimit && servo_pulsewidth2 > servo_lowerlimit))
		servo_countdown_timer = SERVO_STALLTIME;
}


void adjust_signals(UINT8 switches)
{
	UINT8 left_sig=RED, center_sig=RED, right_sig=RED, upper_sig=RED, lower_sig=RED;
	UINT8 servo1_pos = 2;
	UINT8 servo2_pos = 2;
	UINT8 portb_temp = 0;
	UINT8 porta_temp = 0;
	
	if (servo_pulsewidth1 == servo_upperlimit)
		servo1_pos = 1;
	else if (servo_pulsewidth1 == servo_lowerlimit)
		servo1_pos = 0;

	if (servo_pulsewidth2 == servo_upperlimit)
		servo2_pos = 1;
	else if (servo_pulsewidth2 == servo_lowerlimit)
		servo2_pos = 0;

	if (servo1_pos != 2 && servo2_pos != 2)
	{
		/* 		case SWITCH_TURNOUT_L:
			servo1_dir = DECREMENT;
			servo2_dir = INCREMENT;
			break;

		case SWITCH_TURNOUT_C:
			servo1_dir = servo2_dir = DECREMENT;
			break;

		case SWITCH_TURNOUT_R:
			servo2_dir = DECREMENT;
			servo1_dir = INCREMENT;
*/

		// Basically, if both turnouts are in final states
		if (0 == servo1_pos && 1 == servo2_pos && SWITCH_TURNOUT_L == switches)
		{
			left_sig = GREEN;
			lower_sig = GREEN;
		}
		
		if (0 == servo1_pos && 0 == servo2_pos && SWITCH_TURNOUT_C == switches)
		{
			center_sig = GREEN;
			upper_sig = GREEN;
		}
		
		if (1 == servo1_pos && 0 == servo2_pos && SWITCH_TURNOUT_R == switches)
		{
			right_sig = GREEN;
			lower_sig = YELLOW;
		}
	}

	switch(left_sig)
	{
		case GREEN:
			portb_temp |= _BV(PORTB_SIG_L_GRN);
			break;

		case YELLOW:
		case RED:
		default:
			portb_temp |= _BV(PORTB_SIG_L_RED);
			break;
	}	

	switch(center_sig)
	{
		case GREEN:
			portb_temp |= _BV(PORTB_SIG_C_GRN);
			break;

		case YELLOW:
		case RED:
		default:
			portb_temp |= _BV(PORTB_SIG_C_RED);
			break;
	}	

	switch(right_sig)
	{
		case GREEN:
			portb_temp |= _BV(PORTB_SIG_R_GRN);
			break;

		case YELLOW:
		case RED:
		default:
			portb_temp |= _BV(PORTB_SIG_R_RED);
			break;
	}	

	switch(upper_sig)
	{
		case GREEN:
			portb_temp |= _BV(PORTB_USIG_GRN);
			break;

		case YELLOW:
			portb_temp |= _BV(PORTB_USIG_GRN) | _BV(PORTB_USIG_RED);
			break;		

		case RED:
		default:
			portb_temp |= _BV(PORTB_USIG_RED);
			break;
	}	

	switch(lower_sig)
	{
		case GREEN:
			porta_temp |= _BV(PORTA_LSIG_GRN);
			break;

		case YELLOW:
			porta_temp |= _BV(PORTA_LSIG_GRN) | _BV(PORTA_LSIG_RED);
			break;		

		case RED:
		default:
			porta_temp |= _BV(PORTA_LSIG_RED);
			break;
	}	

	portb = portb_temp;
	porta = porta_temp;

}

void main()
{
	UINT8 switches, i, j;
	cmcon = 0x07;

	trisa = _BV(PORTA_SWITCH1_IN) | _BV(PORTA_SWITCH2_IN);
	trisb = 0;

	servo_upperlimit = SERVO_UPPERLIMIT;
	servo_lowerlimit = SERVO_LOWERLIMIT;
	servo_midpoint   = (SERVO_UPPERLIMIT + SERVO_LOWERLIMIT) / 2;
	servo_ramprate   = SERVO_RAMPRATE;
	servo_countdown_timer = SERVO_STALLTIME;

	switches = porta & (_BV(PORTA_SWITCH1_IN) | _BV(PORTA_SWITCH2_IN));
	switch(switches)
	{
		case SWITCH_TURNOUT_L:
			servo_pulsewidth1 = servo_pulsewidth2 = SERVO_UPPERLIMIT;
			break;

		case SWITCH_TURNOUT_C:
			servo_pulsewidth1 = SERVO_LOWERLIMIT;
			servo_pulsewidth2 = SERVO_UPPERLIMIT;
			break;

		case SWITCH_TURNOUT_R:
			servo_pulsewidth1 = servo_pulsewidth2 = SERVO_LOWERLIMIT;
			break;
	
		default:
			servo_pulsewidth1 = servo_pulsewidth2 = (SERVO_UPPERLIMIT + SERVO_LOWERLIMIT) / 2;
			break;	
	}	

	while(1)
	{
		switches = porta & (_BV(PORTA_SWITCH1_IN) | _BV(PORTA_SWITCH2_IN));
		adjust_pwm(switches);
		output_pulse();
		adjust_signals(switches);
		delay_roughly_17ms();		

	}
}
