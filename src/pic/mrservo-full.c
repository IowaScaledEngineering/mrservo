#include <system.h>
#include <pic12f629.h>

#pragma DATA _CONFIG, _CPD_OFF & _CP_OFF & _BODEN_ON & _MCLRE_OFF & _PWRTE_ON & _INTRC_OSC_NOCLKOUT & _WDT_ON
#pragma DATA _EEPROM, 0xFA, 0x05, 0x05, 0x19

#define UINT8 unsigned char


UINT8 EepromRead(UINT8 addr)
{
	eeadr = addr;
	set_bit(eecon1, RD);
	return(eedata);
}

void EepromWrite(UINT8 addr, UINT8 data)
{
	UINT8 temp_intcon = intcon;
	eeadr = addr;
	eedata = data;

	asm
	{
		bsf _eecon1, WREN
		movlw	0x55
		movwf	_eecon2
		movlw	0xAA
		movwf	_eecon2
		bsf		_eecon1, WR
	}

	while(eecon1 & 0x02)
		asm clrwdt
	intcon = temp_intcon;
    clear_bit( eecon1, WREN );
}

#define EEADR_PWM_UPPERLIMIT  0x00
#define EEADR_PWM_LOWERLIMIT  0x01
#define EEADR_PWM_RAMPRATE    0x02
#define EEADR_PWM_FROGDELAY   0x03

#define _BV(n)  (1<<(n))

#define GPIO_SERVO_OUT      0
#define GPIO_FROG_RELAY_OUT 1
#define GPIO_DIR_RELAY_OUT  2
#define GPIO_DIR_IN         3
#define GPIO_BTN1_IN        4
#define GPIO_BTN2_IN        5

#pragma CLOCK_FREQ    4000000

UINT8 servo_pulsewidth = 0x7F;
UINT8 servo_upperlimit = 0xFF;
UINT8 servo_lowerlimit = 0x00;
UINT8 servo_midpoint   = 0x7F;
UINT8 servo_ramprate   = 0x05;
UINT8 frog_poweroff_timer = 25;
UINT8 frog_delay       = 0x19;

void output_pulse()
{
	UINT8 temp1=0, temp2=0;
	asm
	{
		; Send base (1 ms) servo pulse
			bsf		_gpio, 0
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
			movf	_servo_pulsewidth, W
		
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
			bcf		_gpio, 0
	}
	clear_wdt();
}

void delay_roughly_50hz()
{
	UINT8 i=4;
	while(i--)
	{
		delay_ms(5);
		clear_wdt();
	}
}

void adjust_pwm()
{
	if (gpio & _BV(GPIO_DIR_IN))
	{
		// Decrement PWM
		if (servo_pulsewidth >= (servo_lowerlimit + servo_ramprate))
			servo_pulsewidth -= servo_ramprate;
		else
			servo_pulsewidth = servo_lowerlimit;

	}
	else
	{
		// Increment PWM
		if (servo_pulsewidth <= (servo_upperlimit - servo_ramprate))
			servo_pulsewidth += servo_ramprate;
		else
			servo_pulsewidth = servo_upperlimit;
	}
}

inline void direction_relay_logic()
{
	if (servo_pulsewidth > servo_midpoint)
		gpio |= _BV(GPIO_DIR_RELAY_OUT);
	else
		gpio &= ~(_BV(GPIO_DIR_RELAY_OUT));
}


inline void frog_relay_logic()
{
	if (servo_pulsewidth == servo_upperlimit || servo_pulsewidth == servo_lowerlimit)
	{
		if (0 == frog_poweroff_timer)
			gpio |= _BV(GPIO_FROG_RELAY_OUT);
		else
			frog_poweroff_timer--;
	}
	else
	{
		gpio &= ~(_BV(GPIO_FROG_RELAY_OUT));
		frog_poweroff_timer = 25;
	}
	
}

UINT8 application_state = 0;

#define APP_STATE_NORMAL      0
#define APP_STATE_CONFIG_MAX_SETUP  10
#define APP_STATE_CONFIG_MAX        15

#define APP_STATE_CONFIG_MIN_SETUP  20
#define APP_STATE_CONFIG_MIN        25

#define KEY_CHORD_DELAY 50  // Measured in 1/50ths of a second
#define KEY_DELAY 15  // Measured in 1/50ths of a second
UINT8 key_countdown = KEY_DELAY;
UINT8 chord_countdown = KEY_CHORD_DELAY;
#define GPIO_BTN_MASK (_BV(GPIO_BTN1_IN) | _BV(GPIO_BTN2_IN))

#define BTN_PRESS_1    1
#define BTN_PRESS_2    2
#define BTN_PRESS_BOTH 3

UINT8 clock_A=0, clock_B=0, debounced_state=0;

UINT8 debounce(UINT8 raw_inputs)
{
  UINT8 delta;
  UINT8 changes;

  delta = raw_inputs ^ debounced_state;   //Find all of the changes

  clock_A ^= clock_B;                     //Increment the counters
  clock_B  = ~clock_B;

  clock_A &= delta;                       //Reset the counters if no changes
  clock_B &= delta;                       //were detected.

  changes = ~((~delta) | clock_A | clock_B);
  debounced_state ^= changes;
  debounced_state &= GPIO_BTN_MASK;
  return(changes);
}


void main()
{
	UINT8 switch_changes, i, j;
    asm  //Load Factory Calibration Value Into OSCCAL
    {
        bsf _status,RP0
        call 0x3FF
        movwf _osccal
        bcf _status,RP0
    }

	cmcon = 0x07;
	gpio = 0x00;
	option_reg = 0x00;
	wpu = _BV(GPIO_DIR_IN) | _BV(GPIO_BTN1_IN) | _BV(GPIO_BTN2_IN);
	trisio = _BV(GPIO_DIR_IN) | _BV(GPIO_BTN1_IN) | _BV(GPIO_BTN2_IN);
	application_state = APP_STATE_NORMAL;
	
	servo_upperlimit = EepromRead(EEADR_PWM_UPPERLIMIT);
	servo_lowerlimit = EepromRead(EEADR_PWM_LOWERLIMIT);
	servo_ramprate = EepromRead(EEADR_PWM_RAMPRATE);
	frog_delay = EepromRead(EEADR_PWM_FROGDELAY);
	switch_changes = 0;
	debounced_state = 0;
	key_countdown = KEY_CHORD_DELAY;
	
	while(1)
	{
		switch_changes = debounce((gpio ^ 0xFF) & GPIO_BTN_MASK);
		
		if (GPIO_BTN_MASK == debounced_state)
		{
			chord_countdown--;
			// If the following is true, we've held down the chord for a second
			if (0 == chord_countdown)
			{
				gpio &= ~(_BV(GPIO_DIR_RELAY_OUT) | _BV(GPIO_FROG_RELAY_OUT));
				do
				{
					gpio ^= _BV(GPIO_DIR_RELAY_OUT);
					for (j=0; j<8; j++)
					{
						delay_roughly_50hz();
						debounce((gpio ^ 0xFF) & GPIO_BTN_MASK);
					}
				} while (debounced_state != 0);
				
				switch(application_state)
				{
					case APP_STATE_NORMAL:
						application_state = APP_STATE_CONFIG_MAX;
						gpio &= ~(_BV(GPIO_DIR_RELAY_OUT) | _BV(GPIO_FROG_RELAY_OUT));
						servo_pulsewidth = servo_upperlimit = EepromRead(EEADR_PWM_UPPERLIMIT);
						key_countdown = KEY_DELAY;
						
						break;
						
					case APP_STATE_CONFIG_MAX:
						application_state = APP_STATE_CONFIG_MIN;
						gpio &= ~(_BV(GPIO_DIR_RELAY_OUT) | _BV(GPIO_FROG_RELAY_OUT));
						gpio |= _BV(GPIO_DIR_RELAY_OUT);
						servo_pulsewidth = servo_lowerlimit = EepromRead(EEADR_PWM_LOWERLIMIT);
						key_countdown = KEY_DELAY;
						break;

					case APP_STATE_CONFIG_MIN:
						EepromWrite(EEADR_PWM_UPPERLIMIT, servo_upperlimit);
						EepromWrite(EEADR_PWM_LOWERLIMIT, servo_lowerlimit);
						servo_upperlimit = EepromRead(EEADR_PWM_UPPERLIMIT);
						servo_lowerlimit = EepromRead(EEADR_PWM_LOWERLIMIT);
						servo_midpoint = (servo_upperlimit>>1) + (servo_lowerlimit>>1);
						gpio &= ~(_BV(GPIO_DIR_RELAY_OUT) | _BV(GPIO_FROG_RELAY_OUT));
						application_state = APP_STATE_NORMAL;
						break;
					
					default:
						application_state = APP_STATE_NORMAL;
						break;
				
				}
			}
		}	
		else
		{
			chord_countdown = KEY_CHORD_DELAY;
		}



		switch(application_state)
		{
			case APP_STATE_NORMAL:
				adjust_pwm();
				direction_relay_logic();
				frog_relay_logic();
				break;
			
			case APP_STATE_CONFIG_MAX:
				if (0 != key_countdown)
					key_countdown--;
				else
				{
					if (_BV(GPIO_BTN1_IN) == debounced_state )
					{
						// Btn 1 just released
						if (servo_upperlimit >= (0x7F+5))
						{
							servo_upperlimit -= 5;
							servo_pulsewidth = servo_upperlimit;
						} else {
							servo_pulsewidth = servo_upperlimit = 0x7F;
						}
						key_countdown = KEY_DELAY;
					}
					else if (_BV(GPIO_BTN2_IN) == debounced_state )
					{
						// Btn 2 just released
						// Btn 1 just released
						if (servo_upperlimit <= (0xFF-5))
						{
							servo_upperlimit += 5;
							servo_pulsewidth = servo_upperlimit;
						} else {
							servo_pulsewidth = servo_upperlimit = 0xFF;
						}
					}
					key_countdown = KEY_DELAY;
				}
				break;
				
			case APP_STATE_CONFIG_MIN:
				if (0 != key_countdown)
					key_countdown--;
				else
				{
					if (_BV(GPIO_BTN1_IN) == debounced_state )
					{
						// Btn 1 just released
						if (servo_lowerlimit >= 5)
						{
							servo_lowerlimit -= 5;
							servo_pulsewidth = servo_lowerlimit;
						} else {
							servo_pulsewidth = servo_lowerlimit = 0;
						}
					}
					else if (_BV(GPIO_BTN2_IN) == debounced_state )
					{
						// Btn 2 just released
						if (servo_lowerlimit <= (0x7F-5))
						{
							servo_lowerlimit += 5;
							servo_pulsewidth = servo_lowerlimit;
						} else {
							servo_pulsewidth = servo_lowerlimit = 0x7F;
						}
					}
					key_countdown = KEY_DELAY;
				}					
				break;
		}
		delay_roughly_50hz();		
		output_pulse();
	}
}
