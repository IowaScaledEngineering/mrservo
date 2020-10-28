/*************************************************************************
Title:    MRServo v4 Switch Machine
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2020 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint8_t eventFlags = 0;
#define EVENT_DO_BD_READ 0x01
#define EVENT_DO_ADC_RUN 0x02
#define EVENT_1HZ_BLINK  0x04

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 94;  // 9.6MHz / 1024 / 94 ~= 100Hz
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);  // 1024 prescaler
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIM0_COMPA_vect)
{
	static uint8_t ticks = 0;
	static uint8_t decisecs = 0;

	if (++ticks >= 10)
	{
		ticks = 0;
		eventFlags |= EVENT_DO_ADC_RUN;

		if (++decisecs >=5)
		{
			eventFlags ^= EVENT_1HZ_BLINK;
			decisecs = 0;
		}
	}

}


void setRelayOn()
{
	PORTB |= _BV(PB1);
}

void setRelayOff()
{
	PORTB &= ~_BV(PB4);
}

void init(void)
{
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();

	CLKPR = _BV(CLKPCE);
	CLKPR = 0x00;

	// Pin Assignments for PORTB/DDRB
	//  PB0 - Servo PWM output
	//  PB1 - Position Input
	//  PB2 - Not used
	//  PB3 - Analog in from R2
	//  PB4 - Relay/Position LED Drive
	//  PB5 - Not used
	//  PB6 - Not used
	//  PB7 - Not used
	DDRB  = 0b00010001;
	PORTB = 0b00001110;

	initialize100HzTimer();
}

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;					// Clear reset status
	init();
	sei();

	while(1)
	{
		wdt_reset();
	}
}

