/*************************************************************************
Title:    MRServo v4 Switch Machine
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2021 Michael Petersen & Nathan Holmes

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
#include <stdbool.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define min(a,b)  (((a)<(b))?(a):(b))
#define max(a,b)  (((a)>(b))?(a):(b))

volatile uint8_t eventFlags = 0;
volatile uint8_t pwmWidth = 0;
#define EVENT_DO_READ 0x01

// This needs a 4.8MHz clock source
#define PULSE_1MS   (56)
#define PULSE_2MS   (112)

#define OCR_1MS     (0xFF-PULSE_1MS)
#define OCR_15MS    (0xFF-((PULSE_2MS - PULSE_1MS) / 2))
#define OCR_2MS     (0xFF-PULSE_2MS)
#define OCR_OFF     (0xFF)

void initializeTimer(void)
{
	// Set up timer 0 for 3.413mS / 293 Hz interrupts
	TCNT0 = 0;
	OCR0A = OCR_OFF;  // OCR0A is used for output PWM
	TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(CS01) | _BV(CS00);  // 64 prescaler
	TIMSK0 = _BV(TOIE0);
}

ISR(TIM0_OVF_vect)
{
	static uint8_t ticks = 0;
	
	// This will fire every 3.413mS when timer0 rolls over
	// This was chosen to get the entire 1-2mS servo PWM range
	// within the PWM window of OCR0

	if (ticks++ >= 6)
	{
		// Every 20mS (50Hz), trigger the main loop to read inputs and recalculate position
		// Also set OCR0A so that we emit a pulse
		OCR0A = pwmWidth;
		eventFlags |= EVENT_DO_READ;
		ticks = 0;
	} else {
		// Set OCR0A to off so that we don't emit pulses faster than 50Hz
		OCR0A = OCR_OFF;
	}
}

void setRelayOn()
{
	PORTB |= _BV(PB4);
}

void setRelayOff()
{
	PORTB &= ~_BV(PB4);
}

uint8_t getPositionInput()
{
	return ((PINB & _BV(PB1))?1:0);
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

	initializeTimer();
}

uint8_t debounce(uint8_t raw_inputs)
{
	static uint8_t clock_A=0, clock_B=0, debounced_state=0x01;
	uint8_t delta = raw_inputs ^ debounced_state;   //Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debounced_state ^= changes;
	debounced_state &= 0x0F;
	return(debounced_state);
}

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;					// Clear reset status
	wdt_reset();

	uint8_t servoUpperLimit = 170;
	uint8_t servoLowerLimit = 80;
	uint8_t servoHalfway = ((uint16_t)servoLowerLimit + (uint16_t)servoUpperLimit) / 2;
	uint8_t servo = servoLowerLimit;
	uint8_t rampRate = 2;
	
	uint8_t inputState = 0x01;
	uint8_t movingTimeout = 50;

	eventFlags = 0;
	pwmWidth = 0;

	init();
	sei();

	while(1)
	{
		wdt_reset();

		if (eventFlags & EVENT_DO_READ)
		{
			eventFlags &= ~(EVENT_DO_READ);
			inputState = debounce(getPositionInput());

			if (inputState && (servo < servoUpperLimit))
			{
				servo = min(servo+rampRate, servoUpperLimit);
				movingTimeout = 50;
			} else if (!inputState && (servo > servoLowerLimit)) {
				servo = max(servo-rampRate, servoLowerLimit);
				movingTimeout = 50;
			}
			
			if (servo > servoHalfway)
				setRelayOn();
			else
				setRelayOff();

			if (movingTimeout > 0)
			{
				movingTimeout--;
				uint8_t pwmVal = (uint16_t)servo * (PULSE_1MS) / 256;
				pwmWidth = OCR_1MS - pwmVal;
			} else {
				pwmWidth = OCR_OFF; // Stop driving servo ~1s after we've stopped moving
			}

		}
	}
}

