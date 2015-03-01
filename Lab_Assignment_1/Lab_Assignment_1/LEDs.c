#include "LEDs.h"
#include "timer.h"

#include <avr/io.h>
#include <stdio.h>

// RED LED is toggled in a cyclic executive defined in lab1.c
// Yellow and Green are toggled in interrupt handlers defined below.
// The interrupts are initialized in the timers.c file.
// Yellow is on Timer 3. Green is on Timer 1.

// GLOBALS
extern uint32_t G_yellow_ticks;

extern uint32_t G_red_period;
extern uint32_t G_green_period;
extern uint32_t G_yellow_period;

extern uint32_t G_red_toggles;
extern uint32_t G_green_toggles;
extern uint32_t G_yellow_toggles;

void init_LEDs() {

	volatile int i;

	// Clear all data direction ports
	DD_REG_RED    = 0;
	DD_REG_YELLOW = 0;
	DD_REG_GREEN  = 0;

	// Configure data direction as output
	DD_REG_RED    |= BIT_RED;
	DD_REG_YELLOW |= BIT_YELLOW;
	DD_REG_GREEN  |= BIT_GREEN;
	
	PORT_RED    |= BIT_RED;
	PORT_YELLOW |= BIT_YELLOW;
	PORT_GREEN  |= BIT_GREEN;

	// Turn LEDs on to make sure they are working
	LED_ON(RED);
	LED_ON(YELLOW);
	LED_ON(GREEN);
	
	// leave on for 2 seconds
	//TODO THIS DON'T WORK, BUT IN MAIN IT DO
	for (i=0;i<200;i++){
		WAIT_10MS;
	}
	// Start all LEDs off
	LED_OFF(RED);
	LED_OFF(YELLOW);
	LED_OFF(GREEN);

	// clear toggle counters
	G_green_toggles = 0;
	G_red_toggles = 0;
	G_yellow_toggles = 0;
	
}


void set_toggle(char color, int ms) {

	// check toggle ms is positive and multiple of 100
	if (ms<0) {
		printf("Cannot toggle negative ms.\n");
		return;
	}

	if (~((ms%100)==0)) {
		ms = ms - (ms%100);
		printf("Converted to toggle period: %lu.\n",ms);
	}

	// For each color, if ms is 0, turn it off by changing data direction to input.
	// If it is >0, set data direction to output.
	if ((color=='R') || (color=='A')) {
		if (ms==0)
		DD_REG_OFF(RED);
		else
		DD_REG_ON(RED);
		G_red_period = ms;
	}

	if ((color=='Y') || (color=='A')) {
		if (ms==0)
		DD_REG_OFF(YELLOW);
		else
		DD_REG_ON(YELLOW);
		G_yellow_period = ms / 100;
	}

	if ((color=='G') || (color=='A')) {
		if (ms==0)
		{
			DD_REG_OFF(GREEN);
			TIMSK1 = (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1); // Turn off ISR
		}
		else
		{
			DD_REG_ON(GREEN);
			TIMSK1 = (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1); // Turn on ISR
			G_green_period = ms;
		}

		// green has a limit on its period.
		if ( ms > 4000) ms = 4000;
		G_green_period = ms;

		// set the OCR1A (TOP) to get (approximately) the requested frequency.
		if ( ms > 0 ) {
			ICR1 = 19 * G_green_period;
			OCR1A = (uint32_t) ICR1 / 2 ;
			//printf("Green to toggle at freq %luHz (period %lu ms)\n", XXXXX ,G_green_period);
		}
	}
}



// INTERRUPT Names are defined in iom1284p.h

// INTERRUPT HANDLER for yellow LED
ISR(TIMER3_COMPA_vect) {
	// This the Interrupt Service Routine for Toggling the yellow LED.
	// Each time the TCNT count is equal to the OCRxx register, this interrupt is enabled.
	// At creation of this file, it was initialized to interrupt every 100ms (10Hz).
	//
	// Increment ticks. If it is time, toggle YELLOW and increment toggle counter.
	G_yellow_ticks++;
	if ( ( G_yellow_ticks % G_yellow_period ) == 0 )
	{
		LED_TOGGLE(YELLOW);
		G_yellow_toggles++;
	}
}

// INTERRUPT HANDLER for green LED
ISR(TIMER1_COMPA_vect) {

	// This the Interrupt Service Routine for tracking green toggles. The toggling is done in hardware.
	// Each time the TCNT count is equal to the OCRxx register, this interrupt is enabled.
	// This interrupts at the user-specified frequency for the green LED.
	
	G_green_toggles++;
	
}

