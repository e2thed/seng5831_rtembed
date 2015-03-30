#include "timer.h"

#include <avr/interrupt.h>

// GLOBALS
extern uint32_t G_green_ticks;
extern uint32_t G_yellow_ticks;
extern uint32_t G_ms_ticks;

extern uint32_t G_red_period;
extern uint32_t G_timer_period;
extern uint32_t G_yellow_period;

extern uint32_t G_release_red;

void init_timers() {
	
	//<--      MOTOR PWM Controller          -->
	// Compare Output Mode, fast PWM mode = COM2A1:0 = 10 for Clear on OC2A Compare Match
	// WGM mode 3 Fast PWM. WGM2:1:0 = 011
	// Clock speed (Prescaler) will be lowest since it does note matter at this point. 8 010
	TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0x0<<3) | (0x0<<2) | (1<<WGM21) | (1<<WGM20);
	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0x0<<5) | (0x0<<4) | (0<<WGM22) | (0<<CS22) | (1<<CS21) | (0<<CS21);
	OCR2A = 0;
	
	// SET PORT Low
	PORTD &= ~(1<<PIND7);
	PORTC |= (1<<PINC7);	// Try high for experiment; nice it has flipped directions.
	
	DDRD |= (1<<PIND7);
	DDRC |= (1<<PINC7);
	
	//<--	PID Controller   ->
	// Compare Output Mode, COM11:0 = 11
	// WGM mode 10 PWM, Phase Correct, WGM13:2:1:0 = 1010
	// Clock speed (Prescalar 1024) CS12:1:0 = 101
	TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0x0<<3) | (0x0<<2) | (1<<WGM11) | (0<<WGM10);
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0x0<<5) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
	OCR1A = 19531*((float)G_timer_period/2000);
	ICR1  = 39062*((float)G_timer_period/2000);
	TIMSK1 = (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);


}