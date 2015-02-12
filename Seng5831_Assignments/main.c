/* Seng5831_Assignments - an application for the Pololu Orangutan SVP
*
* This application uses the Pololu AVR C/C++ Library.  For help, see:
* -User's guide: http://www.pololu.com/docs/0J20
* -Command reference: http://www.pololu.com/docs/0J18
*
* Created: 1/31/2015 2:39:21 PM
*  Author: US296865
*/

#include <pololu/orangutan.h>

unsigned long timer_counter = 0;

ISR(TIMER0_COMPA_vect){
	cli();
	timer_counter++;
	if (timer_counter % 500 == 0)	// slow down the blink rate
	{
		red_led(TOGGLE);
		timer_counter=0;
	}
	sei();
}

void init_timer()
{
	// This is specific to a non-pwm timer
	// need to calculate the interrupt rate
	TCCR0A = 0b10000010;	// set the register A values - from the data sheet 
	TCCR0B = 0b0000110;		// set the register B values - from the data sheet
	OCR0A = 200;			// set the count for the formula
	TIMSK0 = 0b00000010;	// enable the interrupt watch
}

int main()
{	
	red_led(TOGGLE);	// check led
	init_timer();		// init timer
	sei();				// enable interrupts
	while(1);			// ensure program runs continuously 
}