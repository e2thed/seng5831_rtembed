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

int main()
{
	red_led(TOGGLE);	// check led
	
	DDRD |= _BV(5);
	PORTD |= _BV(5);
	// WGM is MODE 14 refer to table 16-5 in data sheet
	// Prescaler is 1024 [set to highest and work your way down] This coordinates with the TOP
	// COM1A clear on compare match refer to table 16-3 in data sheet
	TCCR1A = 0b10100010; // COM1A1;COM1A0;COM1B1;COM1B0;-;-;WGM11;WGM10
	TCCR1B = 0b00011101; // ICNC1;ICES1;-;WGM13;WGM12;CS12;CS11;CS10
	
	ICR1 = 0x7D0;  // TOP is 2000 [Set this as high as possible, then work down] This coordinates with the Prescaler
	OCR1A = ICR1/2; // Half of ICR1 1000

	while(1);			// ensure program runs continuously
	
}


