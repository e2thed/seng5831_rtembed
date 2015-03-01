/* Seng5831_Assignments - an application for the Pololu Orangutan SVP
*
* This application uses the Pololu AVR C/C++ Library.  For help, see:
* -User's guide: http://www.pololu.com/docs/0J20
* -Command reference: http://www.pololu.com/docs/0J18
*
* Created: 1/31/2015 2:39:21 PM
*  Author: US296865
*
* This is the lab in progress
*/

#include <pololu/orangutan.h>
#include <stdio.h>

/////// DEFINE For Lab 1 Assignment Part 1 - 100 Million seems a bit excessive
#define FOR_COUNT_10MS 16230
uint32_t __ii;
uint32_t i;
#define WAIT_10MS {for (__ii=0;__ii<FOR_COUNT_10MS;__ii++);}

unsigned long timer_counter = 0;
unsigned long blink_rate_8bit = 1500;

void calculate_wcet()
{
	unsigned long StartTimeEmptyLoop = 0;
	unsigned long StopTimeEmptyLoop = 0;
	unsigned long FunctionLoopDuration = 0;
	
	StartTimeEmptyLoop = get_ms();
	WAIT_10MS;
	StopTimeEmptyLoop = get_ms();
	
	FunctionLoopDuration = FOR_COUNT_10MS / (StopTimeEmptyLoop - StartTimeEmptyLoop);
	clear();
	print_unsigned_long(StopTimeEmptyLoop);
	lcd_goto_xy(0,1);
	print_unsigned_long(FunctionLoopDuration);
}
/// Software Timer 8-bit ////////
ISR(TIMER0_COMPA_vect){ ///// Part 2 - 8-bit Software Timer user specified rate
	cli();
	timer_counter++;
	if (timer_counter % blink_rate_8bit == 0)	// slow down the blink rate
	{
		red_led(TOGGLE);
		timer_counter=0;
	}
	sei();
}
void init_timer_0()
{
	// This is specific to a non-pwm timer
	// need to calculate the interrupt rate
	TCCR0A = 0b10000010;	// set the register A values - from the data sheet
	TCCR0B = 0b0000011;		// set the register B values - from the data sheet
	OCR0A = 200;			// set the count for the formula
	TIMSK0 = 0b00000010;	// enable the interrupt watch
}
/* END Software Timer 8-bit */

//// Software Timer 16-bit
void init_timer_1()
{
	// WGM is MODE 14 refer to table 16-5 in data sheet
	// Prescaler is 1024 [set to highest and work your way down] This coordinates with the TOP
	// COM1A clear on compare match refer to table 16-3 in data sheet
	TCCR1A = 0b10100010; // COM1A1;COM1A0;COM1B1;COM1B0;-;-;WGM11;WGM10
	TCCR1B = 0b00011101; // ICNC1;ICES1;-;WGM13;WGM12;CS12;CS11;CS10

	ICR1 = 0x7D0;  // TOP is 2000 [Set this as high as possible, then work down] This coordinates with the Prescaler
	OCR1A = ICR1/2; // Half of ICR1 1000
}
/* END Software Timer 16-bit */
int main()
{
	clear();                 // clear the lcd
	//calculate_wcet();      // uncomment this for the calculation
	//uint32_t delay_red = 0;  // this will be used for the main loop
	//init_timer_0();          // Software Timer  8-bit
	init_timer_1();          // Software Timer 16-bit
	DDRD |= _BV(5);
	PORTD |= _BV(5);

	sei();                   // SEt Interrupts
	while(1)
	{
		////// This will blink the red led @ 1hz
		//delay_red++;
		//WAIT_10MS;
		//if (delay_red >= 100)
		//{
		//red_led(TOGGLE);
		//delay_red = 0;
		//}
		
	};
	
}