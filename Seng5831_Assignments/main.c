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

int main()
{
	unsigned long blink_rate = 500;
	unsigned char toggle_green = 0;
	unsigned char toggle_red = 0;
	unsigned char green_state = 0;
	unsigned char red_state = 0;
	
	time_reset();
	unsigned long mytimer = get_ms();
	
	clear();
	print("Ready");
	while(1)
	{
		// when either the top or bottom buttons is pressed
		// store the value of the pressed button in the variable 'button'
		unsigned char button = get_single_debounced_button_press(ANY_BUTTON);
		
		// Evaluate pressed button
		// change led state then set led to that state
		if (button & TOP_BUTTON)
		{
			toggle_green=!toggle_green;
			green_led(toggle_green);
		}
		if (button & BOTTOM_BUTTON)
		{
			toggle_red=!toggle_red;
			red_led(toggle_red);
		}
		
		// when the toggle value is active 
		// blink the led
		if (toggle_green && (mytimer >= blink_rate))
		{
			green_state = !green_state;
			green_led(green_state);
		}		
		if (toggle_red && (mytimer >= blink_rate))
		{
			red_state = !red_state;
			red_led(red_state);
		}
		
		// reset timer after the limit has been breached
		if (mytimer >= blink_rate)
		{
			time_reset();
		}

		// increment timer
		mytimer = get_ms();
		
	}
}
