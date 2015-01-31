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

// Blinks the LEDs
void blink_leds()
{
	red_led(1);               // red LED on
	delay_ms(1000);           // waits for a second
	red_led(0);               // red LED off
	delay_ms(1000);           // waits for a second
	green_led(1);             // green LED on (will not work on the Baby Orangutan)
	delay_ms(500);            // waits for 0.5 seconds
	green_led(0);             // green LED off (will not work on the Baby Orangutan)
	delay_ms(500);            // waits for 0.5 seconds
}

// toggle the green LED
void start_green_led_blink()
{
	green_led(1);             // green LED on
	delay_ms(1000);           // waits for a second
	green_led(0);             // green LED off
	delay_ms(500);           // waits for half a second
}

// Toggle the red LED
void start_red_led_blink()
{
	red_led(1);               // red LED on
	delay_ms(1000);           // waits for a second
	red_led(0);               // red LED off
	delay_ms(500);           // waits for half a second
}


int main()
{
	print("LED Blink");
	blink_leds();
	clear();
	print("Top or Bottom");
	lcd_goto_xy(0,1);
	print("Green or Red");
	while(1)
	{
		// wait for either the top or bottom buttons to be pressed
		// store the value of the pressed button in the variable 'button'
		unsigned char button = wait_for_button_press(TOP_BUTTON | BOTTOM_BUTTON);
		clear();
		if (button == TOP_BUTTON)     // display the button that was pressed
		{
			clear();
			lcd_goto_xy(0,0);
			print("Green Light");
			start_green_led_blink(button);
		}
		else
		{
			clear();
			print("Red Light");
			start_red_led_blink(button);
		}
		
		if (!button_is_pressed(TOP_BUTTON) & !button_is_pressed(BOTTOM_BUTTON))
		{
			clear();
			print("Top or Bottom");
			lcd_goto_xy(0,1);
			print("Green or Red");
		}
	}
}
