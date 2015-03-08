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

#define forward -1
#define reverse 1
#define speed_increase 25
#define speed_max 255
#define speed_min 0

volatile int direction = forward;
volatile int m1_speed = 0;

int main()
{
	clear();
	print("Ready");
	delay_ms(1000);
	clear();
	encoders_init(IO_D3, IO_D2,0,0);
	clear();
	print_long(m1_speed);
	delay_ms(1000);
	m1_speed = 0;
	// push buttons
	while(1)
	{
		// Get button value
		unsigned char button =	get_single_debounced_button_press(ANY_BUTTON);
		
		// motor control
		if (button & TOP_BUTTON) // increase speed
		{
			m1_speed += direction * speed_increase;
			if (abs(m1_speed) >= speed_max)
			{
				m1_speed = direction * speed_max;
			}
			set_m1_speed(m1_speed);
		}
		if (button & MIDDLE_BUTTON) // reverse
		{
			lcd_goto_xy(0,1);
			print_long(m1_speed);
			
			// for some reason the values go wonky when I send it to the function
			// so I will leave the control here in the main
			while (m1_speed != 0)
			{
				m1_speed -= direction * speed_increase;
				if ((direction * m1_speed) < speed_min)
				{
					m1_speed = 0;
				}
				set_m1_speed(m1_speed);
				delay_ms(500);
			}
			
			if (direction == forward) { direction = reverse; }
			else { direction = forward; }
		}
		if (button & BOTTOM_BUTTON) // slow down
		{
			m1_speed -= direction * speed_increase;
			if ((direction * m1_speed) < speed_min)
			{
				m1_speed = 0;
			}
			set_m1_speed(m1_speed);
		}
		
		// display encoder counts
		lcd_goto_xy(0,0);
		print_long(encoders_get_counts_m1());
		lcd_goto_xy(0,1);
		print_long(m1_speed);
	}
}