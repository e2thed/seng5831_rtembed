/* ObstacleAvoidance - an application for the Pololu Orangutan SVP
*
* This application uses the Pololu AVR C/C++ Library.  For help, see:
* -User's guide: http://www.pololu.com/docs/0J20
* -Command reference: http://www.pololu.com/docs/0J18
*
* Created: 5/8/2015 9:50:17 AM
*  Author: US296865
*/

#include <pololu/orangutan.h>
#include "menu.h"

#define L_SNSR 0
#define R_SNSR 1
// TODO: Define motors. If it makes sense the last time I found it easier to handle it without defining




int main()
{
	play_from_program_space(PSTR(">g32>>c32"));  // Play welcoming notes.
	
	int throttlePrint = 0;
	int lastReading = 4316;
	int currentReading = 0;
	//init_menu();
	
	set_m1_speed(70);
	delay_ms(2000);
	set_m1_speed(0);
	delay_ms(100);
	set_m1_speed(-70);
	delay_ms(2000);
	set_m1_speed(0);
	
	while(1)
	{
		if (throttlePrint % 5000 == 0)
		{
			clear();
			lcd_goto_xy(0,0);
			//print_unsigned_long(analog_read_millivolts(0));
			currentReading = analog_read_millivolts(0);
			if ((lastReading >= currentReading + 5)
				|| (lastReading <= currentReading - 5))
			{
				lastReading = currentReading;
			}
			print_unsigned_long(lastReading);

			throttlePrint = 0;
		}
		throttlePrint++;
	}
}
