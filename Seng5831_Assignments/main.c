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

char receive_buffer[32];
unsigned char receive_buffer_position = 0;
char send_buffer[32];
unsigned long blink_rate = 50;

void wait_for_sending_to_finish()
{
	while(!serial_send_buffer_empty(USB_COMM))
	serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
}

void process_received_byte(char byte)
{
	clear();		// clear LCD
	print("RX: ");
	print_character(byte);
	lcd_goto_xy(0, 1);	// go to start of second LCD row

	switch(byte)
	{
		// If the character 'G' or 'g' is received, toggle the green LED.
		case '+':
		blink_rate -= 30;
		break;

		// If the character 'R' or 'r' is received, toggle the red LED.
		case '-':
		blink_rate += 30;
		break;

		// If any other character is received, change its capitalization and
		// send it back.
		default:
		wait_for_sending_to_finish();
		send_buffer[0] = byte ^ 0x20;
		serial_send(USB_COMM, send_buffer, 1);
		print("TX: ");
		print_character(send_buffer[0]);
		break;
	}
}

void check_for_new_bytes_received()
{
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// Process the new byte that has just been received.
		process_received_byte(receive_buffer[receive_buffer_position]);

		// Increment receive_buffer_position, but wrap around when it gets to
		// the end of the buffer.
		if (receive_buffer_position == sizeof(receive_buffer)-1)
		{
			receive_buffer_position = 0;
		}
		else
		{
			receive_buffer_position++;
		}
	}
}

int main()
{
	// Set up flags and configuration variables
	unsigned char led1_state = 0;
	unsigned char led2_state = 0;
	
	serial_set_baud_rate(USB_COMM, 9600);

	serial_receive_ring(USB_COMM, receive_buffer, sizeof(receive_buffer));
	
	// set up DDR's and ports
	DDRA |= 0x08;
	DDRD |= 0x08;
	
	PORTA |= 0x08;
	PORTD |= 0x08;
	
	time_reset();
	unsigned long mytimer = 0;
	
	clear();
	print("Ready");
	while(1)
	{
		serial_check();
		check_for_new_bytes_received();

		// when either the top or bottom buttons is pressed
		// store the value of the pressed button in the variable 'button'
		unsigned char button = get_single_debounced_button_press(ANY_BUTTON);
		
		// Evaluate pressed button
		// change led state then set led to that state
		if (button & TOP_BUTTON)
		{
			led1_state=1;
			PORTA &= ~(1<<0x08); // Turn light off
			led2_state=0;
			PORTD &= ~(1<<0x08); // Turn light off
		}
		if (button & BOTTOM_BUTTON)
		{
			led1_state=0;
			PORTA &= ~(1<<0x08); // Turn light off
			led2_state=1;
			PORTD &= ~(1<<0x08); // Turn light off
		}
		if (button & MIDDLE_BUTTON)
		{
			led1_state=0;
			PORTA &= ~(1<<0x08); // Turn light off
			led2_state=0;
			PORTD &= ~(1<<0x08); // Turn light off
		}
		// when the toggle value is active
		// blink the led
		if (mytimer >= blink_rate)
		{
			if (led1_state)
			{
				DDRA ^= 0x08;
			}
			if (led2_state)
			{
				DDRD ^= 0x08;
			}
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
