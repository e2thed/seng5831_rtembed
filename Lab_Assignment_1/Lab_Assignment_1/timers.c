#include "timer.h"
#include "LEDs.h"

#include <avr/interrupt.h>

// GLOBALS
extern uint32_t G_green_ticks;
extern uint32_t G_yellow_ticks;
extern uint32_t G_ms_ticks;

extern uint32_t G_red_period;
extern uint32_t G_green_period;
extern uint32_t G_yellow_period;

extern uint32_t G_release_red;

void init_timers() {
	
	// -------------------------  RED --------------------------------------//
	// Software Clock Using Timer/Counter 0.
	// THE ISR for this is below.

	//// SET appropriate bits in TCCR....
	//// Using CTC mode with OCR0 for TOP. This is mode 2, thus WGM0/1/0 = 010.
	//// Using pre-scaler 256. This is CS0/2/1/0 = 100
	//// Software Clock Interrupt Frequency: 1000 = f_IO / (prescaler*OCR0)
	//// Set OCR0 appropriately for TOP to generate desired frequency of 1KHz
	printf("Initializing software clock to freq 1000Hz (period 1 ms)\n");

	//Enable output compare match interrupt on timer 0A
	TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0x0<<3) | (0x0<<2) | (1<<WGM01) | (0<<WGM00);
	//      --^--                                                                         --^--
	TCCR0B = (0<<FOC0A)  | (0<<FOC0B)  | (0x0<<5) | (0x0<<4) | (0<<WGM02) | (1<<CS02) |  (0<<CS01) | (0<<CS00);
	//                                                                     --^--
	OCR0A = 0x4E; // 78
	TIMSK0 = (0x0<<7) | (0x0<<6) | (0x0<<5) | (0x0<<4) | (0x0<<3) | (0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);
	//                                                                           --^--
	G_ms_ticks = 0;	// Initialize counter

	//--------------------------- YELLOW ----------------------------------//
	// Set-up of interrupt for toggling yellow LEDs.
	// This task is "self-scheduled" in that it runs inside the ISR that is
	// generated from a COMPARE MATCH of Timer/Counter 3 to OCR3A.
	// Obviously, we could use a single timer to schedule everything, but we are experimenting here!
	// THE ISR for this is in the LEDs.c file

	// SET appropriate bits in TCCR ...
	// Using CTC mode with OCR3A for TOP. This is mode 4, thus WGM1(3?)/3210 = 0,1,0,0 = 0x4.
	// Using pre-scaler 64. This is CS1(3?)_210 = 0,1,1 = 0x3.
	TCCR3A = (1<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0x0<<3) | (0x0<<2) | (0<<WGM31) | (0<<WGM30);
	//      --^--
	TCCR3B = (0<<ICNC3) | (0<<ICES3) | (0x0<<5) | (0<<WGM33) | (1<<WGM32) | (0<<CS32) | (1<<CS31) | (1<<CS30);
	//                                                        --^--                    --^--       --^--
	// Interrupt Frequency: 10 = f_IO / (prescaler*OCR3A)
	// Set OCR3A appropriately for TOP to generate desired frequency using Y_TIMER_RESOLUTION (100 ms).
	// NOTE: This is not the toggle frequency, rather a tick frequency used to time toggles.
	OCR3A = 0x7A12; //31250
	printf("Initializing yellow clock to freq %luHz (period %lu ms)\n",(int)(20,000,000/(1024*OCR3A)),Y_TIMER_RESOLUTION);

	//Enable output compare match interrupt on timer 3A
	TIMSK3 = (0x0<<7) | (0x0<<6) | (0<<ICIE3) | (0x0<<4) | (0x0<<3) | (0<<OCIE3B) | (1<<OCIE3A) | (0<<TOIE3);
	//                                                                             --^--
	G_yellow_ticks = 0;
	
	//--------------------------- GREEN ----------------------------------//
	// Set-up of interrupt for toggling green LED.
	// This "task" is implemented in hardware, because the OC1A pin will be toggled when
	// a COMPARE MATCH is made of
	//      Timer/Counter 1 to OCR1A.
	// We will keep track of the number of matches (thus toggles) inside the ISR (in the LEDs.c file)
	// Limits are being placed on the frequency because the frequency of the clock
	// used to toggle the LED is limited.

	// Using CTC mode with OCR1A for TOP. This is mode 10, thus WGM3/3210 = 1010.
	// Toggle OC1A on a compare match. Thus COM1A_10 = 10
	// Using pre-scaler 1024. This is CS1/2/1/0 = 101
	TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0x0<<3) | (0x0<<2) | (1<<WGM11) | (0<<WGM10);
	//      --^--                                                                         --^--
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0x0<<5) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
	//                                           --^--                     --^--                   --^--

	//// Interrupt Frequency: (?) 51ms = f_IO / (1024*OCR1A)
	//// Set OCR1A appropriately for TOP to generate desired frequency.
	//// NOTE: This IS the toggle frequency.
	printf("green period %lu\n",G_green_period);
	ICR1 = 19 * G_green_period;
	OCR1A = (uint32_t) ICR1 / 2 ;
	TIMSK1 = (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
	//                                                                             --^--
	printf("Set OCR1A to %lu\n",OCR1A);
	printf("Initializing green clock to freq %luHz (period %lu ms)\n",(int)(51),G_green_period);

	//// A match to this will toggle the green LED.
	//// Regardless of its value (provided it is less than OCR1A), it will match at the frequency of timer 1.
	//OCR1B = 1;

	////Enable output compare match interrupt on timer 1B
}

//INTERRUPT HANDLERS
ISR(TIMER0_COMPA_vect) {
	// This is the Interrupt Service Routine for Timer0 (10ms clock used for scheduling red).
	// Each time the TCNT count is equal to the OCR0 register, this interrupt is "fired".
	
	G_ms_ticks++; // Increment ticks

	// if time to toggle the RED LED, set flag to release
	if ( ( G_ms_ticks % G_red_period ) == 0 )
	{
		G_release_red = 1;		
	}	
}

