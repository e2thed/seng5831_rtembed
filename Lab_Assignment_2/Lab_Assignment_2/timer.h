/*******************************************
*
* Header file for Timer stuff.
*
*******************************************/
#ifndef __MOTOR_ISR_HANDLER_H
#define __MOTOR_ISR_HANDLER_H

#include <inttypes.h> //gives us uintX_t

// number of empty for loops to eat up about 1 ms
#define FOR_COUNT_10MS 5630

volatile uint32_t __ii;

#define WAIT_10MS {for (__ii=0;__ii<FOR_COUNT_10MS; __ii++);}

#define G_TIMER_RESOLUTION 100
#define Y_TIMER_RESOLUTION 100

void init_timers();

#endif //__TIMER_H
