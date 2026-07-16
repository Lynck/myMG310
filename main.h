#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "interrupt.h"

/* Main loop flags shared with timer/ADC interrupts. */
extern volatile uint16_t ADC_Val;
extern volatile bool start_100ms_timer;
extern volatile bool check_100ms_flag;
extern bool OLED_Flag;
extern bool timer_10ms_flag;

#endif  /* _MAIN_H_ */
