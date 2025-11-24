#ifndef __LED_TASK_H
#define __LED_TASK_H

// LED task update period in ms
#define LED_UPDATE_PERIOD 10

#define LED_TASK_PRIORITY     2

#define LED_TASK_STACK_SIZE   10000

enum Led_Stat {LED_STAT_STARTUP=0,LED_ERROR_CRSF, LED_ERROR_GPS, LED_STAT_GOOD, LED_STAT_LINK_LOST, LED_STAT_WIFI};

extern StackType_t LED_task_stack[LED_TASK_STACK_SIZE];

extern StaticTask_t LED_task;

extern Led_Stat board_led_state;

void LED_task_func(void *p);

#endif
