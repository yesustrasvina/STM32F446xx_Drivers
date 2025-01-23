/*
 * Scheduler_Round_Robin.h
 *
 *  Created on: Dec 9, 2024
 *      Author: Yesus
 */

#ifndef INC_SCHEDULER_ROUND_ROBIN_H_
#define INC_SCHEDULER_ROUND_ROBIN_H_

#include "stm32f446xx.h"

/*Stack Memory Calculations*/
/*SRAM1 BEGINS 0x20000000 end in 0x2001BFFF*/
/*SRAM2 BEGINS 0x2001C000 end in 0x2001FFFF*/
/*SRAM1 + SRAM2 = 0x20020000*/
#define SIZE_TASK_STACK				1024U //1KByte
#define SIZE_SCHEDULER_STACK		1024U //1KByte

#define SRAM_START 					0x20000000U
#define SRAM_SIZE					((128)*(1024))
#define SRAM_END					((SRAM_START)+(SRAM_SIZE))

#define T1_STACK_START				SRAM_END
#define T2_STACK_START				((SRAM_END)-(SIZE_TASK_STACK))
#define T3_STACK_START				((SRAM_END) - (2 * SIZE_TASK_STACK))
#define T4_STACK_START				((SRAM_END) - (3 * SIZE_TASK_STACK))
#define IDLE_STACK_START			((SRAM_END) - (4 * SIZE_TASK_STACK))
#define SCHEDULER_STACK_START		((SRAM_END) - (5 * SIZE_TASK_STACK))

__attribute__ ((naked)) void init_scheduler_stack(uint32_t scheduler_stack_start);

/*Task Handlers*/
void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);
void idle_task(void);

#endif /* INC_SCHEDULER_ROUND_ROBIN_H_ */
