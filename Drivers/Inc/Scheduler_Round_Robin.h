/*
 * Scheduler_Round_Robin.h
 *
 *  Created on: Dec 9, 2024
 *      Author: Yesus
 */

#ifndef INC_SCHEDULER_ROUND_ROBIN_H_
#define INC_SCHEDULER_ROUND_ROBIN_H_

#include "stm32f446xx.h"

#define MAX_TASKS		5

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

/* Systick Timer Configurations defines */
#define TICK_HZ				1000U //Tick Exception frequency
#define HSI_CLOCK 			16000000U // Internal MCU frequency
#define SYSTIM_TIM_CLK		HSI_CLOCK // Systick timer source clock

/* Defines for context initial values */
#define DUMMY_XPSR			0x1000000U // T_Bit = 1
#define EXC_RETURN			0xFFFFFFFDU /// Return to thread mode and use PSP

#define TASK_RUNNING_STATE 		0x00
#define TASK_BLOCKED_STATE 		0xFF

#define INTERRUPT_ENABLE()		do{ __asm volatile ("MOV R0, #0x1"); __asm volatile("MSR PRIMASK,R0"); } while(0)
#define INTERRUPT_DISABLE()		do{ __asm volatile ("MOV R0, #0x0"); __asm volatile("MSR PRIMASK,R0"); } while(0)

/* Task Control Block (TCB) Structure Definition */
typedef struct
{
	uint32_t psp_value;					// Process stack pointer (PSP)
	uint32_t block_count;				// Tick count when task is unblocked
	uint8_t current_state;				// Current state of the task (RUNNING,BLOCKED)
	void (*task_handler)(void);   		// Pointer o the function (task handler) that the task executes
}TCB_t;

extern uint8_t current_task;
extern uint32_t g_tick_count;

__attribute__ ((naked)) void init_scheduler_stack(uint32_t scheduler_stack_start);
void init_systick_timer(uint32_t tick_hz);
void init_tasks_stack(void);
__attribute__((naked)) void switch_sp_to_psp(void);
uint32_t get_psp_value(void);
void SysTick_Handler(void);
void update_global_tick_count(void);
__attribute__((naked)) void PendSV_Handler(void);
void save_psp_value(uint32_t current_psp_value);
void unblock_tasks(void);
void task_delay(uint32_t tick_count);
void schedule(void);

/*Task Handlers*/
void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);
void idle_task(void);

#endif /* INC_SCHEDULER_ROUND_ROBIN_H_ */
