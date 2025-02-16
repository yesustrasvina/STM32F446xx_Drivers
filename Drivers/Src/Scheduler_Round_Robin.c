/*
 * Scheduler_Round_Robin.c
 *
 *  Created on: Dec 9, 2024
 *      Author: Yesus
 */

#include "Scheduler_Round_Robin.h"

/* Creating control task block for each task */
TCB_t user_tasks[MAX_TASKS];

/*********************************
 * @fn								-init_scheduler_stack
 *
 * @brief							-Initializes the scheduler stack pointer to the specified start address
 *
 * @param scheduler_stack_start		-Start address of the scheduler stack.
 *
 * @return							-None
 *
 */

__attribute__((naked)) void init_scheduler_stack(uint32_t scheduler_stack_start)
{
	__asm volatile("MSR  MSP,%0": : "r"  (scheduler_stack_start) : ); //Copy start value to MSP
	__asm volatile("BX LR"); //Write it to return since there is not epilogue
}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t count_value = SYSTIM_TIM_CLK / tick_hz; // This is the reload value

	/* Clear RVR */
	*SYST_RVR &= ~(0xFFFFFFFF);

	/* Load the value into SysTick Reload Value Register */
	*SYST_RVR |= (count_value - 1); // Reload value should be N-1, Example: 16000 - 1

	/* SysTick Setting in SysTick Control and Status Register */
	/* Enabling Systick exception request */
	*SYST_CSR |= ( 1 << TICKINT_BIT );
	/* Selecting processor clock */
	*SYST_CSR |= ( 1 << CLKSOURCE_BIT );

	/* Enabling SysTick timer */
	*SYST_CSR |= ( 1 << SYSTICK_ENABLE_BIT);
}

/***************************
 * @fn				- init_tasks_stack
 *
 * @brief			- Initializes the Process Stack Pointer(PSP) and state for all user tasks.
 *
 * @param			- None
 *
 * @return			-None
 */
void init_tasks_stack(void)
{
	/* Setting initial tasks state */
	user_tasks[0].current_state = TASK_RUNNING_STATE;
	user_tasks[1].current_state = TASK_RUNNING_STATE;
	user_tasks[2].current_state = TASK_RUNNING_STATE;
	user_tasks[3].current_state = TASK_RUNNING_STATE;
	user_tasks[4].current_state = TASK_RUNNING_STATE;

	/* Setting initial PSP values for tasks */
	user_tasks[0].psp_value = IDLE_STACK_START;
	user_tasks[1].psp_value = T1_STACK_START;
	user_tasks[2].psp_value = T2_STACK_START;
	user_tasks[3].psp_value = T3_STACK_START;
	user_tasks[4].psp_value = T4_STACK_START;

	/* Init handlers pointers for tasks */
	user_tasks[0].task_handler = idle_task;
	user_tasks[1].task_handler = task1_handler;
	user_tasks[2].task_handler = task2_handler;
	user_tasks[3].task_handler = task3_handler;
	user_tasks[4].task_handler = task4_handler;

	uint32_t *pPSP;

	for(unsigned char i = 0; i < MAX_TASKS; i++)
	{
		pPSP = (uint32_t *) user_tasks[i].psp_value; // Point o the task PSP

		pPSP--; //Decrement stack position
		*pPSP = DUMMY_XPSR; // Saving XPSR value

		pPSP--; //Decrement stack position
		*pPSP = (uint32_t) user_tasks[i].task_handler; // Saving task handler

		pPSP--; //Decrement stack position
		*pPSP = EXC_RETURN; // Saving LR with special value

		for(unsigned char j = 0; j < 13; j++)
		{
			pPSP--; //Decrement stack position
			*pPSP = 0; // Initializing with 0 General register x
		}
		user_tasks[i].psp_value = (uint32_t) pPSP;
	}

}

/*********************
 * @fn			- switch_sp_to_psp
 *
 * @brief		- Switches the stack pointer from MSP to PSP
 *
 *
 * @param		- None
 *
 * @return		- None
 * */
__attribute__((naked)) void switch_sp_to_psp(void)
{
	/* Initialize the PSP with TASK1 stack start address */
	// Get value of PSP of current task
	__asm volatile ("PUSH {LR}"); // Saving LR before calling function get PSP value
	__asm volatile ("BL get_psp_value"); // Call function with Branch with Link, By standard return value is stored in R0
	__asm volatile ("MSR PSP, R0"); // Copy R0 value (current task psp) to PSP register
	__asm volatile ("POP {LR}"); // Restoring LR value

	/* Change SP to PSP using CONTROL Register */
	__asm volatile ("MOV R0, #0X02");
	__asm volatile ("MSR CONTROL, R0");
	__asm volatile ("BX LR");
}

/*********************
 * @fn			- get_psp_value
 *
 * @brief		- Retrieves the current PSP value for the active task
 *
 *
 * @param		- None
 *
 * @return		- Current PSP value
 * */
uint32_t get_psp_value(void)
{
	return user_tasks[current_task].psp_value;
}

void SysTick_Handler(void)
{
	update_global_tick_count();
	unblock_tasks();
	// Pend the PENDSV exception
	*ICSR |= (1 << PENDSVSET_BIT);
}

void update_global_tick_count(void)
{
	g_tick_count++;
}
/***********************************************
 * @fn			- unblock_tasks
 *
 * @brief		- Unblocks tasks whose block count matches the global tick count.
 *
 * @param		- None
 *
 * @return		- None
 */
void unblock_tasks(void)
{
	for(unsigned char i = 1; i < MAX_TASKS; i++)
	{
		/* Check if the task is not running and if its blocked */
		if(user_tasks[i].current_state != TASK_RUNNING_STATE)
		{
			/* Check if the task has reached the unblock time */
			if(user_tasks[i].block_count == g_tick_count)
			{
				/* Unblock the task and set its state to running */
				user_tasks[i].current_state = TASK_RUNNING_STATE;
			}
		}
	}
}

__attribute__((naked)) void PendSV_Handler(void)
{
	/* Save the context of current task */
	// 1. Get current running task's PSP value
	__asm volatile ("MRS R0, PSP");
	// 2. Using that PSP value STORE SF2 ( R4 - R11 )
	__asm volatile ("STMDB R0!, {R4,R11}");

	__asm volatile ("PUSH {LR}"); // Saving LR before calling function Save psp value

	__asm volatile ("BL save_psp_value"); 	// Is not necessary to put the PSP value as a parameter because R0 already has that value,
											// When save_psp_value is called, R0 value is passed  as parameter, this because to the Procedure Call Standard


	/* Retrieve the context of the next task */
	// 1. Decide next task to run
	__asm volatile ("BL update_next_task");

	// 2. Get its past PSP value
	__asm volatile ("BL get_psp_value"); // PSP value is returned  in R0 by standard

	// 3. Using that PSP value to retrieve SF2 (R4 - R11)
	__asm volatile ("LDMIA R0!,{R4-R11}");

	// 4. Update PSP and Exit
	__asm volatile ("MSR PSP, R0");

	//  5. Restore LR value (0xFFFFFFFD) from stack
	__asm volatile ("POP {LR}");

	__asm volatile ("BX LR");
}


/************************************************************************
 * @fn			- save_psp_value
 *
 * @brief		- Saves the current PSP value for the active task.
 *
 * @param		- PSP value to save.
 *
 * @return		- None
 */
void save_psp_value(uint32_t current_psp_value)
{
	user_tasks[current_task].psp_value = current_psp_value;
}

void update_next_task(void)
{
	uint8_t state = TASK_BLOCKED_STATE;

	for(unsigned char i = 0; i < MAX_TASKS; i++)
	{
		/* Move to the next task in the round-robin queue */
		current_task++;

		if(current_task == MAX_TASKS)
		{
			current_task = 0;
		}

		/* Check the state of the next task */
		state = user_tasks[current_task].current_state;

		/* Select the task if it is runnable and not the idle task (task 0) */
		if( (state == TASK_RUNNING_STATE) && (current_task != 0) )
		{
			break;
		}
	}
	/* Go to idle task if no other task is runnable */
	if(state != TASK_RUNNING_STATE)
	{
		current_task = 0;
	}
}









