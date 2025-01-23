/*
 * Scheduler_Round_Robin.c
 *
 *  Created on: Dec 9, 2024
 *      Author: Yesus
 */

#include "Scheduler_Round_Robin.h"

/*********************************
 * @fn								-init_scheduler_stack
 *
 * @brief							-Initiializes the scheduler stack pointer to the specified start address
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
