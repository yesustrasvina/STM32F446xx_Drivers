#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "Scheduler_Round_Robin.h"



void GPIOx_Init(void);

uint8_t current_task = 1;
uint32_t g_tick_count = 0;

int main(void)
{

	/*GPIOs Initializations*/
	GPIOx_Init();

	/*Set MSP to point to scheduler task*/
	init_scheduler_stack(SCHEDULER_STACK_START);

	/* Initializing Tasks Stacks with Initial values */
	init_tasks_stack();

	/* Switching to use PSP instead of MSP */
	switch_sp_to_psp();

	/* Configuring SysTick timer
	 * 1 system timer interruption every 1ms */
	init_systick_timer(TICK_HZ);

	task1_handler(); // Launching Task1

	while(1)
	{

	}
}

void GPIOx_Init(void)
{
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Handle_t GPIO_LED_struct;

	memset(&GPIO_LED_struct, 0, sizeof(GPIO_LED_struct));
	GPIO_LED_struct.pGPIOx = GPIOA;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

	GPIO_Init(&GPIO_LED_struct); /* Initializing LED PA5 */

	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8; /* Initializing LED PA8 */
	GPIO_Init(&GPIO_LED_struct);

	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9; /* Initializing LED PA9 */
	GPIO_Init(&GPIO_LED_struct);

	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10; /* Initializing LED PA10 */
	GPIO_Init(&GPIO_LED_struct);

}

void task1_handler(void)
{
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
	}
}
void task2_handler(void)
{
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_8);
	}
}
void task3_handler(void)
{
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_9);
	}
}
void task4_handler(void)
{
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_10);
	}
}

void idle_task(void)
{
	while(1)
	{

	}
}
