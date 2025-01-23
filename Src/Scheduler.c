#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "Scheduler_Round_Robin.h"



void GPIOx_Init(void);
void Clock_Init(void);

int main(void)
{

	/*GPIOs Initializations*/
	GPIOx_Init();

	/*Set MSP to point to scheduler task*/
	__attribute__ ((naked)) void init_scheduler_stack(uint32_t scheduler_stack_start);

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

	GPIO_Init(&GPIO_LED_struct);
}

void task1_handler(void)
{
	while(1)
	{

	}
}
void task2_handler(void)
{
	while(1)
	{

	}
}
void task3_handler(void)
{
	while(1)
	{

	}
}
void task4_handler(void)
{
	while(1)
	{

	}
}
