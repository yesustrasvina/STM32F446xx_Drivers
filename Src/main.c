#include <stdint.h>
#include <string.h> //For memset
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void GPIOxInit(void);
void Delay(void);

/*BUTTON*/
//PC13
/*LED*/
//PA5

uint8_t flag = 0;

int main(void)
{
	GPIOxInit();
	GPIO_IRQInterruptConfig(EXTI15_10_IRQn, ENABLE);
	GPIO_IRQPriorityConfig(EXTI15_10_IRQn, NVIC_IRQ_PRI5);

	while(1)
	{
		if(flag == 1)
		{
			Delay(); //Debounce
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
			flag = 0;
		}
	}
	return 0;
}


void GPIOxInit(void)
{
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Handle_t LED;

	memset (&LED,0,sizeof(LED)); //Initialize all in 0

	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Init(&LED);

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Handle_t BUTTON;

	memset (&BUTTON,0,sizeof(BUTTON)); //Initialize all in 0

	BUTTON.pGPIOx = GPIOC;
	BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PD;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&BUTTON);

}

void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_13);
	flag = 1;
 }

void Delay(void)
{
	for(uint32_t i = 0; i <= 60000; i++);
}
