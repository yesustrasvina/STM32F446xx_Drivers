/*
 * stm32f446xx.h
 *
 *  Created on: Oct 21, 2024
 *      Author: Yesus
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include <stdint.h>

#define __vo volatile
#define __weak __atribbute__((weak))

/*CORTEX M4 Processor specific*/
/*Interrupt Set-enable Registers*/
#define NVIC_ISER0         ((__vo uint32_t*)0xE000E100U) //0-31
#define NVIC_ISER1         ((__vo uint32_t*)0xE000E104U) //32-63
#define NVIC_ISER2         ((__vo uint32_t*)0xE000E108U) //64-95

/*Interrupt Clear-enable Registers*/
#define NVIC_ICER0         ((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1         ((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2         ((__vo uint32_t*)0XE000E188U)

/*Interrupt Priority Registers*/
#define NVIC_IPR_BASE_ADDR ((__vo uint32_t*)0xE000E400)

/*Systick registers*/
#define SYST_RVR		((__vo uint32_t*)0xE000E014U) // Systick reload value register
#define SYST_CSR		((__vo uint32_t*)0xE000E010U) // Systick control and status register

/* Systick Control and Status Register Bits positions */
#define TICKINT_BIT			1
#define CLKSOURCE_BIT		2
#define SYSTICK_ENABLE_BIT	0

/* System Control Block Registers */
#define ICSR			((__vo uint32_t*)0xE000ED04U)

/* Interrupt Control and State Register Bits positions */
#define PENDSVSET_BIT	28


/*----------BASE ADDRESSES BEGIN----------*/

/*Memories*/
/*All this definitions can be find in the page 57 of the reference manual
 * Figure 2. Memory map*/
#define FLASH_BASEADDR      	  0x08000000U
#define SRAM1_BASEADDR      	  0x20000000U
#define SRAM2_BASEADDR      	  0x2001C000U
#define ROM_BASEADDR        	  0x1FFF0000U /*System memory*/

/*Peripheral buses*/
#define APB1_BASEADDR       	  0x40000000U
#define APB2_BASEADDR       	  0x40010000U
#define AHB1_BASEADDR       	  0x40020000U
#define AHB2_BASEADDR       	  0x50000000U
#define AHB3_BASEADDR       	  0x60000000U

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * */
#define TIM2_BASEADDR       	  0x40000000U
#define TIM3_BASEADDR       	  0x40000400U
#define TIM4_BASEADDR       	  0x40000800U
#define TIM5_BASEADDR      		  0x40000C00U
#define TIM6_BASEADDR       	  0x40001000U
#define TIM7_BASEADDR       	  0x40001400U
#define TIM12_BASEADDR      	  0x40001800U
#define TIM13_BASEADDR      	  0x40001C00U
#define TIM14_BASEADDR      	  0x40002000U
#define RTC_BKP_BASEADDR    	  0x40002800U
#define WWDG_BASEADDR       	  0x40002C00U
#define IWDG_BASEADDR       	  0x40003000U
#define SPI2_I2S2_BASEADDR  	  0x40003800U
#define SPI3_I2S3_BASEADDR  	  0x40003C00U
#define SPDIF_RX_BASEADDR   	  0x40004000U
#define USART2_BASEADDR     	  0x40004400U
#define USART3_BASEADDR    	  	  0x40004800U
#define UART4_BASEADDR      	  0x40004C00U
#define UART5_BASEADDR      	  0x40005000U
#define I2C1_BASEADDR       	  0x40005400U
#define I2C2_BASEADDR       	  0x40005800U
#define I2C3_BASEADDR       	  0x40005C00U
#define CAN1_BASEADDR       	  0x40006400U
#define CAN2_BASEADDR       	  0x40006800U
#define HDMI_CEC_BASEADDR   	  0x40006C00U
#define PWR_BASEADDR          	  0x40007000U
#define DAC_BASEADDR        	  0x40007400U

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * */
#define TIM1_BASEADDR      		  0x40010000U
#define TIM8_BASEADDR      		  0x40010400U
#define USART1_BASEADDR   		  0x40011000U
#define USART6_BASEADDR           0x40011400U
#define ADC1_ADC2_ADC3_BASEADDR   0x40012000U
#define SDMMC_BASEADDR            0x40012C00U
#define SPI1_BASEADDR             0x40013000U
#define SPI4_BASEADDR             0x40013400U
#define SYSCFG_BASEADDR           0x40013800U
#define EXTI_BASEADDR             0x40013C00U
#define TIM9_BASEADDR             0x40014000U
#define TIM10_BASEADDR            0x40014400U
#define TIM11_BASEADDR 			  0x40014800U
#define SAI1_BASEADDR			  0x40015800U
#define SAI2_BASEADDR       	  0x40015C00U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * */
#define GPIOA_BASEADDR			  0x40020000U
#define GPIOB_BASEADDR			  0x40020400U
#define GPIOC_BASEADDR			  0x40020800U
#define GPIOD_BASEADDR			  0x40020C00U
#define GPIOE_BASEADDR			  0x40021000U
#define GPIOF_BASEADDR			  0x40021400U
#define GPIOG_BASEADDR			  0x40021800U
#define GPIOH_BASEADDR			  0x40021C00U
#define CRC_BASEADDR			  0x40023000U
#define RCC_BASEADDR			  0x40023800U
#define FLASH_INTERFACE_BASEADDR  0x40023C00U
#define BKPSRAM_BASEADDR		  0x40024000U
#define DMA1_BASEADDR			  0x40026000U
#define DMA2_BASEADDR			  0x40026400U
#define USB_OTG_HS_BASEADDR		  0x40040000U

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 * */
#define USB_OTG_FS_BASEADDR  	  0x50000000U
#define DCMI_BASEADDR			  0x50050000U

/*
 * Base addresses of peripherals which are hanging on AHB3 bus
 * */
#define QUADSPI_BASEADDR          0xA0001000U
#define FMC_BASEADDR			  0xA0000000U


/***********Peripheral Register Definitions Structure***********/
/*Note: Registers of a peripheral are specific for stm32f446xx*/

/*Peripheral register definition structure for GPIO*/
typedef struct
{
	__vo uint32_t GPIOx_MODER;       /*GPIO port mode register. These bits are written by software to configure the I/O direction mode.                   OFFSET (0x00)*/
	__vo uint32_t GPIOx_OTYPER;      /*GPIO port output type register. These bits are written by software to configure the output type of the I/O port.   OFFSET (0x04)*/
	__vo uint32_t GPIOx_OSPEEDR;     /*GPIO port output speed register. These bits are written by software to configure the I/O output speed.             OFFSET (0x08)*/
	__vo uint32_t GPIOx_PUPDR;       /*GPIO port pull-up/pull-down register. These bits are written by software to configure the I/O pull-up or pull-down OFFSET (0x0C)*/
	__vo uint32_t GPIOx_IDR;         /*GPIO port input data register. Read-only bits. Contain the input value of the I/O port                             OFFSET (0x10)*/
	__vo uint32_t GPIOx_ODR;         /*GPIO port output data register. These bits can be read and written by software. ODR 0 to 15                        OFFSET (0x14)*/
	__vo uint32_t GPIOx_BSRR;        /*GPIO port bit set/reset register                                                                                   OFFSET (0x18)*/
	__vo uint32_t GPIOx_LCKR;        /*GPIO port configuration lock register.                                                                             OFFSET (0x1C)*/
	__vo uint32_t GPIOx_AFR[2];      /*GPIO alternate function register. [0] low register and [1] high register                                           OFFSET (0x20)(0x24)*/
}GPIO_Reg_Def_t;

/*Peripheral register definition structure for RCC*/
typedef struct
{
	__vo uint32_t RCC_CR;            /*RCC clock control register. These bits are written by software to turn on the clocks                               OFFSET (0x00)*/
	__vo uint32_t RCC_PLLCFGR;       /*RCC PLL configuration register. This register is used to configure the PLL clock outputs                           OFFSET (0x04)*/
	__vo uint32_t RCC_CFGR;          /*RCC clock configuration register                                                                                   OFFSET (0x08)*/
	__vo uint32_t RCC_CIR;           /*RCC clock interrupt register                                                                                       OFFSET (0x0C)*/
	__vo uint32_t RCC_AHB1RSTR;      /*RCC AHB1 peripheral reset register                                                                                 OFFSET (0x10)*/
	__vo uint32_t RCC_AHB2RSTR;      /*RCC AHB2 peripheral reset register                                                                                 OFFSET (0x14)*/
	__vo uint32_t RCC_AHB3RSTR;      /*RCC AHB3 peripheral reset register                                                                                 OFFSET (0x18)*/
	__vo uint32_t RCC_RESERVED_1;    /*These 4 bytes are reserved                                                                                         OFFSET (0x1C)*/
	__vo uint32_t RCC_APB1RSTR;      /*RCC APB1 peripheral reset register                                                                                 OFFSET (0x20)*/
	__vo uint32_t RCC_APB2RSTR;      /*RCC APB2 peripheral reset register                                                                                 OFFSET (0x24)*/
	__vo uint32_t RCC_RESERVED_2;    /*These 4 bytes are reserved                                                                                         OFFSET (0x28)*/
	__vo uint32_t RCC_RESERVED_3;    /*These 4 bytes are reserved                                                                                         OFFSET (0x2C)*/
	__vo uint32_t RCC_AHB1ENR;       /*RCC AHB1 peripheral clock enable register                                                                          OFFSET (0x30)*/
	__vo uint32_t RCC_AHB2ENR;       /*RCC AHB2 peripheral clock enable register                                                                          OFFSET (0x34)*/
	__vo uint32_t RCC_AHB3ENR;       /*RCC AHB3 peripheral clock enable register                                                                          OFFSET (0x38)*/
	__vo uint32_t RCC_RESERVED_4;    /*These 4 bytes are reserved                                                                                         OFFSET (0x3C)*/
	__vo uint32_t RCC_APB1ENR;       /*RCC APB1 peripheral clock enable register                                                                          OFFSET (0x40)*/
	__vo uint32_t RCC_APB2ENR;       /*RCC APB2 peripheral clock enable register                                                                          OFFSET (0x44)*/
	__vo uint32_t RCC_RESERVED_5;    /*These 4 bytes are reserved                                                                                         OFFSET (0x48)*/
	__vo uint32_t RCC_RESERVED_6;    /*These 4 bytes are reserved                                                                                         OFFSET (0x4C)*/
	__vo uint32_t RCC_RESERVED_7;    /*These 4 bytes are reserved                                                                                         OFFSET (0x50)*/
	__vo uint32_t RCC_AHB2LPENR;     /*RCC AHB2 peripheral clock enable in low power mode register                                                        OFFSET (0x54)*/
	__vo uint32_t RCC_AHB3LPENR;     /*RCC AHB3 peripheral clock enable in low power mode register                                                        OFFSET (0x58)*/
	__vo uint32_t RCC_RESERVED_8;    /*These 4 bytes are reserved                                                                                         OFFSET (0x5C)*/
	__vo uint32_t RCC_RESERVED_9;    /*These 4 bytes are reserved                                                                                         OFFSET (0x60)*/
	__vo uint32_t RCC_APB2LPENR;     /*RCC APB2 peripheral clock enabled in low power mode register                                                       OFFSET (0x64)*/
	__vo uint32_t RCC_RESERVED_10;   /*These 4 bytes are reserved                                                                                         OFFSET (0x68)*/
	__vo uint32_t RCC_RESERVED_11;   /*These 4 bytes are reserved                                                                                         OFFSET (0x6C)*/
	__vo uint32_t RCC_BDCR;          /*RCC Backup domain control register                                                                                 OFFSET (0x70)*/
	__vo uint32_t RCC_CSR;           /*RCC clock control & status register                                                                                OFFSET (0x74)*/
	__vo uint32_t RCC_RESERVED_12;   /*These 4 bytes are reserved                                                                                         OFFSET (0x78)*/
	__vo uint32_t RCC_RESERVED_13;   /*These 4 bytes are reserved                                                                                         OFFSET (0x7C)*/
	__vo uint32_t RCC_SSCGR;         /*RCC spread spectrum clock generation register                                                                      OFFSET (0x80)*/
	__vo uint32_t RCC_PLLI2SCFGR;    /*RCC PLLI2S configuration register                                                                                  OFFSET (0x84)*/
	__vo uint32_t RCC_PLLSAICFGR;    /*RCC PLL configuration register                                                                                     OFFSET (0x88)*/
	__vo uint32_t RCC_DCKCFGR;       /*RCC dedicated clock configuration register                                                                         OFFSET (0x8C)*/
	__vo uint32_t CKGATENR;          /*RCC clocks gated enable register                                                                                   OFFSET (0x90)*/
	__vo uint32_t DCKCFGR2;          /*RCC dedicated clocks configuration register 2                                                                      OFFSET (0x94)*/

}RCC_Reg_Def_t;


/*Peripheral register definition structure for EXTI*/
typedef struct
{
	__vo uint32_t EXTI_IMR;          /*Interrupt mask register                                                                                            OFFSET (0x00)*/
	__vo uint32_t EXTI_EMR;          /*Event mask register                                                                                                OFFSET (0x04)*/
	__vo uint32_t EXTI_RTSR;         /*Rising trigger selection register                                                                                  OFFSET (0x08)*/
	__vo uint32_t EXTI_FTSR;         /*Falling trigger selection register                                                                                 OFFSET (0x0C)*/
	__vo uint32_t EXTI_SWIER;        /*Software interrupt event register                                                                                  OFFSET (0x10)*/
	__vo uint32_t EXTI_PR;           /*Pending register                                                                                                   OFFSET (0x14)*/
}EXTI_Reg_Def_t;

/*Peripheral register definition structure for SYSCFG*/
typedef struct
{
	__vo uint32_t SYSCFG_MEMRMP;     /*SYSCFG memory remap register                                                                                       OFFSET (0x00)*/
	__vo uint32_t SYSCFG_PMC;        /*SYSCFG peripheral mode configuration register                                                                      OFFSET (0x04)*/
	__vo uint32_t SYSCFG_EXTICR[4];  /*SYSCFG external interrupt configuration register 1 2 3 4                                                           OFFSET (0x08)to (0x14)*/
	__vo uint32_t SYSCFG_RESERVED1;  /*These 4 bytes are reserved                                                                                         OFFSET (0x18)*/
	__vo uint32_t SYSCFG_RESERVED2;  /*These 4 bytes are reserved                                                                                         OFFSET (0x1C)*/
	__vo uint32_t SYSCFG_CMPCR;      /*Compensation cell control register                                                                                 OFFSET (0x20)*/
	__vo uint32_t SYSCFG_RESERVED3;  /*These 4 bytes are reserved                                                                                         OFFSET (0x24)*/
	__vo uint32_t SYSCFG_RESERVED4;  /*These 4 bytes are reserved                                                                                         OFFSET (0x28)*/
	__vo uint32_t SYSCFG_CFGR;       /*SYSCFG configuration register                                                                                      OFFSET (0x2C)*/
}SYSCFG_Reg_Def_t;

/*Peripheral register definition structure for SPI*/
typedef struct
{
	__vo uint32_t SPI_CR1;           /*SPI control register 1. Not used in I2S mode                                                                       OFFSET (0x00)*/
	__vo uint32_t SPI_CR2;           /*SPI control register 2                                                                                             OFFSET (0x04)*/
	__vo uint32_t SPI_SR;            /*SPI status register                                                                                                OFFSET (0x08)*/
	__vo uint32_t SPI_DR;            /*SPI data register. Data received or to be transmitted.                                                             OFFSET (0x0C)*/
	__vo uint32_t SPI_CRCPR;         /*SPI CRC polynomial register. Not used in I2S mode                                                                  OFFSET (0x10)*/
	__vo uint32_t SPI_RXCRCR;        /*SPI RX CRC register. Not used in I2S mode                                                                          OFFSET (0x14)*/
	__vo uint32_t SPI_TXCRCR;        /*SPI TX CRC register. Not used in I2S mode                                                                          OFFSET (0x18)*/
	__vo uint32_t SPI_I2SCFGR;       /*SPI_I2S configuration register                                                                                     OFFSET (0x1C)*/
	__vo uint32_t SPI_I2SPR;         /*SPI_I2S prescaler register                                                                                         OFFSET (0x20)*/
}SPI_Reg_Def_t;

/*Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )*/
#define GPIOA  ((GPIO_Reg_Def_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_Reg_Def_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_Reg_Def_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_Reg_Def_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_Reg_Def_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_Reg_Def_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_Reg_Def_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_Reg_Def_t*)GPIOH_BASEADDR)

#define RCC    ((RCC_Reg_Def_t*)RCC_BASEADDR)

#define EXTI   ((EXTI_Reg_Def_t*)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_Reg_Def_t*)SYSCFG_BASEADDR)

#define SPI1        ((SPI_Reg_Def_t*)SPI1_BASEADDR)
#define SPI2        ((SPI_Reg_Def_t*)SPI2_I2S2_BASEADDR)
#define SPI3        ((SPI_Reg_Def_t*)SPI3_I2S3_BASEADDR)
#define SPI4        ((SPI_Reg_Def_t*)SPI4_BASEADDR)

/*
 * Peripheral clocks enable macros
 * */

/*Clock enable macros for GPIOx peripherals*/
#define GPIOA_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PERI_CLOCK_ENABLE() (RCC->RCC_AHB1ENR |= (1 << 7))

/*Clock enable macro for SYSCFG peripheral*/
#define SYSCFG_PERI_CLOCK_ENABLE() (RCC->RCC_APB2ENR) |= (1 << 14)

/*Clock enable macro for SPIx peripheral*/
#define SPI1_PERI_CLOCK_ENABLE()   (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PERI_CLOCK_ENABLE()   (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PERI_CLOCK_ENABLE()   (RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PERI_CLOCK_ENABLE()   (RCC->RCC_APB2ENR |= (1 << 13))

/*
 * Peripheral clock disable macros
 * */
/*Clock disable macros for GPIOx peripherals*/
#define GPIOA_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PERI_CLOCK_DISABLE() (RCC->RCC_AHB1ENR &= ~(1 << 7))

/*Clock disable macro for SYSCFG peripheral*/
#define SYSCFG_PERI_CLOCK_DISABLE() (RCC->RCC_APB2ENR) &= ~(1 << 14)

/*Clock Disable Macros for SPIx peripherals*/
#define SPI1_PERI_CLOCK_DISABLE()   (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PERI_CLOCK_DISABLE()   (RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PERI_CLOCK_DISABLE()   (RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_PERI_CLOCK_DISABLE()   (RCC->RCC_APB2ENR &= ~(1 << 13))

/*Reset macros for GPIOx peripherals*/
#define GPIOA_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_RESET()    do{(RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7));}while(0)

/*General macros definitions*/
#define DISABLE 0
#define ENABLE  1


static inline uint8_t GPIO_BASEADDR_TO_CODE(GPIO_Reg_Def_t *pGPIOx)
{
	if       (pGPIOx == GPIOA) return 0;
	else if  (pGPIOx == GPIOB) return 1;
	else if  (pGPIOx == GPIOC) return 2;
	else if  (pGPIOx == GPIOD) return 3;
	else if  (pGPIOx == GPIOE) return 4;
	else if  (pGPIOx == GPIOF) return 5;
	else if  (pGPIOx == GPIOG) return 6;
	else if  (pGPIOx == GPIOH) return 7;
	else                       return 0;
}

/*STM32 specific Interrupt Numbers*/
typedef enum
{
	WWDG_IRQn                        = 0,   /*Window Watchdog interrupt                                                                               ADDRESS (0x00000040)*/
	PVD_IRQn                         = 1,   /*PVD through EXTI line detection interrupt                                                               ADDRESS (0x00000044)*/
	TAMP_STAMP_IRQn                  = 2,   /*Tamper and TimeStamp interrupts through the EXTI line*/
	RTC_WKUP_IRQn                    = 3,   /*RTC Wakeup interrupt through the EXTI line*/
	FLASH_IRQn                       = 4,   /* Flash global interrupt*/
	RCC_IRQn                         = 5,   /*RCC global interrupt*/
	EXTI0_IRQn                       = 6,   /*EXTI Line0 interrupt*/
	EXTI1_IRQn                       = 7,   /*EXTI Line1 interrupt*/
	EXTI2_IRQn                       = 8,   /*EXTI Line2 interrupt*/
	EXTI3_IRQn                       = 9,   /*EXTI Line3 interrupt*/
	EXTI4_IRQn                       = 10,  /*EXTI Line4 interrupt*/
	DMA1_Stream0_IRQn                = 11,  /*DMA1 Stream0 global interrupt*/
	DMA1_Stream1_IRQn                = 12,  /*DMA1 Stream1 global interrupt*/
	DMA1_Stream2_IRQn                = 13,  /*DMA1 Stream2 global interrupt*/
	DMA1_Stream3_IRQn                = 14,  /*DMA1 Stream3 global interrupt*/
	DMA1_Stream4_IRQn                = 15,  /*DMA1 Stream4 global interrupt*/
	DMA1_Stream5_IRQn                = 16,  /*DMA1 Stream5 global interrupt*/
	DMA1_Stream6_IRQn                = 17,  /*DMA1 Stream6 global interrupt*/
	ADC_IRQn                         = 18,  /* ADC1, ADC2 and ADC3 global interrupts*/
	CAN1_TX_IRQn                     = 19,  /*CAN1 TX interrupts*/
	CAN1_RX0_IRQn                    = 20,  /* CAN1 RX0 interrupts*/
	CAN1_RX1_IRQn                    = 21,  /* CAN1 RX1 interrupts*/
	CAN1_SCE_IRQn                    = 22,  /* CAN1 SCE interrupt*/
	EXTI9_5_IRQn                     = 23,  /*EXTI Line[9:5] interrupts*/
	TIM1_BRK_TIM9_IRQn               = 24,  /*TIM1 Break interrupt and TIM9 global interrupt*/
	TIM1_UP_TIM10_IRQn               = 25,  /*TIM1 Update interrupt and TIM10 global interrupt*/
	TIM1_TRG_COM_TIM11_IRQn          = 26,  /*TIM1 Trigger and Commutation interrupts and TIM11 global interrupt*/
	TIM1_CC_IRQn                     = 27,  /*TIM1 Capture compare interrupt*/
	TIM2_IRQn                        = 28,  /*TIM2 global interrupt*/
	TIM3_IRQn                        = 29,  /*TIM3 global interrupt*/
	TIM4_IRQn                        = 30,  /*TIM4 global interrupt*/
	I2C1_EV_IRQn                     = 31,  /*I2C1 event interrupt*/
	I2C1_ER_IRQn                     = 32,  /*I2C1 error interrupt*/
	I2C2_EV_IRQn                     = 33,  /*I2C2 event interrupt*/
	I2C2_ER_IRQn                     = 34,  /*I2C2 error interrupt*/
	SPI1_IRQn                        = 35,  /*SPI1 global interrupt*/
	SPI2_IRQn                        = 36,  /*SPI2 global interrupt*/
	USART1_IRQn                      = 37,  /*USART1 global interrupt*/
	USART2_IRQn                      = 38,  /*USART2 global interrupt*/
	USART3_IRQn                      = 39,  /*USART3 global interrupt*/
	EXTI15_10_IRQn                   = 40,  /*EXTI Line[15:10] interrupts*/
	RTC_Alarm_IRQn                   = 41,  /*RTC Alarms (A and B) through EXTI line interrupt*/
	OTG_FS_WKUP_IRQn                 = 42,  /*USB On-The-Go FS Wakeup through EXTI line interrupt*/
	TIM8_BRK_TIM12_IRQn              = 43,  /*TIM8 break interrupt and TIM12 global interrupt*/
	TIM8_UP_TIM13_IRQn               = 44,  /*TIM8 Update interrupt and TIM13 global interrupt*/
	TIM8_TRG_COM_TIM14_IRQn          = 45,  /*TIM8 Trigger and Commutation interrupts and TIM14 global interrupt*/
	TIM8_CC_IRQn                     = 46,  /*TIM8 Capture compare interruptTIM8 Capture compare interrupt*/
	DMA1_Stream7_IRQn                = 47,  /*DMA1 Stream7 global interrupt*/
	FMC_IRQn                         = 48,  /*FMC global interrupt*/
	SDIO_IRQn                        = 49,  /*SDIO global interrupt*/
	TIM5_IRQn                        = 50,  /*TIM5 global interrupt*/
	SPI3_IRQn                        = 51,  /*SPI3 global interrupt*/
	UART4_IRQn                       = 52,  /*UART4 global interrupt*/
	UART5_IRQn                       = 53,  /*UART5 global interrupt*/
	TIM6_DAC_IRQn                    = 54,  /*TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts*/
	TIM7_IRQn                        = 55,  /*TIM7 global interrupt*/
	DMA2_Stream0_IRQn                = 56,  /*DMA2 Stream0 global interrupt*/
	DMA2_Stream1_IRQn                = 57,  /*DMA2 Stream1 global interrupt*/
	DMA2_Stream2_IRQn                = 58,  /*DMA2 Stream2 global interrupt*/
	DMA2_Stream3_IRQn                = 59,  /*DMA2 Stream3 global interrupt*/
	DMA2_Stream4_IRQn                = 60,  /*DMA2 Stream4 global interrupt*/
	CAN2_TX_IRQn                     = 63,  /*CAN2 TX interrupts*/
	CAN2_RX0_IRQn                    = 64,  /*CAN2 RX0 interrupts*/
	CAN2_RX1_IRQn                    = 65,  /*CAN2 RX1 interrupt*/
	CAN2_SCE_IRQn                    = 66,  /*CAN2 SCE interrupt*/
	OTG_FS_IRQn                      = 67,  /*USB On The Go FS global interrupt*/
	DMA2_Stream5_IRQn                = 68,  /*DMA2 Stream5 global interrupt*/
	DMA2_Stream6_IRQn                = 69,  /*DMA2 Stream6 global interrupt*/
	DMA2_Stream7_IRQn                = 70,  /*DMA2 Stream7 global interrupt*/
	USART6_IRQn                      = 71,  /*USART6 global interrupt*/
	I2C3_EV_IRQn                     = 72,  /*I2C3 event interrupt*/
	I2C3_ER_IRQn                     = 73,  /*I2C3 error interrupt*/
	OTG_HS_EP1_OUT_IRQn              = 74,  /*USB On The Go HS End Point 1 Out global interrupt*/
	OTG_HS_EP1_IN_IRQn               = 75,  /*USB On The Go HS End Point 1 In global interrupt*/
	OTG_HS_WKUP_IRQn                 = 76,  /*USB On The Go HS Wakeup through EXTI interrupt*/
	OTG_HS_IRQn                      = 77,  /*USB On The Go HS global interrupt*/
	DCMI_IRQn                        = 78,  /*DCMI global interrupt*/
	FPU_IRQn                         = 81,  /*FPU global interrupt*/
	SPI4_IRQn                        = 84,  /*SPI 4 global interrupt*/
	SAI1_IRQn                        = 87,  /*SAI1 global interrupt*/
	SAI2_IRQn                        = 91,  /*SAI2 global interrupt*/
	QuadSPI_IRQn                     = 92,  /*QuadSPI global interrupt*/
	HDMI_CEC_IRQn                    = 93,  /*HDMI-CEC global interrupt*/
	SPDIF_Rx_IRQn                    = 94,  /*SPDIF-Rx global interrupt*/
	FMPI2C1_IRQn                     = 95,  /*FMPI2C1 event interrupt*/
	FMPI2C1_error_IRQn               = 96   /*FMPI2C1 error interrupt*/
}IRQn_Type;

typedef enum
{
	NVIC_IRQ_PRI0   = 0,
	NVIC_IRQ_PRI1   = 1,
	NVIC_IRQ_PRI2   = 2,
	NVIC_IRQ_PRI3   = 3,
	NVIC_IRQ_PRI4   = 4,
	NVIC_IRQ_PRI5   = 5,
	NVIC_IRQ_PRI6   = 6,
	NVIC_IRQ_PRI7   = 7,
	NVIC_IRQ_PRI8   = 8,
	NVIC_IRQ_PRI9   = 9,
	NVIC_IRQ_PRI10  = 10,
	NVIC_IRQ_PRI11  = 11,
	NVIC_IRQ_PRI12  = 12,
	NVIC_IRQ_PRI13  = 13,
	NVIC_IRQ_PRI14  = 14,
	NVIC_IRQ_PRI15  = 15,
}NVIC_Priority;

typedef enum
{
	AF0 = 0,
	AF1 = 1,
	AF2 = 2,
	AF3 = 3,
	AF4 = 4,
	AF5 = 5,
	AF6 = 6,
	AF7 = 7,
	AF8 = 8,
	AF9 = 9,
	AF10 = 10,
	AF11 = 11,
	AF12 = 12,
	AF13 = 13,
	AF14 = 14,
	AF15 = 15,

}AF_Type;


#endif /* INC_STM32F446XX_H_ */
