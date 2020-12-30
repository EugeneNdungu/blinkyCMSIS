/*
* stm32f103xx.h
*
*  Created on: Dec 9, 2020
*      Author: Eugene
*
*/

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>		// We are using the uint32_t data type defined in this header file.

/*
* Base addresses of the Flash and SRAM memories
* The compiler considers the numbers as signed integers, but addresses are unsigned.
* We therefore have to inform the comiler that the addresses are unsigned integers.
* The U denotes the values as unsigned
*/

#define	FLASH_BASEADDR										0x08000000U		// Base address of the flash memory
#define	SRAM_BASEADDR											0x20000000U 	// Base address of the sram memory.
#define	ROM_BASEADDR											0x1FFFF000U		// Base address of the system memory.

/* Base addresses for the various bus domains */

#define PERIPH_BASEADDR										0x40000000U		// Base address of the peripheral
#define APB1PERIPH_BASEADDR								PERIPH_BASE		// Base address of TIM2 timer peripheral(1st peripheral attached to APB1)
#define APB2PERIPH_BASEADDR								0x40010000U		// Base address of the AFIO peripheral(1st peripheral attached to APB2)
#define AHBPERIPH_BASEADDR									0x40018000U		// Base address of the SDIO peripheral(1st peripheral attached to AHB)

/**
* Base addresses of AHB, APB1 and APB2 Peripherals
* For this tutorial, we shall not be using any of the peripherals hanging on AHB Bus.
* All the GPIO Ports hang on the APB2 bus.
*/
/* Base Addresses of AHB Peripherals */

#define RCC_BASEADDR												(AHBPERIPH_BASEADDR + 0x9000)

/* Base Addresses of APB1 Peripherals */

#define I2C1_BASEADDR											(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR											(APB1PERIPH_BASEADDR + 0x5800)

#define SPI2_BASEADDR											(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR											(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR										(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR										(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR											(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR											(APB1PERIPH_BASEADDR + 0x5000)

/* Base addresses of APB2 Peripherals */

#define GPIOA_BASEADDR											(APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR											(APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR											(APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR											(APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR											(APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR											(APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR											(APB2PERIPH_BASEADDR + 0x2000)

#define AFIO_BASEADDR											APB2PERIPH_BASEADDR

#define EXTI_BASEADDR											(APB2PERIPH_BASEADDR + 0x0400)

#define SPI1_BASEADDR											(APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR										(APB2PERIPH_BASEADDR + 0x3800)

/**
* Peripheral register definition structure for GPIO.
* Instead of defining the base address for ech peripheral register for all the
* the peripherals, we can create a structure that holds the peripheral register
* addresses of peipherals register common to specific peripheral. The peripheral
* registers are 32 bits and therefore will be offset from each other by 4 bytes(
* i.e the second reg is offset the first reg by 4 bytes). We then declare a struct
* pointer variable for one of the peipherals that have the defined registers and
* initialize it with the base address of that peripheral. In so doing, the address
* of the first variable in the structure will be equal to the base address of the
* peripheral, with the second being 4 bytes offset this base address, and the third
* being 8 bytes offset of this base address and so on and so forth. To have an offset
* of 4 bytes, the variables in the structure should be of type uint32_t.
* The variables declared in the structure are placeholders to hold values of the
* peripheral's registers.
*/

/************************ Peripheral Register Definition Structures ****************************/

/*
* Note : Registers of a peripheral are specific to MCU
* e.g : Number of Registers of SPI peripheral of STM32F4x Family of MCUs may be different
* (more or less) compared to number of registers of SPI  peripheral of STM32F1x family of
* MCUs.
* Please check your Device RM
* Given how the data in some of these register changes without the knowledge of the
* compiler, it is very important to use the volatile keyword when defining these
* registers
*/

/* GPIO Peripheral Register Definition Structure*/

typedef struct
{
volatile uint32_t CRL;							/*|< Port configuration register low,								Adrress offset: 0x00 */
volatile uint32_t CRH;							/*|< Port configuration register high,							Address offset: 0x04 */
volatile uint32_t IDR;							/*|< Port input data register,											Address offset: 0x08 */
volatile uint32_t ODR;							/*|< Port output data register,											Address offset: 0x0C */
volatile uint32_t BSRR;						  /*|< Port bit set/reset register,										Address offset: 0x10 */
volatile uint32_t BRR;							/*|< Port bit reset register,												Address offset: 0x14 */
volatile uint32_t LCKR;						  /*|< Port configuration lock register,							Address offset: 0x18 */

}GPIOx_RegDef_t;

/* RCC Peripheral Register Definition Structure */

typedef struct
{
volatile uint32_t RCC_CR;						/*|< Clock control register ,									Adrress offset: 0x00 */
volatile uint32_t RCC_CFGR;						/*|< Clock configuration register ,								Adrress offset: 0x04 */
volatile uint32_t RCC_CIR;						/*|< Clock interrupt register ,									Adrress offset: 0x08 */
volatile uint32_t RCC_APB2RSTR;					/*|< APB2 peripheral reset register  ,							Adrress offset: 0x0C */
volatile uint32_t RCC_APB1RSTR;					/*|< APB1 peripheral reset register ,							Adrress offset: 0x10 */
volatile uint32_t RCC_AHBENR;					/*|< AHB Peripheral Clock enable registe,						Adrress offset: 0x14 */
volatile uint32_t RCC_APB2ENR;					/*|< APB2 peripheral clock enable register  ,					Adrress offset: 0x18 */
volatile uint32_t RCC_APB1ENR;					/*|< APB1 peripheral clock enable register  ,					Adrress offset: 0x1C */
volatile uint32_t RCC_BDCR;						/*|< Backup domain control register,						  	Adrress offset: 0x20 */
volatile uint32_t RCC_CSR;						/*|< Control/status register,									Adrress offset: 0x24 */
volatile uint32_t RCC_AHBSTR;					/*|< AHB peripheral clock reset register ,						Adrress offset: 0x28 */
volatile uint32_t RCC_CFGR2;					/*|< Clock configuration register2 ,							Adrress offset: 0x2C */

}RCC_RegDef_t;

/* AFIO Peripheral Register Definition Structure */
typedef struct
{
	volatile uint32_t AFIO_EVCR;				/*< Event control register (AFIO_EVCR) >*/
	volatile uint32_t AFIO_MAPR;				/*< AF remap and debug I/O configuration register (AFIO_MAPR) >*/
	volatile uint32_t AFIO_EXTICR1;				/*< External interrupt configuration register 1 (AFIO_EXTICR1) >*/
	volatile uint32_t AFIO_EXTICR2;				/*< External interrupt configuration register 2 (AFIO_EXTICR2) >*/
	volatile uint32_t AFIO_EXTICR3;				/*< External interrupt configuration register 3 (AFIO_EXTICR3) >*/
	volatile uint32_t AFIO_EXTICR4;				/*< External interrupt configuration register 4 (AFIO_EXTICR4) >*/
	volatile uint32_t AFIO_MAPR2;				/*< AF remap and debug I/O configuration register2 (AFIO_MAPR2) >*/

}AFIO_RegDef_t;

/*
* Peripheral definitions( Peripheral base addresses typecasted to XXXX_RegDef_t)
*/

#define GPIOA													((GPIOx_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB													((GPIOx_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC													((GPIOx_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD													((GPIOx_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE													((GPIOx_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF													((GPIOx_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG													((GPIOx_RegDef_t*)GPIOG _BASEADDR)

#define RCC														((RCC_RegDef_t*)RCC_BASEADDR)

/*
* We use Bitwise OR to set a bit and Bitwise AND to reset a bit
*/
/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_CLCK_EN()								( RCC->RCC_APB2ENR |= (1 << 2))
#define GPIOB_CLCK_EN()								( RCC->RCC_APB2ENR |= (1 << 3))
#define GPIOC_CLCK_EN()								( RCC->RCC_APB2ENR |= (1 << 4))
#define GPIOD_CLCK_EN()								( RCC->RCC_APB2ENR |= (1 << 5))
#define GPIOE_CLCK_EN()								( RCC->RCC_APB2ENR |= (1 << 6))
//	 #define GPIOF_CLCK_EN()								( RCC->RCC_APB2ENR |= (1 << 0))
//	 #define GPIOG_CLCK_EN()								( RCC->RCC_APB2ENR |= (1 << 0))

/* Clock Enable Macros for I2C's peripherals */
#define I2C1_CLCK_EN()									( RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_CLCK_EN()									( RCC->RCC_APB1ENR |= (1<<22))

/* Clock Enable Macros for USART's peripherals */
#define USART1_CLCK_EN()								( RCC->RCC_APB2ENR |= (1<<14))
#define USART2_CLK_EN()								( RCC->RCC_APB1ENR |= (1<<17))
#define USART3_CLK_EN()								( RCC->RCC_APB1ENR |= (1<<18))
#define UART4_CLK_EN()									( RCC->RCC_APB1ENR |= (1<<19))
#define UART5_CLK_EN()									( RCC->RCC_APB1ENR |= (1<<20))

/* Clock Enable Macros for SPI's peripherals */
#define SPI1_CLCK_EN()									( RCC->RCC_APB2ENR |= (1<<12))
#define SPI2_CLCK_EN()									( RCC->RCC_APB1ENR |= (1<<14))
#define SPI3_CLCK_EN()									( RCC->RCC_APB1ENR |= (1<<15))

/* Clock Enable Macros for SYSCFG peripherals */

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1 << 2))
#define GPIOB_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1 << 3))
#define GPIOC_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1 << 4))
#define GPIOD_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1 << 5))
#define GPIOE_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1 << 6))
//	#define GPIOF_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1 << 0))
//	#define GPIOG_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1 << 0))

/* clock Disable Macros for I2C's peripherals */
#define I2C1_CLCK_DI()									( RCC->RCC_APB1ENR &= ~(1<<21))
#define I2C2_CLCK_DI()									( RCC->RCC_APB1ENR &= ~(1<<22))

/* Clock Disable Macros for UART's peripherals */
#define USART1_CLCK_DI()								( RCC->RCC_APB2ENR &= ~(1<<14))
#define USART2_CLCK_DI()								( RCC->RCC_APB1ENR &= ~(1<<17))
#define USART3_CLCK_DI()								( RCC->RCC_APB1ENR &= ~(1<<18))
#define UART4_CLCK_DI()									( RCC->RCC_APB1ENR &= ~(1<<19))
#define UART5_CLCK_DI()									( RCC->RCC_APB1ENR &= ~(1<<20))

/* Clock Disable Macros for SPI's peripherals */
#define SPI1_CLCK_DI()									( RCC->RCC_APB2ENR &= ~(1<<12))
#define SPI2_CLCK_DI()									( RCC->RCC_APB1ENR &= ~(1<<14))
#define SPI3_CLCK_DI()									( RCC->RCC_APB1ENR &= ~(1<<15))

/* Macros to reset GPIOx Peripheral Registers */
#define GPIOA_REG_RST()										do{( RCC->RCC_APB2RSTR |= (1 << 2)); ( RCC->RCC_APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOB_REG_RST()										do{( RCC->RCC_APB2RSTR |= (1 << 3)); ( RCC->RCC_APB2RSTR &= ~(1 << 3));}while(0)
#define GPIOC_REG_RST()										do{( RCC->RCC_APB2RSTR |= (1 << 4)); ( RCC->RCC_APB2RSTR &= ~(1 << 4));}while(0)
#define GPIOD_REG_RST()										do{( RCC->RCC_APB2RSTR |= (1 << 5)); ( RCC->RCC_APB2RSTR &= ~(1 << 5));}while(0)
#define GPIOE_REG_RST()										do{( RCC->RCC_APB2RSTR |= (1 << 6)); ( RCC->RCC_APB2RSTR &= ~(1 << 6));}while(0)

/* Miscellaneous Macros*/
#define ENABLE									1
#define DISABLE									0
#define SET										ENABLE
#define RESET									DISABLE
#define GPIO_PIN_SET							SET
#define GPIO_PIN_RESET							RESET


#endif /* INC_STM32F103XX_H_ */
