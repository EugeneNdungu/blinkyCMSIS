/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Dec 9, 2020
 *      Author: Eugene
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"

/**
 * This is a Configuration Structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;						/*< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode; 						/*< possible values from @GPIO_PIN_CFG >*/
	uint8_t GPIO_PinCfg; 						/*< possible values from @GPIO_PIN_CFG >*/
//	uint8_t GPIO_PinMode;
//	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdCtrl;
//	uint8_t GPIO_PinOType;
//	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/**
 * This is a Handle Structure for a GPIO pin
 */
typedef struct
{
	GPIOx_RegDef_t *pGPIOx;						/*< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;			/*< This holds the GPIO pin configuration settings >*/
}GPIO_Handle_t;

/**
 * @GPIO_PIN_NUMBERS
 * GPIO possible pin numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/**
 * @GPIO_PIN_CFG
 * GPIO Modes & configurations(refer to reference manual)
 * There are two registers to set the mode of a GPIO Pin.
 * The first 8 pins(0-7) of a GPIO port are set using the Port configuration register low.
 * The last 8 pins(8-15) of a GPIO port are set using the Port configuration register high.
 */
//GPIO Modes
#define GPIO_MODE_IN			0			/*< Input mode (reset state) >*/
#define GPIO_MODE_OUT_10		1			/*< Output mode, max speed 10 MHz. >*/
#define GPIO_MODE_OUT_2			2			/*< Output mode, max speed 2 MHz. >*/
#define GPIO_MODE_OUT_50		3			/*< Output mode, max speed 50 MHz. >*/
//GPIO configurations(Input Mode)
#define GPIO_IN_ANA				0			/*< Analog mode >*/
#define GPIO_IN_FLOAT			1			/*< Floating input (reset state) >*/
#define GPIO_IN_PUPD			2			/*< Input with pull-up / pull-down >*/
/*
 * GPIO pin pull up pull down configuration macros
 * The Port output data register is used to set the PU/PD state of an input pin.
 */
#define GPIO_PIN_PD					0			/*< PxODR register >*/
#define GPIO_PIN_PU					1			/*< PxODR register >*/
//GPIO CONFIGURATION(Output Mode)
#define GPIO_OUT_GP_PP			0			/*< General purpose output push-pull >*/
#define GPIO_OUT_GP_OD			1			/*< General purpose output Open-drain >*/
#define GPIO_OUT_ALT_PP			2			/*< Alternate function output Push-pull >*/
#define GPIO_OUT_ALT_OD			3			/*< Alternate function output Open-drain	 >*/




/********************************************************************************************
 * 									APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 ********************************************************************************************/

/**
 * Peripheral clock configuration
 */
void GPIO_PeriClockConfig(GPIOx_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * GPIO Initialization and De-initialization(Reset)
 */
void GPIOinit(GPIO_Handle_t *pGPIOHandle);
void GPIODeInit(GPIOx_RegDef_t *pGPIOx);

/**
 * GPIO Read and Write Operations
 */
uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIOx_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * GPIO Interrupt Configuration and Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
