/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Dec 9, 2020
 *      Author: Eugene
 */

#include "stm32f103xx_gpio_driver.h"

/*****************************************************************
 * @fn				- GPIO_PeriClockConfig
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_PeriClockConfig(GPIOx_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLCK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLCK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLCK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLCK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLCK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLCK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLCK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLCK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLCK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLCK_DI();
		}
	}
}

/*****************************************************************
 * @fn				- GPIOinit
 *
 * @brief			- This function initializes the GPIO pin in use
 *
 * @param[in]		- pointer to the port and pin configuration settings in use
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			-
 *
 */
void GPIOinit(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. configure the mode and configuration of the pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_OUT_50)
	{
		//Pin 0-7 are configured using the Port Configuration register Low
		//The mode sets the pin to an input or an output of a specific speed.
		//The configuration specifies what type of input or output the pin will be.
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_NO_7)
		{
			//a. set the mode of the pin
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			//B. set the configuration of the pin
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinCfg << ((4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) + 2));
			pGPIOHandle->pGPIOx->CRL &= ~(0xf << 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
			pGPIOHandle->pGPIOx->CRL |= temp;
		}
		//Pin 8-15 are configured using the Port Configuration register High
		else
		{
			//a. set the mode of the pin
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
			//B. set the configuration of the pin
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinCfg << ((4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)) + 2));
			pGPIOHandle->pGPIOx->CRH &= ~(0xf << 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
			pGPIOHandle->pGPIOx->CRH |= temp;
		}
		temp = 0;

	}
	else
	{
		//To do later.
	}

	//2. configure the input pin pull up pull down
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->ODR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->ODR |= temp;
		temp = 0;
	}

	//3. configure the alternate function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinCfg == GPIO_OUT_ALT_OD || pGPIOHandle->GPIO_PinConfig.GPIO_PinCfg == GPIO_OUT_ALT_PP)
	{
		//To do later.
	}

}

/*****************************************************************
 * @fn				- GPIODeInit
 *
 * @brief			- resets all the registers of the peripheral
 *
 * @param[in]		- pointer to a port
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void GPIODeInit(GPIOx_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RST();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RST();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RST();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RST();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RST();
	}
}

/*****************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*****************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef_t *pGPIOx)
{

}

/*****************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void GPIO_WriteToOutputPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

/*****************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void GPIO_WriteToOutputPort(GPIOx_RegDef_t *pGPIOx, uint16_t Value)
{

}

/*****************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void GPIO_ToggleOutputPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*****************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*****************************************************************
 * @fn				-
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

