/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jan 2, 2025
 *      Author: dell
 */

#include "stm32f446xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */

/*********************************************************************
 *  function            -   GPIO_PeriClockControl
 *  
 *  brief               -   This function enables or disables peripherals clock for the given GPIO port
 * 
 *  parameter [in]      -   base address of the GPIO peripheral
 *  parameter [in]      -   Enable or Disable macros
 * 
 *  return              -   None
 * 
 *  NOTE                -   None
 *********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_REG_RESET();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_REG_RESET();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_REG_RESET();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_REG_RESET();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_REG_RESET();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_REG_RESET();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_REG_RESET();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_REG_RESET();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
 */

/*********************************************************************
 *  function            -   GPIO_Init
 *  
 *  brief               -   This function Initializes the GPIO Port
 * 
 *  parameter [in]      -   Pointer to the handle structure
 * 
 *  return              -   none
 * 
 *  NOTE                -   none
 *********************************************************************/
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle)
{
    uint32_t temp = 0;          // Temporary register

    // Configure the model of GPIO Pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        // Non interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        // The code will be updated later (Interrupt Mode)
    }
    temp = 0;
    
    // Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDER |= temp;
    temp = 0;

    // Configure the PUPD Control
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    // Configure the Output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;

    // Configure the alternate functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        // Configure Alternate function Registers
        uint32_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2) );
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
    }

}

/*********************************************************************
 *  function            -   GPIO_Deinit
 *  
 *  brief               -   This function De-initializes the GPIO Port
 * 
 *  parameter [in]      -   Base address of GPIO Periphereal
 * 
 *  return              -   none
 * 
 *  NOTE                -   none
 *********************************************************************/
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_PCLK_EN();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_PCLK_EN();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_PCLK_EN();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_PCLK_EN();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_PCLK_EN();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_PCLK_EN();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_PCLK_EN();
    }
}

/*
 * Data Read and Write
 */

/*********************************************************************
 *  function            -   GPIO_ReadFromInputPin
 *  
 *  brief               -   This function reads the data from the Input Pin
 * 
 *  parameter [in]      -   Base address of the GPIO Peripheral 
 *  parameter [in]      -   Pin number of the pin used
 * 
 *  return              -   
 * 
 *  NOTE                -   
 *********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

/*********************************************************************
 *  function            -   GPIO_ReadFromInputPort
 *  
 *  brief               -   This function reads the data from the Input Port
 * 
 *  parameter [in]      -   Base address of the GPIO Peripheral 
 * 
 *  return              -   
 * 
 *  NOTE                -   
 *********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)(pGPIOx->IDR)
    return value;
}

/*********************************************************************
 *  function            -   GPIO_WriteToOutputPin
 *  
 *  brief               -   This functions writes the data to Output Pin
 * 
 *  parameter [in]      -   Base address of the GPIO Peripheral
 *  parameter [in]      -   Pin number of the pin used
 *  parameter [in]      -   macros
 * 
 *  return              -   
 * 
 *  NOTE                -   
 *********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        // Write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1<<PinNumber);
    }
    else
    {
        // Write 0
        pGPIOx->ODR &= ~(1<<PinNumber);
    }
}

/*********************************************************************
 *  function            -   GPIO_WriteToOutputPort
 *  
 *  brief               -   This functions writes the data to Output Port
 * 
 *  parameter [in]      -   Base address of the GPIO Peripheral 
 *  parameter [in]      -   macros
 * 
 *  return              -   
 * 
 *  NOTE                -   
 *********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*********************************************************************
 *  function            -   GPIO_ToggleOutputPin
 *  
 *  brief               -   This function toggles the GPIO Pin State
 * 
 *  parameter [in]      -   Base address of the GPIO Peripheral 
 *  parameter [in]      -   Pin number of the pin used
 * 
 *  return              -   
 * 
 *  NOTE                -   
 *********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR handling
 */

/*********************************************************************
 *  function            -   GPIO_IRQConfig
 *  
 *  brief               -   This function configures the IRQ number of the GPIO pin
 * 
 *  parameter [in]      -   IRQ number to identify the specific source
 *  parameter [in]      -   IRQ priority
 *  parameter [in]      -   macros
 * 
 *  return              -   
 * 
 *  NOTE                -   
 *********************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*********************************************************************
 *  function            -   GPIO_IRQHandling
 *  
 *  brief               -   This function is called when an interrupt occurs to process that interrupt
 * 
 *  parameter [in]      -   Pin number of the pin used
 
 * 
 *  return              -   
 * 
 *  NOTE                -   
 *********************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}