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

}

/*
 * Init and De-init
 */

/*********************************************************************
 *  function            -   GPIO_Init
 *  
 *  brief               -   This function Initializes the GPIO Port
 * 
 *  parameter [in]      -   Pointer to the handlestructure
 * 
 *  return              -   none
 * 
 *  NOTE                -   none
 *********************************************************************/
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle)
{

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
