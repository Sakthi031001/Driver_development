/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jan 2, 2025
 *      Author: dell
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIOPinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */

typedef struct
{
    GPIO_RegDef_t *pGPIOx;              // Holds the base address of the GPIO port to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig;    // Holds the GPIO pin configuration settings

}GPIO_HANDLE_t;


/*************************************************************************************************************************************************************
 *                                                              APIs Supported by this driver
 *                                                  For more info about the APIs Check the function defenitions
 ************************************************************************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
