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

}GPIO_HANDLE;


/*************************************************************************************************************************************************************
 *                                                              APIs Supported by this driver
 *                                                  For more info about the APIs Check the function defenitions
 ************************************************************************************************************************************************************/

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
