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
    uint8_t GPIO_PinMode;           // For Possible values refer @GPIO_Pin_possible_modes
    uint8_t GPIO_PinSpeed;          // For Possible values refer @GPIO_Pin_possible_Output_Speed
    uint8_t GPIO_PinPuPdControl;    // For Possible values refer @GPIO_Pin_Pull-up_and_Pull-down_configuration macros
    uint8_t GPIO_PinOPType;         // For Possible values refer @GPIO_Pin_possible_OUTPUT_Types
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

/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0           0       // GPIO Pin Number 0
#define GPIO_PIN_NO_1           1       // GPIO Pin Number 1       
#define GPIO_PIN_NO_2           2       // GPIO Pin Number 2
#define GPIO_PIN_NO_3           3       // GPIO Pin Number 3
#define GPIO_PIN_NO_4           4       // GPIO Pin Number 4
#define GPIO_PIN_NO_5           5       // GPIO Pin Number 5
#define GPIO_PIN_NO_6           6       // GPIO Pin Number 6
#define GPIO_PIN_NO_7           7       // GPIO Pin Number 7
#define GPIO_PIN_NO_8           8       // GPIO Pin Number 8
#define GPIO_PIN_NO_9           9       // GPIO Pin Number 9
#define GPIO_PIN_NO_10          10      // GPIO Pin Number 10
#define GPIO_PIN_NO_11          11      // GPIO Pin Number 11
#define GPIO_PIN_NO_12          12      // GPIO Pin Number 12
#define GPIO_PIN_NO_13          13      // GPIO Pin Number 13
#define GPIO_PIN_NO_14          14      // GPIO Pin Number 14
#define GPIO_PIN_NO_15          15      // GPIO Pin Number 15

/*
 * @GPIO_Pin_possible_modes
 * GPIO Pin possible modes
 */
#define GPIO_MODE_IN            0       // GPIO Input Mode
#define GPIO_MODE_OUT           1       // GPIO Output Mode
#define GPIO_MODE_ALTFN         2       // GPIO Alternate Function Mode
#define GPIO_MODE_ANALOG        3       // GPIO Analog Mode
#define GPIO_MODE_IT_FT         4       // GPIO Input Falling Edge
#define GPIO_MODE_IT_RT         5       // GPIO Input Raising Edge
#define GPIO_MODE_IT_RFT        6       // GPIO Raising edge Falling Edge Trigger

/*
 * @GPIO_Pin_possible_Output_Speed
 * GPIO Pin possible OUTPUT Types
 */
#define GPIO_OP_TYPE_PP         0       // GPIO Output type Push-Pull
#define GPIO_OP_TYPE_OD         1       // GPIO Output type Open-Drain

/*
 * @GPIO_Pin_possible_Output_Speed
 * GPIO Pin possible Output Speed
 */
#define GPIO_SPEED_LOW          0       // Low output speed
#define GPIO_SPEED_MEDIUM       1       // Medium output speed
#define GPIO_SPEED_FAST         2       // Fast output speed
#define GPIO_SPEED_HIGH         3       // High output speed

/*
 * @GPIO_Pin_Pull-up_and_Pull-down_configuration macros
 * GPIO Pin Pull-up and Pull-down configuration macros
 */
#define GPIO_NO_PUPD            0       // No Pull-up and Pull-down
#define GPIO_PIN_PU             1       // GPIO Pull-Up
#define GPIO_PIN_PD             2       // GPIO Pull-Down

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
