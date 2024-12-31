/*
 * stm32446xx.h
 *
 *  Created on: Dec 31, 2024
 *      Author: dell
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

/*
 * Base address for Flash and SRAM memories
 * NOTE : 'U' is used in end of address to represent Unsigned integer
 */

#define FLASH_BASEADDR                  0x08000000U /* mentioned on reference manual as system memory in Flash Memory Organization */
#define SRAM1_BASEADDR                  0x20000000U /* 112 KB, mentioned on reference manual */
#define SRAM2_BASEADDR                  0x2001C000U /* 16 KB, mentioned on reference manual */
#define ROM                             0x1FFF0000U /* mentioned on reference manual as system memory in Flash Memory Organization */
#define SRAM                            SRAM1_BASEADDR

/*
 * Base addresses of AHB and APB bus peripheral base address
 */

#define PERIPH_BASE                     0x40000000U /* Refer Memory map from Reference Manual for APB1 */
#define APB1PERIPH_BASE                 PERIPH_BASE
#define APB2PERIPH_BASE                 0x40010000U /* Refer Memory map from Reference Manual for APB2 */
#define AHB1PERIPH_BASE                 0x40020000U /* Refer Memory map from Reference Manual for AHB1 */
#define AHB2PERIPH_BASE                 0x50000000U /* Refer Memory map from Reference Manual for AHB2 */

/*
 * Base address of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR                  (AHB1PERIPH_BASE + 0x0000) /* Refer regester boundary address in Reference Manual for GPIOA and add its least 4 bits */
#define GPIOB_BASEADDR                  (AHB1PERIPH_BASE + 0x0400) /* Refer regester boundary address in Reference Manual for GPIOB and add its least 4 bits */
#define GPIOC_BASEADDR                  (AHB1PERIPH_BASE + 0x0800) /* Refer regester boundary address in Reference Manual for GPIOC and add its least 4 bits */
#define GPIOD_BASEADDR                  (AHB1PERIPH_BASE + 0x0C00) /* Refer regester boundary address in Reference Manual for GPIOD and add its least 4 bits */
#define GPIOE_BASEADDR                  (AHB1PERIPH_BASE + 0x1000) /* Refer regester boundary address in Reference Manual for GPIOE and add its least 4 bits */
#define GPIOF_BASEADDR                  (AHB1PERIPH_BASE + 0x1400) /* Refer regester boundary address in Reference Manual for GPIOF and add its least 4 bits */
#define GPIOG_BASEADDR                  (AHB1PERIPH_BASE + 0x1800) /* Refer regester boundary address in Reference Manual for GPIOG and add its least 4 bits */
#define GPIOH_BASEADDR                  (AHB1PERIPH_BASE + 0x1C00) /* Refer regester boundary address in Reference Manual for GPIOH and add its least 4 bits */

/*
 * Base addresses of Peripherals which are hanging on APB1
 */

/* I2C */
#define I2C1_BASEADDR                   (APB1PERIPH_BASE + 0x5400) /* Refer regester boundary address in Reference Manual for I2C1 and add its least 4 bits */
#define I2C2_BASEADDR                   (APB1PERIPH_BASE + 0x5800) /* Refer regester boundary address in Reference Manual for I2C2 and add its least 4 bits */
#define I2C3_BASEADDR                   (APB1PERIPH_BASE + 0x5C00) /* Refer regester boundary address in Reference Manual for I2C3 and add its least 4 bits */

/* SPI */
#define SPI2_BASEADDR                   (APB1PERIPH_BASE + 0x3800) /* Refer regester boundary address in Reference Manual for SPI2 and add its least 4 bits */
#define SPI3_BASEADDR                   (APB1PERIPH_BASE + 0x3C00) /* Refer regester boundary address in Reference Manual for SPI3 and add its least 4 bits */

/* USART */
#define USART2_BASEADDR                 (APB1PERIPH_BASE + 0x4400) /* Refer regester boundary address in Reference Manual for USART2 and add its least 4 bits */
#define USART3_BASEADDR                 (APB1PERIPH_BASE + 0x4800) /* Refer regester boundary address in Reference Manual for USART3 and add its least 4 bits */

/* UART */
#define UART4_BASEADDR                  (APB1PERIPH_BASE + 0x4C00) /* Refer regester boundary address in Reference Manual for UART4 and add its least 4 bits */
#define UART5_BASEADDR                  (APB1PERIPH_BASE + 0x5000) /* Refer regester boundary address in Reference Manual for UART5 and add its least 4 bits */

/*
 * Base addresses of Peripherals which are hanging on APB2
 */

/* SPI */
#define SPI1_BASEADDR                   (APB2PERIPH_BASE + 0x3000) /* Refer regester boundary address in Reference Manual for SPI1 and add its least 4 bits */

/* USART */
#define USART1_BASEADDR                 (APB2PERIPH_BASE + 0x1000) /* Refer regester boundary address in Reference Manual for USART1 and add its least 4 bits */
#define USART6_BASEADDR                 (APB2PERIPH_BASE + 0x1400) /* Refer regester boundary address in Reference Manual for USART6 and add its least 4 bits */

/* EXTI */
#define EXTI_BASEADDR                   (APB2PERIPH_BASE + 0x3C00) /* Refer regester boundary address in Reference Manual for EXTI and add its least 4 bits */

/* SYSCFG */
#define SYSCFG_BASEADDR                 (APB2PERIPH_BASE + 0x3800) /* Refer regester boundary address in Reference Manual for SYSCFG and add its least 4 bits */

#endif /* INC_STM32F446XX_H_ */
