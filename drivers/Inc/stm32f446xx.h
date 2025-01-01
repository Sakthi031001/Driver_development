/*
 * stm32446xx.h
 *
 *  Created on: Dec 31, 2024
 *      Author: SAKTHIVEL B
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

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

#define GPIOA_BASEADDR                  (AHB1PERIPH_BASE + 0x0000) /* Refer register boundary address in Reference Manual for GPIOA and add its least 4 bits */
#define GPIOB_BASEADDR                  (AHB1PERIPH_BASE + 0x0400) /* Refer register boundary address in Reference Manual for GPIOB and add its least 4 bits */
#define GPIOC_BASEADDR                  (AHB1PERIPH_BASE + 0x0800) /* Refer register boundary address in Reference Manual for GPIOC and add its least 4 bits */
#define GPIOD_BASEADDR                  (AHB1PERIPH_BASE + 0x0C00) /* Refer register boundary address in Reference Manual for GPIOD and add its least 4 bits */
#define GPIOE_BASEADDR                  (AHB1PERIPH_BASE + 0x1000) /* Refer register boundary address in Reference Manual for GPIOE and add its least 4 bits */
#define GPIOF_BASEADDR                  (AHB1PERIPH_BASE + 0x1400) /* Refer register boundary address in Reference Manual for GPIOF and add its least 4 bits */
#define GPIOG_BASEADDR                  (AHB1PERIPH_BASE + 0x1800) /* Refer register boundary address in Reference Manual for GPIOG and add its least 4 bits */
#define GPIOH_BASEADDR                  (AHB1PERIPH_BASE + 0x1C00) /* Refer register boundary address in Reference Manual for GPIOH and add its least 4 bits */
#define RCC_BASEADDR                    (AHB1PERIPH_BASE + 0x3800) /* Refer register boundary address in Reference Manual for RCC and add its least 4 bits */

/*
 * Base addresses of Peripherals which are hanging on APB1
 */

/* I2C */
#define I2C1_BASEADDR                   (APB1PERIPH_BASE + 0x5400) /* Refer register boundary address in Reference Manual for I2C1 and add its least 4 bits */
#define I2C2_BASEADDR                   (APB1PERIPH_BASE + 0x5800) /* Refer register boundary address in Reference Manual for I2C2 and add its least 4 bits */
#define I2C3_BASEADDR                   (APB1PERIPH_BASE + 0x5C00) /* Refer register boundary address in Reference Manual for I2C3 and add its least 4 bits */

/* SPI */
#define SPI2_BASEADDR                   (APB1PERIPH_BASE + 0x3800) /* Refer register boundary address in Reference Manual for SPI2 and add its least 4 bits */
#define SPI3_BASEADDR                   (APB1PERIPH_BASE + 0x3C00) /* Refer register boundary address in Reference Manual for SPI3 and add its least 4 bits */

/* USART */
#define USART2_BASEADDR                 (APB1PERIPH_BASE + 0x4400) /* Refer register boundary address in Reference Manual for USART2 and add its least 4 bits */
#define USART3_BASEADDR                 (APB1PERIPH_BASE + 0x4800) /* Refer register boundary address in Reference Manual for USART3 and add its least 4 bits */

/* UART */
#define UART4_BASEADDR                  (APB1PERIPH_BASE + 0x4C00) /* Refer register boundary address in Reference Manual for UART4 and add its least 4 bits */
#define UART5_BASEADDR                  (APB1PERIPH_BASE + 0x5000) /* Refer register boundary address in Reference Manual for UART5 and add its least 4 bits */

/*
 * Base addresses of Peripherals which are hanging on APB2
 */

/* SPI */
#define SPI1_BASEADDR                   (APB2PERIPH_BASE + 0x3000) /* Refer register boundary address in Reference Manual for SPI1 and add its least 4 bits */

/* USART */
#define USART1_BASEADDR                 (APB2PERIPH_BASE + 0x1000) /* Refer register boundary address in Reference Manual for USART1 and add its least 4 bits */
#define USART6_BASEADDR                 (APB2PERIPH_BASE + 0x1400) /* Refer register boundary address in Reference Manual for USART6 and add its least 4 bits */

/* EXTI */
#define EXTI_BASEADDR                   (APB2PERIPH_BASE + 0x3C00) /* Refer register boundary address in Reference Manual for EXTI and add its least 4 bits */

/* SYSCFG */
#define SYSCFG_BASEADDR                 (APB2PERIPH_BASE + 0x3800) /* Refer register boundary address in Reference Manual for SYSCFG and add its least 4 bits */

/****************************** Peripheral Register Definition Structures ******************************/

typedef struct 
{
    /*
     * Since we are dealing with registors it is good to use volatile element
     */
    volatile uint32_t MODER;            /* GPIO port mode register                                  Address offset: 0x00 */
    volatile uint32_t OTYPER;           /* GPIO port output type register                           Address offset: 0x04 */
    volatile uint32_t OSPEEDER;         /* GPIO port output speed register                          Address offset: 0x08 */
    volatile uint32_t PUPDR;            /* GPIO port pull-up/pull-down register                     Address offset: 0x0C */
    volatile uint32_t IDR;              /* GPIO port input data register                            Address offset: 0x10 */
    volatile uint32_t ODR;              /* GPIO port output data register                           Address offset: 0x14 */
    volatile uint32_t BSRR;             /* GPIO port bit set/reset register                         Address offset: 0x18 */
    volatile uint32_t LCKR;             /* GPIO port configuration lock register                    Address offset: 0x1C */
    volatile uint32_t AFR[2];           /* GPIO alternate function low and high register    AFR[0]:Low      AFR[1]:High     Address offset: AFR[0]: 0x20 AFR[1]: 0x24 */
}GPIO_RefDef_t;

typedef struct 
{
    /*
     * Since we are dealing with registors it is good to use volatile element
     */
    volatile uint32_t   CR;             /* RCC clock control register Address offset: 0x00*/
    volatile uint32_t   PLLCFGR;        /* RCC PLL configuration register Address offset: 0x04 */
    volatile uint32_t   CFGR;           /* RCC clock configuration register Address offset: 0x08 */
    volatile uint32_t   CIR;            /* RCC clock interrupt register Address offset: 0x0C */
    volatile uint32_t   AHB1RSTR;       /* RCC AHB1 peripheral reset register Address offset: 0x10 */
    volatile uint32_t   AHB2RSTR;       /* RCC AHB2 peripheral reset register Address offset: 0x14 */
    volatile uint32_t   AHB3RSTR;       /* RCC AHB3 peripheral reset register Address offset: 0x18 */
    uint32_t            RESERVED0;
    volatile uint32_t   APB1RSTR;       /* RCC APB1 peripheral reset register Address offset: 0x20 */
    volatile uint32_t   APB2RSTR;       /* RCC APB2 peripheral reset register Address offset: 0x24 */
    uint32_t            RESERVED1[2];
    volatile uint32_t   AHB1ENR;        /* RCC AHB1 peripheral clock enable register Address offset: 0x30 */
    volatile uint32_t   AHB2ENR;        /* RCC AHB1 peripheral clock enable register Address offset: 0x34 */
    volatile uint32_t   AHB3ENR;        /* RCC AHB1 peripheral clock enable register Address offset: 0x38 */
    uint32_t            RESERVED2;
    volatile uint32_t   APB1ENR;        /* RCC APB1 peripheral clock enable register Address offset: 0x40 */
    volatile uint32_t   APB2ENR;        /* RCC APB1 peripheral clock enable register Address offset: 0x44 */
    uint32_t            RESERVED3[2];
    volatile uint32_t   AHB1LPENR;
    volatile uint32_t   AHB2LPENR;
    volatile uint32_t   AHB3LPENR;
    uint32_t            RESERVED4;
    volatile uint32_t   APB1LPENR;
    volatile uint32_t   APB2LPENR;
    uint32_t            RESERVED5[2];
    volatile uint32_t   BDCR;
    volatile uint32_t   CSR;
    uint32_t            RESERVED6[2];
    volatile uint32_t   SSCGR;
    volatile uint32_t   PLLI2SCFGR;
    volatile uint32_t   PLLSAICFGR;
    volatile uint32_t   DCKCFGR;
    volatile uint32_t   CKGATENR;
    volatile uint32_t   DCKCFGR2;
    }RCC_RefDef_t;

/*
 * Peripheral defenitions (Peripheral base addresses typecasted to _RegDef_t)
 */

#define GPIOA       ((GPIO_RefDef_t*)GPIOA_BASEADDR)
#define GPIOB       ((GPIO_RefDef_t*)GPIOB_BASEADDR)
#define GPIOC       ((GPIO_RefDef_t*)GPIOC_BASEADDR)
#define GPIOD       ((GPIO_RefDef_t*)GPIOD_BASEADDR)
#define GPIOE       ((GPIO_RefDef_t*)GPIOE_BASEADDR)
#define GPIOF       ((GPIO_RefDef_t*)GPIOF_BASEADDR)
#define GPIOG       ((GPIO_RefDef_t*)GPIOG_BASEADDR)
#define GPIOH       ((GPIO_RefDef_t*)GPIOH_BASEADDR)

#define RCC         ((RCC_RefDef_t*)RCC_BASEADDR)

/*
 * Clock enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()       (RCC -> AHB1ENR |= (1<<0))

#endif /* INC_STM32F446XX_H_ */