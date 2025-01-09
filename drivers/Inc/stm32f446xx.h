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
     * Since we are dealing with registers it is good to use volatile element
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
}GPIO_RegDef_t;

typedef struct 
{
    /*
     * Since we are dealing with registers it is good to use volatile element
     */
    volatile uint32_t   CR;             /* RCC clock control register                                       Address offset: 0x00 */
    volatile uint32_t   PLLCFGR;        /* RCC PLL configuration register                                   Address offset: 0x04 */
    volatile uint32_t   CFGR;           /* RCC clock configuration register                                 Address offset: 0x08 */
    volatile uint32_t   CIR;            /* RCC clock interrupt register                                     Address offset: 0x0C */
    volatile uint32_t   AHB1RSTR;       /* RCC AHB1 peripheral reset register                               Address offset: 0x10 */
    volatile uint32_t   AHB2RSTR;       /* RCC AHB2 peripheral reset register                               Address offset: 0x14 */
    volatile uint32_t   AHB3RSTR;       /* RCC AHB3 peripheral reset register                               Address offset: 0x18 */
    uint32_t            RESERVED0;
    volatile uint32_t   APB1RSTR;       /* RCC APB1 peripheral reset register                               Address offset: 0x20 */
    volatile uint32_t   APB2RSTR;       /* RCC APB2 peripheral reset register                               Address offset: 0x24 */
    uint32_t            RESERVED1[2];
    volatile uint32_t   AHB1ENR;        /* RCC AHB1 peripheral clock enable register                        Address offset: 0x30 */
    volatile uint32_t   AHB2ENR;        /* RCC AHB1 peripheral clock enable register                        Address offset: 0x34 */
    volatile uint32_t   AHB3ENR;        /* RCC AHB1 peripheral clock enable register                        Address offset: 0x38 */
    uint32_t            RESERVED2;
    volatile uint32_t   APB1ENR;        /* RCC APB1 peripheral clock enable register                        Address offset: 0x40 */
    volatile uint32_t   APB2ENR;        /* RCC APB2 peripheral clock enable register                        Address offset: 0x44 */
    uint32_t            RESERVED3[2];
    volatile uint32_t   AHB1LPENR;      /* RCC AHB1 peripheral clock enable in low power mode register      Address offset: 0x50 */
    volatile uint32_t   AHB2LPENR;      /* RCC AHB2 peripheral clock enable in low power mode register      Address offset: 0x54 */
    volatile uint32_t   AHB3LPENR;      /* RCC AHB3 peripheral clock enable in low power mode register      Address offset: 0x58 */
    uint32_t            RESERVED4;
    volatile uint32_t   APB1LPENR;      /* RCC APB1 peripheral clock enable in low power mode register      Address offset: 0x60 */
    volatile uint32_t   APB2LPENR;      /* RCC APB2 peripheral clock enable in low power mode register      Address offset: 0x64 */
    uint32_t            RESERVED5[2];
    volatile uint32_t   BDCR;           /* RCC Backup domain control register                               Address offset: 0x70 */
    volatile uint32_t   CSR;            /* RCC clock control & status register                              Address offset: 0x74 */
    uint32_t            RESERVED6[2];
    volatile uint32_t   SSCGR;          /* RCC spread spectrum clock generation register                    Address offset: 0x80 */
    volatile uint32_t   PLLI2SCFGR;     /* RCC PLLI2S configuration register                                Address offset: 0x84 */
    volatile uint32_t   PLLSAICFGR;     /* RCC PLL configuration register                                   Address offset: 0x88 */
    volatile uint32_t   DCKCFGR;        /* RCC dedicated clock configuration register                       Address offset: 0x8C */
    volatile uint32_t   CKGATENR;       /* RCC clocks gated enable register                                 Address offset: 0x90 */
    volatile uint32_t   DCKCFGR2;       /* RCC dedicated clocks configuration register 2                    Address offset: 0x94 */
    }RCC_RefDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to _RegDef_t)
 */

#define GPIOA       ((GPIO_RegDef_t*)GPIOA_BASEADDR)            /* GPIO-A Peripheral definition */
#define GPIOB       ((GPIO_RegDef_t*)GPIOB_BASEADDR)            /* GPIO-B Peripheral definition */
#define GPIOC       ((GPIO_RegDef_t*)GPIOC_BASEADDR)            /* GPIO-C Peripheral definition */
#define GPIOD       ((GPIO_RegDef_t*)GPIOD_BASEADDR)            /* GPIO-D Peripheral definition */
#define GPIOE       ((GPIO_RegDef_t*)GPIOE_BASEADDR)            /* GPIO-E Peripheral definition */
#define GPIOF       ((GPIO_RegDef_t*)GPIOF_BASEADDR)            /* GPIO-F Peripheral definition */
#define GPIOG       ((GPIO_RegDef_t*)GPIOG_BASEADDR)            /* GPIO-G Peripheral definition */
#define GPIOH       ((GPIO_RegDef_t*)GPIOH_BASEADDR)            /* GPIO-H Peripheral definition */

#define RCC         ((RCC_RefDef_t*)RCC_BASEADDR)               /* Reset and Clock Control peripheral definition */

/*
 * Clock enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1<<0))      /* GPIOA clock enable */
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1<<1))      /* GPIOB clock enable */
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1<<2))      /* GPIOC clock enable */
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1<<3))      /* GPIOD clock enable */
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1<<4))      /* GPIOE clock enable */
#define GPIOF_PCLK_EN()         (RCC->AHB1ENR |= (1<<5))      /* GPIOF clock enable */
#define GPIOG_PCLK_EN()         (RCC->AHB1ENR |= (1<<6))      /* GPIOG clock enable */
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1<<7))      /* GPIOH clock enable */

/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1<<21))     /* I2C1 Clock Enable */
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1<<22))     /* I2C2 Clock Enable */
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1<<23))     /* I2C3 Clock Enable */

/*
 * Clock Enable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1<<12))     /* SPI1 Clock Enable */
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1<<14))     /* SPI2 Clock Enable */
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1<<15))     /* SPI3 Clock Enable */
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1<<13))     /* SPI4 Clock Enable */

/*
 * Clock Enable Macros for USARTx Peripherals
 */

#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1<<4))      /* USART1 Clock Enable */
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1<<17))     /* USART2 Clock Enable */
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1<<18))     /* USART3 Clock Enable */
#define UART4_PCLK_EN()         (RCC->APB1ENR |= (1<<19))     /* UART4 Clock Enable */
#define UART5_PCLK_EN()         (RCC->APB1ENR |= (1<<20))     /* UART5 Clock Enable */
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1<<5))      /* USART6 Clock Enable */

/*
 * Clock Enable Macros for SYSCFG Peripheral
 */

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1<<14))     /* System configuration controller clock enable */


/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<0))      /* GPIOA clock Disable */
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<1))      /* GPIOB clock Disable */
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<2))      /* GPIOC clock Disable */
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<3))      /* GPIOD clock Disable */
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<4))      /* GPIOE clock Disable */
#define GPIOF_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<5))      /* GPIOF clock Disable */
#define GPIOG_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<6))      /* GPIOG clock Disable */
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1<<7))      /* GPIOH clock Disable */

/*
 * Clock Disable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1<<21))     /* I2C1 Clock Disable */
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1<<22))     /* I2C2 Clock Disable */
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1<<23))     /* I2C3 Clock Disable */

/*
 * Clock Disable Macros for SPIx Peripherals
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1<<12))     /* SPI1 Clock Disable */
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1<<14))     /* SPI2 Clock Disable */
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1<<15))     /* SPI3 Clock Disable */
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~(1<<13))     /* SPI4 Clock Disable */

/*
 * Clock Disable Macros for USARTx Peripherals
 */

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1<<4))      /* USART1 Clock Disable */
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1<<17))     /* USART2 Clock Disable */
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~(1<<18))     /* USART3 Clock Disable */
#define UART4_PCLK_DI()         (RCC->APB1ENR &= ~(1<<19))     /* UART4 Clock Disable */
#define UART5_PCLK_DI()         (RCC->APB1ENR &= ~(1<<20))     /* UART5 Clock Disable */
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1<<5))      /* USART6 Clock Disable */

/*
 * Clock Disable Macros for SYSCFG Peripheral
 */

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1<<14))     /* System configuration controller clock Disable */

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));} while (0)
#define GPIOB_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<1));  (RCC->AHB1RSTR &= ~(1<<1));} while (0)
#define GPIOC_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<2));  (RCC->AHB1RSTR &= ~(1<<2));} while (0)
#define GPIOD_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<3));  (RCC->AHB1RSTR &= ~(1<<3));} while (0)
#define GPIOE_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<4));  (RCC->AHB1RSTR &= ~(1<<4));} while (0)
#define GPIOF_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<5));  (RCC->AHB1RSTR &= ~(1<<5));} while (0)
#define GPIOG_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<6));  (RCC->AHB1RSTR &= ~(1<<6));} while (0)
#define GPIOH_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<7));  (RCC->AHB1RSTR &= ~(1<<7));} while (0)



// Some generic macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#endif /* INC_STM32F446XX_H_ */