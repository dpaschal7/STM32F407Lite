/*
 * stm32f407xx.h
 *
 *  Created on: Jun 1, 2022
 *      Author: Drew
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))


/*   
* ARM Cortex MX Processor NVIC ISERx register addresses
*/

#define NVIC_ISER0                          ((volatile uint32_t*) 0xE000E100 )
#define NVIC_ISER1                          ((volatile uint32_t*) 0xE000E104 )
#define NVIC_ISER2                          ((volatile uint32_t*) 0xE000E108 )
#define NVIC_ISER3                          ((volatile uint32_t*) 0xE000E10C )

#define NVIC_ICER0                          ((volatile uint32_t*) 0xE000E180 )
#define NVIC_ICER1                          ((volatile uint32_t*) 0xE000E184 )
#define NVIC_ICER2                          ((volatile uint32_t*) 0xE000E188 )
#define NVIC_ICER3                          ((volatile uint32_t*) 0xE000E18C )


/*
* ARM Cortex MX Processor Priority Register
*/

#define NVIC_PR_BASE_ADDR                   ((volatile uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED              4 /* number of priority bits implemented */ 


/*
 * Base addresses of Flash and SRAM 
 */

#define FLASH_BASEADDR						0x08000000U /* flash memory base address   */
#define SRAM1_BASEADDR						0x20000000U /* sram base address 		   */
#define SRAM2_BASEADDR						0x2001C000U /* auxillary sram base address */
#define ROM									0x1FFF0000U /* system memory base address  */
#define SRAM 								SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 *
 */

#define PERIPH_BASEADDR							0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U


/*
 * BASEADDR addresses for peripherals on AHB1 bus
 */

#define GPIOA_BASEADDR					    	(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR						    (AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR                            (AHB1PERIPH_BASEADDR + 0x3800)



/* 
* BASEADDR addresses for peripherals on APB1 bus
*/

#define I2C1_BASEADDR                       (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                       (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                       (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR                       (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                       (APB1PERIPH_BASEADDR + 0x3C00)


#define USART2_BASEADDR                     (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                     (APB1PERIPH_BASEADDR + 0x4800)


#define UART4_BASEADDR                      (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                      (APB1PERIPH_BASEADDR + 0x5000)


/* 
* BASEADDR addresses for peripherals on APB2 bus
*/

#define EXTI_BASEADDR                       (APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR                       (APB2PERIPH_BASEADDR + 0x3000)
//#define SPI4_BASEADDR                       (APB2PERIPH_BASEADDR + 0x3400) 

#define SYSCFG_BASEADDR                     (APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR                     (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                     (APB2PERIPH_BASEADDR + 0x1400)





/*
* Peripheral Register Structures (please refer to the reference manual)
*/

/*
 * Peripheral register structure for GPIOx register
 */

typedef struct {

    volatile uint32_t MODER;                     // GPIO port mode register                                     address offset: 0x00
    volatile uint32_t OTYPER;                    // GPIO output type register                                   address offset: 0x04
    volatile uint32_t OSPEEDR;                   // GPIO output speed register                                  address offset: 0x08
    volatile uint32_t PUPDR;                     // GPIO pull up/pull down register                             address offset: 0x0C
    volatile uint32_t IDR;                       // GPIO input data register                                    address offset: 0x10
    volatile uint32_t ODR;                       // GPIO output data register                                   address offset: 0x14
    volatile uint32_t BSRR;                      // GPIO bit set/reset register                                 address offset: 0x18
    volatile uint32_t LCKR;                      // GPIO configuration lock register                            address offset: 0x1C
    volatile uint32_t AFR[2];                    // AFR[0]: GPIO alt. func. low register                        address offset: 0x20
                                                 // AFR[1]: GPIO alt. func. high register                       address offset: 0x24
} GPIO_RegDef_t;

/*
 * Peripheral register structure for RCC register
 */

typedef struct {

    volatile uint32_t CR;                        // RCC clock control register                                  address offset: 0x00
    volatile uint32_t PLLCFGR;                   // RCC PLL configuration register                              address offset: 0x04
    volatile uint32_t CFGR;                      // RCC clock configuration register                            address offset: 0x08
    volatile uint32_t CIR;                       // RCC clock interrupt register                                address offset: 0x0C
    volatile uint32_t AHB1RSTR;                  // RCC AHB1 peripheral reset register                          address offset: 0x10
    volatile uint32_t AHB2RSTR;                  // RCC AHB2 peripheral reset register                          address offset: 0x14
    volatile uint32_t AHB3RSTR;                  // RCC AHB3 peripheral reset register                          address offset: 0x18
             uint32_t RESERVED0;                 // Reserved                                                    address offset: 0x1C 
    volatile uint32_t APB1RSTR;                  // RCC APB1 peripheral reset register                          address offset: 0x20
    volatile uint32_t APB2RSTR;                  // RCC APB2 peripheral reset register                          address offset: 0x24
             uint32_t RESERVED1[2];              // Reserved                                                    address offset: 0x28
                                                 // Reserved                                                    address offset: 0x2C
    volatile uint32_t AHB1ENR;                   // RCC AHB1 peripheral clock enable register                   address offset: 0x30
    volatile uint32_t AHB2ENR;                   // RCC AHB2 peripheral clock enable register                   address offset: 0x34
    volatile uint32_t AHB3ENR;                   // RCC AHB3 peripheral clock enable register                   address offset: 0x38
             uint32_t RESERVED2;                 // Reserved                                                    address offset: 0x3C
    volatile uint32_t APB1ENR;                   // RCC APB1 peripheral clock enable register                   address offset: 0x40
    volatile uint32_t APB2ENR;                   // RCC APB2 peripheral clock enable register                   address offset: 0x44
             uint32_t RESERVED3[2];              // Reserved                                                    address offset: 0x48
                                                 // Reserved                                                    address offset: 0x4C
    volatile uint32_t AHB1LPENR;                 // RCC AHB1 peripheral clock enable register (low power)       address offset: 0x50
    volatile uint32_t AHB2LPENR;                 // RCC AHB2 peripheral clock enable register (low power)       address offset: 0x54
    volatile uint32_t AHB3LPENR;                 // RCC AHB3 peripheral clock enable register (low power)       address offset: 0x58
             uint32_t RESERVED4;                 // Reserved                                                    address offset: 0x5C
    volatile uint32_t APB1LPENR;                 // RCC APB1 peripheral clock enable register (low power)       address offset: 0x60
    volatile uint32_t APB2LPENR;                 // RCC APB2 peripheral clock enable register (low power)       address offset: 0x64
             uint32_t RESERVED5[2];              // Reserved                                                    address offset: 0x68
                                                 // Reserved                                                    address offset: 0x6C
    volatile uint32_t BDCR;                      // RCC backup domain control register                          address offset: 0x70
    volatile uint32_t CSR;                       // RCC clock control & status register                         address offset: 0x74
             uint32_t RESERVED6[2];              // Reserved                                                    address offset: 0x78
                                                 // Reserved                                                    address offset: 0x7C
    volatile uint32_t SSCGR;                     // RCC spread spectrum clock generation register               address offset: 0x80
    volatile uint32_t PLLI2SCFGR;                // RCC PLLI2S configuration register                           address offset: 0x84
} RCC_RegDef_t;



/*
 * Peripheral register structure for SYSCFG register
 */

typedef struct {
    volatile uint32_t MEMRMP;                    // Memory remap register                                       address offset: 0x00
    volatile uint32_t PMC;                       // Peripheral mode configuration register                      address offset: 0x04
    volatile uint32_t EXTICR[4];                 // EXTI configuration register 1                               address offset: 0x08
                                                 // EXTI configuration register 2                               address offset: 0x0C
                                                 // EXTI configuration register 3                               address offset: 0x10
                                                 // EXTI configuration register 4                               address offset: 0x14
            uint32_t RESERVED[2];                // Reserved                                                    address offset: 0x18
                                                 // Reserved                                                    address offset: 0x1C
    volatile uint32_t CMPCR;                     // Compensation cell control register                          address offset: 0x20


} SYSCFG_RegDef_t;

/*
 * Peripheral register structure for EXTI register
 */

typedef struct {
    volatile uint32_t IMR;                       // Interrupt mask register                                     address offset: 0x00
    volatile uint32_t EMR;                       // Event mask register                                         address offset: 0x04
    volatile uint32_t RTSR;                      // Rising edge trigger selection register                      address offset: 0x08
    volatile uint32_t FTSR;                      // Falling edge trigger selection register                     address offset: 0x0C
    volatile uint32_t SWIER;                     // Software interrupt event register                           address offset: 0x10
    volatile uint32_t PR;                        // Pending register                                            address offset: 0x14
} EXTI_RegDef_t;

/*
* SPIx configuration structure 
*/

typedef struct {  
    volatile uint32_t CR1;                       // SPI control register 1                                      address offset: 0x00
    volatile uint32_t CR2;                       // SPI control register 2                                      address offset: 0x04
    volatile uint32_t SR;                        // SPI status register                                         address offset: 0x08
    volatile uint32_t DR;                        // SPI data register                                           address offset: 0x0C
    volatile uint32_t CRCPR;                     // SPI CRC polynomial register                                 address offset: 0x10
    volatile uint32_t RXCRCR;                    // SPI RX CRC register                                         address offset: 0x14
    volatile uint32_t TXCRCR;                    // SPI TX CRC register                                         address offset: 0x18
    volatile uint32_t I2SCFGR;                   // I2S configuration register                                  address offset: 0x1C
    volatile uint32_t I2SPR;                     // I2S prescaler register                                      address offset: 0x20
} SPI_RegDef_t;


/*
* I2Cx configuration structure
*/

typedef struct {
    volatile uint32_t CR1;                       // I2C control register 1                                      address offset: 0x00                    
    volatile uint32_t CR2;                       // I2C control register 2                                      address offset: 0x04
    volatile uint32_t OAR1;                  // I2C Own address register 1                                  address offset: 0x08
    volatile uint32_t OAR2;                  // I2C Own address register 2                                  address offset: 0x0C
    volatile uint32_t DR;                    // I2C Data register                                           address offset: 0x10
    volatile uint32_t SR1;                   // I2C status register 1                                       address offset: 0x14
    volatile uint32_t SR2;                   // I2C status register 2                                       address offset: 0x18
    volatile uint32_t CCR;                   // I2C clock control register                                  address offset: 0x1C
    volatile uint32_t TRISE;                 // I2C TRISE register                                          address offset: 0x20
    volatile uint32_t FLTR;                  // I2C digital noise filter register                           address offset: 0x24
} I2C_RegDef_t;

/*
* Peripheral definitions (Peripheral base addresses typecasted to xxxx_RegDef_t)
*/

#define GPIOA                       ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                       ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                       ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                       ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                       ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                       ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                       ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                       ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI                       ((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC                         ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI                        ((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG                      ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1                        ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2                        ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3                        ((SPI_RegDef_t*) SPI3_BASEADDR)

//#define SPI4                        ((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1                        ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2                        ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3                        ((I2C_RegDef_t*) I2C3_BASEADDR)

/*
* Clock Enable Macros for GPIOx peripherals
*/

#define GPIOA_PCLK_EN()             (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()             (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()             (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()             (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()             (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()             (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()             (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()             (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()             (RCC->AHB1ENR |= (1 << 8))

/*
* Clock Enable Macros for I2Cx peripherals
*/

#define I2C1_PCLK_EN()              (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()              (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()              (RCC->APB1ENR |= (1 << 23))

/*
* Clock Enable Macros for SPIx peripherals
*/

#define SPI1_PCLK_EN()              (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()              (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()              (RCC->APB1ENR |= (1 << 15))
//#define SPI4_PCLK_EN()              (RCC->APB2ENR |= (1 << 13))               // uncomment for stm32f42xx and stm32f43xx 

/*
* Clock Enable Macros for USARTx peripherals
*/

#define USART1_PCLK_EN()            (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()            (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()            (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()             (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()             (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()            (RCC->APB1ENR |= (1 << 5))
/*
* Clock Enable Macros for SYSCFG peripherals
*/

#define SYSCFG_PCLK_EN()            (RCC->APB2ENR |= (1 << 14))


/*
* Clock Disable Macros for GPIOx peripherals
*/

#define GPIOA_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()             (RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()              (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()              (RCC->APB1ENR &= ~(1 << 14))               
#define SPI3_PCLK_DI()              (RCC->APB1ENR &= ~(1 << 15))


/*
* Clock Disable Macros for I2Cx peripherals
*/

#define I2C1_PCLK_DI()              (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()              (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()              (RCC->APB1ENR &= ~(1 << 23))

/*
* GPIO Reset macros
*/

#define GPIOA_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()           do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)

/*
 * returns port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)    ( (x == GPIOA)?0:\
                                      (x == GPIOB)?1:\
                                      (x == GPIOC)?2:\
                                      (x == GPIOD)?3:\
                                      (x == GPIOE)?4:\
                                      (x == GPIOF)?5:\
                                      (x == GPIOG)?6:\
                                      (x == GPIOH)?7:\
                                      (x == GPIOI)?8: 0)


/*
 *  SPI Reset Macros
 */

#define SPI1_REG_RESET()            do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET()            do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()            do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));} while(0)


/*
 *  I2C Reset Macros
 */

#define I2C_REG_RESET()             do {(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while(0)
#define I2C_REG_RESET()             do {(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define I2C_REG_RESET()             do {(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));} while(0)

/*
 * IRQ Number Macro
 */

#define IRQ_NO_EXTI0                6
#define IRQ_NO_EXTI1                7
#define IRQ_NO_EXTI2                8
#define IRQ_NO_EXTI3                9
#define IRQ_NO_EXTI4                10
#define IRQ_NO_EXTI5_9              23
#define IRQ_NO_EXTI10_15            40
#define IRQ_NO_SPI1                 35
#define IRQ_NO_SPI2                 36
#define IRQ_NO_SPI3                 51

/**
 * NVIC IRQ Priority Macros 
 */

#define NVIC_IRQ_PRI0               0
#define NVIC_IRQ_PRI1               1
#define NVIC_IRQ_PRI2               2
#define NVIC_IRQ_PRI3               3
#define NVIC_IRQ_PRI4               4
#define NVIC_IRQ_PRI5               5
#define NVIC_IRQ_PRI6               6
#define NVIC_IRQ_PRI7               7
#define NVIC_IRQ_PRI8               8
#define NVIC_IRQ_PRI9               9
#define NVIC_IRQ_PRI10              10
#define NVIC_IRQ_PRI11              11
#define NVIC_IRQ_PRI12              12
#define NVIC_IRQ_PRI13              13
#define NVIC_IRQ_PRI14              14
#define NVIC_IRQ_PRI15              15


/*
 * Bit position definitions of SPI_CR1 peripheral register
 */

#define SPI_CR1_CPHA                0   
#define SPI_CR1_CPOL                1
#define SPI_CR1_BR                  3
#define SPI_CR1_MSTR                2
#define SPI_CR1_SPE                 6                 
#define SPI_CR1_LSBFIRST            7
#define SPI_CR1_SSI                 8
#define SPI_CR1_SSM                 9
#define SPI_CR1_RXONLY              10
#define SPI_CR1_DFF                 11
#define SPI_CR1_CRCNEXT             12    
#define SPI_CR1_CRCEN               13
#define SPI_CR1_BIDIOE              14
#define SPI_CR1_BIDIMODE            15

/*
 * Bit position definitions of SPI_CR2 register
 */

#define SPI_CR2_RXDMAEN             0                    
#define SPI_CR2_TXDMAEN             1       
#define SPI_CR2_SSOE                2
#define SPI_CR2_FRF                 4   
#define SPI_CR2_ERRIE               5     
#define SPI_CR2_RXNEIE              6      
#define SPI_CR2_TXEIE               7     

/*
 * Bit position definitions of SPI_SR register
 */


#define SPI_SR_RXNE                 0                         
#define SPI_SR_TXE                  1       
#define SPI_SR_CHSIDE               2          
#define SPI_SR_UDR                  3       
#define SPI_SR_CRCERR               4          
#define SPI_SR_MODF                 5        
#define SPI_SR_OVR                  6       
#define SPI_SR_BSY                  7       
#define SPI_SR_FRE                  8       

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	 15




/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					 14
#define I2C_CCR_FS  				 	 15


/*
 * Generic Macros
 * 
 */

#define ENABLE                      1
#define DISABLE                     0
#define SET                         ENABLE
#define RESET                       DISABLE
#define GPIO_PIN_SET                SET
#define GPIO_PIN_RESET              RESET
#define FLAG_RESET                  RESET
#define FLAG_SET                    SET

#endif /* INC_STM32F407XX_H_ */
