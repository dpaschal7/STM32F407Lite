/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 3, 2022
 *      Author: Drew
 */

/* 
* Takes care of:
* SPI Initialization
* Enable/Disable SPI clock
* SPI clock control
* SPI TX & RX 
* Configures SPI interrupt 
* Interrupt Handling
*/


#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx Peripheral 
 */

typedef struct {
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct {
    SPI_RegDef_t* pSPIx;                //pointer to hold base address of SPI peripheral being used
    SPI_Config_t SPI_Config;            //SPI configuration settings
} SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER                  1
#define SPI_DEVICE_MODE_SLAVE                   0


/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD                       1
#define SPI_BUS_CONFIG_HD                       2
//#define SPI_BUS_CONFIG_SIMP_TXONLY              3
#define SPI_BUS_CONFIG_SIMP_RXONLY              3


/*
 * @SPI_SCLKSpeed
 */

#define SPI_SCLK_SPEED_DIV2                     0
#define SPI_SCLK_SPEED_DIV4                     1
#define SPI_SCLK_SPEED_DIV8                     2
#define SPI_SCLK_SPEED_DIV16                    3
#define SPI_SCLK_SPEED_DIV32                    4
#define SPI_SCLK_SPEED_DIV64                    5
#define SPI_SCLK_SPEED_DIV128                   6
#define SPI_SCLK_SPEED_DIV256                   7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS                           0
#define SPI_DFF_16BITS                          1


/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH                           1
#define SPI_CPOL_LOW                            0


/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH                           1
#define SPI_CPHA_LOW                            0


/*
 * @SPI_SSM
 */

#define SPI_SSM_EN                              1
#define SPI_SSM_DI                              0


/*
 * SPI status flag macros
 */

#define SPI_TXE_FLAG                            (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG                           (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG                           (1 << SPI_SR_BSY)
#define SPI_CHSIDE_FLAG                         (1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG                            (1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG                         (1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG                           (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG                            (1 << SPI_SR_OVR)
#define SPI_FRE_FLAG                            (1 << SPI_SR_FRE)



/* 
 * Peripheral Clock setup 
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* 
 * Init & Deinit 
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/* 
 * Data TX & RX 
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t FlagName);

/* 
 * Peripheral Controls Setup 
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* 
 * IRQ Configuration and ISR handling 
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */