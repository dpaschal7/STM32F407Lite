
#include "stm32f407xx_spi_driver.h"


/**
 * @brief  
 * @note   
 * @param  *pSPIx: 
 * @param  EnorDi: 
 * @retval None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {

        if (pSPIx == SPI1){
            SPI1_PCLK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_EN();
        }
    } else {
        if (pSPIx == SPI1){ 
            SPI1_PCLK_DI();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_EN();
        }
    }
}


/**
 * @brief  
 * @note   
 * @param  pSPIHandle: 
 * @retval None
 */

void SPI_Init(SPI_Handle_t* pSPIHandle) {
    
    uint32_t tempreg = 0;

    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    //1. configure the device mode
    tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

    //2. Configure bus config

    if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        //clear bidirectional mode 
        //pSPIHandle->pSPIx->CR1 &= ~(1 << 
        tempreg &= ~(1 << 15);


    } else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        //set bidirectional mode
        tempreg |= (1 << 15);
    } else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RXONLY) {
        //clear bidirectional mode
        tempreg &= ~(1 << 15);
        //set RXONLY bit
        tempreg |= (1 << 10);

    }
 
    //3. Configure Serial Clock
    tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;



    //4. Configure DFF

    tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;


    //5. Configure CPOL

    tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

    
    //6. Configure CPHA

    tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

    //7. Configure SSM

    tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;


    pSPIHandle->pSPIx->CR1 = tempreg;


}

/**
 * @brief  
 * @note   
 * @param  pSPIx: 
 * @param  FlagName: 
 * @retval 
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t FlagName) {
    if (pSPIx->SR & FlagName) {
        return FLAG_SET;
    }
    	return FLAG_RESET;

}



/**
 * @brief  
 * @note   
 * @param  pSPIx: 
 * @retval None
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx) {
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    }
}


/**
 * @brief  
 * @note   
 * @param  *pSPIx: 
 * @param  *pTxBuffer: 
 * @param  Len: 
 * @retval None
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {



    while (Len) {
        //1. wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        //2. check DFF bit in CR1

        if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
            //16 bit DFF
            //1. load the data into the DR
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*) pTxBuffer++;
        } else {
            //8 bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }

    }

}

/**
 * @brief  
 * @note   
 * @param  *pSPIx: 
 * @param  EnorDi: 
 * @retval None
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/**
 * @brief  
 * @note   
 * @param  *pSPIx: 
 * @param  EnorDi: 
 * @retval None
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}
