
#include "stm32f407xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


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
            SPI3_PCLK_DI();
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


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
    
     while (Len) {
        //1. wait until RXNE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        //2. check DFF bit in CR1

        if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
            //16 bit DFF
            //1. load data from DR to RxBuffer
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len--;
            Len--;
            (uint16_t*) pRxBuffer++;
        } else {
            //8 bit DFF
            *pRxBuffer = pSPIx->DR  ;
            Len--;
            pRxBuffer++;
        }

    }

}

/**
 * @brief  
 * @note   
 * @param  *pSPIHandle: 
 * @param  *pTxBuffer: 
 * @param  Len: 
 * @retval None
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
    uint8_t state = pSPIHandle->TxState;
    if (state != SPI_BUSY_IN_TX) {

        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE); //Enables TXEIE bit to trigger on interrupt from the TXE flag in SR


    }

    return state;
}

/**
 * @brief  
 * @note   
 * @param  *pSPIHandle: 
 * @param  *pRxBuffer: 
 * @param  Len: 
 * @retval None
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
        uint8_t state = pSPIHandle->RxState;
    if (state != SPI_BUSY_IN_RX) {

        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE); //Enables RXNEIE bit to trigger on interrupt from the RXNE flag in SR


    }

    return state;
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


/**
 * @brief  
 * @note   
 * @param  *pSPIx: 
 * @param  EnorDi: 
 * @retval None
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}


/**
 * @brief  
 * @note   
 * @param  IRQNumber: 
 * @param  EnorDi: 
 * @retval None
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


/**
 * @brief  
 * @note   
 * @param  IRQNumber: 
 * @param  IRQPriority: 
 * @retval None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}


/**
 * @brief  
 * @note   
 * @param  *pSPIHandle: 
 * @retval None
 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
    uint8_t temp1, temp2;

    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (temp1 && temp2) {
        //handle TXE
        spi_txe_interrupt_handle(pSPIHandle);
    }

    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (temp1 && temp2) {
        //handle RXNE
        spi_rxne_interrupt_handle(pSPIHandle);

    }

    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (temp1 && temp2) {
        spi_ovr_err_interrupt_handle(pSPIHandle);
    }
    

}


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
            //16 bit DFF
            //1. load the data into the DR
            pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
            pSPIHandle->TxLen--;
            pSPIHandle->TxLen--;
            (uint16_t*) pSPIHandle->pTxBuffer++;
        } else {
            //8 bit DFF
            pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
            pSPIHandle->TxLen--;
            pSPIHandle->pTxBuffer++;
        }

    if(!pSPIHandle->TxLen) {
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
        pSPIHandle->pTxBuffer = NULL;
        pSPIHandle->TxLen = 0;
        pSPIHandle->TxState = SPI_READY;
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

        if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) ) {
            //16 bit DFF
            //1. load data from DR to RxBuffer
            *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
            pSPIHandle->RxLen--;
            pSPIHandle->RxLen--;
            (uint16_t*) pSPIHandle->pRxBuffer++;
        } else {
            //8 bit DFF
            *pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
            pSPIHandle->RxLen--;
            pSPIHandle->pRxBuffer++;
        }

        if (!pSPIHandle->RxLen) {

            SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
        }

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	uint8_t temp;
	
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {

}