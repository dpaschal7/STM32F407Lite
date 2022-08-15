#include "stm32f407xx_usart_driver.h"



/**
 * @brief  
 * @note   
 * @param  *pUSARTx: 
 * @param  BaudRate: 
 * @retval None
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {

    uint32_t PCLKx,
             usartdiv,
             M_part,
             F_part;
    
    uint32_t tempreg = 0;

    if (pUSARTx == USART1 || pUSARTx == USART6) {
        PCLKx = RCC_GetPCLK2Value();
    } else {
        PCLKx = RCC_GetPCLK1Value();
    }
    
  
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
    	   //OVER8 = 1 , over sampling by 8
    	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
    } else {
    	   //oversampling by 16
    	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
    }

    M_part = usartdiv/100;
    
    tempreg |= (M_part << USART_BRR_MANT);

    //Extract the fraction
    F_part = (usartdiv - (M_part * 100));

    if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8)) {
    	  //OVER8 = 1 , over sampling by 8
    	  F_part = ((( F_part * 8)+ 50) / 100) & ((uint8_t)0x07);
     } else {
    	   //oversampling by 16
    	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
     }

    tempreg |= (F_part << USART_BRR_FRAC);
    pUSARTx->BRR = tempreg;
}

/**
 * @brief  
 * @note   
 * @param  *pUSARTHandle: 
 * @retval None
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
    uint32_t tempreg = 0;

    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
        
        tempreg |= (1 << USART_CR1_RE);
    } else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
        tempreg |= (1 << USART_CR1_TE);
    } else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
        tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
    }

    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
        tempreg |= (1 << USART_CR1_PCE);
    } else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
        tempreg |= (1 << USART_CR1_PCE);
        tempreg |= (1 << USART_CR1_PS); //  1 = odd parity, 0 = even parity
    }

    pUSARTHandle->pUSARTx->CR1 = tempreg;

    tempreg = 0;
    tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);
    pUSARTHandle->pUSARTx->CR2 = tempreg;

    tempreg = 0;
    
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
        tempreg |= (1 << USART_CR3_CTSE);
    } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
        tempreg |= (1 << USART_CR3_RTSE);
    } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
        tempreg |= (1 << USART_CR3_CTSE);
        tempreg |= (1 << USART_CR3_RTSE);
    }

    pUSARTHandle->pUSARTx->CR3 = tempreg;

    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/**
 * @brief  
 * @note   
 * @param  pUSARTx: 
 * @retval None
 */
void USART_DeInit(USART_RegDef_t* pUSARTx) {
    if (pUSARTx == USART1) {
        USART1_REG_RESET();  
    } else if (pUSARTx == USART2) {
        USART2_REG_RESET();
    } else if (pUSARTx == USART3) {
        USART3_REG_RESET();
    } else if (pUSARTx == UART4) {
        UART4_REG_RESET();
    } else if (pUSARTx == UART5) {
        UART5_REG_RESET();
    } else if (pUSARTx == USART6) {
        USART6_REG_RESET();
    }
}

/**
 * @brief  
 * @note   
 * @param  *pUSARTx: 
 * @param  EnorDi: 
 * @retval None
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi) {
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4) {
		 	UART4_PCLK_EN();
		} else if (pUSARTx == UART5) {
            UART5_PCLK_EN();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_EN();
        }
	} else {
      if(pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4) {
		 	UART4_PCLK_DI();
		} else if (pUSARTx == UART5) {
            UART5_PCLK_DI();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_DI();
        }  
	}
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pUSARTx->CR1 |= (1 << USART_CR1_UE);
    } else {
        pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
    }
}

/**
 * @brief  
 * @note   
 * @param  *pUSARTx: 
 * @param  StatusFlagName: 
 * @retval 
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName) {
    if (pUSARTx->SR & StatusFlagName) {
    	return SET;
    }
   return RESET;
}


/**
 * @brief  
 * @note   
 * @param  IRQNumber: 
 * @param  IRQPriority: 
 * @retval None
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section  = IRQNumber %4 ;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED) ;

    *(NVIC_PR_BASE_ADDR + iprx) |=  ( IRQPriority << shift_amount );

}

/**
 * @brief  
 * @note   
 * @param  IRQNumber: 
 * @param  EnorDi: 
 * @retval None
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (IRQNumber <= 31) {
            *NVIC_ISER0 |= ( 1 << IRQNumber );

        } else if (IRQNumber > 31 && IRQNumber < 64 ) {
            *NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
        } else if (IRQNumber >= 64 && IRQNumber < 96 ) {
            *NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
        }
    } else {
        if (IRQNumber <= 31) {
            *NVIC_ICER0 |= ( 1 << IRQNumber );
        } else if (IRQNumber > 31 && IRQNumber < 64 ) {
            *NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
        } else if (IRQNumber >= 6 && IRQNumber < 96 ) {
            *NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
        }
    }

}