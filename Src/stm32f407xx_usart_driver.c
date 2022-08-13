#include "stm32f407xx_usart_driver.h"



/**
 * @brief  
 * @note   
 * @param  *pUSARTx: 
 * @param  EnorDi: 
 * @retval None
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
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

    uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

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