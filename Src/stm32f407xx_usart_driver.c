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

/**
 * @brief  
 * @note   
 * @param  *pUSARTx: 
 * @param  EnorDi: 
 * @retval None
 */
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

/**
 * @brief  
 * @note   
 * @param  *pUSARTHandle: 
 * @param  *pTxBuffer: 
 * @param  Len: 
 * @retval None
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {

    uint16_t *pdata;

    for (uint32_t i = 0; i < Len; i++) {
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TXE));
        
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            
            pdata = (uint16_t *) pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                
                // if no parity is used then 9 bits of data will sent (i.e. inc buffer twice);
                pTxBuffer++;
                pTxBuffer++;
            } else {
                pTxBuffer++;
            }
        } else {

            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
            pTxBuffer++;
        }
    }

    while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}


/**
 * @brief  
 * @note   
 * @param  *pUSARTHandle: 
 * @param  *pRxBuffer: 
 * @param  Len: 
 * @retval None
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {

    for (uint32_t i = 0; i < Len; i++) {
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            //parity
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {

                //all nine bits will be user data
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				pRxBuffer++;
				pRxBuffer++;

            } else {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
                pRxBuffer++;
            }

        } else {// 8 bit data frame

            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
            } else {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
            }

            pRxBuffer++;
        }
    }
}

/**
 * @brief  
 * @note   
 * @param  *pUSARTHandle: 
 * @param  *pTxBuffer: 
 * @param  Len: 
 * @retval 
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {

        uint8_t txstate = pUSARTHandle->TxBusyState;

        if (txstate != USART_BUSY_IN_TX) {
            pUSARTHandle->TxLen = Len;
            pUSARTHandle->pTxBuffer = pTxBuffer;
            pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
        }
        
        return txstate;
}


/**
 * @brief  
 * @note   
 * @param  *pUSARTHandle: 
 * @param  *pRxBuffer: 
 * @param  Len: 
 * @retval 
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len) {

    	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (rxstate != USART_BUSY_IN_RX) {
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);

	}

	return rxstate;
}


/**
 * @brief  
 * @note   
 * @param  *pUSARTx: 
 * @param  StatusFlagName: 
 * @retval None
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);

}

/**
 * @brief  
 * @note   
 * @param  *pUSARTHandle: 
 * @retval None
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {
    uint32_t temp1, temp2, temp3;
    uint16_t *pdata;
     

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

    if (temp1 && temp2) {

        if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {

            if (!pUSARTHandle->TxLen) {
                pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;
				//Reset Buffer address
				pUSARTHandle->pTxBuffer = NULL;
				//Reset the length
				pUSARTHandle->TxLen = 0;
			
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
            }

        }

    }


    
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);

    if (temp1 && temp2) {


		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			
			if (pUSARTHandle->TxLen > 0) {
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}


    temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if (temp1 && temp2 ) {
		//RXNE interrupt
		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) {
			if (pUSARTHandle->RxLen > 0) {
                
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					
					//check USART_ParityControl
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						//No parity is used; all 9bits = user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					} else {
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				} else {
					//Now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						//No parity is used , so all 8bits will be of user data
						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					} else {
						//Parity is used, so , 7 bits = user data; 1 bit = parity
						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}


			} 

			if (!pUSARTHandle->RxLen) {
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}


    //Note : CTS feature is not applicable for UART4 and UART5
    //Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if (temp1  && temp2 ) {
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

    	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);


	if (temp1 && temp2) {
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

    
    temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if (temp1  && temp2) {
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag.
		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}


    	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if (temp2) {
		temp1 = pUSARTHandle->pUSARTx->SR;
		if (temp1 & ( 1 << USART_SR_FE)) {
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if (temp1 & ( 1 << USART_SR_NE) ) {
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if (temp1 & ( 1 << USART_SR_ORE) ) {
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}


}


/**
 * @brief  
 * @note   
 * @param  *pUSARTHandle: 
 * @param  event: 
 * @retval None
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event) {

}
