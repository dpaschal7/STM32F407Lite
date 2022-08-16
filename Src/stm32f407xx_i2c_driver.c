#include "stm32f407xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SubnodeAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SubnodeAddr);

static void I2C_MainHandleTXEInterrupt(I2C_Handle_t  *pI2CHandle);
static void I2C_MainHandleRXNEInterrupt(I2C_Handle_t  *pI2CHandle);


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SubnodeAddr) {
    SubnodeAddr = SubnodeAddr << 1;
    SubnodeAddr &= ~(1); // subnode addr + r/!w bit (r/!w set to 0)
    pI2Cx->DR = SubnodeAddr;
}

static void I2C_ExecuteAddressRead(I2C_RegDef_t *pI2Cx, uint8_t SubnodeAddr) {
    SubnodeAddr = SubnodeAddr << 1;
    SubnodeAddr |= 1; // subnode addr + r/!w bit (r/!w set to 1)
    pI2Cx->DR = SubnodeAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
    uint32_t tempread;
    
    //check if device is in main or subnode mode
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { //I2C_SR2_MSL == 0 (subnode)
                                                       //I2C_SR2_MSL == 1 (main)
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) { //if I2C is busy in rx, disable the ack and then clear the addr flag
            I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
            //reading from SR1 and then SR2 to clear ADDR flag
            tempread = pI2CHandle->pI2Cx->SR1;
		    tempread = pI2CHandle->pI2Cx->SR2;
            (void)tempread;

        } else {
            tempread = pI2CHandle->pI2Cx->SR1;
		    tempread = pI2CHandle->pI2Cx->SR2;
            (void)tempread;
        }
    } else {
        tempread = pI2CHandle->pI2Cx->SR1;
		tempread = pI2CHandle->pI2Cx->SR2;
        (void)tempread;
    }

}
 

void I2C_Init(I2C_Handle_t *pI2CHandle) {

    uint32_t tempreg = 0;
    // enable clock based on handler pI2Cx
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    // assign ack control bit
    pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);


    //configure freq
    tempreg = 0;
    tempreg = RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);
    
    //configure device address (7 bit address mode) 

    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << 14); // the 14th bit of OAR1 should always be keep to 1 via software
    pI2CHandle->pI2Cx->OAR1 |= tempreg;


    uint16_t ccr_value = 0;
    tempreg = 0;

    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        pI2CHandle->pI2Cx->CCR &= ~(1 << 15);
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempreg |= (ccr_value & 0xFFF);
        
    } else {
        // I2C_SCL_SPEED_FM
     
        tempreg |= (1 << 15); // enable fast mode
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));

        } else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9) {
            ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        tempreg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;


    //TRISE Config
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1;

	} else {
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}



/**
 * @brief  
 * @note   
 * @param  *pI2Cx: 
 * @retval None
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
    if (pI2Cx == I2C1) {
        I2C1_REG_RESET();
    } else if (pI2Cx == I2C2) {
        I2C2_REG_RESET();
    } else if (pI2Cx == I2C3) {
        I2C3_REG_RESET();
    }   
}


/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @param  *pTxBuffer: 
 * @param  Len: 
 * @param  SubnodeAddr: 
 * @param  Sr: 
 * @retval None
 */
void I2C_MainSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SubnodeAddr, uint8_t Sr) {

    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));//check SB flag to make sure start condition has been generated

    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SubnodeAddr);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); // check that address phase is completed

    I2C_ClearADDRFlag(pI2CHandle);

    while (Len) {
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    //after data length is zero wait for transmission buffer to be empty and then check the byte transfer finished flag
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); //btf == 0 (data transfer not done)
                                                                //btf == 1 (data transfer succeeded)
    if (Sr == I2C_DISABLE_SR) {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }

}


/**
 * @brief   
 * @note   
 * @param  *pI2CHandle: 
 * @param  *pTxBuffer: 
 * @param  Len: 
 * @param  SubnodeAddr: 
 * @param  Sr: 
 * @retval None
 */

void I2C_MainReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SubnodeAddr, uint8_t Sr) {
    
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SubnodeAddr);

    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    

    if (Len == 1) {
        
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
        
        I2C_ClearADDRFlag(pI2CHandle);

        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

        if (Sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        *pRxBuffer = pI2CHandle->pI2Cx->DR;
    }

    if (Len > 1) {

        I2C_ClearADDRFlag(pI2CHandle);

        for (uint32_t i = Len; i > 0; --i) {

            while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

            if (i == 2) {
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                if (Sr == I2C_DISABLE_SR) {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
            }

            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            pRxBuffer++;
        } 

    }

    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
        

}


/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @param  *pTxBuffer: 
 * @param  Len: 
 * @param  SubnodeAddr: 
 * @param  Sr: 
 * @retval 
 */
uint8_t I2C_MainSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SubnodeAddr, uint8_t Sr) {

    uint8_t state = pI2CHandle->TxRxState;

    if ((state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX)) {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SubnodeAddr;
        pI2CHandle->Sr = Sr;

        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return state;
}

/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @param  *pRxBuffer: 
 * @param  Len: 
 * @param  SubnodeAddr: 
 * @param  Sr: 
 * @retval 
 */
uint8_t I2C_MainReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SubnodeAddr, uint8_t Sr) {
    
    uint8_t state = pI2CHandle->TxRxState;

    if ((state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX)) {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->RxSize = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->DevAddr = SubnodeAddr;
        pI2CHandle->Sr = Sr;

        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return state;
}

/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @retval None
 */
static void I2C_MainHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {
    if(pI2CHandle->TxLen > 0) {
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        pI2CHandle->TxLen--;
        pI2CHandle->pTxBuffer++;
    }
}

/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @retval None
 */
static void I2C_MainHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {

    if (pI2CHandle->RxSize == 1) {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxSize > 1) {

        if (pI2CHandle->RxLen == 2) {
            I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
        }

        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;

    }

    if (pI2CHandle->RxLen == 0) {
        //close data reception and change state to CMPLT

        if (pI2CHandle->Sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        I2C_CloseReceiveData(pI2CHandle);

        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }


}

/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @retval None
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	//ITBUFEN Control Bit = 0
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//ITEVFEN Control Bit = 0
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}


/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @retval None
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	//ITBUFEN Control Bit = 0
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//ITEVFEN Control Bit = 0
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}

/**
 * @brief  
 * @note   
 * @param  *pI2C: 
 * @param  data: 
 * @retval None
 */
void I2C_SubnodeSendData(I2C_RegDef_t *pI2C, uint8_t data) {
    pI2C->DR = data;
}

/**
 * @brief  
 * @note   
 * @param  *pI2C: 
 * @retval 
 */
uint8_t I2C_SubnodeReceiveData(I2C_RegDef_t *pI2Cx) {
    return (uint8_t) pI2Cx->DR;
}

/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @retval None
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

    if (temp1 && temp3) {
        //handle for interrupt generated by SB event

		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
    }

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//handle for interrupt generated by ADDR event
	//Note : When main mode : Address is sent
	//		 When subnode mode : Address matched with own address
	if (temp1 && temp3) {
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    //handle for interrupt generated by BTF event

    if (temp1 && temp3) {
        
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			//confirm that TXE is set
			if (pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE)) {
				//BTF, TXE = 1
				if (pI2CHandle->TxLen == 0) {
					
					if (pI2CHandle->Sr == I2C_DISABLE_SR) {
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }
					I2C_CloseSendData(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

				}
			}

		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
            ; //LOOK AT PAGE 849 OF REFERENCE MANUAL (you shouldn't need to do anything because communication is closed via the RXNE handle function)
		}
    }

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//Handle for interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only subnode mode. For main this flag will never be set
	//The below code block will not be executed by the main since STOPF will not set in main mode
	if (temp1 && temp3) {
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}



    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//Handle For interrupt generated by TXE event
    if (temp1 && temp2 && temp3) {
		//Check for device mode
		if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)) {			
            //TXE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MainHandleTXEInterrupt(pI2CHandle);
			}
		} else {
			//subnode
			//make sure that the subnode is in transmitter mode
		    if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)) {
		    	I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		    }
		}
	}

    temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//handle for interrupt generated by RXNE event
	if (temp1 && temp2 && temp3) {
		//check device mode
		if (pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)) {

			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MainHandleRXNEInterrupt(pI2CHandle);
			}

		} else {
			//subnode
			//make sure that the subnode is in receiver mode
			if (!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/**
 * @brief  
 * @note   
 * @param  *pI2CHandle: 
 * @retval None
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);


	temp1 = (pI2CHandle->pI2Cx->SR1) & (1<< I2C_SR1_BERR);
	if (temp1  && temp2 ) {
	   pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
	   I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO );
	if (temp1  && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if (temp1  && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if (temp1  && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if (temp1  && temp2) {
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/**
 * @brief  
 * @note   
 * @param  *pI2Cx: 
 * @param  EnorDi: 
 * @retval None
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

/**
 * @brief  
 * @note   
 * @param  *pI2Cx: 
 * @param  EnorDi: 
 * @retval None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {

        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        if (pI2Cx == I2C1) { 
            I2C1_PCLK_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_DI();
        }
    }
}



/**
 * @brief  Enables or disables the register pointed to by pI2Cx with the ENABLE or DISABLE macro 
 * @note   
 * @param  *pI2Cx: 
 * @param  EnorDi: 
 * @retval None
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pI2Cx->CR1 |= (1 < I2C_CR1_PE);
    } else {
        pI2Cx->CR1 &= ~(1 < I2C_CR1_PE);
    }
}


/**
 * @brief  
 * @note   
 * @param  pI2Cx: 
 * @param  FlagName: 
 * @retval 
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t * pI2Cx, uint32_t FlagName) {
    if (pI2Cx->SR1 & FlagName) {
        return FLAG_SET;
    } else {
        return FLAG_RESET;
    }
}


/**
 * @brief  
 * @note   
 * @param  IRQNumber: 
 * @param  EnorDi: 
 * @retval None
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section  = IRQNumber %4;

    uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |=  (IRQPriority << shift_amount );

}

/**
 * @brief  
 * @note   
 * @param  *pI2Cx: 
 * @param  EnorDi: 
 * @retval None
 */
void I2C_SubnodeEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
    } else {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
    }
}