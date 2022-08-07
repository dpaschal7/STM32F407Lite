#include "stm32f407xx_i2c_driver.h"





void I2C_Init(I2C_Handle_t *pI2CHandle) {

    uint32_t tempreg = 0;
    // enable clock based on handler pI2Cx
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    // assign ack control bit
    pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);


    //configure freq
    
    
    // device address???

    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << 14); // the 14th bit of OAR1 should always be keep to 1 via software
    pI2CHandle->pI2Cx->OAR1 = tempreg;


    //TRISE Config

    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {

    } else {

    }
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
 * @param  *pI2Cx: 
 * @param  EnorDi: 
 * @retval None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {

        if (pI2Cx == I2C1){
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        if (pI2Cx == I2C1){ 
            I2C1_PCLK_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_DI();
        }
    }
}



/**
 * @brief  
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
    
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section  = IRQNumber %4 ;

    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

    *(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}