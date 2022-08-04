#include "stm32f407xx_i2c_driver.h"




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