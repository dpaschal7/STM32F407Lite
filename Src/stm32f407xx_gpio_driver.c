


#include "stm32f407xx_gpio_driver.h"

/*
* Peripheral Clock setup
*/

/**
 * @brief  Enables or disables the peripheral clock for the given GPIO port
 * @note   
 * @param  pGPIOx: Base address of the GPIO port
 * @param  EnorDi: ENABLE or DISABLE macros
 * @retval None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    
    if (EnorDi == ENABLE) {

        if (pGPIOx == GPIOA){
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_EN();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_EN();
        }
 
    } else {
        if (pGPIOx == GPIOA){
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_DI();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_DI();
        }
    }


}


/*
* Init & De-init
*/

/**
 * @brief  
 * @note   
 * @param  pGPIOHandle: 
 * @retval None
 */

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{

    uint32_t temp = 0;
    //enable GPIO Port clock
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure gpio mode

    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
            temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
            pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing register bits
            pGPIOHandle->pGPIOx->MODER |= temp; // setting register bits
            temp = 0;
    } else {

            if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {

                //1. configure the FTSR
                EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

                //clear the RTSR bit
                EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {

                //2. configure the RTSR
                EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

                //clear the FTSR bit
                EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {

                //3. configure both the FTSR and RSTR
                EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
                EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            }
        //2. configure the GPIO port selection in SYSCFG_EXTICR

        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

        //3. enable the EXTI interrupt delivery using the interrupt mask register
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }
    temp = 0;
    //2. configure gpio speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;



    //3. configure gpio pull up/pull down resistors

    temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    //4. configure gpio output type
    temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;

    //5. configure alt. func.
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN) {
        uint8_t temp1, temp2; 
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }

}

/**
 * @brief  
 * @note   
 * @param  pGPIOx: 
 * @retval None
 */

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
    if (pGPIOx == GPIOA){
            GPIOA_REG_RESET();
        } else if (pGPIOx == GPIOB) {
            GPIOB_REG_RESET();
        } else if (pGPIOx == GPIOC) {
            GPIOC_REG_RESET();
        } else if (pGPIOx == GPIOD) {
            GPIOD_REG_RESET();
        } else if (pGPIOx == GPIOE) {
            GPIOE_REG_RESET();
        } else if (pGPIOx == GPIOG) {
            GPIOG_REG_RESET();
        } else if (pGPIOx == GPIOF) {
            GPIOF_REG_RESET();
        } else if (pGPIOx == GPIOH) {
            GPIOH_REG_RESET();
        } else if (pGPIOx == GPIOI) {
            GPIOI_REG_RESET();
        }
}



/*
* Data read & write
*/

/**
 * @brief  
 * @note   returns 0 or 1
 * @param  pGPIOx: 
 * @param  PinNumber: 
 * @retval uint8_t:
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
    value = (uint8_t) (pGPIOx->IDR >> PinNumber) & 0x00000001;
    return value;
}


/**
 * @brief  
 * @note   
 * @param  pGPIOx: 
 * @retval 
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
    value = (uint16_t) pGPIOx->IDR;
    return value;
}

/**
 * @brief  
 * @note   
 * @param  pGPIOx: 
 * @param  PinNumber: 
 * @param  Value: 
 * @retval None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET) {
        	pGPIOx->ODR |= (0x1 << PinNumber);
    } else {
            pGPIOx->ODR &= ~(1 << PinNumber);
    }

}
/**
 * @brief  
 * @note   
 * @param  pGPIOx: 
 * @param  Value: 
 * @retval None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}
/**
 * @brief  
 * @note   
 * @param  pGPIOx: 
 * @param  PinNumber: 
 * @retval None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
* IRQ Configuration and ISR handling
*/

/**
 * @brief  
 * @note   
 * @param  IRQNumber: 
 * @param  EnorDi: 
 * @retval None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE) {
        if (IRQNumber <= 31) { 
        
            *NVIC_ISER0 |= (1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber < 64) {
            
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));

        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
            
        }
    } else {
        if (IRQNumber <= 31) { 
        
            *NVIC_ICER0 |= (1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber < 64) {
            
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));

        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
            
        }
    }
}

/**
 * @brief clears the EXTI pending register corresponding to the provide pin number 
 * @note   
 * @param  PinNumber: 
 * @retval None
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
    if((EXTI->PR) & (1 << PinNumber)) {
        //clear
        EXTI->PR |= (1 << PinNumber);
    }
}

/**
 * @brief  
 * @note   
 * @param  IRQPriority: 
 * @retval None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) 
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);


    *(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount); // refer to the NVIC section of the Cortex M4 user guide to find out how many bits are used for each priority field
}
