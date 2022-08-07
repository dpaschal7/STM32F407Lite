#include "stm32f407xx_rcc_driver.h"

uint8_t APB_PreScaler[4] = {2, 4, 8, 16};
uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};


/**
 * @brief  
 * @note   
 * @retval uint32_t
 */
uint32_t RCC_GetPCLK1Value(void) {
    uint32_t pclk, SystemClock;
    uint8_t clksrc, apb1p, ahbp, temp;


    clksrc = ((RCC->CFGR >> 2) & 0x3);
   
    if (clksrc == 0) {
        SystemClock = 16000000;
    } else if (clksrc == 1) {
        SystemClock = 8000000;
    } else if (clksrc == 2) {
        SystemClock = RCC_GetPLLOutputClock();
    }

    //get ahb prescaler
    temp = ((RCC->CFGR >> 4) & 0xF);
    
    if (temp < 8) {
        ahbp = 1;
    } else {
        ahbp = AHB_PreScaler[temp - 8];
    }

    //get apb1 prescaler
    temp = ((RCC->CFGR >> 10 ) & 0x7);

    if (temp < 4) {
        apb1p = 1;

    } else {
        apb1p = APB_PreScaler[temp - 4];
    }


    pclk = (SystemClock /ahbp) / apb1p;

    return pclk;
    
}




/**
 * @brief  
 * @note   
 * @retval 
 */
uint32_t RCC_GetPCLK2Value(void) {

    uint32_t pclk, SystemClock;
    uint8_t clksrc, apb2p, ahbp, temp;


    clksrc = ((RCC->CFGR >> 2) & 0x3);
   
    if (clksrc == 0) {
        SystemClock = 16000000;
    } else if (clksrc == 1) {
        SystemClock = 8000000;
    } else if (clksrc == 2) {
        SystemClock = RCC_GetPLLOutputClock();
    }

    //get ahb prescaler
    temp = ((RCC->CFGR >> 4) & 0xF);
    
    if (temp < 0x08) {
        ahbp = 1;
    } else {
        ahbp = AHB_PreScaler[temp - 8];
    }

    //get apb2 prescaler
    temp = ((RCC->CFGR >> 13 ) & 0x7);

    if (temp < 0x04) {
        apb2p = 1;

    } else {
        apb2p = APB_PreScaler[temp - 4];
    }


    pclk = (SystemClock /ahbp) / apb2p;

    return pclk;
}

/**
 * @brief  
 * @note   
 * @retval 
 */
uint32_t RCC_GetPLLOutputClock(void) {
    return 0;
}