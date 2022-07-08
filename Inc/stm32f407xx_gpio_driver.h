/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jun 4, 2022
 *      Author: Drew
 */


/* 
* Takes care of:
* GPIO Initialization
* Enable/Disable GPIO port clock
* Read/Write to GPIO pin/port 
* Configures alt. func.
* Interrupt Handling
*/

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
* GPIO pin configuration structure
*/

typedef struct {
  
  uint8_t GPIO_PinNumber;                       //check @GPIO PIN NO
  uint8_t GPIO_PinMode;                         //check @GPIO MODES
  uint8_t GPIO_PinSpeed;                        //check @GPIO SPEED
  uint8_t GPIO_PinPuPdControl;                  //check @GPIO PUPD
  uint8_t GPIO_PinOPType;                       //check @GPIO OUTPUT TYPE
  uint8_t GPIO_PinAltFunMode;

} GPIO_PinConfig_t;


/*
* Handle structure for a GPIO pin
*/

typedef struct {

   GPIO_RegDef_t* pGPIOx;                           //pointer to hold the base address of the GPIO peripheral port to which the pin belongs
   GPIO_PinConfig_t GPIO_PinConfig;                 //GPIO Pin Configuration settings

} GPIO_Handle_t;


/*
* @GPIO PIN NO
*  GPIO pin number
*/

#define GPIO_PIN_NO_0            0
#define GPIO_PIN_NO_1            1
#define GPIO_PIN_NO_2            2
#define GPIO_PIN_NO_3            3
#define GPIO_PIN_NO_4            4
#define GPIO_PIN_NO_5            5
#define GPIO_PIN_NO_6            6
#define GPIO_PIN_NO_7            7
#define GPIO_PIN_NO_8            8
#define GPIO_PIN_NO_9            9
#define GPIO_PIN_NO_10           10
#define GPIO_PIN_NO_11           11
#define GPIO_PIN_NO_12           12
#define GPIO_PIN_NO_13           13
#define GPIO_PIN_NO_14           14
#define GPIO_PIN_NO_15           15


/*
* @GPIO MODES
* GPIO modes 
*/

#define GPIO_MODE_IN              0
#define GPIO_MODE_OUT             1
#define GPIO_MODE_ALTFUN          2
#define GPIO_MODE_ANALOG          3
#define GPIO_MODE_IT_FT           4
#define GPIO_MODE_IT_RT           5
#define GPIO_MODE_IT_RFT          6


/* 
* @GPIO OUTPUT TYPE
* GPIO output types 
*/

#define GPIO_OP_TYPE_PP           0         // pushpull  output type
#define GPIO_OP_TYPE_OD           1         // open drain output type


/* 
* @GPIO SPEED
* GPIO output speeds
*/

#define GPIO_SPEED_LOW            0
#define GPIO_SPEED_MEDIUM         1
#define GPIO_SPEED_FAST           2
#define GPIO_SPEED_HIGH           3


/*
* @GPIO PUPD
* GPIO pull up/ pull down configuration
*/

#define GPIO_NO_PUPD               0
#define GPIO_PU                    1
#define GPIO_PD                    2


/*
* Peripheral Clock setup
*/


void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

/*
* Init & De-init
*/

void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);


/*
* Data read & write
*/



uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
* IRQ Configuration and ISR handling
*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);





#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
