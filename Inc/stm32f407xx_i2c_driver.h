#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

typedef struct {
    uint32_t I2C_SCLSpeed;
    uint8_t  I2C_DeviceAddress;
    uint8_t  I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
} I2C_Handle_t;


#endif