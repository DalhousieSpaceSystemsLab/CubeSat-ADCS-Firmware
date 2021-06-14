#ifndef I2CB1_H_
#define I2CB1_H_

#include <stdint.h>

#define TYPE_0_LENGTH   2
#define MAX_BUFFER_SIZE     20

//******************************************************************************
//*************************General I2C State Machine ***************************
//******************************************************************************
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

void i2c_b1_init();

I2C_Mode I2CB1_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

I2C_Mode I2CB1_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count);

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);

#endif /* #ifndef I2CB1_H_ */
