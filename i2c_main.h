
#ifndef __I2C_MAIN_H
#define __I2C_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


#define GPIO_PIN__6      6
#define GPIO_PIN__9      9

#define I2C_SCL_LINE    GPIO_PIN__6
#define I2C_SDA_LINE    GPIO_PIN__9

void i2c_gpio_init();

static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);





#endif
