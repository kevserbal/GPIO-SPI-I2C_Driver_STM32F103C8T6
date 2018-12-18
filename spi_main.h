#ifndef __SPI_MAIN_H
#define __SPI_MAIN_H



#include<stdint.h>


// For Transfer Communication
#define CMD_MASTER_READ  		((uint16_t)0x1234)
#define CMD_MASTER_WRITE 	  ((uint16_t)0x5672)
#define CMD_LENGTH        	 2
#define DATA_LENGTH       	 4
#define ACK_LEN           	 2
#define SPI_ACK_BYTES     	 0xD5E5


//definition of SPIs NVIC
#define SPIx_IRQn       	 	 SPI1_IRQn
#define SPIx_IRQHandler 		 SPI1_IRQHandler

#define EXTIx_IRQn           EXTI0_IRQn
#define EXTIx_IRQHandler     EXTI0_IRQHandler


//define button pin and port
#define GPIO_BUTTON_PIN     0
#define GPIO_BUTTON_PORT    GPIOA


//GPIO config for SPI functionality
#define GPIOB_PIN_5         5
#define GPIOB_PIN_6         6
#define GPIOB_PIN_7         7


#define SPI_CLK_PIN         GPIOB_PIN_5
#define SPI_MISO_PIN        GPIOB_PIN_6
#define SPI_MOSI_PIN        GPIOB_PIN_7


#define GPIO_PIN_AF5_SPI1   0x00  //bunu degistir

#define ALT_REMAP_ENABLE    0X01

#define RED_ERR   9
#define RED       8
#define YELLOW    6
#define BLUE      7





#endif

