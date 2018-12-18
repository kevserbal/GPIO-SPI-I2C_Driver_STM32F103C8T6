#include <stdint.h>
#include "hal_gpio_driver.h"
#include "hal_spi_driver.h"
#include "spi_main.h"
#include "stm32f10x.h"
#include <string.h> 
spi_handler_typedef SpiHandler;

uint16_t testRead=0;
uint8_t  ack_buf[2]={0,0};  

uint8_t slave_reply_data[4] ={0x02,0x05,0x45,0x24};

uint8_t master_write_data[]={0xa, 0xb, 0xc, 0xd };

uint8_t master_read_buffer[4];


void spi_gpio_init(){

 
        gpio_pin_config_t  gpio_spi;
				
	
				_HAL_RCC_GPIOA_CLOCK_ENABLE();
	
	      gpio_spi.pin=SPI_CLK_PIN;
				gpio_spi.mode=GPIO_PIN_ALT_PUSH_PULL;
	      gpio_spi.speed=GPIO_PIN_OUTPUT_MODE_SPEED_MEDIUM;
	
				hal_gpio_altarnete_conf(ALT_PORTB,gpio_spi.pin,ALT_SPI1_REG,gpio_spi.mode);      
				hal_gpio_init(GPIOA,&gpio_spi);
        


	      gpio_spi.pin=SPI_MISO_PIN;
				gpio_spi.mode=GPIO_PIN_ALT_PUSH_PULL;
				hal_gpio_altarnete_conf(ALT_PORTB,gpio_spi.pin,ALT_SPI1_REG,gpio_spi.mode);
				hal_gpio_init(GPIOA,&gpio_spi);



	      gpio_spi.pin=SPI_MOSI_PIN;
				gpio_spi.mode=GPIO_PIN_ALT_PUSH_PULL;
				hal_gpio_altarnete_conf(ALT_PORTB,gpio_spi.pin,ALT_SPI1_REG,gpio_spi.mode);
				hal_gpio_init(GPIOA,&gpio_spi);

     
}



void delay_gen( )
{
		uint32_t i=0;
		
		for(i=0;i< 800000;i++);
}

void assert_error(void)
{
	while(1)
	{
	  led_toggle(GPIOB,RED_ERR);
		delay_gen();
	}
}


uint16_t BufferComp(uint8_t* pBuff1,uint8_t* pBuff2,uint8_t BufferLength)
{
	while(BufferLength--)
	{
	    if(pBuff1[BufferLength] != pBuff2[BufferLength])
				 return BufferLength;
	
	}
	return 0;
	
}



void SPI1_IRQHandler(void)
{

}



void EXTI0_IRQHandler(void)
{

	
	
}