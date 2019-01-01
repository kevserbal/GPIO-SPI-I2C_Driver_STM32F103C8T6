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
/*
int main(void)
{   	
    
		uint16_t i =0;
	  uint8_t  AdrrCMD[CMD_LENGTH];
 
	
	  spi_gpio_init();
	  

	  led_init();
	
    _HAL_RCC_GPIOA_CLOCK_ENABLE();


    _HAL_RCC_SPI1_CLK_ENABLE();
	  hal_gpio_config_interrupt(GPIO_BUTTON_PIN,INT_FALLING_EDGE);
	
    hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);


    SpiHandler.Instance=SPI1;
	  SpiHandler.init.BoundRatePreScaler=spi_bound_rate_f_div_256;
	  SpiHandler.init.direction=spi_enable_2_line_uni_dir;
	  SpiHandler.init.FirstBit=spi_tx_msbfirst;
	  SpiHandler.init.CLKPol=spi_clock_pol_low;
	  SpiHandler.init.DataSize=spi_8bit_dff;
	  SpiHandler.init.CLKPhase=spi_clock_cpha_first_trans;
	  SpiHandler.init.NSS=spi_ssm_enabled;
 	  SpiHandler.init.mode=spi_master_config;
	 
	  SpiHandler.State=HAL_SPI_STATE_READY;

		hal_spi_init(&SpiHandler);
		
		NVIC_EnableIRQ(SPI1_IRQn);
		
	  while(testRead != SET)
    { 
			led_toggle(GPIOB,BLUE);
			//LED3 (orange)

	
			
    }
	
	  led_turn_off(GPIOB,BLUE); 

		
		
		
		while(1)
		{
			while(SpiHandler.State != HAL_SPI_STATE_READY);
			while((SPI1->SR&(1<<7)));
		 Master write command 
			AdrrCMD[0]=(uint8_t)   CMD_MASTER_WRITE;
			AdrrCMD[1]=(uint8_t)  (CMD_MASTER_WRITE>>8);
	//		while(!(SpiHandler.Instance->SR & 0X01));

			SPI1->DR=AdrrCMD[0];

			while((SPI1->SR&(1<<7)));
		  uint16_t read=0;
			read=SPI1->DR;

			while((SPI1->SR&(1<<7)));
		
		 if( !(SPI1->CR1 & spi_enable_reg) )
		    SPI1->CR1 |= spi_enable_reg;
	  
		 hal_spi_master_tx(&SpiHandler,AdrrCMD,1);
		 while(!(SPI1->SR&(1<<1)));
		 while(SpiHandler.State != HAL_SPI_STATE_READY );
			delay_gen();	
	
			//Read ack byte
		//	while(!(SpiHandler.Instance->SR & 0X01));
		  hal_spi_master_rx(&SpiHandler,ack_buf,1);
		 
			while(SpiHandler.State != HAL_SPI_STATE_READY);
		 	while(!(SPI1->SR|(~(1<<0))));
			if(ack_buf[1] == 0XE5 && ack_buf[0] == 0xD5)
			{
				led_turn_on(GPIOB,RED);
			  memset(ack_buf,0,2);
				
			}
			else
			{
				assert_error();
				memset(ack_buf,0,2);
	
			}
			hal_spi_master_rx(&SpiHandler,master_read_buffer,DATA_LENGTH);
			
			while(SpiHandler.State != HAL_SPI_STATE_READY);
			led_turn_off(GPIOB,RED);
			if(BufferComp(master_read_buffer,slave_reply_data,DATA_LENGTH))
			{
				led_toggle(GPIOB,BLUE);//YANLIS
				  delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
        	delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
						delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
						delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
						delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
			}
			else
			{
			
				led_toggle(GPIOB,YELLOW);//DOGRU
				  delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
				
						delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
						delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
						delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
						delay_gen();
					delay_gen();
					delay_gen();
					delay_gen();
			}
			
			delay_gen();
	  while(SpiHandler.State != HAL_SPI_STATE_READY);
		}
		
		
		return 0;



}
	


		*/




void SPI1_IRQHandler(void)
{
	/* call the driver api to process this interrupt */
  hal_spi_i2c_irq_handler(&SpiHandler);
				if(ack_buf[0] == 0XD5 )
			{
			  led_turn_on(GPIOB,RED);
			}
			if(ack_buf[0] != 0)
			{
			  led_turn_on(GPIOB,BLUE);
			}
}



void EXTI0_IRQHandler(void)
{

	//Clear the interrupt 
	
	
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	testRead = SET;
	
	
}