#include <stdint.h>
#include "hal_gpio_driver.h"
#include "hal_spi_driver.h"
#include "hal_i2c_driver.h"
#include "i2c_main.h"
#include "stm32f10x.h"
#include <string.h> 

#define SLAVE_OWN_ADDRESS      (uint8_t) 0x52;
#define SLAVE_ADDRESS_READ     (uint8_t) 0xA7
#define SLAVE_ADDRESS_WRITE    (uint8_t) 0xA6


#define GENERAL_CALL_ADDRESS    (uint8_t)0x00

#define MASTER_WRITE_CMD       0xC1
#define MASTER_READ_CMD        0XC2

#define READ_LEN    5
#define WRITE_LEN   5



uint8_t master_tx_buffer[5]={0xa5, 0x55, 0xa5, 0x55, 0xb0};
uint8_t master_rx_buffer[5];

uint8_t slave_tx_buffer[5]={0xa5, 0x55, 0xa5, 0x55, 0xc0};
uint8_t slave_rx_buffer[5];

uint8_t master_write_req;
uint8_t master_read_req;

uint8_t slave_rcv_cmd;

i2c_handle_t  handleI2C;






int main()
{
	uint32_t vall, slave_data_len;
	led_init();
 
	
		
	//CALISMAZSA PUSH PULLU DEGISTIR
	i2c_gpio_init();

	_HAL_RCC_I2C1_CLK_ENABLE();
	handleI2C.state = HAL_I2C_STATE_READY;

	
	handleI2C.I2Cx=I2C1;
	handleI2C.init.AckEnable= I2C_REG_CR1_ACK_ENABLE;
	handleI2C.init.ADDMODE  = I2C_Addr_Mode_7;
	handleI2C.init.ClockSpeed= I2C_PERIPHERAL_CLK_FREQ_10MHZ;
	handleI2C.init.DUTY =I2C_FM_DUTY_2;
	handleI2C.init.master=I2C_Master_Mode;
	handleI2C.init.stretch =I2C_Stretching_Enabled;
	handleI2C.init.generalCall=0;
	handleI2C.init.OwnAdd1=SLAVE_OWN_ADDRESS; 
	
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	
	
	hal_i2c_init(&handleI2C);
	
		  while(hal_gpio_read_to_pin(GPIOB,GPIO_PIN_1) == SET)
    { 
			led_turn_on(GPIOB,GPIO_PIN_8);
			
    }
	  led_turn_off(GPIOB,GPIO_PIN_8);
	
		
		while(1)
		{	
			while(handleI2C.state != HAL_I2C_STATE_READY)
			
			master_write_req=MASTER_WRITE_CMD;
		
			hal_i2c_master_tx(&handleI2C,SLAVE_ADDRESS_WRITE,(uint8_t *) &master_write_req,1);
		
			while(handleI2C.state != HAL_I2C_STATE_READY)
		
			master_write_req=WRITE_LEN;
		
			hal_i2c_master_tx(&handleI2C,SLAVE_ADDRESS_WRITE,(uint8_t *) &master_write_req,1);

		
			while(handleI2C.state != HAL_I2C_STATE_READY)

			hal_i2c_master_tx(&handleI2C,SLAVE_ADDRESS_WRITE,master_tx_buffer,1);

			
			
			while(handleI2C.state != HAL_I2C_STATE_READY)
		
			master_read_req=MASTER_READ_CMD;
		
			hal_i2c_master_rx(&handleI2C,SLAVE_ADDRESS_WRITE,(uint8_t *) &master_read_req,1);
		
	
	    while(handleI2C.state != HAL_I2C_STATE_READY)
		
			master_read_req=READ_LEN;
		
			hal_i2c_master_rx(&handleI2C,SLAVE_ADDRESS_WRITE,(uint8_t *) &master_read_req,1);
		
		
		  while(handleI2C.state != HAL_I2C_STATE_READY)
		
			memset(master_rx_buffer,0, 5);
			hal_i2c_master_rx(&handleI2C,SLAVE_ADDRESS_WRITE,master_rx_buffer,1);
			
			if(Buffercmp(slave_tx_buffer,master_rx_buffer,READ_LEN))
			{
			   led_turn_on(GPIOB,GPIO_PIN_7);
			}
			else{
			
				 led_turn_off(GPIOB,GPIO_PIN_7);
			
			}	
			
			
			
			
			
			
			
			
		}
		
}











void  i2c_gpio_init()
{

	gpio_pin_config_t i2c_scl,i2c_sda;
	
	_HAL_RCC_GPIOB_CLOCK_ENABLE();
	
	i2c_scl.pin= I2C_SCL_LINE;
	i2c_scl.mode= GPIO_PIN_ALT_PUSH_PULL;
	hal_gpio_altarnete_conf(ALT_PORTB,i2c_scl.pin,ALT_I2C1_REG,i2c_scl.mode);
	hal_gpio_init(GPIOB,&i2c_scl);

	i2c_sda.pin= I2C_SDA_LINE;
	i2c_sda.mode= GPIO_PIN_ALT_PUSH_PULL;
	hal_gpio_altarnete_conf(ALT_PORTB,i2c_sda.pin,ALT_I2C1_REG,i2c_sda.mode);
	hal_gpio_init(GPIOB,&i2c_sda);

}



void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ERR_IRQnHandler(& handleI2C);
}



void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EVENT_IRQnHandler(& handleI2C);
}


static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}


