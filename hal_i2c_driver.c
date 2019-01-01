#include <stdint.h>
#include "hal_i2c_driver.h"
#include "hal_gpio_driver.h"


void hal_i2c_init(i2c_handle_t *handle){

	/* I2C Clock Inits */
	hal_i2c_set_clk(handle->I2Cx,handle->init.ClockSpeed,handle->init.DUTY);
	
	/* Sers I2C addressing mode */
  hal_i2c_set_addressing_mode(handle->I2Cx,handle->init.ADDMODE);
	
	/*Enable the ACKing*/
	hal_i2c_manage_ack(handle->I2Cx,handle->init.AckEnable);
	
	/*Enable the clock stretching */ 
	hal_i2c_manage_clock_streching(handle->I2Cx,handle->init.stretch);

	/*Configure the own address */
	hal_i2c_set_ownadress1(handle->I2Cx,handle->init.OwnAdd1);
	
	/* Finally, enable the IC2 perhipheral */ 
	hal_i2c_enable(handle->I2Cx);

	
	hal_i2c_disable(handle->I2Cx);
	
	gpio_pin_config_t sda,scl;
	
	//6 ,9 
	
		 // _HAL_RCC_GPIOB_CLOCK_ENABLE();
			
	sda.pin=GPIO_PIN_7;
	sda.mode=GPIO_PIN_OUTPUT_MODE_SPEED_LOW;
	sda.output_type=GPIO_PIN_OUTPUT_PUSH_PULL;
	 
	hal_gpio_init(GPIOB,&sda);
	
	scl.pin=GPIO_PIN_6;
	scl.mode=GPIO_PIN_OUTPUT_MODE_SPEED_LOW;
	scl.output_type=GPIO_PIN_OUTPUT_PUSH_PULL;
	 
	hal_gpio_init(GPIOB,&scl);
	hal_gpio_write_to_pin(GPIOB,scl.pin,1);
	hal_gpio_write_to_pin(GPIOB,sda.pin,1);	
	

	
	hal_gpio_write_to_pin(GPIOB,sda.pin,0);			


	hal_gpio_write_to_pin(GPIOB,scl.pin,0);			


	hal_gpio_write_to_pin(GPIOB,scl.pin,1);
	hal_gpio_write_to_pin(GPIOB,sda.pin,1);	

	handle->I2Cx->CR1 |= I2C_CR1_SWRST;
	handle->I2Cx->CR1 &= ~I2C_CR1_SWRST;




	scl.mode= GPIO_PIN_OUTPUT_MODE_SPEED_HIGH;
	scl.output_type=GPIO_PIN_ALT_OPEN_DRAIN;
	hal_gpio_altarnete_conf(ALT_PORTB,scl.pin,ALT_I2C1_REG,scl.mode);
	hal_gpio_init(GPIOB,&scl);


	sda.mode= GPIO_PIN_OUTPUT_MODE_SPEED_HIGH;
	sda.output_type=GPIO_PIN_ALT_OPEN_DRAIN;
	hal_gpio_altarnete_conf(ALT_PORTB,sda.pin,ALT_I2C1_REG,sda.mode);
	hal_gpio_init(GPIOB,&sda);
	/* I2C Clock Inits */
	hal_i2c_set_clk(handle->I2Cx,handle->init.ClockSpeed,handle->init.DUTY);
	
	/* Sers I2C addressing mode */
  hal_i2c_set_addressing_mode(handle->I2Cx,handle->init.ADDMODE);
	
	/*Enable the ACKing*/
	hal_i2c_manage_ack(handle->I2Cx,handle->init.AckEnable);
	
	/*Enable the clock stretching */ 
	hal_i2c_manage_clock_streching(handle->I2Cx,handle->init.stretch);

	/*Configure the own address */
	hal_i2c_set_ownadress1(handle->I2Cx,handle->init.OwnAdd1);
  hal_i2c_enable(handle->I2Cx);

}



void hal_i2c_enable(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |=  I2C_REG_CR1_ENABLE;
   
}

void hal_i2c_disable(I2C_TypeDef *i2cx)
{
	i2cx->CR1 &= ~I2C_REG_CR1_ENABLE;

}


void hal_i2c_manage_clock_streching(I2C_TypeDef *i2cx,uint32_t nostrech)
{
	if(nostrech)
		i2cx->CR1 |=  I2C_REG_CR1_NOSTRETCH;
	else
		i2cx->CR1 &= ~I2C_REG_CR1_NOSTRETCH;
}



void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx,uint32_t addressMode)
{
	
	if(addressMode==I2C_Addr_Mode_10)
		i2cx->OAR1 |=  I2C_REG_OAR1_10Addr_Mode;
	else
		i2cx->OAR1 &= ~I2C_REG_OAR1_10Addr_Mode;

	

}
 void hal_i2c_set_ownadress1(I2C_TypeDef *i2cx,uint32_t ownadress)
{
	
	i2cx->OAR1 &= ~(0x7f<<1);
	i2cx->OAR1 |=  (ownadress<<1);
}


void hal_i2c_set_mode(I2C_TypeDef *i2cx,uint32_t i2cMode)
{
	if(i2cMode==I2C_Master_Mode)
		i2cx->SR2 |=  I2C_REG_SR2_MASTER_MODE;
	else 
		i2cx->SR2 &= ~I2C_REG_SR2_MASTER_MODE;

}

void hal_i2c_set_FMdutyCycle(I2C_TypeDef *i2cx,uint32_t duty)
{
	if(duty==I2C_FM_DUTY_16_9)
		i2cx->CCR |=  I2C_REG_CCR_FM_DUTY_16_9;
	else
		i2cx->CCR &= ~I2C_REG_CCR_FM_DUTY_16_9;

}


 void hal_i2c_set_clk(I2C_TypeDef *i2cx,uint32_t clkSpeed ,uint32_t dutyCycle)
{
	i2cx->CR2 &= ~(0X1F << 0);
	uint32_t pclk;
	pclk=I2C_PERIPHERAL_CLK_FREQ_10MHZ;
  i2cx->CR2 |= (pclk << 0);
	
	hal_i2c_set_ccr(i2cx,pclk ,clkSpeed ,dutyCycle);
	hal_i2c_set_rise_time(i2cx,clkSpeed ,pclk);
}


void hal_i2c_set_ccr(I2C_TypeDef *i2cx,uint32_t pclk ,uint32_t clkSpeed ,uint32_t dutyCycle)
{
	uint32_t ccr;
	
	if(clkSpeed<=10000)
	{
		ccr=(pclk*10000)/(clkSpeed<<1);
	}
	else
	{
		if(dutyCycle==I2C_FM_DUTY_2)
			ccr=(pclk*10000)/(clkSpeed*3);
		else
			ccr=(pclk*10000)/(clkSpeed*25);   /* this is to reach 400khz in fm mode */
	}
	
	
	i2cx->CCR |=ccr;
}


void hal_i2c_set_rise_time(I2C_TypeDef *i2cx,uint32_t clkSpeed ,uint32_t freqrange)
{

	uint32_t Trise;
	
	if(clkSpeed<=10000)
		Trise=freqrange+1;
	else
		Trise = (((freqrange * 300) / 1000) + 1);
	
	i2cx->TRISE  &= ~(0X3F);
	i2cx->TRISE |= Trise;
	
}

void hal_i2c_generate_start_con(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_START_GEN;

}

void hal_i2c_generate_stop_con(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_STOP_GEN;

}

void hal_i2c_conf_buffer_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if(enable)
		i2cx->CR2 |=  I2C_REG_CR2_Buffer_Int_Enable;
	else
		i2cx->CR2 |= ~I2C_REG_CR2_Buffer_Int_Enable;
}


void hal_i2c_conf_event_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if(enable)
		i2cx->CR2 |=  I2C_REG_CR2_Event_Int_Enable;
	else
		i2cx->CR2 |= ~I2C_REG_CR2_Event_Int_Enable;
}


void hal_i2c_conf_err_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if(enable)
		i2cx->CR2 |=  I2C_REG_CR2_Error_Int_Enable;
	else
		i2cx->CR2 |= ~I2C_REG_CR2_Error_Int_Enable;
	
}


uint8_t hal_i2c_is_busy(I2C_TypeDef *i2cx)
{
	if (i2cx->SR2 & I2C_REG_SR2_BUS_BUSY_FLAG)
		return 1;
	else
		return 0;
}

void hal_i2c_wait_until_sb_set(I2C_TypeDef *i2cx)
{

  uint32_t x;
  while(!(i2cx->SR1 & I2C_REG_SR1_START_BIT_FLAG));
  x=i2cx->SR1;
}

void hal_i2c_wait_addr_set(I2C_TypeDef *i2cx)
{

  while(!(i2cx->SR1 & I2C_REG_SR1_ADDR_SENT_FLAG));

}




/******* TX-RX ***********/

void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slaveAdrr,uint8_t *buffer, uint32_t len)
{
	// populate handle with len and buffer information 
	handle->pBufferTx=buffer;
	handle->TrSize=len;
	handle->XferCount=len; //burasi biraz sa�ma olmus gibi gibi 
	handle->state=HAL_I2C_STATE_BUSY_TX;
	
	// make sure I2Cx is enabled
	hal_i2c_enable(handle->I2Cx);

	// Generate the start condition
	handle->I2Cx->CR1 |= I2C_CR1_START;

	// wait until the sb is set
	hal_i2c_wait_until_sb_set(handle->I2Cx);
	
	// send 8 bit address first 
	hal_i2c_send_addr_first(handle->I2Cx,slaveAdrr);
	
	//wait until addr reg is set 
	hal_i2c_wait_addr_set(handle->I2Cx);
	
	//clear addr regs reading SR1 and SR2
	hal_i2c_clear_addr(handle->I2Cx);
	
	hal_i2c_conf_buffer_interrupt(handle->I2Cx,I2C_Buffer_Int_Enable);
  hal_i2c_conf_event_interrupt(handle->I2Cx,I2C_Event_Int_Enable);
	hal_i2c_conf_err_interrupt(handle->I2Cx,I2C_Error_Int_Enable);

}

void hal_i2c_send_addr_first(I2C_TypeDef *i2cx,uint8_t slaveAdrr)
{
	i2cx->DR = slaveAdrr;
}


void hal_i2c_clear_addr(I2C_TypeDef *i2cx)
{
	uint32_t sr;
	
 	sr=i2cx->SR1;
  sr=i2cx->SR2;	
}



void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slaveAdrr,uint8_t *buffer, uint32_t len)
{
	// populate handle with len and buffer information 
	handle->pBufferTx=buffer;
	handle->TrSize=len;
	handle->XferCount=len; //burasi biraz sa�ma olmus gibi gibi 
	handle->state=HAL_I2C_STATE_BUSY_RX;
	
	// make sure I2Cx is enabled
	hal_i2c_enable(handle->I2Cx);
	
	//make sure pos is 0
	handle->I2Cx->CR1 &= ~I2C_REG_CR1_POS_Data;
	
	//make sure acking is enabled
	handle->I2Cx->CR1 |=  I2C_REG_CR1_ACK_ENABLE;

	// Generate the start condition
	hal_i2c_generate_start_con(handle->I2Cx);
	
	// wait until the sb is set
	hal_i2c_wait_until_sb_set(handle->I2Cx);
	
	// send 8 bit address first 
	hal_i2c_send_addr_first(handle->I2Cx,slaveAdrr);
	
	//wait until addr reg is set 
	hal_i2c_wait_addr_set(handle->I2Cx);
	
	//clear addr regs reading SR1 and SR2
	hal_i2c_clear_addr(handle->I2Cx);
	
	hal_i2c_conf_buffer_interrupt(handle->I2Cx,I2C_Buffer_Int_Enable);
  hal_i2c_conf_event_interrupt(handle->I2Cx,I2C_Event_Int_Enable);
	hal_i2c_conf_err_interrupt(handle->I2Cx,I2C_Error_Int_Enable);

}

void hal_i2c_slave_tx(i2c_handle_t *handle,uint8_t *buffer, uint32_t len)
{
	// populate handle with len and buffer information 
	handle->pBufferTx=buffer;
	handle->TrSize=len;
	handle->XferCount=len; //burasi biraz sa�ma olmus gibi gibi 
	handle->state=HAL_I2C_STATE_BUSY_TX;
	
	//make sure pos is 0
	handle->I2Cx->CR1 &= ~I2C_REG_CR1_POS_Data;
	
	
	// make sure I2Cx is enabled
	hal_i2c_enable(handle->I2Cx);


	//make sure acking is enabled
	handle->I2Cx->CR1 |=  I2C_REG_CR1_ACK_ENABLE;

	
	hal_i2c_conf_buffer_interrupt(handle->I2Cx,I2C_Buffer_Int_Enable);
  hal_i2c_conf_event_interrupt(handle->I2Cx,I2C_Event_Int_Enable);
	hal_i2c_conf_err_interrupt(handle->I2Cx,I2C_Error_Int_Enable);

}

void hal_i2c_slave_rx(i2c_handle_t *handle,uint8_t *buffer, uint32_t len)
{
	// populate handle with len and buffer information 
	handle->pBufferTx=buffer;
	handle->TrSize=len;
	handle->XferCount=len; //burasi biraz sa�ma olmus gibi gibi 
	handle->state=HAL_I2C_STATE_BUSY_RX;
	
	//make sure pos is 0
	handle->I2Cx->CR1 &= ~I2C_REG_CR1_POS_Data;
	
	
	// make sure I2Cx is enabled
	hal_i2c_enable(handle->I2Cx);


	//make sure acking is enabled
	handle->I2Cx->CR1 |=  I2C_REG_CR1_ACK_ENABLE;

	
	hal_i2c_conf_buffer_interrupt(handle->I2Cx,I2C_Buffer_Int_Enable);
  hal_i2c_conf_event_interrupt(handle->I2Cx,I2C_Event_Int_Enable);
	hal_i2c_conf_err_interrupt(handle->I2Cx,I2C_Error_Int_Enable);

}


void HAL_I2C_EVENT_IRQnHandler(i2c_handle_t *handle)
{
	uint32_t mode_slave=0,mode_master=1,rxInt=0,txInt=0,rxE=0,txE=0,BTF=0;
	
	mode_master=handle->I2Cx->SR2 & I2C_REG_SR2_MASTER_MODE;
	mode_slave=handle->I2Cx->SR2 & (~I2C_REG_SR2_MASTER_MODE);
	rxInt=handle->I2Cx->SR1 & I2C_REG_SR1_RX_NOTempty_FLAG;
	txInt=handle->I2Cx->SR1 & I2C_REG_SR1_TX_empty_FLAG;
	txE=handle->I2Cx->SR1 & I2C_SR1_TXE;
	rxE=handle->I2Cx->SR1 & I2C_SR1_RXNE;
	BTF=handle->I2Cx->SR1 & I2C_REG_SR1_BTF_FLAG;
	
	if(mode_master)
	{
			if(txInt)
		{
			if(BTF)
				hal_i2c_master_handle_tx_btf_int(handle);
			else
				hal_i2c_master_handle_txe_int(handle);
		
		}

		if(rxInt)
		{
			if(BTF)
				hal_i2c_master_handle_rx_btf_int(handle);
			else
			  hal_i2c_master_handle_rxe_int(handle);
		}
	
		
		
	}
	if(mode_slave)
	{
		uint32_t addFlag=0,evenInt=0,stopFlag=0,txFlag=0,buffer=0;
		
		addFlag = handle->I2Cx->SR1 & I2C_REG_SR1_ADDR_SENT_FLAG;
    evenInt = handle->I2Cx->CR2 & I2C_REG_CR2_Event_Int_Enable;
		buffer= handle->I2Cx->CR2 & I2C_REG_CR2_Buffer_Int_Enable;
		stopFlag =handle->I2Cx->SR1 & I2C_REG_SR1_Stop_detection_FLAG;
		txFlag =handle->I2Cx->SR2 & I2C_SR2_TRA;
	
		//if address matched
	  if(addFlag && evenInt)
			hal_i2c_clear_addr(handle->I2Cx);
		else if(evenInt && stopFlag)
			hal_i2c_slave_stop_condition(handle);
		
	
		if( txFlag )
		{
				if( (txE && BTF) && buffer )
  				hal_i2c_slave_handle_txe_int(handle);					//transmission did not finished
        else if (BTF && evenInt)
		    	hal_i2c_slave_handle_tx_btf_int(handle);		
		}	
		else
		{
				if( (rxE && (!BTF)) && buffer )
					hal_i2c_slave_handle_rxe_int(handle);	  			//recieving did not finished
				else if (BTF && evenInt)
					hal_i2c_slave_handle_rx_btf_int(handle);			//recieving finished

		}
		
		
		

		

		
	}


}

void hal_i2c_master_handle_txe_int(i2c_handle_t *handle)
{
	handle->I2Cx->DR |= *handle->pBufferTx++;
	handle->XferCount--;
	
	if(handle->XferCount==0)
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Buffer_Int_Enable;

}

void hal_i2c_master_handle_tx_btf_int(i2c_handle_t *handle)
{
	if(handle->XferCount !=0)
	{
		handle->I2Cx->DR |= *handle->pBufferTx++;
		handle->XferCount--;
	}
	else
	{		
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Buffer_Int_Enable;
	 	handle->I2Cx->CR2 &= ~I2C_REG_CR2_Error_Int_Enable;
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Event_Int_Enable;
	
		handle->I2Cx->CR1 |= I2C_REG_CR1_STOP_GEN;
		
		handle->state = HAL_I2C_STATE_READY;
		
		hal_i2c_master_tx_complete(handle);
	}


}


void	hal_i2c_master_tx_complete(i2c_handle_t *handle)
{



}

void hal_i2c_master_handle_rxe_int(i2c_handle_t *handle)
{
	*handle->pBufferTx = handle->I2Cx->DR;  //burasi yanlis olabilir
	handle->XferCount--;
	
	if(handle->XferCount==0)
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Buffer_Int_Enable;

}

void hal_i2c_master_handle_rx_btf_int(i2c_handle_t *handle)
{
	if(handle->XferCount !=0)
	{
		*handle->pBufferTx = handle->I2Cx->DR;
		handle->XferCount--;
	}
	else
	{		
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Buffer_Int_Enable;
	 	handle->I2Cx->CR2 &= ~I2C_REG_CR2_Error_Int_Enable;
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Event_Int_Enable;
	
		handle->I2Cx->CR1 |= I2C_REG_CR1_STOP_GEN;
		
		handle->state = HAL_I2C_STATE_READY;
		
		hal_i2c_master_rx_complete(handle);
	}


}

void hal_i2c_master_rx_complete(i2c_handle_t *handle)
{



}

void hal_i2c_slave_handle_txe_int(i2c_handle_t *handle)
{
	if(handle->XferCount !=0)
	{
		handle->I2Cx->DR |= *handle->pBufferTx++;
		handle->XferCount--;
	}

}
void hal_i2c_slave_handle_tx_btf_int(i2c_handle_t *handle)
{
	if(handle->XferCount !=0)
	{
		handle->I2Cx->DR |= *handle->pBufferTx++;
		handle->XferCount--;
	}


}

void hal_i2c_slave_handle_rxe_int(i2c_handle_t *handle)
{
		if(handle->XferCount!=0){
			(*handle->pBufferTx++)= handle->I2Cx->DR;  //burasi yanlis olabilir
			handle->XferCount--;
	  }


}

void hal_i2c_slave_handle_rx_btf_int(i2c_handle_t *handle)
{
		if(handle->XferCount!=0){
			(*handle->pBufferTx++)= handle->I2Cx->DR;  //burasi yanlis olabilir
			handle->XferCount--;
	  }

}


void hal_i2c_slave_stop_condition(i2c_handle_t *handle)
{
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Buffer_Int_Enable;
	 	handle->I2Cx->CR2 &= ~I2C_REG_CR2_Error_Int_Enable;
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Event_Int_Enable;
	
		hal_i2c_clear_stop_flag(handle);
		handle->I2Cx->CR1 |= ~I2C_REG_CR1_ACK_ENABLE;
		
		handle->state = HAL_I2C_STATE_READY;
		
		hal_i2c_slave_tx_complete(handle);

}

void	hal_i2c_clear_stop_flag(i2c_handle_t *handle)
{
   handle->I2Cx->SR1 &= ~I2C_REG_SR1_Stop_detection_FLAG;

}
void hal_i2c_slave_tx_complete(i2c_handle_t *handle)
{
  

}


void HAL_I2C_ERR_IRQnHandler(i2c_handle_t *handle)
{

		uint32_t bus=0,errInt=0,arlo,ackFail,ovr,count,state;
		
			bus    = handle->I2Cx->SR1 &I2C_REG_SR1_Bus_Error_FLAG ;
			errInt = handle->I2Cx->CR2 & I2C_REG_CR2_Error_Int_Enable;
      
			if(bus && errInt)
			{	
				handle->ErrorCode |=  I2C_REG_SR1_Bus_Error_FLAG;
        handle->I2Cx->SR1 &= ~I2C_REG_SR1_Bus_Error_FLAG ;
			}

			errInt = handle->I2Cx->CR2 & I2C_REG_CR2_Error_Int_Enable;
			arlo   = handle->I2Cx->SR1 & I2C_REG_SR1_ARLO_FLAG;
			
			if(arlo && errInt)
			{	
				handle->ErrorCode |=  I2C_REG_SR1_ARLO_FLAG;
        handle->I2Cx->SR1 &= ~I2C_REG_SR1_ARLO_FLAG ;
			}			

			errInt = handle->I2Cx->CR2 & I2C_REG_CR2_Error_Int_Enable;
			ackFail   = handle->I2Cx->SR1 & I2C_REG_SR1_ACK_FAIL_FLAG;			
			
			if(ackFail && errInt)
			{	
				errInt = handle->I2Cx->SR2 & I2C_REG_SR2_MASTER_MODE;
				count  = handle->XferCount;
				state  = handle->state;
				
				if( ((!errInt) && count) && (state == HAL_I2C_STATE_BUSY))
				{
					hal_i2c_slave_ackFail_handler(handle);
				
				}
				else
				{
					handle->ErrorCode |=  I2C_REG_SR1_ACK_FAIL_FLAG;
					handle->I2Cx->SR1 &= ~I2C_REG_SR1_ACK_FAIL_FLAG;
				}
				

			}	
			
			errInt = handle->I2Cx->CR2 & I2C_REG_CR2_Error_Int_Enable;
			ovr   = handle->I2Cx->SR1 & I2C_REG_SR1_OVR_FLAG;			
			
			if(ovr && errInt)
			{	
				handle->ErrorCode |=  I2C_REG_SR1_OVR_FLAG;
        handle->I2Cx->SR1 &= ~I2C_REG_SR1_OVR_FLAG ;
			}	


}

void hal_i2c_slave_ackFail_handler(i2c_handle_t *handle)
{
	  handle->I2Cx->CR2 &= ~I2C_REG_CR2_Buffer_Int_Enable;
	 	handle->I2Cx->CR2 &= ~I2C_REG_CR2_Error_Int_Enable;
		handle->I2Cx->CR2 &= ~I2C_REG_CR2_Event_Int_Enable;
	
    handle->I2Cx->SR1 &= ~I2C_REG_SR1_ACK_FAIL_FLAG;
	
		handle->I2Cx->CR1 |= ~I2C_REG_CR1_ACK_ENABLE;
		
		handle->state = HAL_I2C_STATE_READY;
		
		hal_i2c_slave_tx_complete(handle);

}



void hal_i2c_manage_ack(I2C_TypeDef *I2Cx,uint32_t noAck)
{

	if(noAck == I2C_REG_CR1_ACK_ENABLE)
		I2Cx->CR1 |= I2C_REG_CR1_ACK_ENABLE;
	else
		I2Cx->CR1 &= ~I2C_REG_CR1_ACK_ENABLE;



}



