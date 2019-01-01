#include <stdint.h>
#include "hal_spi_driver.h"



void hal_spi_init (spi_handler_typedef *spiHandle){

    spiHandle->init.BoundRatePreScaler  |= spiHandle->init.BoundRatePreScaler;
	  

	  hal_spi_cnfg_phase_pol(spiHandle->Instance,spiHandle->init.CLKPhase,spiHandle->init.CLKPol);
	  hal_spi_cnfg_datasize_direction(spiHandle->Instance,spiHandle->init.DataSize,spiHandle->init.FirstBit);
    hal_spi_cnfg_device_mode(spiHandle->Instance,spiHandle->init.mode);	  
    hal_spi_cnfg_boundrate(spiHandle->Instance,spiHandle->init.BoundRatePreScaler);
	  if( spiHandle->Instance->CR1 & spi_master_reg_config)
			hal_spi_cnfg_NSS_master(spiHandle->Instance,spi_ssm_disabled);
		else
			hal_spi_cnfg_NSS_slave(spiHandle->Instance,spi_ssm_enabled);
		
		spiHandle->Instance->CR1 |=spiHandle->init.direction;
}
//
void hal_spi_cnfg_boundrate(SPI_TypeDef *SPIx,uint32_t boundrate){
           
	     if(boundrate > 7 )
            SPIx->CR1 |= (0x00 << 3); //if pre_scalar_value > 7,then use default . that is 0
        else
            SPIx->CR1 |= (boundrate << 3);
 
        

}
void hal_spi_cnfg_device_mode(SPI_TypeDef *SPIx,uint32_t master){

    if(master)
			 SPIx->CR1 |=  spi_master_reg_config;

    else
			 SPIx->CR1 &= ~spi_master_reg_config;


}
//


void hal_spi_cnfg_phase_pol(SPI_TypeDef *SPIx,uint32_t phase, uint32_t polarity){

    if(phase)
		   SPIx->CR1 |=  spi_clock_cpha_reg;
		else 
			 SPIx->CR1 &= ~spi_clock_cpha_reg;
     
    
		if(polarity)
			 SPIx->CR1 |=  spi_clock_pol_reg;
		else 
			 SPIx->CR1 &= ~spi_clock_pol_reg;
		
}
//


void hal_spi_cnfg_datasize_direction(SPI_TypeDef *SPIx,uint32_t datasize,uint32_t msb_lsb){
    
	  if(datasize)
			SPIx->CR1  |=  spi_dff;
    else
			SPIx->CR1  &= ~spi_dff;

		
		if(msb_lsb)
			SPIx->CR1  |=  spi_lsbfirst;
		else
			SPIx->CR1  &= ~spi_lsbfirst;
		
		

}
//


void hal_spi_cnfg_NSS_master(SPI_TypeDef *SPIx,uint32_t ssm_enable){

  if(ssm_enable)
	{	
   	SPIx->CR1  |=  spi_ssm_reg;
	  SPIx->CR1  |=  spi_internal_slave_select;
	}
	else
		SPIx->CR1  &= ~spi_ssm_reg;


}
//


void hal_spi_enable(SPI_TypeDef *SPIx){

    	if( !(SPIx->CR1 & spi_enable_reg) )
		    SPIx->CR1 |= spi_enable_reg;
        SPIx->CR1 |= spi_master_reg_config;

}
void hal_spi_disable(SPI_TypeDef *SPIx){

  SPIx->CR1 &= ~spi_enable_reg;


}

void hal_spi_cnfg_NSS_slave(SPI_TypeDef *SPIx,uint32_t ssm_enable){

  if(ssm_enable)
	  SPIx->CR1  |=  spi_ssm_reg;
  else
		SPIx->CR1  &= ~spi_ssm_reg;

}

void hal_spi_write_to_transmit(spi_handler_typedef *spiHandle,uint16_t data){
     
    spiHandle->Instance->DR &= ~data;
 
}

void hal_spi_master_tx(spi_handler_typedef *spiHandle,uint8_t *buffer,uint32_t len){
  
	 spiHandle->pTXBuffPtr=buffer;
	 spiHandle->TransferSize=len;
	 spiHandle->TransferCount=len;
	
	 spiHandle->State =HAL_SPI_STATE_BUSY_TX;
	
 
	 hal_spi_enable_interrpt_txe(spiHandle->Instance);
   hal_spi_enable(spiHandle->Instance);


}
void hal_spi_enable_interrpt_txe(SPI_TypeDef *SPIx){

    if(!(SPIx->CR2 & spi_tx_buff_emty_int_en_reg))
      SPIx->CR2 |=spi_tx_buff_emty_int_en_reg;
}

void hal_spi_disable_interrpt_txe(SPI_TypeDef *SPIx){

    
      SPIx->CR2 &= ~spi_tx_buff_emty_int_en_reg;
}
void hal_spi_master_rx(spi_handler_typedef *spiHandle,uint8_t *rcv_buffer,uint32_t len){
   
	uint32_t i=0,val;
	
	 // dummy tx to pop rx 
	 spiHandle->pTXBuffPtr=rcv_buffer;
	 spiHandle->TransferSize=len;
	 spiHandle->TransferCount=len;
	
	
	
	 spiHandle->pRXBuffPtr=rcv_buffer;
   spiHandle->ReceiveSize =len;
	 spiHandle->ReceiveCount=len;
	
	 spiHandle->State=HAL_SPI_STATE_BUSY_RX;
	 
	 val=spiHandle->Instance->DR; // read buffer once to make sure the DR is empty
	 

	 hal_spi_enable_interrpt_rxe(spiHandle->Instance);
   hal_spi_enable_interrpt_txe(spiHandle->Instance);
	// hal_spi_enable(spiHandle->Instance);
}

void hal_spi_enable_interrpt_rxe(SPI_TypeDef *SPIx){

   if(!(SPIx->CR2 & spi_recieve_buff_empty))
     SPIx->CR2 |=spi_rx_buff_not_emty_int_en_reg;
}

void hal_spi_disable_interrpt_rxe(SPI_TypeDef *SPIx){
    
     SPIx->CR2 &= ~spi_rx_buff_not_emty_int_en_reg;
}
uint8_t dummu_rx[2]={0};
void hal_spi_slave_tx(spi_handler_typedef *spiHandle,uint8_t *buffer,uint32_t len){

   
   spiHandle->pTXBuffPtr=dummu_rx;
	 spiHandle->TransferSize=len;
	 spiHandle->TransferCount=len;
	
	
	
	 spiHandle->pRXBuffPtr=buffer;
   spiHandle->ReceiveSize =len;
	 spiHandle->ReceiveCount=len;
	
	 spiHandle->State=HAL_SPI_STATE_BUSY_TX;
	 
	

	 hal_spi_enable_interrpt_rxe(spiHandle->Instance);
   hal_spi_enable_interrpt_txe(spiHandle->Instance);
	 hal_spi_enable(spiHandle->Instance);

}

uint8_t dummu_tx[2]={0};
void hal_spi_slave_rx(spi_handler_typedef *spiHandle,uint8_t *rcv_buffer,uint32_t len){
   
   
   spiHandle->pTXBuffPtr=dummu_tx;
	 spiHandle->TransferSize=len;
	 spiHandle->TransferCount=len;
	
	
	 spiHandle->pRXBuffPtr=rcv_buffer;
   spiHandle->ReceiveSize =len;
	 spiHandle->ReceiveCount=len;
	
	 spiHandle->State=HAL_SPI_STATE_BUSY_RX;
	
	

	 hal_spi_enable_interrpt_rxe(spiHandle->Instance);
	 hal_spi_enable_interrpt_txe(spiHandle->Instance);
	 hal_spi_enable(spiHandle->Instance);
}

void hal_spi_i2c_irq_handler(spi_handler_typedef *spiHandle){
  
	uint32_t temp1=0, temp2=0;
	
	
	temp1 = (spiHandle->Instance->SR & spi_recieve_buff_empty);
  temp2 = (spiHandle->Instance->CR2 & spi_rx_buff_not_emty_int_en_reg);

 if((temp1 != RESET) && (temp2 != RESET) )
  {
	  hal_spi_rx_interrupt_handler(spiHandle);
	}
	uint32_t temp3=0, temp4=0;
	
	temp3 = (spiHandle->Instance->SR & spi_transmit_buff_empty);
  temp4 = (spiHandle->Instance->CR2 & spi_tx_buff_emty_int_en_reg);
	
		if(!(temp3==RESET) && !(temp4==RESET))
	{
	  hal_spi_tx_interrupt_handler(spiHandle);
	  return;
	}
	
}


void hal_spi_rx_interrupt_handler(spi_handler_typedef *spiHandle){
   
	     if(spiHandle->init.DataSize== spi_8bit_dff)
		  {
				while(spiHandle->TransferCount)
				{
					spiHandle->Instance->DR =(*spiHandle->pTXBuffPtr++);
				  spiHandle->TransferCount--;
				}
		  } 
		 else
		  {
	      spiHandle->Instance->DR = *((uint16_t*)spiHandle->pTXBuffPtr++);
				
				spiHandle->pTXBuffPtr+=2;
				spiHandle->TransferCount-=2;  // we send 2 bytes in one go 
		 
		  }
	  if( spiHandle->init.DataSize==spi_8bit_dff)
		  {
					while(spiHandle->ReceiveCount)
					{
						if(spiHandle->pRXBuffPtr++)
							(*spiHandle->pRXBuffPtr++)=spiHandle->Instance->DR;
						spiHandle->ReceiveCount--;
					}
	  	}
	  else
		  {	
				*((uint16_t*)spiHandle->pRXBuffPtr)=spiHandle->Instance->DR;
		     spiHandle->pRXBuffPtr+=2;
				 spiHandle->ReceiveCount-=2;
	   	} 
		
		if(spiHandle->ReceiveCount<=0)
			hal_spi_rx_close_intterput(spiHandle);
		hal_spi_tx_close_intterput(spiHandle);
}

void hal_spi_tx_interrupt_handler(spi_handler_typedef *spiHandle){

     if(spiHandle->init.DataSize== spi_8bit_dff)
		  {
				while(spiHandle->TransferCount)
				{
					spiHandle->Instance->DR =0x55;
				  spiHandle->TransferCount--;
				}
		  } 
		 else
		  {
	      spiHandle->Instance->DR = *((uint16_t*)spiHandle->pTXBuffPtr++);
				
				spiHandle->pTXBuffPtr+=2;
				spiHandle->TransferCount-=2;  // we send 2 bytes in one go 
		 
		  }
			
			
			if(spiHandle->TransferCount<=0)
			{
			  hal_spi_tx_close_intterput(spiHandle);
			 
			}

}


void hal_spi_tx_close_intterput(spi_handler_typedef *spiHandle){

      hal_spi_disable_interrpt_txe(spiHandle->Instance);
	    
	    if(spiHandle->init.mode && (spiHandle->State != HAL_SPI_STATE_BUSY))
				spiHandle->State=HAL_SPI_STATE_READY;

}


void hal_spi_rx_close_intterput(spi_handler_typedef *spiHandle){

     while(hal_spi_is_busy(spiHandle->Instance));
	  
	   hal_spi_disable_interrpt_rxe(spiHandle->Instance);
   
	   spiHandle->State=HAL_SPI_STATE_READY;
}


uint8_t hal_spi_is_busy(SPI_TypeDef *SPIx){

     if((SPIx->SR & spi_busy_flag_reg))
			 return SPI_IS_BUSY;
		 else
			 return SPI_IS_NOT_BUSY;

}