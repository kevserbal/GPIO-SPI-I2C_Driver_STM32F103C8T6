
#ifndef __HAL_I2C_DRIVER_H
#define __HAL_I2C_DRIVER_H


#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include  <stdint.h>


#define _HAL_RCC_I2C1_CLK_ENABLE()       (RCC->APB1ENR|=((uint32_t ) 1<<21))
#define _HAL_RCC_I2C2_CLK_ENABLE()       (RCC->APB1ENR|=((uint32_t ) 1<<22))
/**********  I2C CR1 REGS ******************************/

#define I2C_REG_CR1_SowftwareReset  				((uint32_t ) 1<<15)
#define I2C_UnderReset      								1
#define I2C_NOTUnderReset  									0

#define I2C_REG_CR1_POS_Data 								((uint32_t ) 1<<11)
#define I2C_POS_CurrentData        			    1
#define I2C_POS_NextData           			    0


#define I2C_REG_CR1_ACK_ENABLE    					((uint32_t ) 1<<10)
#define I2C_ACK_RETURNED 										1
#define I2C_ACK_notRETURNED 								1

#define I2C_REG_CR1_STOP_GEN   							((uint32_t ) 1<<9)
#define I2C_REG_CR1_START_GEN  							((uint32_t ) 1<<8)

#define I2C_REG_CR1_NOSTRETCH 						  ((uint32_t ) 1<<7)
#define I2C_Stretching_Enabled   					  0
#define I2C_Stretching_Disabled  						1

#define I2C_REG_CR1_GeneralCall_Enable      ((uint32_t ) 1<<6)
#define I2C_GeneralCall_Enable_ACKed   		 1
#define I2C_GeneralCall_Disable_NACKed		  0

#define I2C_REG_CR1_PEC_ENABLED             ((uint32_t ) 1<<5)
#define I2C_PEC_ENABLED             		    1
#define I2C_PEC_DISABLED            		    0

#define I2C_REG_CR1_ARP_ENABLED             ((uint32_t ) 1<<4)
#define I2C_ARP_ENABLED  										1
#define I2C_ARP_DISABLED  									0


#define I2C_REG_CR1_SMBus_Host    					((uint32_t ) 1<<3)
#define I2C_SMBus_Device            		    0
#define I2C_SMBus_Host              		    1
 
#define I2C_REG_CR1_SMBus_TYPE_HOST    	  	((uint32_t ) 1<<2)
#define I2C_SMBus_TYPE_HOST    		  		    1
#define I2C_SMBus_TYPE_DEVICE     		      0
 
#define I2C_REG_CR1_Mode_SMBus        			((uint32_t ) 1<<1)
#define I2C_Mode_I2C   					    		    0
#define I2C_Mode_SMBus  					 	  	    1

#define I2C_REG_CR1_ENABLE            			((uint32_t ) 1<<0)
#define I2C_DISABLE_Peripheral              0
#define I2C_ENABLE_Peripheral               1



/**********  I2C CR2 REGS ******************************/

#define I2C_REG_CR2_DMA_Last_Transfer        ((uint32_t ) 1<<12)
#define I2C_DMA_Last_Transfer                1
#define I2C_DMA_notLast_Transfer             0

#define I2C_REG_CR2_DMA_REQ_ENABLE           ((uint32_t ) 1<<11)
#define I2C_DMA_REQ_ENABLE                   1
#define I2C_DMA_REQ_DISABLE                  0                     //WHEN TxE=1 or RxNE =1

#define I2C_REG_CR2_Buffer_Int_Enable        ((uint32_t ) 1<<10)
#define I2C_Buffer_Int_Enable                1
#define I2C_Buffer_Int_Disable               0

#define I2C_REG_CR2_Event_Int_Enable         ((uint32_t ) 1<<9)
#define I2C_Event_Int_Enable                 1
#define I2C_Event_Int_Disable                0
/*
Event interrupt is generated when:
– SB = 1 (Master)
– ADDR = 1 (Master/Slave)
– ADD10= 1 (Master)
– STOPF = 1 (Slave)
– BTF = 1 with no TxE or RxNE event
– TxE event to 1 if ITBUFEN = 1
– RxNE event to 1if ITBUFEN = 1
*/


#define I2C_REG_CR2_Error_Int_Enable         ((uint32_t ) 1<<9)
#define I2C_Error_Int_Enable                 1
#define I2C_Error_Int_Disable                0
/*
Error interrupt is generated when:
– BERR = 1
– ARLO = 1
– AF = 1
– OVR = 1
– PECERR = 1
– TIMEOUT = 1
– SMBALERT = 1
*/

#define I2C_PERIPHERAL_CLK_FREQ_2MHZ      ((uint32_t)2 )  
#define I2C_PERIPHERAL_CLK_FREQ_3MHZ      ((uint32_t)3 )  
#define I2C_PERIPHERAL_CLK_FREQ_4MHZ      ((uint32_t)4 )
#define I2C_PERIPHERAL_CLK_FREQ_5MHZ      ((uint32_t)5 ) 
#define I2C_PERIPHERAL_CLK_FREQ_8MHZ      ((uint32_t)8 )
#define I2C_PERIPHERAL_CLK_FREQ_5MHZ      ((uint32_t)5 )  
#define I2C_PERIPHERAL_CLK_FREQ_10MHZ     ((uint32_t)10 )  
#define I2C_PERIPHERAL_CLK_FREQ_20MHZ     ((uint32_t)20 )  
#define I2C_PERIPHERAL_CLK_FREQ_30MHZ     ((uint32_t)30 )  
#define I2C_PERIPHERAL_CLK_FREQ_40MHZ     ((uint32_t)40 ) 
#define I2C_PERIPHERAL_CLK_FREQ_50MHZ     ((uint32_t)50 )  



/******** I2C Own address register 1 (I2C_OAR1) ***************/

#define I2C_REG_OAR1_10Addr_Mode           ((uint32_t ) 1<<15)
#define I2C_Addr_Mode_7                   0
#define I2C_Addr_Mode_10                  1


/********* I2C Own address register 2 (I2C_OAR2) **************/


#define I2C_REG_OAR2_ENDUAL               ((uint32_t ) 1<<0)
#define I2C_ENDUAL_ONLY_OAR1_7Bits        0
#define I2C_ENDUAL_BOTH_OAR1_OAR2_7Bits   1


/******** I2C Status register 1 (I2C_SR1)   ****************/
#define I2C_REG_SR1_TIMEOUT_FLAG           ( (uint32_t) 1 << 14)
#define I2C_REG_SR1_PECERR                 ( (uint32_t) 1 << 13)
#define I2C_REG_SR1_OVR_FLAG               ( (uint32_t) 1 << 11)
#define I2C_REG_SR1_ACK_FAIL_FLAG          ( (uint32_t) 1 << 10)
#define I2C_REG_SR1_ARLO_FLAG              ( (uint32_t) 1 << 9)
#define I2C_REG_SR1_Bus_Error_FLAG         ( (uint32_t) 1 << 8)
#define I2C_REG_SR1_TX_empty_FLAG          ( (uint32_t) 1 << 7)
#define I2C_REG_SR1_RX_NOTempty_FLAG       ( (uint32_t) 1 << 6)
#define I2C_REG_SR1_Stop_detection_FLAG    ( (uint32_t) 1 << 4)
#define I2C_REG_SR1_ADD_SENT_FLAG          ( (uint32_t) 1 << 3)
#define I2C_REG_SR1_BTF_FLAG               ( (uint32_t) 1 << 2)
#define I2C_REG_SR1_ADDR_SENT_FLAG         ( (uint32_t) 1 << 1)
#define I2C_REG_SR1_START_BIT_FLAG         ( (uint32_t) 1 << 0)


/******** I2C Status register 2 (I2C_SR2)   ********************/
#define I2C_REG_SR2_TX_RX_FLAG             ( (uint32_t) 1 << 2)
#define I2C_TX_Mode                        1
#define I2C_RX_Mode                        0


#define I2C_REG_SR2_BUS_BUSY_FLAG          ( (uint32_t) 1 << 1)
#define I2C_BUS_BUSY                       1
#define I2C_BUS_NOT_BUSY                   0



#define I2C_REG_SR2_MASTER_MODE            ( (uint32_t) 1 << 0)
#define I2C_Master_Mode                    1
#define I2C_Slave_Mode                     0

/* Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag, even if the ADDR flag was
set after reading I2C_SR1. Consequently, I2C_SR2 must be read only when ADDR is found
set in I2C_SR1 or when the STOPF bit is cleared.*/

/******** I2C Clock control register (I2C_CCR) *******************************/

#define I2C_REG_CCR_Enable_FM                    ( (uint32_t) 1 << 15)
#define I2C_Enable_FM                            1
#define I2C_Enable_SM                            0

#define I2C_REG_CCR_FM_DUTY_16_9                   ( (uint32_t) 1 << 14)
#define I2C_FM_DUTY_16_9                		        1
#define I2C_FM_DUTY_2                 		          0




/************ DATA STRUCTURES **************************/


typedef enum {
	
	HAL_I2C_STATE_RESET  					  = 0x00,
	HAL_I2C_STATE_READY   					= 0x01,
	HAL_I2C_STATE_BUSY    				  = 0x02,
	HAL_I2C_STATE_BUSY_TX					  = 0x12,
	HAL_I2C_STATE_BUSY_RX 				  = 0x22,
  HAL_I2C_STATE_MEM_BUSY_TX       = 0x32,  /*!< Memory Data Transmission process is ongoing */
  HAL_I2C_STATE_MEM_BUSY_RX       = 0x42,  /*!< Memory Data Reception process is ongoing    */
	HAL_I2C_STATE_TIMEOUT 				  = 0x03,
	HAL_I2C_STATE_OVR      				  = 0x04,
	HAL_I2C_STATE_ERR
	
}hal_i2c_state_t;


typedef struct{
	
	uint32_t  stretch;
	uint32_t  generalCall;
	uint32_t  SMBus_type;
  uint32_t  ADDMODE;
	uint32_t  Interface_address;
	uint32_t  AckEnable;
	uint32_t  DualAdressMode;
	uint32_t  ClockSpeed;
	uint32_t	DUTY;
	uint32_t  OwnAdd1;
	uint32_t  OwnAdd2;
	uint8_t   master;
	
}i2c_init_t;


typedef struct{

	I2C_TypeDef       *I2Cx;
	  
	i2c_init_t        init;
	
	uint8_t           *pBufferTx;
	
	uint32_t          TrSize;
	
	__IO uint32_t     XferCount;

	hal_i2c_state_t   state;
	
	uint8_t           ErrorCode;


}i2c_handle_t;


#define  RESET  0 
#define  SET  !RESET


typedef enum{
	
	HAL_OK 					      = 0x00,
	HAL_ERROR  				  	= 0x01,
	HAL_BUSY    				  = 0x02,
	HAL_TIMEOUT					  = 0x03

}HAL_Status_t;


#define UNUSED(x) ((void)(x))




/*********** APIs *******************************/

void hal_i2c_init(i2c_handle_t *handle);

void hal_i2c_manage_ack(I2C_TypeDef *I2Cx,uint32_t noAck);

void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slaveAdrr,uint8_t *buffer, uint32_t len);
void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slaveAdrr,uint8_t *buffer, uint32_t len);



void hal_i2c_slave_tx(i2c_handle_t *handle,uint8_t *buffer, uint32_t len);
void hal_i2c_slave_rx(i2c_handle_t *handle,uint8_t *buffer, uint32_t len);

void HAL_I2C_EVENT_IRQnHandler(i2c_handle_t *handle);
void HAL_I2C_ERR_IRQnHandler(i2c_handle_t *handle);



void hal_i2c_set_FMdutyCycle(I2C_TypeDef *i2cx,uint32_t duty);
void hal_i2c_set_mode(I2C_TypeDef *i2cx,uint32_t i2cMode);
void hal_i2c_set_ownadress1(I2C_TypeDef *i2cx,uint32_t ownadress);
void hal_i2c_set_ccr(I2C_TypeDef *i2cx,uint32_t pclk ,uint32_t clkSpeed ,uint32_t dutyCycle);
void hal_i2c_set_rise_time(I2C_TypeDef *i2cx,uint32_t clkSpeed ,uint32_t freqrange);
void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx,uint32_t addressMode); 
void hal_i2c_set_clk(I2C_TypeDef *i2cx,uint32_t clkSpeed ,uint32_t dutyCycle);
void hal_i2c_manage_clock_streching(I2C_TypeDef *i2cx,uint32_t nostrech);

void hal_i2c_disable(I2C_TypeDef *i2cx);
void hal_i2c_enable(I2C_TypeDef *i2cx);

void hal_i2c_generate_start_con(I2C_TypeDef *i2cx);
void hal_i2c_generate_stop_con(I2C_TypeDef *i2cx);

void hal_i2c_conf_event_interrupt(I2C_TypeDef *i2cx, uint32_t enable);
void hal_i2c_conf_buffer_interrupt(I2C_TypeDef *i2cx, uint32_t enable);
void hal_i2c_conf_err_interrupt(I2C_TypeDef *i2cx, uint32_t enable);

uint8_t hal_i2c_is_busy(I2C_TypeDef *i2cx);
void hal_i2c_wait_until_sb_set(I2C_TypeDef *i2cx);
void hal_i2c_wait_addr_set(I2C_TypeDef *i2cx);

void hal_i2c_send_addr_first(I2C_TypeDef *i2cx,uint8_t slaveAdrr);
void hal_i2c_clear_addr(I2C_TypeDef *i2cx);


void hal_i2c_master_handle_txe_int(i2c_handle_t *handle);
void hal_i2c_master_tx_complete(i2c_handle_t *handle);
void hal_i2c_master_handle_tx_btf_int(i2c_handle_t *handle);
void hal_i2c_master_handle_rxe_int(i2c_handle_t *handle);
void hal_i2c_master_rx_complete(i2c_handle_t *handle);
void hal_i2c_master_handle_rx_btf_int(i2c_handle_t *handle);

void hal_i2c_slave_handle_rx_btf_int(i2c_handle_t *handle);
void hal_i2c_slave_handle_tx_btf_int(i2c_handle_t *handle);

void hal_i2c_slave_handle_rxe_int(i2c_handle_t *handle);
void hal_i2c_slave_handle_txe_int(i2c_handle_t *handle);
void hal_i2c_slave_stop_condition(i2c_handle_t *handle);

void hal_i2c_slave_tx_complete(i2c_handle_t *handle);
void	hal_i2c_clear_stop_flag(i2c_handle_t *handle);
void hal_i2c_slave_ackFail_handler(i2c_handle_t *handle);
#endif