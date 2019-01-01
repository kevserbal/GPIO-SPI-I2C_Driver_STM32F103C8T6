#ifndef HAL_SPI_DRIVER_H
#define HAL_SPI_DRIVER_H


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/**************** SPI control register 1 (SPI_CR1) ***************************/

#define spi_bidi_mode                         ( (uint32_t ) 1<<15 )
#define spi_enable_2_line_uni_dir             0
#define spi_enable_1_line_bidi                1


#define spi_dff                               ((uint32_t) 1<<11)
#define spi_8bit_dff                          0
#define spi_16bit_dff                         1

#define spi_ssm_reg                           ((uint32_t) 1<<9)
#define spi_ssm_disabled                      0
#define spi_ssm_enabled                       1

#define spi_internal_slave_select             ((uint32_t) 1<<8)

#define spi_lsbfirst                          ((uint32_t) 1<<7)
#define spi_tx_msbfirst                       0
#define spi_tx_lsbfirst                       1

#define spi_enable_reg                        ((uint32_t)  1<<6)


#define spi_bound_rate_f_div_2                0
#define spi_bound_rate_f_div_4                1
#define spi_bound_rate_f_div_8                2
#define spi_bound_rate_f_div_16               3
#define spi_bound_rate_f_div_32               4
#define spi_bound_rate_f_div_64               5
#define spi_bound_rate_f_div_128              6
#define spi_bound_rate_f_div_256              7

#define spi_master_reg_config                 ((uint32_t)  1<<2)
#define spi_slave_config                      0
#define spi_master_config                     1

#define spi_clock_pol_reg                     ((uint32_t)  1<<1)
#define spi_clock_pol_low                     0
#define spi_clock_pol_high                    1


#define spi_clock_cpha_reg                    ((uint32_t)  1<<0)
#define spi_clock_cpha_first_trans            0
#define spi_clock_cpha_scnd_trans             1

/**************** SPI control register 2 (SPI_CR2) ***************************/

#define spi_tx_buff_emty_int_en_reg           ((uint32_t)  1<<7)
#define spi_rx_buff_not_emty_int_en_reg       ((uint32_t)  1<<6)
#define spi_error_int_reg                     ((uint32_t)  1<<5)
#define spi_ss_enable_reg                     ((uint32_t)  1<<2)
#define spi_ss_multi_maste_mode               0
#define spi_ss_outpu_enable                   1

#define spi_tx_buff_dma_enable                ((uint32_t)  1<<1) //When this bit is set, the DMA request is made whenever the TXE flag is set.
#define spi_rx_buff_dma_enable                ((uint32_t)  1<<0) //When this bit is set, the DMA request is made whenever the RXNE flag is set.




/*************** SPI Status Register (SPI_SR)      ****************************/
#define spi_busy_flag_reg                     ((uint32_t)  1<<7)
#define spi_ovverun_flag                      ((uint32_t)  1<<6)
#define spi_mode_fault                        ((uint32_t)  1<<5)
#define spi_crc_err_flag                      ((uint32_t)  1<<4)
#define spi_underrun_flag                     ((uint32_t)  1<<3)
#define spi_channel_side                      ((uint32_t)  1<<2)
#define spi_transmit_buff_empty               ((uint32_t)  1<<1)
#define spi_recieve_buff_empty                ((uint32_t)  1<<0)

#define RESET 0
#define SET   !RESET

#define SPI_IS_BUSY      1
#define SPI_IS_NOT_BUSY  0



/*************** Macros to enable clock for different spi's ********************/

#define _HAL_RCC_SPI1_CLK_ENABLE()   (RCC->APB2ENR|=((uint32_t ) 1<<12))
#define _HAL_RCC_SPI2_CLK_ENABLE()   (RCC->APB1ENR|=((uint32_t ) 1<<14))
#define _HAL_RCC_SPI3_CLK_ENABLE()   (RCC->APB1ENR|=((uint32_t ) 1<<15))





/*************** Data Sructures for SPI *****************************************/

typedef enum
{ 
  HAL_SPI_STATE_RESET      = 0X00,
	HAL_SPI_STATE_READY      = 0x01,
	HAL_SPI_STATE_BUSY       = 0x02,
	HAL_SPI_STATE_BUSY_TX    = 0x012,
  HAL_SPI_STATE_BUSY_RX    = 0x022,
  HAL_SPI_STATE_BUSY_TX_RX = 0x032,
	HAL_SPI_STATE_ERROR      = 0X03

}hal_spi_state_t;



typedef struct
{
  uint32_t mode;  
	uint32_t direction;
	uint32_t DataSize;
	uint32_t CLKPol;
	uint32_t CLKPhase;
	uint32_t NSS;
	uint32_t BoundRatePreScaler;
	uint32_t FirstBit;            // MSB OR LSB

} spi_init_typedef;



typedef struct __spi_handler_typedef
{
	
	SPI_TypeDef        *Instance;   //Registers Base Address
	spi_init_typedef    init; 
	uint8_t            *pTXBuffPtr; //Pointer to SPI transfer buffer ----uzunlugu ayarlamak için pointer kullandik
	uint16_t            TransferSize;
	uint16_t            TransferCount;
	
	uint8_t            *pRXBuffPtr; //Pointer to SPI receiver buffer ----uzunlugu ayarlamak için pointer kullandik
	uint16_t            ReceiveSize;
	uint16_t            ReceiveCount;
	
	hal_spi_state_t     State;
	
}spi_handler_typedef;




/************* SPI APIs ***********************************/



void hal_spi_init (spi_handler_typedef *spiHandle);

void hal_spi_cnfg_device_mode(SPI_TypeDef *SPIx,uint32_t master);
void hal_spi_cnfg_phase_pol(SPI_TypeDef *SPIx,uint32_t phase, uint32_t polarity);
void hal_spi_cnfg_datasize_direction(SPI_TypeDef *SPIx,uint32_t datasize,uint32_t msb_lsb);
void hal_spi_cnfg_NSS_master(SPI_TypeDef *SPIx,uint32_t ssm_enable);
void hal_spi_cnfg_NSS_slave(SPI_TypeDef *SPIx,uint32_t ssm_enable);
void hal_spi_cnfg_boundrate(SPI_TypeDef *SPIx,uint32_t boundrate);

void hal_spi_enable(SPI_TypeDef *SPIx);
void hal_spi_disable(SPI_TypeDef *SPIx);

void hal_spi_master_tx(spi_handler_typedef *spiHandle,uint8_t *buffer,uint32_t len);
void hal_spi_master_rx(spi_handler_typedef *spiHandle,uint8_t *rcv_buffer,uint32_t len);
void hal_spi_slave_rx(spi_handler_typedef *spiHandle,uint8_t *buffer,uint32_t len);
void hal_spi_slave_tx(spi_handler_typedef *spiHandle,uint8_t *rv_buffer,uint32_t len);

void hal_spi_i2c_irq_handler(spi_handler_typedef *spiHandle);
void hal_spi_rx_interrupt_handler(spi_handler_typedef *spiHandle);
void hal_spi_rx_close_intterput(spi_handler_typedef *spiHandle);

void hal_spi_tx_interrupt_handler(spi_handler_typedef *spiHandle);
void hal_spi_tx_close_intterput(spi_handler_typedef *spiHandle);


void hal_spi_write_to_transmit(spi_handler_typedef *spiHandle,uint16_t data);
void hal_spi_enable_interrpt_txe(SPI_TypeDef *SPIx);
void hal_spi_disable_interrpt_txe(SPI_TypeDef *SPIx);

void hal_spi_enable_interrpt_rxe(SPI_TypeDef *SPIx);
void hal_spi_disable_interrpt_rxe(SPI_TypeDef *SPIx);

uint8_t hal_spi_is_busy(SPI_TypeDef *SPIx);


#endif
