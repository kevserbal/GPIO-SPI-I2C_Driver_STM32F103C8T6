#ifndef __HAL_GPIO_DRIVER_H
#define __HAL_GPIO_DRIVER_H

#include "stm32f10x.h"

#define EXTIx_IRQn           EXTI0_IRQn
#define EXTIx_IRQHandler     EXTI0_IRQHandler











//Modes
#define GPIO_PIN_INPUT_MODE                  ( (uint32_t) 0x00 )
#define GPIO_PIN_OUTPUT_MODE_SPEED_LOW       ( (uint32_t) 0x02 )
#define GPIO_PIN_OUTPUT_MODE_SPEED_MEDIUM    ( (uint32_t) 0x01 )
#define GPIO_PIN_OUTPUT_MODE_SPEED_HIGH      ( (uint32_t) 0x03 )




//Input Modes 
#define GPIO_PIN_INPUT_ANALOG_MODE       ( (uint32_t) 0x00 )
#define GPIO_PIN_INPUT_FLOATING_MODE     ( (uint32_t) 0x01 )
#define GPIO_PIN_INPUT_PULLUP_PULLDOWN   ( (uint32_t) 0x02 )


//Output Modes
#define GPIO_PIN_OUTPUT_PUSH_PULL        ( (uint32_t) 0x00 )
#define GPIO_PIN_OUTPUT_OPEN_DRAIN       ( (uint32_t) 0x01 )
#define GPIO_PIN_ALT_PUSH_PULL   				 ( (uint32_t) 0x02 )
#define GPIO_PIN_ALT_OPEN_DRAIN   			 ( (uint32_t) 0x03 )




#define GPIO_PORT_A  GPIOA
#define GPIO_PORT_B  GPIOB
#define GPIO_PORT_C  GPIOC
#define GPIO_PORT_D  GPIOD


//To enable clocks for differenr gpio ports
#define _HAL_RCC_GPIOA_CLOCK_ENABLE()    (RCC->APB2ENR|=(1<<2))
#define _HAL_RCC_GPIOB_CLOCK_ENABLE()    (RCC->APB2ENR|=(1<<3))
#define _HAL_RCC_GPIOC_CLOCK_ENABLE()    (RCC->APB2ENR|=(1<<4))
#define _HAL_RCC_GPIOD_CLOCK_ENABLE()    (RCC->APB2ENR|=(1<<5))


//alternate function config

#define ALT_PORTA ( (uint8_t) 0x00 )
#define ALT_PORTB ( (uint8_t) 0x01 )
#define ALT_PORTC ( (uint8_t) 0x02 )
#define ALT_PORTD ( (uint8_t) 0x03 )

#define GPIO_PIN_0  0x00
#define GPIO_PIN_1  0x01
#define GPIO_PIN_2  0x02
#define GPIO_PIN_3  0x03
#define GPIO_PIN_4  0x04
#define GPIO_PIN_5  0x05
#define GPIO_PIN_6  0x06
#define GPIO_PIN_7  0x07
#define GPIO_PIN_8  0x08
#define GPIO_PIN_9  0x09
#define GPIO_PIN_10  0x0A
#define GPIO_PIN_11  0x0B
#define GPIO_PIN_12  0x0C
#define GPIO_PIN_13  0x0D
#define GPIO_PIN_14  0x0E
#define GPIO_PIN_15  0x0F

// Alternate function registers 

#define ALT_SPI1_REG      0
#define ALT_I2C1_REG      1
#define ALT_USART1_REG    2
#define ALT_USART2_REG    3
#define ALT_USART3_REG    4
#define ALT_TIM1_REG      6
#define ALT_TIM2_REG      8
#define ALT_TIM3_REG      10
#define ALT_TIM4_REG      12
#define ALT_CAN_REG       13
#define ALT_PD01_REG      15
#define ALT_TIM5CH4_REG   16
#define ALT_ADC1_ETRGINJ  17
#define ALT_ADC1_ETRGREG  18
#define ALT_ADC2_ETRGINJ  19
#define ALT_ADC2_ETRGREG  20
#define ALT_SWJ_CFG       24


//GPIO Interrupt config

#define GPIOA_INTERRUPT    0x00;
#define GPIOB_INTERRUPT    0x01;
#define GPIOC_INTERRUPT    0x02;
#define GPIOD_INTERRUPT    0x03;
#define GPIOE_INTERRUPT    0x04;
#define GPIOF_INTERRUPT    0x05;
#define GPIOG_INTERRUPT    0x06;


/********* Data structures ********/


typedef struct
{
   uint32_t pin;
	
   uint32_t mode;
	
   uint32_t output_type;

	 uint32_t speed;
	
   uint32_t input_type;
	
	 uint32_t pull;
	
	  
}gpio_pin_config_t;

typedef enum
{
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE
}int_edge_sel_t;


/********* Driver exposed APIs ********/

void hal_gpio_init(GPIO_TypeDef *GPIOx,gpio_pin_config_t *gpio_pin_conf);

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint8_t val);

uint8_t hal_gpio_read_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no);


 void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx,uint32_t pin_no,uint32_t mode);

void hal_gpio_configure_pin_outtype(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t out_type);
void hal_gpio_configure_pin_input(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t in_conf);
void hal_gpio_cnfg_pin_outtype_speed(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t speed);

void hal_gpio_altarnete_conf(uint32_t GPIOx,uint32_t pin_no,uint32_t selected_func,uint32_t cnf);

void hal_gpio_config_interrupt(uint32_t gpio_pin,int_edge_sel_t edge_config);

void hal_gpio_enable_interrupt(uint32_t gpio_pin,IRQn_Type IRqn);
void hal_gpio_clear_interrupt(uint32_t gpio_pin);



//led.c de olmmasi lazim aslinda
void led_init(void);

void led_turn_on(GPIO_TypeDef *GPIOx,uint16_t pin);

void led_turn_off(GPIO_TypeDef *GPIOx,uint16_t pin);


void led_toggle(GPIO_TypeDef *GPIOx,uint16_t pin);

#endif