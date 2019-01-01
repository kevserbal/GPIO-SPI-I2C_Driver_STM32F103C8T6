#include "hal_gpio_driver.h"

void led_init(void)
{
     gpio_pin_config_t led_pin_conf;
	
	  _HAL_RCC_GPIOB_CLOCK_ENABLE();
			
		 led_pin_conf.pin=GPIO_PIN_8;
	   led_pin_conf.mode=GPIO_PIN_OUTPUT_MODE_SPEED_LOW;
	   led_pin_conf.output_type=GPIO_PIN_OUTPUT_PUSH_PULL;
	 
	   hal_gpio_init(GPIOB,&led_pin_conf);
	
	

	
		 led_pin_conf.pin=GPIO_PIN_7;
	   led_pin_conf.mode=GPIO_PIN_OUTPUT_MODE_SPEED_LOW;
	   led_pin_conf.output_type=GPIO_PIN_OUTPUT_PUSH_PULL;
	 
	   hal_gpio_init(GPIOB,&led_pin_conf);
		 
	
	
		 led_pin_conf.pin=GPIO_PIN_1;
		 led_pin_conf.input_type=GPIO_PIN_INPUT_ANALOG_MODE;
	   led_pin_conf.mode=GPIO_PIN_INPUT_MODE;

	   hal_gpio_init(GPIOB,&led_pin_conf);
		 


}

void led_turn_on(GPIO_TypeDef *GPIOx,uint16_t pin)
{
   hal_gpio_write_to_pin(GPIOx,pin,1);

}

void led_turn_off(GPIO_TypeDef *GPIOx,uint16_t pin)
{
   hal_gpio_write_to_pin(GPIOx,pin,0);

}

void led_toggle(GPIO_TypeDef *GPIOx,uint16_t pin)
{
    if(hal_gpio_read_to_pin(GPIOx,pin))
			 hal_gpio_write_to_pin(GPIOx,pin,0);
    else 
			 hal_gpio_write_to_pin(GPIOx,pin,1);
		
		uint32_t i=0;
		
		for(i=0;i<10000;i++);
}
///

void hal_gpio_init(GPIO_TypeDef *GPIOx,gpio_pin_config_t *gpio_pin_conf)
{
  
	 hal_gpio_configure_pin_mode(GPIOx,gpio_pin_conf->pin,gpio_pin_conf->mode);
	 if(gpio_pin_conf->mode!=0x00)
	 {
     hal_gpio_configure_pin_outtype(GPIOx,gpio_pin_conf->pin,gpio_pin_conf->output_type);	 
	 }
	 else 
	   hal_gpio_configure_pin_input(GPIOx,gpio_pin_conf->pin,gpio_pin_conf->input_type);
}

 void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx,uint32_t pin_no,uint32_t mode)
{
    if(pin_no <0x08)
		   GPIOx->CRL|= (mode << pin_no*4); 
		else
  		 GPIOx->CRH|= (mode << (pin_no-8)*4);
		
}


void hal_gpio_configure_pin_outtype(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t out_type)
{
    if(pin_no <0x08)
		{
		  GPIOx->CRL &= ~(0X03 << (  pin_no*4    +2));
			GPIOx->CRL |= (out_type << (  pin_no*4    +2)); 
		}			
		else
		{
		  GPIOx->CRH &= ~(0X03 << ( (pin_no-8)*4 +2));
			GPIOx->CRH |= (out_type << (  (pin_no-8)*4    +2)); 
		}
}


void hal_gpio_cnfg_pin_outtype_speed(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t speed)
{
    if(pin_no <0x08)
		  if(speed!=0)   
    	   GPIOx->CRL|= (speed << (  pin_no*4 )); 
			else
		     GPIOx->CRL&= ~(0X03 << (  3*4 )); 	
		else
			
		  if(speed!=0)   
    	   GPIOx->CRH|= (speed << (  pin_no*4  )); 
			else
		     GPIOx->CRH&= ~(0X03 << ( (pin_no-8)*4 ));
		
}

 void hal_gpio_configure_pin_input(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t in_conf)
{

    if(pin_no <0x08)
		   GPIOx->CRL|= (in_conf << (  pin_no*4    +2)); 
		else
  		 GPIOx->CRH|= (in_conf << ( (pin_no-8)*4 +2));
		

}




 void hal_gpio_altarnete_conf(uint32_t GPIOx,uint32_t pin_no,uint32_t selected_func,uint32_t cnf)
{
    
  AFIO->EVCR|=(GPIOx<<4);
  AFIO->EVCR|=(pin_no<<0);
	//AFIO->MAPR|=(cnf<<selected_func);
	
}

void hal_gpio_config_interrupt(uint32_t gpio_pin,int_edge_sel_t edge_config)
{
   //disable intterupt mask reg
	
	 if(edge_config==INT_RISING_EDGE)
		 EXTI->RTSR  |=(SET<<gpio_pin);
	 
	 else if (edge_config==INT_FALLING_EDGE)
		 EXTI->FTSR  |=(SET<<gpio_pin);
	 
	 else if (edge_config==INT_RISING_FALLING_EDGE)
	 { 
		 EXTI->RTSR  |=(SET<<gpio_pin);		 
		 EXTI->FTSR  |=(SET<<gpio_pin);
	 }
   else
	 {
	 
	 }


}

uint8_t hal_gpio_read_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
   uint8_t value;
	
	 value=((GPIOx->IDR>>pin_no)& 0x00000001);
   
	 return value;
}

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint8_t val)
{

   /*GPIOx->ODR |=  (val<<pin_no); bunu yaptigimiz zaman bu registerda
    1 degeri varsa or dan dolayi sifiri degerini yazdiramayiz*/

  if (val != RESET)
  {
    GPIOx->BSRR |=(1<<pin_no);
  }
  else
  {
    GPIOx->BRR |=(1<<pin_no);
  }
	 
}

void hal_gpio_enable_interrupt(uint32_t gpio_pin,IRQn_Type IRqn)
{   
/*
	 if(gpio_pin < 4)
		 	AFIO->EXTICR[0] &= ~(~gpiox_interrupt<< gpio_pin*4);
	
   else if(gpio_pin < 8)
			AFIO->EXTICR[1] |= (gpiox_interrupt<< gpio_pin*4);
	 
   else if(gpio_pin < 12)
			AFIO->EXTICR[2] |= (gpiox_interrupt<< gpio_pin*4);
	 
   else 
			AFIO->EXTICR[3] |= (gpiox_interrupt<< gpio_pin*4);
*/
	 EXTI->IMR |= (SET<<gpio_pin);
	 NVIC_EnableIRQ(IRqn);

}




void hal_gpio_clear_interrupt(uint32_t gpio_pin)
{
	if(EXTI->PR & (1 << gpio_pin ))
	{
		EXTI->PR |= 1 << gpio_pin;
	}
}
