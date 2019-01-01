#include "led.h"

void led_init(void)
{
     gpio_pin_config_t led_pin_conf;
	
	   _HAL_RCC_GPIOA_CLOCK_ENABLE();
	
	   led_pin_conf.pin=3;
	   led_pin_conf.mode=GPIO_PIN_OUTPUT_MODE_SPEED_LOW;
	   led_pin_conf.output_type=GPIO_PIN_OUTPUT_PUSH_PULL;
	 
	   hal_gpio_init(GPIOA,&led_pin_conf);
	  

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
		
		for(i=0;i<1000000;i++);
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
		  if(out_type!=0)   
    	   GPIOx->CRL|= (out_type << (  pin_no*4    +2)); 
			else
		     GPIOx->CRL&= ~(0X03 << (  3*4    +2)); 	
		else
			
		  if(out_type!=0)   
    	   GPIOx->CRH|= (out_type << (  pin_no*4    +2)); 
			else
		     GPIOx->CRH&= ~(0X03 << ( (pin_no-8)*4 +2));
		
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
	AFIO->MAPR|=(cnf<<selected_func);
	
}



uint8_t hal_gpio_read_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
   uint8_t value;
	
	 value=((GPIOx->ODR>>pin_no)& 0x00000001);
   
	 return value;
}

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint8_t val)
{

   /*GPIOx->ODR |=  (val<<pin_no); bunu yaptigimiz zaman bu registerda
    1 degeri varsa or dan dolayi sifiri degerini yazdiramayiz*/
	 if(val)
		  GPIOx->ODR|=(1<<pin_no);
   else
	    GPIOx->ODR&=~(1<<pin_no);
		
}

int main(){

	led_init();
  while(1)
  led_toggle(GPIOA,3);
	
	
return 0;
	
}