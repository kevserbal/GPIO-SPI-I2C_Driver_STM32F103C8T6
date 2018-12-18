#include "gpio_driver.h"
void hal_gpio_init(GPIO_TypeDef *GPIOx,gpio_pin_config_t *gpio_pin_conf)
{

	 hal_gpio_configure_pin_mode(GPIOx,gpio_pin_conf->pin,gpio_pin_conf->mode);
  
}

static  void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t mode)
{
    if(pin_no <0x08)
		   GPIOx->CRL|= (mode << pin_no*4); 
		else
  		 GPIOx->CRH|= (mode << (pin_no-8)*4);
		
}


static  void hal_gpio_configure_pin_outtype(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t out_type)
{
    if(pin_no <0x08)
		   GPIOx->CRL|= (out_type << (  pin_no*4    +2)); 
		else
  		 GPIOx->CRH|= (out_type << ( (pin_no-8)*4 +2));
		
}

static  void hal_gpio_configure_pin_input(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t in_conf)
{

    if(pin_no <0x08)
		   GPIOx->CRL|= (in_conf << (  pin_no*4    +2)); 
		else
  		 GPIOx->CRH|= (in_conf << ( (pin_no-8)*4 +2));
		

}




static  void hal_gpio_altarnete_conf(uint32_t GPIOx,uint32_t pin_no,uint32_t selected_func,uint32_t cnf)
{
    
  AFIO->EVCR|=(GPIOx<<4);
  AFIO->EVCR|=(pin_no<<0);
	AFIO->MAPR|=(cnf<<selected_func);
	
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
	 if(val)
		  GPIOx->ODR|=(1<<pin_no);
   else
	    GPIOx->ODR&=~(1<<pin_no);
		
}