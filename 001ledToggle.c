/*
 * 001ledToggle.c
 *
 *  Created on: Sep 10, 2024
 *      Author: joita
 */


#include "stm32f401re.h"

void delay(){
	for(uint32_t i = 0 ;i<=500000 ; i++);
}

int main(void){

	GPIO_Handle_t gpio_led;
	//select the GPIO_port
	gpio_led.pGPIOx = GPIO_A;

	//Configure the port

	gpio_led.GPIO_pinConfig.GPIO_pinNumber = GPIO_PIN_NO_5;
	gpio_led.GPIO_pinConfig.GPIO_mode = GPIO_MODE_OUT;
	gpio_led.GPIO_pinConfig.GPIO_outputType = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_pinConfig.GPIO_speed = GPIO_SPEED_LOW ;
	gpio_led.GPIO_pinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIO_A,ENABLE);
	GPIO_init(&gpio_led);

	while(1){
		GPIO_ToggleOutputPin(GPIO_A,GPIO_PIN_NO_5);
		delay();
	}

}
