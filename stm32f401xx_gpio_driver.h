/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Sep 4, 2024
 *      Author: joita
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401re.h"


typedef struct{
	uint32_t GPIO_portName;
	uint32_t GPIO_pinNumber;
	uint32_t GPIO_mode;      /*Possible GPIO modes can be referred @GPIO_pin_modes*/
	uint32_t GPIO_speed;     /*Possible output speed can be referred @GPIO_output_speed*/
	uint32_t GPIO_outputType;/* Possible output types can be referred @GPIO_Output_type*/
	uint32_t GPIO_PuPdControl; /*Possible pull-up and pull-down configurations can be referred @GPIO_PU_PD_Config*/
	uint32_t GPIO_AltFuncMode;

}GPIO_pinConfig_t;


typedef struct {
	GPIO_Reg_Def_t *pGPIOx;  // this pointer variable holds the address of the GPIO we want to use
    GPIO_pinConfig_t GPIO_pinConfig; //this variable holds GPIO pin configuration settings

}GPIO_Handle_t;


/* @GPIO_Pin_no
 * GPIO possible pin numbers
 * */
#define GPIO_PIN_NO_0     0
#define GPIO_PIN_NO_1     1
#define GPIO_PIN_NO_2     2
#define GPIO_PIN_NO_3     3
#define GPIO_PIN_NO_4     4
#define GPIO_PIN_NO_5     5
#define GPIO_PIN_NO_6     6
#define GPIO_PIN_NO_7     7
#define GPIO_PIN_NO_8     8
#define GPIO_PIN_NO_9     9
#define GPIO_PIN_NO_10    10
#define GPIO_PIN_NO_11    11
#define GPIO_PIN_NO_12    12
#define GPIO_PIN_NO_13    13
#define GPIO_PIN_NO_14    14
#define GPIO_PIN_NO_15    15

/*  @GPIO_pin_modes
 * GPIO pin Possible Modes
 *
 * If value is less than or equal to 3 then it is non-interrupt mode*/

#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG    3
#define GPIO_MODE_IT_FT     4    //Falling edge trigger
#define GPIO_MODE_IT_RT     5    //Rising edge trigger
#define GPIO_MODE_IT_FRT    6    //Falling-Rising edge trigger

/* @GPIO_Output_type
 * GPIO pin Possible output mode */

#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1

/* @GPIO_output_speed
 * GPIO pin possible output speed */

#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MED      1
#define GPIO_SPEED_HIGH     2
#define GPIO_SPEED_VHIGH    3

/*  @GPIO_PU_PD_Config
 *  GPIO pin pull-up and pull-down configuration macros */

#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2





/**************************************************************************************************
 *             APIs Supported by this driver
 *      For more information check the function definition
 ************************************************************************************************** */

//For controlling Peripheral clock

void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t En_Di); //We will be passing the base address of gpio and weather we have to enable or disable the gpio

//For Initializing and de-initializing GPIO

void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx);

//For data read write

uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx); // return the content of input data register
void GPIO_WriteFromOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t pinNumber, uint8_t value);
void GPIO_WriteFromOutputPort(GPIO_Reg_Def_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t  pinNumber);

//IRQ configure and ISR handling

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_Di);
void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
