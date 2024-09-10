/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Sep 4, 2024
 *      Author: joita
 */

#include "stm32f401xx_gpio_driver.h"

//For controlling Peripheral clock
/**********************************************************************************************
 * @fn              -GPIO_PeriClockControl
 *
 * @brief           -This function is used to enable or disable the peripheral clock of the given peripheral
 *
 * @param[in]       -Base address of GPIO Peripheral
 * @param[in]       -ENABLE or DISABLE MACROS
 * @param[in]       -
 *
 * @return          -none
 *
 * @note            -none
 * */

void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t En_Di){

	if(En_Di == ENABLE){
		if(pGPIOx == GPIO_A){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIO_B){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIO_C){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIO_D){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIO_E){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIO_H){
			GPIOH_PCLK_EN();
		}
	}
	else{

		if(pGPIOx == GPIO_A){
					GPIOA_PCLK_DI();
				}
				else if(pGPIOx == GPIO_B){
					GPIOB_PCLK_DI();
				}
				else if(pGPIOx == GPIO_C){
					GPIOC_PCLK_DI();
				}
				else if(pGPIOx == GPIO_D){
					GPIOD_PCLK_DI();
				}
				else if(pGPIOx == GPIO_E){
					GPIOE_PCLK_DI();
				}
				else if(pGPIOx == GPIO_H){
					GPIOH_PCLK_DI();
				}

	}

}



//For Initializing and de-initializing GPIO

/**********************************************************************************************
 * @fn              -GPIO_init
 *
 * @brief           -This function is used to initialize the given Peripheral register
 *
 * @param[in]       -A pointer to handle structure is passed
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -none
 *
 * @note            -none
 * */

void GPIO_init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0; //temporary register

	//1) configure the mode of the gpio pin

	if(pGPIOHandle->GPIO_pinConfig.GPIO_mode<= GPIO_MODE_ANALOG ){
		temp = (pGPIOHandle->GPIO_pinConfig.GPIO_mode<<(2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else{
		//This part I'll code later its the interrupt mode
	}

	temp =0;

	//2) configure the speed

	temp = (pGPIOHandle->GPIO_pinConfig.GPIO_speed<<(2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |=  temp;

	temp = 0;

	//3) configure the pu-pd settings
	temp = (pGPIOHandle->GPIO_pinConfig.GPIO_PuPdControl<<(2*pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;
	//4) configure the op_type
    temp = (pGPIOHandle->GPIO_pinConfig.GPIO_outputType<<(pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber));
    pGPIOHandle->pGPIOx->OTYPER  &= ~(0x1<<pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber);
    pGPIOHandle->pGPIOx->OTYPER |=temp;

    temp = 0;
	//5) configure the alt_functionality

    if(pGPIOHandle->GPIO_pinConfig.GPIO_mode==GPIO_MODE_ALTFN){

    	uint32_t temp1,temp2,temp3;
    	temp1 = pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber/8; //this will tell us whether we have to choose higher or lower register
    	temp2 = pGPIOHandle->GPIO_pinConfig.GPIO_pinNumber%8; //this will tell us which bits field we have to configure
    	temp3 = pGPIOHandle->GPIO_pinConfig.GPIO_AltFuncMode<<(4*temp2);
    	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf<<temp2);
    	pGPIOHandle->pGPIOx->AFR[temp1] |= temp3;

    }

}



/**********************************************************************************************
 * @fn              -GPIO_DeInit
 *
 * @brief           -This function is used to de-initialize all the registers of the given peripheral
 *
 * @param[in]       -Base Address of the required peripheral is passed
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -none
 *
 * @note            -none
 * */

void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx){

	if(pGPIOx == GPIO_A){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIO_B){
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIO_C){
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIO_D){
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIO_E){
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIO_H){
				GPIOH_REG_RESET();
			}

}




//For data read write

/**********************************************************************************************
 * @fn              -GPIO_ReadFromInputPin
 *
 * @brief           -This function is used for reading data from a specific pin of a given port
 *
 * @param[in]       -Base address of the given gpio port
 * @param[in]       -Pin number of the pin you want to read data from
 * @param[in]       -
 *
 * @return          -0&1
 *
 * @note            -none
 * */

uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t pinNumber){
	uint8_t value;

	value = (uint8_t)(( pGPIOx->IDR >> pinNumber) & 0x00000001);

	return value;
}



/**********************************************************************************************
 * @fn              -GPIO_ReadFromInputPort
 *
 * @brief           -This function is used for read the content of the data register or the value from all the pins in the port
 *
 * @param[in]       -Base address of the Port you want to read from
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -uint16_t
 *
 * @note            -none
 * */


uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx){
	uint16_t value;
	value =(uint16_t) pGPIOx->IDR;
	return value;
}

/**********************************************************************************************
 * @fn              -GPIO_WriteFromOutputPin
 *
 * @brief           -This function is used to write a value to a specific pin
 *
 * @param[in]       -Base address of the gpio
 * @param[in]       -Pin number of the pin you want to write to
 * @param[in]       -Value you want to write
 *
 * @return          -none
 *
 * @note            -none
 * */

void GPIO_WriteFromOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t pinNumber, uint8_t value){

	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1<<pinNumber);
		//write 1 to the bits field corresponding to the pin number
	}
	else{
		pGPIOx->ODR &= ~(1<<pinNumber);
				//write 1 to the bits field cor
		//write 0 to the bits field corresponding to the pin number
	}

}



/**********************************************************************************************
 * @fn              -GPIO_WriteFromOutputPort
 *
 * @brief           -This function is used to write to the output register
 *
 * @param[in]       -Base Address of the GPIO
 * @param[in]       -Value you want to write
 * @param[in]       -
 *
 * @return          -
 *
 * @note            -
 * */

void GPIO_WriteFromOutputPort(GPIO_Reg_Def_t *pGPIOx, uint8_t value){
	pGPIOx->ODR=value;


}




/**********************************************************************************************
 * @fn              -GPIO_ToggleOutputPin
 *
 * @brief           -This function is used to complement the value already present in the pin
 *
 * @param[in]       -Base address of the GPIO
 * @param[in]       -Pin number of the pin whose vaslue you want to complement
 * @param[in]       -
 *
 * @return          -
 *
 * @note            -
 * */

void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t  pinNumber){
	pGPIOx->ODR ^= (1<<pinNumber);

}



//IRQ configure and ISR handling

/**********************************************************************************************
 * @fn              -GPIO_IRQConfig
 *
 * @brief           -This function is used to configure the IRQ number of the GPIO, and setting the priority
 *
 * @param[in]       - Interrupt request number
 * @param[in]       - Interrupt request priority
 * @param[in]       - Enable or Disable Macro
 *
 * @return          -
 *
 * @note            -
 * */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_Di){

}



/**********************************************************************************************
 * @fn              - GPIO_IRQHandling
 *
 * @brief           - This function is used to manage the interrupt from a given GPIO pin
 *
 * @param[in]       - Pin Number is passed
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @note            -
 * */

void GPIO_IRQHandling(uint8_t pinNumber){

}
