/*
 * stm32f401re.h
 *
 *  Created on: Sep 1, 2024
 *      Author: joita
 */

#ifndef INC_STM32F401RE_H_
#define INC_STM32F401RE_H_

#include<stdint.h>

/*Base Addresses of Flash, SRAM, ROM*/

#define FLASH_BASE_ADDRESS           0x08000000U //these values can be found in referrence manual
#define SRAM1_BASE_ADDRESS           0x20000000U
#define SRAM                         SRAM1_BASE_ADDRESS
#define ROM_BASE_ADDRESS             0x1FFF0000U

/*Base Addresses of bus Domains*/

#define PERIPH_BASE_ADDRESS          0x40000000U
#define APB1_PERIPH_BASE_ADDRESS     PERIPH_BASE_ADDRESS
#define APB2_PERIPH_BASE_ADDRESS     0x40010000U
#define AHB1_PERIPH_BASE_ADDRESS     0x40020000U
#define AHB2_PERIPH_BASE_ADDRESS     0x50000000U

//Base Addresses of Peripherals  which are hanging on AHB1 Bus

#define GPIOA_BASE_ADDRESS           0x40020000U
#define GPIOB_BASE_ADDRESS           0x40020400U
#define GPIOC_BASE_ADDRESS           0x40020800U
#define GPIOD_BASE_ADDRESS           0x40020C00U
#define GPIOE_BASE_ADDRESS           0x40021000U
#define GPIOH_BASE_ADDRESS           0x40021C00U
#define RCC_BASE_ADDRESS             0x40023800U

//Base Addresses of Peripherals which are hanging on APB1 Bus

#define I2C1_BASE_ADDRESS            0x40005400U
#define I2C2_BASE_ADDRESS            0x40005800U
#define I2C3_BASE_ADDRESS            0x40005C00U
#define SPI2_BASE_ADDRESS            0x40003800U
#define SPI3_BASE_ADDRESS            0x40003C00U
#define USART2_BASE_ADDRESS          0x40004400U

//Base Addresses of Peripherals which are hanging on APB2 Bus

#define SPI1_BASE_ADDRESS            0x40013000U
#define SPI4_BASE_ADDRESS            0x40013400U
#define USART1_BASE_ADDRESS          0x40011000U
#define USART6_BASE_ADDRESS          0x40011400U
#define EXTI_BASE_ADDRESS            (APB2_PERIPH_BASE_ADDRESS + 0x3C00U)
#define SYSCFG_BASE_ADDRESS          (APB2_PERIPH_BASE_ADDRESS + 0x3800U)

/************************* Peripheral Register Definition *******************************/
typedef struct{
	volatile uint32_t MODER ;                 //To set the mode in which the MCU will work Input/Output/Alternate_function/Analog
	volatile uint32_t OTYPER ;                //To define the output type push_pull/open_drain
	volatile uint32_t OSPEEDR ;               //To configure the I/O output speed
	volatile uint32_t PUPDR ;                 //To configure pull-up or pull-down register
	volatile uint32_t IDR ;                   //These bits are read-only and can be accessed in word mode only. They contain the input value of the corresponding I/O port.
	volatile uint32_t ODR ;                   //These bits can be read and written by software.
	volatile uint32_t BSRR ;                  //To sets or resets the corresponding bit in ODR
	volatile uint32_t LCKR ;                  //This register is used to lock the configuration of the port bits when a correct write sequence is applied to bit 16 (LCKK). The value of bits [15:0] is used to lock the configuration of the GPIO. During the write sequence, the value of LCKR[15:0] must not change. When the LOCK sequence has been applied on a port bit, the value of this port bit can no longer be modified until the next MCU or peripheral reset.
	volatile uint32_t AFR[2] ;                //This register will configure the pin for Alternate functions for ALF[0] = pins[0:7] ; ALF[1]= pin[8:15]

} GPIO_Reg_Def_t;



//************************ RCC Register Definition Structure ************************
typedef struct{
	volatile uint32_t CR;                     //RCC clock control register
	volatile uint32_t PLLCFGR;                //RCC PLL configuration register
	volatile uint32_t CFGR;                   //RCC clock configuration register
	volatile uint32_t CIR;                    //RCC clock interrupt register
	volatile uint32_t AHB1RSTR;               //RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;               //RCC AHB2 peripheral reset register
	uint32_t reserved0[2];
	volatile uint32_t APB1RSTR;               //RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;               //RCC APB2 peripheral reset register
	uint32_t reserved1[2];
	volatile uint32_t AHB1ENR;                //RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;                //RCC AHB2 peripheral clock enable register
	uint32_t reserved2[2];
	volatile uint32_t APB1ENR;                //RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;                //RCC APB2 peripheral clock enable register
	uint32_t reserved3[2];
	volatile uint32_t AHB1LPENR;              //RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;              //RCC AHB2 peripheral clock enable in low power mode register
	uint32_t reserved4[2];
	volatile uint32_t APB1LPENR;              //RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;              //RCC APB2 peripheral clock enable in low power mode register
	uint32_t reserved5[2];
	volatile uint32_t BDCR;                   //RCC Backup domain control register
	volatile uint32_t CSR;                    //RCC clock control & status register
	uint32_t reserved6[2];
	volatile uint32_t SSCGR;                  //RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;             //RCC Backup domain control register
	uint32_t reserved7;
	volatile uint32_t DCKCFGR;                //RCC Dedicated Clocks Configuration Register


}RCC_Reg_Def_t;

#define GPIO_A                      ((GPIO_Reg_Def_t*)GPIOA_BASE_ADDRESS)
#define GPIO_B                      ((GPIO_Reg_Def_t*)GPIOB_BASE_ADDRESS)
#define GPIO_C                      ((GPIO_Reg_Def_t*)GPIOC_BASE_ADDRESS)
#define GPIO_D                      ((GPIO_Reg_Def_t*)GPIOD_BASE_ADDRESS)
#define GPIO_E                      ((GPIO_Reg_Def_t*)GPIOE_BASE_ADDRESS)
#define GPIO_H                      ((GPIO_Reg_Def_t*)GPIOH_BASE_ADDRESS)

#define RCC                         ((RCC_Reg_Def_t*) RCC_BASE_ADDRESS)

//GPIO_Reg_Def_t *pGPIO_A = GPIO_A ;

//************* Enabling Clock for GPIOx Peripherals ***********************

#define GPIOA_PCLK_EN()             ( (RCC->AHB1ENR) |= (1<<0) )
#define GPIOB_PCLK_EN()             ( (RCC->AHB1ENR) |= (1<<1) )
#define GPIOC_PCLK_EN()             ( (RCC->AHB1ENR) |= (1<<2) )
#define GPIOD_PCLK_EN()             ( (RCC->AHB1ENR) |= (1<<3) )
#define GPIOE_PCLK_EN()             ( (RCC->AHB1ENR) |= (1<<4) )
#define GPIOH_PCLK_EN()             ( (RCC->AHB1ENR) |= (1<<7) )

//************ Enabling Clock for I2C Peripherals **********************
#define I2C1_PCLK_EN()             ( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()             ( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()             ( RCC->APB1ENR |= (1<<23) )

//**************** Enabling Clock for SPI Peripherals **********************

#define SPI1_PCLK_EN()             ( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()             ( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()             ( RCC->APB1ENR |= (1<<15) )
#define SPI4_PCLK_EN()             ( RCC->APB2ENR |= (1<<13) )

//********************** Enabling Clock For USART Peripheral ***********************

#define USART1_PCLK_EN()             ( RCC->APB2ENR |= (1<<4) )
#define USART2_PCLK_EN()             ( RCC->APB1ENR |= (1<<17) )
#define USART6_PCLK_EN()             ( RCC->APB2ENR |= (1<<5) )

//********************** Enabling Clock for SYSCFG Peripheral *********************
#define SYSCFG_PCLK_EN()             ( RCC->APB2ENR |= (1<<14) )


//************* Disabling Clock for GPIOx Peripherals ***********************

#define GPIOA_PCLK_DI()             ( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()             ( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()             ( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()             ( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()             ( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOH_PCLK_DI()             ( RCC->AHB1ENR &= ~(1<<7) )

//************ Disabling Clock for I2C Peripherals **********************
#define I2C1_PCLK_DI()             ( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()             ( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()             ( RCC->APB1ENR &= ~(1<<23) )

//**************** Disabling Clock for SPI Peripherals **********************

#define SPI1_PCLK_DI()             ( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI()             ( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()             ( RCC->APB1ENR &= ~(1<<15) )
#define SPI4_PCLK_DI()             ( RCC->APB2ENR &= ~(1<<13) )

//********************** Disabling Clock For USART Peripheral ***********************

#define USART1_PCLK_DI()             ( RCC->APB2ENR &= ~(1<<4) )
#define USART2_PCLK_DI()             ( RCC->APB1ENR &= ~(1<<17) )
#define USART6_PCLK_DI()             ( RCC->APB2ENR &= ~(1<<5) )

//********************** Disabling Clock for SYSCFG Peripheral *********************
#define SYSCFG_PCLK_DI()             ( RCC->APB2ENR &= ~(1<<14) )


/***********************Reseting the All the register of a given Peripheral********************************/
#define GPIOA_REG_RESET()            do{( (RCC->AHB1RSTR) |= (1<<0) ); ((RCC->AHB1RSTR) &= ~(1<<0) );}while(0)
#define GPIOB_REG_RESET()            do{( (RCC->AHB1RSTR) |= (1<<0) ); ((RCC->AHB1RSTR) &= ~(1<<1) );}while(0)
#define GPIOC_REG_RESET()            do{( (RCC->AHB1RSTR) |= (1<<0) ); ((RCC->AHB1RSTR) &= ~(1<<2) );}while(0)
#define GPIOD_REG_RESET()            do{( (RCC->AHB1RSTR) |= (1<<0) ); ((RCC->AHB1RSTR) &= ~(1<<3) );}while(0)
#define GPIOE_REG_RESET()            do{( (RCC->AHB1RSTR) |= (1<<0) ); ((RCC->AHB1RSTR) &= ~(1<<4) );}while(0)
#define GPIOH_REG_RESET()            do{( (RCC->AHB1RSTR) |= (1<<0) ); ((RCC->AHB1RSTR) &= ~(1<<7) );}while(0)


//Some general Macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#include "stm32f401xx_gpio_driver.h"


#endif /* INC_STM32F401RE_H_ */
