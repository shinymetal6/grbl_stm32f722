/*
 * stm32_hal_port.h
 *
 *  Created on: May 1, 2020
 *      Author: fil
 */

#ifndef GRBL_INC_STM32_HAL_PORT_H_
#define GRBL_INC_STM32_HAL_PORT_H_

void   		GPIO_Write(GPIO_TypeDef* GPIOx , uint16_t value);
uint16_t	GPIO_ReadOutputData(GPIO_TypeDef* GPIOx );
uint16_t	GPIO_ReadInputData(GPIO_TypeDef* GPIOx );
uint16_t	GPIO_ReadInputBit(GPIO_TypeDef* GPIOx, uint16_t bit);
void 		Led1On(void);
void 		Led1Off(void);
uint32_t	grbl_start(void);

#endif /* GRBL_INC_STM32_HAL_PORT_H_ */
