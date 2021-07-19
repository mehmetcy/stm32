/*
 * buttondriver.c
 *
 *  Created on: 8 May 2021
 *      Author: yasak
 */

#include "stm32f0xx_hal.h"

void buttondriver_init()
{
	RCC->AHBENR |= (1<<19);
	GPIOC->MODER &= ~(1<<10);
	GPIOC->MODER &= ~(1<<11);
}
int buttondriver_get_state()
{
	if(!(GPIOC->IDR & (1<<13)))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
