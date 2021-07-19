/*
 * leddriver.c
 *
 *  Created on: 8 May 2021
 *      Author: yasak
 */

#include "stm32f0xx_hal.h"

void leddriver_init(void)
{
	RCC->AHBENR |= (1<<17);

	GPIOA->MODER |= (1<<10);
	GPIOA->MODER &= ~(1<<11);

	GPIOA->OTYPER &= ~(1<<5);

	GPIOA->OSPEEDR |= (11<<0);
	GPIOA->OSPEEDR |= (10<<0);
}
void leddriver_on(void)
{
	GPIOA->BSRR = (1<<5);
}
void leddriver_off(void)
{
	GPIOA->BRR = (1<<5);
}
