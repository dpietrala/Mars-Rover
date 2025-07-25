#include "AddDevice.h"

static void Led_Conf(void)
{
	GPIOE->MODER 	|= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0;
	GPIOE->PUPDR 	|= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR4_0;
}
static void AddDev_LaserConf(void)
{
	GPIOA->MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR11_1 | GPIO_PUPDR_PUPDR12_1;
	LASER_OFF;
	SOLENOID_OFF;
}
void AddDev_Conf(void)
{
	Led_Conf();
	AddDev_LaserConf();
}
