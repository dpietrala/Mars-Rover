#ifndef _MOTORS
#define _MOTORS

#include "Control.h"

#define M0A_SET			GPIOA->ODR |= GPIO_ODR_ODR_6
#define M0A_CLR			GPIOA->ODR &= ~GPIO_ODR_ODR_6
#define M0B_SET			GPIOA->ODR |= GPIO_ODR_ODR_7
#define M0B_CLR			GPIOA->ODR &= ~GPIO_ODR_ODR_7

#define M1A_SET			GPIOC->ODR |= GPIO_ODR_ODR_4
#define M1A_CLR			GPIOC->ODR &= ~GPIO_ODR_ODR_4
#define M1B_SET			GPIOC->ODR |= GPIO_ODR_ODR_5
#define M1B_CLR			GPIOC->ODR &= ~GPIO_ODR_ODR_5

#define M2A_SET			GPIOB->ODR |= GPIO_ODR_ODR_0
#define M2A_CLR			GPIOB->ODR &= ~GPIO_ODR_ODR_0
#define M2B_SET			GPIOB->ODR |= GPIO_ODR_ODR_1
#define M2B_CLR			GPIOB->ODR &= ~GPIO_ODR_ODR_1

#define M3A_SET			GPIOB->ODR |= GPIO_ODR_ODR_2
#define M3A_CLR			GPIOB->ODR &= ~GPIO_ODR_ODR_2
#define M3B_SET			GPIOE->ODR |= GPIO_ODR_ODR_7
#define M3B_CLR			GPIOE->ODR &= ~GPIO_ODR_ODR_7
#define PWM0_REG		TIM1->CCR1
#define PWM1_REG		TIM1->CCR2
#define PWM2_REG		TIM1->CCR3
#define PWM3_REG		TIM1->CCR4

#define	MOT0_FOR		{M0A_SET; M0B_CLR;}
#define	MOT0_BACK		{M0A_CLR; M0B_SET;}
#define	MOT0_STOP		{M0A_CLR; M0B_CLR;}

#define	MOT1_BACK		{M1A_SET; M1B_CLR;}
#define	MOT1_FOR		{M1A_CLR; M1B_SET;}
#define	MOT1_STOP		{M1A_CLR; M1B_CLR;}

#define	MOT2_FOR		{M2A_SET; M2B_CLR;}
#define	MOT2_BACK		{M2A_CLR; M2B_SET;}
#define	MOT2_STOP		{M2A_CLR; M2B_CLR;}

#define	MOT3_BACK		{M3A_SET; M3B_CLR;}
#define	MOT3_FOR		{M3A_CLR; M3B_SET;}
#define	MOT3_STOP		{M3A_CLR; M3B_CLR;}

void Mot_Conf(void);


#endif
