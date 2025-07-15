#ifndef _MOTORS
#define _MOTORS

#include "Control.h"

#define M0A_SET			GPIOD->ODR |= GPIO_ODR_ODR_0
#define M0A_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_0
#define M0B_SET			GPIOD->ODR |= GPIO_ODR_ODR_1
#define M0B_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_1

#define M1A_SET			GPIOD->ODR |= GPIO_ODR_ODR_2
#define M1A_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_2
#define M1B_SET			GPIOD->ODR |= GPIO_ODR_ODR_3
#define M1B_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_3

#define M2A_SET			GPIOD->ODR |= GPIO_ODR_ODR_4
#define M2A_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_4
#define M2B_SET			GPIOD->ODR |= GPIO_ODR_ODR_5
#define M2B_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_5

#define M3A_SET			GPIOD->ODR |= GPIO_ODR_ODR_6
#define M3A_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_6
#define M3B_SET			GPIOD->ODR |= GPIO_ODR_ODR_7
#define M3B_CLR			GPIOD->ODR &= ~GPIO_ODR_ODR_7

#define M4A_SET			GPIOE->ODR |= GPIO_ODR_ODR_12
#define M4A_CLR			GPIOE->ODR &= ~GPIO_ODR_ODR_12
#define M4B_SET			GPIOE->ODR |= GPIO_ODR_ODR_13
#define M4B_CLR			GPIOE->ODR &= ~GPIO_ODR_ODR_13

#define M5A_SET			GPIOE->ODR |= GPIO_ODR_ODR_14
#define M5A_CLR			GPIOE->ODR &= ~GPIO_ODR_ODR_14
#define M5B_SET			GPIOE->ODR |= GPIO_ODR_ODR_15
#define M5B_CLR			GPIOE->ODR &= ~GPIO_ODR_ODR_15

#define M6A_SET			GPIOB->ODR |= GPIO_ODR_ODR_10
#define M6A_CLR			GPIOB->ODR &= ~GPIO_ODR_ODR_10
#define M6B_SET			GPIOB->ODR |= GPIO_ODR_ODR_11
#define M6B_CLR			GPIOB->ODR &= ~GPIO_ODR_ODR_11

#define M7A_SET			GPIOB->ODR |= GPIO_ODR_ODR_12
#define M7A_CLR			GPIOB->ODR &= ~GPIO_ODR_ODR_12
#define M7B_SET			GPIOB->ODR |= GPIO_ODR_ODR_13
#define M7B_CLR			GPIOB->ODR &= ~GPIO_ODR_ODR_13

#define	MOT0_FOR		{M0A_SET; M0B_CLR;}
#define	MOT0_BACK		{M0A_CLR; M0B_SET;}
#define	MOT0_STOP		{M0A_CLR; M0B_CLR;}
#define	MOT1_FOR		{M1A_SET; M1B_CLR;}
#define	MOT1_BACK		{M1A_CLR; M1B_SET;}
#define	MOT1_STOP		{M1A_CLR; M1B_CLR;}

#define	MOT2_FOR		{M2A_SET; M2B_CLR;}
#define	MOT2_BACK		{M2A_CLR; M2B_SET;}
#define	MOT2_STOP		{M2A_CLR; M2B_CLR;}
#define	MOT3_FOR		{M3A_SET; M3B_CLR;}
#define	MOT3_BACK		{M3A_CLR; M3B_SET;}
#define	MOT3_STOP		{M3A_CLR; M3B_CLR;}

#define	MOT4_FOR		{M4A_SET; M4B_CLR;}
#define	MOT4_BACK		{M4A_CLR; M4B_SET;}
#define	MOT4_STOP		{M4A_CLR; M4B_CLR;}
#define	MOT5_FOR		{M5A_SET; M5B_CLR;}
#define	MOT5_BACK		{M5A_CLR; M5B_SET;}
#define	MOT5_STOP		{M5A_CLR; M5B_CLR;}

#define	MOT6_FOR		{M6A_SET; M6B_CLR;}
#define	MOT6_BACK		{M6A_CLR; M6B_SET;}
#define	MOT6_STOP		{M6A_CLR; M6B_CLR;}
#define	MOT7_FOR		{M7A_SET; M7B_CLR;}
#define	MOT7_BACK		{M7A_CLR; M7B_SET;}
#define	MOT7_STOP		{M7A_CLR; M7B_CLR;}

#define PWM0_REG		TIM10->CCR1
#define PWM1_REG		TIM11->CCR1
#define PWM2_REG		TIM9->CCR1
#define PWM3_REG		TIM9->CCR2
#define PWM4_REG		TIM13->CCR1
#define PWM5_REG		TIM14->CCR1
#define PWM6_REG		TIM12->CCR1
#define PWM7_REG		TIM12->CCR2

void Mot_Conf(void);

#endif
