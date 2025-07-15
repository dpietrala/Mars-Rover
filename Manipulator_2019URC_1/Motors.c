#include "Motors.h"
extern sControl* pC;
static void Mot_ConfGpio(void)
{
	GPIOD->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0 | GPIO_PUPDR_PUPD2_0 | GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
	GPIOE->MODER |= GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0;
	GPIOE->PUPDR |= GPIO_PUPDR_PUPD12_0 | GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;
	GPIOB->MODER |= GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD12_0 | GPIO_PUPDR_PUPD13_0;
}
static void Mot_ConfTim(void)
{
	//Encoder 0 TIM1 CH1, CH2, PE9, PE11 ********************************************************************
	GPIOE->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER11_1;
	GPIOE->PUPDR |= GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR11_0;
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR11;
	GPIOE->AFR[1] |= 0x00001010;
	TIM1->ARR = TIMMAX;
	TIM1->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM1->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM1->CNT = 0;
	TIM1->CR1 |= TIM_CR1_CEN;

	//Encoder 1 TIM2 CH1, CH2, PA15, PB3 ********************************************************************
	GPIOA->MODER |= GPIO_MODER_MODER15_1;
	GPIOB->MODER |= GPIO_MODER_MODER3_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR3_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->AFR[1] |= 0x10000000;
	GPIOB->AFR[0] |= 0x00001000;
	TIM2->ARR = TIMMAX;
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;

	//Encoder 2 TIM3 CH1, CH2, PB4, PB5 ********************************************************************
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;
	GPIOB->AFR[0] |= 0x00220000;
	TIM3->ARR = TIMMAX;
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM3->CNT = 0;
	TIM3->CR1 |= TIM_CR1_CEN;

	//Encoder 3 TIM4 CH1, CH2, PB6, PB7 ********************************************************************
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->AFR[0] |= 0x22000000;
	TIM4->ARR = TIMMAX;
	TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM4->CNT = 0;
	TIM4->CR1 |= TIM_CR1_CEN;

	//Encoder 4 TIM5 CH1, CH2, PA0, PA1 ********************************************************************
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOA->AFR[0] |= 0x00000022;
	TIM5->ARR = TIMMAX;
	TIM5->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM5->CCMR1 |= TIM_CCMR1_CC1S_1 | TIM_CCMR1_CC2S_1;
	TIM5->CNT = 0;
	TIM5->CR1 |= TIM_CR1_CEN;

	//Encoder 5 TIM8 CH1, CH2, PC6, PC7 ********************************************************************
	GPIOC->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOC->AFR[0] |= 0x33000000;
	TIM8->ARR = TIMMAX;
	TIM8->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM8->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM8->CNT = 0;
	TIM8->CR1 |= TIM_CR1_CEN;
	
	//PWM0 PB8 TIM10 CH1 ********************************************************************************
	GPIOB->MODER	|= GPIO_MODER_MODER8_1;
	GPIOB->PUPDR	|= GPIO_PUPDR_PUPDR8_0;
	GPIOB->AFR[1]	|= 0x00000003;
	TIM10->PSC	= 12-1;
	TIM10->ARR	= 1000-1;
	TIM10->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM10->CCR1 = 0;
	TIM10->CCER	|= TIM_CCER_CC1E;
	TIM10->CR1	|= TIM_CR1_CEN;
	
	//PWM1 PB9 TIM11 CH1 ********************************************************************************
	GPIOB->MODER	|= GPIO_MODER_MODER9_1;
	GPIOB->PUPDR	|= GPIO_PUPDR_PUPDR9_1;
	GPIOB->AFR[1]	|= 0x00000030;
	TIM11->PSC	= 12-1;
	TIM11->ARR	= 1000-1;
	TIM11->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM11->CCR1 = 0;
	TIM11->CCER	|= TIM_CCER_CC1E;
	TIM11->CR1	|= TIM_CR1_CEN;
	
	//PWM2 PWM3 PE5, PE6, TIM9 CH1,2 ********************************************************************************
	GPIOE->MODER	|= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1;
	GPIOE->PUPDR	|= GPIO_PUPDR_PUPDR5_1 | GPIO_PUPDR_PUPDR6_1;
	GPIOE->AFR[0]	|= 0x03300000;
	TIM9->PSC	= 12-1;
	TIM9->ARR	= 1000-1;
	TIM9->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM9->CCR1 = 0;
	TIM9->CCR2 = 0;
	TIM9->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM9->CR1	|= TIM_CR1_CEN;
	
	//PWM4 PA6 TIM13 CH1 ********************************************************************************
	GPIOA->MODER	|= GPIO_MODER_MODER6_1;
	GPIOA->PUPDR	|= GPIO_PUPDR_PUPDR6_1;
	GPIOA->AFR[0]	|= 0x09000000;
	TIM13->PSC	= 6-1;
	TIM13->ARR	= 1000-1;
	TIM13->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM13->CCR1 = 0;
	TIM13->CCER	|= TIM_CCER_CC1E;
	TIM13->CR1	|= TIM_CR1_CEN;
	
	//PWM5 PA7 TIM14 CH1 ********************************************************************************
	GPIOA->MODER	|= GPIO_MODER_MODER7_1;
	GPIOA->PUPDR	|= GPIO_PUPDR_PUPDR7_1;
	GPIOA->AFR[0]	|= 0x90000000;
	TIM14->PSC	= 6-1;
	TIM14->ARR	= 1000-1;
	TIM14->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM14->CCR1 = 0;
	TIM14->CCER	|= TIM_CCER_CC1E;
	TIM14->CR1	|= TIM_CR1_CEN;
	
	//PWM6 PWM7 PB14, PB15, TIM12 CH1,2 ********************************************************************************
	GPIOB->MODER	|= GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->PUPDR	|= GPIO_PUPDR_PUPDR14_1 | GPIO_PUPDR_PUPDR15_1;
	GPIOB->AFR[1]	|= 0x99000000;
	TIM12->PSC	= 6-1;
	TIM12->ARR	= 1000-1;
	TIM12->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM12->CCR1 = 0;
	TIM12->CCR2 = 0;
	TIM12->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM12->CR1	|= TIM_CR1_CEN;
	
	//TIM6 Interrupt ********************************************************************************
	TIM6->PSC = 84-1;
	TIM6->ARR = 10000-1;	//Przerwanie co 10 ms
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
static void Mot_ConfAdc(void)
{
	GPIOC->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
	GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)pC->Mot.adcval;
	DMA2_Stream0->NDTR = (uint16_t)(AXESMAX*1000);
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC1->CR1 |= ADC_CR1_SCAN;
	
	ADC1->SMPR1 |= ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13 | ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15;
	ADC1->SMPR2 |= ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;
	ADC1->SQR1 |= ADC_SQR1_L_0 | ADC_SQR1_L_1 | ADC_SQR1_L_2;
	ADC1->SQR3 |= (10<<0) | (11<<5) | (12<<10) | (13<<15) | (14<<20) | (15<<25);
	ADC1->SQR2 |= (8<<0) | (9<<5);
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
void Mot_Conf(void)
{
	Mot_ConfGpio();
	Mot_ConfTim();
	Mot_ConfAdc();
}
static void Mot_ReadPos(void)
{
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		int32_t r=0;
		pC->Mot.implast[i] = pC->Mot.imp[i];
		pC->Mot.imp[i] = (int16_t)pC->Mot.tims[i]->CNT;
		r = pC->Mot.imp[i] - pC->Mot.implast[i];
		pC->Mot.speed[i] = 100.0 * (double)r;
		if(r < (-TIMMAX/2))
			pC->Mot.impfullturn[i]++;
		else if(r > (TIMMAX/2))
			pC->Mot.impfullturn[i]--;
		
		pC->Mot.posabs[i] =  pC->Mot.posstart[i] + (double)(pC->Mot.impfullturn[i] * TIMMAX + pC->Mot.imp[i]) / (double)pC->Mot.impperrad[i];

		double x = fmod(pC->Mot.posabs[i], M_2_PI);
		if(fabs(x) < M_PI)
			pC->Mot.pos[i] = x;
		else if(x > M_PI)
			pC->Mot.pos[i] = x - M_2_PI;
		else if(x < -M_PI)
			pC->Mot.pos[i] = M_2_PI - fabs(x);
		
		pC->Mot.posfullturn[i] = (int32_t)(pC->Mot.posabs[i] / M_2_PI);
	}
	
}
static void Mot_Regulator(void)
{
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		if(pC->Mot.pid[i] == On)
		{
			double kp=pC->Mot.kp[i], ki=pC->Mot.ki[i], kd=pC->Mot.kd[i];
			pC->Mot.elast[i] = pC->Mot.e[i];
			pC->Mot.e[i] = pC->Mot.posref[i] - pC->Mot.posabs[i];
			
			pC->Mot.etotal[i] += pC->Mot.e[i];
			if(pC->Mot.etotal[i] > pC->Mot.etotalmax[i])
				pC->Mot.etotal[i] = pC->Mot.etotalmax[i];
			else if(pC->Mot.etotal[i] < -pC->Mot.etotalmax[i])
				pC->Mot.etotal[i] = -pC->Mot.etotalmax[i];
			
			pC->Mot.out[i] = pC->Mot.e[i]*kp + pC->Mot.etotal[i]*ki*PIDTIME + (pC->Mot.e[i]-pC->Mot.elast[i])*kd/PIDTIME;
			if(pC->Mot.out[i] > pC->Mot.outmax[i])
				pC->Mot.out[i] = pC->Mot.outmax[i];
			else if(pC->Mot.out[i] < -pC->Mot.outmax[i])
				pC->Mot.out[i] = -pC->Mot.outmax[i];
		}
		else if(pC->Mot.pid[i] == Off)
			pC->Mot.out[i] = pC->Mot.speedref[i] / 100.0;
	}
}
static void Mot_DirPwmAct(void)
{
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Mot.pwm[i] = pC->Mot.out[i] * 1000.0f;
		if(pC->Mot.pwm[i] > pC->Mot.pwmmax[i])
			pC->Mot.pwm[i] = pC->Mot.pwmmax[i];
		else if(pC->Mot.pwm[i] < -pC->Mot.pwmmax[i])
			pC->Mot.pwm[i] = -pC->Mot.pwmmax[i];
		if(abs(pC->Mot.pwm[i]) >= pC->Mot.pwmdeath[i])
			pC->Mot.pwmout[i] = abs(pC->Mot.pwm[i]) - pC->Mot.pwmdeath[i];
		else
			pC->Mot.pwmout[i] = 0;
	}
	PWM0_REG = pC->Mot.pwmout[0];
	PWM1_REG = pC->Mot.pwmout[1];
	PWM2_REG = pC->Mot.pwmout[2];
	PWM3_REG = pC->Mot.pwmout[3];
	PWM4_REG = pC->Mot.pwmout[4];
	PWM5_REG = pC->Mot.pwmout[5];
	PWM6_REG = pC->Mot.pwmout[6];
	PWM7_REG = pC->Mot.pwmout[7];
	
	
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		if(pC->Mot.out[i] > 0)
			pC->Mot.dir[i] = Forward;
		else if(pC->Mot.out[i] < 0)
			pC->Mot.dir[i] = Backward;
		else pC->Mot.dir[i] = Stop;
	}
	
	if(pC->Mot.dir[0] == Forward)				MOT0_BACK
	else if(pC->Mot.dir[0] == Backward)	MOT0_FOR
	else if(pC->Mot.dir[0] == Stop)			MOT0_STOP

	if(pC->Mot.dir[1] == Forward)				MOT1_FOR
	else if(pC->Mot.dir[1] == Backward)	MOT1_BACK
	else if(pC->Mot.dir[1] == Stop)			MOT1_STOP
		
	if(pC->Mot.dir[2] == Forward)				MOT2_BACK
	else if(pC->Mot.dir[2] == Backward)	MOT2_FOR
	else if(pC->Mot.dir[2] == Stop)			MOT2_STOP

	if(pC->Mot.dir[3] == Forward)				MOT3_FOR
	else if(pC->Mot.dir[3] == Backward)	MOT3_BACK
	else if(pC->Mot.dir[3] == Stop)			MOT3_STOP
		
	if(pC->Mot.dir[4] == Forward)				MOT4_BACK
	else if(pC->Mot.dir[4] == Backward)	MOT4_FOR
	else if(pC->Mot.dir[4] == Stop)			MOT4_STOP

	if(pC->Mot.dir[5] == Forward)				MOT5_FOR
	else if(pC->Mot.dir[5] == Backward)	MOT5_BACK
	else if(pC->Mot.dir[5] == Stop)			MOT5_STOP
		
	if(pC->Mot.dir[6] == Forward)				MOT6_BACK
	else if(pC->Mot.dir[6] == Backward)	MOT6_FOR
	else if(pC->Mot.dir[6] == Stop)			MOT6_STOP

	if(pC->Mot.dir[7] == Forward)				MOT7_FOR
	else if(pC->Mot.dir[7] == Backward)	MOT7_BACK
	else if(pC->Mot.dir[7] == Stop)			MOT7_STOP
}
static void Mot_ReadCurrent(void)
{
	uint32_t suma[AXESMAX] = {0,0,0,0,0,0,0,0};
	uint32_t adcval[AXESMAX];
	for(uint32_t i=0;i<1000;i++)
	{
		suma[0] += pC->Mot.adcval[AXESMAX*i+0];
		suma[1] += pC->Mot.adcval[AXESMAX*i+1];
		suma[2] += pC->Mot.adcval[AXESMAX*i+2];
		suma[3] += pC->Mot.adcval[AXESMAX*i+3];
		suma[4] += pC->Mot.adcval[AXESMAX*i+4];
		suma[5] += pC->Mot.adcval[AXESMAX*i+5];
		suma[6] += pC->Mot.adcval[AXESMAX*i+6];
		suma[7] += pC->Mot.adcval[AXESMAX*i+7];
	}
	for(uint32_t i=0;i<AXESMAX;i++)
	{
		adcval[i] = suma[i] / 1000;
		pC->Mot.current[i] = (double)adcval[i] / 191.146667;
	}
}
static void Mot_ReadErrors(void)
{
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		if(fabs(pC->Mot.posabs[i] - pC->Mot.posabsmin[i]) < 0.02)
			pC->Mot.error[i] = On;
		else if(fabs(pC->Mot.posabs[i] - pC->Mot.posabsmax[i]) < 0.02)
			pC->Mot.error[i] = On;
		else
			pC->Mot.error[i] = Off;
	}
}
static void Mot_Act(void)
{
	Mot_ReadPos();
	Mot_ReadErrors();
	Mot_ReadCurrent();
	Mot_Regulator();
	Mot_DirPwmAct();
}
//----------------Przerwania--------------------------------------
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_SR_UIF) != RESET)
	{
		Mot_Act();
		TIM6->SR &= ~TIM_SR_UIF;
	}
}
