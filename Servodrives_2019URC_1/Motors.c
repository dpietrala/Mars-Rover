#include "Motors.h"
extern sControl* pC;
static void Mot_GpioConf(void)
{
	GPIOA->MODER |= GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
	GPIOC->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0;
	GPIOB->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0 | GPIO_PUPDR_PUPD2_0;
	GPIOE->MODER |= GPIO_MODER_MODE7_0;
	GPIOE->PUPDR |= GPIO_PUPDR_PUPD7_0;
}
static void Mot_TimConf(void)
{
	//Encoder 0 TIM2 CH1, CH2, PA15, PB3 ********************************************************************
	GPIOA->MODER |= GPIO_MODER_MODER15_1;
	GPIOB->MODER |= GPIO_MODER_MODER3_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_0;		
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR3_0;
	GPIOA->AFR[1] |= 0x10000000;
	GPIOB->AFR[0] |= 0x00001000;
	TIM2->ARR = TIMMAX;
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;
	TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM2->CR1 |= TIM_CR1_CEN;

	//Encoder 1 TIM3 CH1, CH2, PB4, PB5 ********************************************************************
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0;					
	GPIOB->AFR[0] |= 0x00220000;
	TIM3->ARR = TIMMAX;
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;
	TIM3->CR1 |= TIM_CR1_CEN;
 
	//Encoder 2 TIM4 CH1, CH2, PB6, PB7 ********************************************************************
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
	GPIOB->AFR[0] |= 0x22000000;
	TIM4->ARR = TIMMAX;
	TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;
	TIM4->CR1 |= TIM_CR1_CEN;

	//Encoder 3 TIM5 CH1, CH2, PA0, PA1 ********************************************************************
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;
	GPIOA->AFR[0] |= 0x00000022;
	TIM5->ARR = TIMMAX;
	TIM5->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM5->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM5->CR1 |= TIM_CR1_CEN;

	//PWM PE9, PE11, PE13, PE14,  TIM1 CH1-4 ********************************************************************************
	GPIOE->MODER	|= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1;
	GPIOE->PUPDR	|= GPIO_PUPDR_PUPDR9_1 | GPIO_PUPDR_PUPDR11_1 | GPIO_PUPDR_PUPDR13_1 | GPIO_PUPDR_PUPDR14_1;
	GPIOE->AFR[1]	|= 0x01101010;
	TIM1->PSC	= 12-1;
	TIM1->ARR	= 1000-1;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM1->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	TIM1->CR1	|= TIM_CR1_CEN;

	//TIM7 Interrupt ********************************************************************************
	TIM7->PSC = 84-1;
	TIM7->ARR = 10000-1;	//Przerwanie co 10 ms
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
}
static void Mot_AdcConf(void)
{
	GPIOA->MODER |= GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)pC->Mot.adcval;
	DMA2_Stream0->NDTR = (uint16_t)(MOTMAX*1000);
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	
	ADC->CCR 	|= ADC_CCR_ADCPRE;
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->SMPR2 |= ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5;
	ADC1->SQR1 |= ADC_SQR1_L_0 | ADC_SQR1_L_1;
	ADC1->SQR3 |= (2<<0) | (3<<5) | (4<<10) | (5<<15);
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
void Mot_Conf(void)
{
	Mot_GpioConf();
	Mot_TimConf();
	Mot_AdcConf();
}
static void Mot_ReadPos(void)
{
	int32_t r=0;
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Mot.poslast[i] = pC->Mot.pos[i];
		pC->Mot.pos[i] = (int16_t)pC->Mot.tims[i]->CNT;
		r = pC->Mot.pos[i] - pC->Mot.poslast[i];
		if(r < (-TIMMAX/2))
			pC->Mot.fullturn[i]++;
		else if(r > (TIMMAX/2))
			pC->Mot.fullturn[i]--;
		pC->Mot.postotallast[i] = pC->Mot.postotal[i];
		pC->Mot.postotal[i] = pC->Mot.fullturn[i] * TIMMAX + pC->Mot.pos[i];
		pC->Mot.speed[i] = (double)(pC->Mot.postotal[i] - pC->Mot.postotallast[i]) / PIDTIME / 4000.0 * 60.0 * (12.0 / 42.0);
		pC->Mot.distance[i] = (double)pC->Mot.postotal[i] / 4000.0 * (12.0 / 42.0) * 0.94247779607;
	}
	pC->Mot.roverspeed = (pC->Mot.speed[0] + pC->Mot.speed[1]) / 2.0;
	pC->Mot.azimuth = (pC->Mot.distance[0] - pC->Mot.distance[1]) / 1.2;
	pC->Mot.posx += sin(pC->Mot.azimuth) * pC->Mot.roverspeed;
	pC->Mot.posy += cos(pC->Mot.azimuth) * pC->Mot.roverspeed;
}
static void Mot_PidSpeedOn(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(pC->Mot.pid[i] == On)
		{
			double kp=pC->Mot.kp[i], ki=pC->Mot.ki[i];
			pC->Mot.elast[i] = pC->Mot.e[i];
			pC->Mot.e[i] = pC->Mot.speedref[i] - pC->Mot.speed[i];
			pC->Mot.etotal[i] += pC->Mot.e[i];
			
			if(pC->Mot.etotal[i] > pC->Mot.etotalmax[i])
				pC->Mot.etotal[i] = pC->Mot.etotalmax[i];
			else if(pC->Mot.etotal[i] < -pC->Mot.etotalmax[i])
				pC->Mot.etotal[i] = -pC->Mot.etotalmax[i];
			
			if(fabs(pC->Mot.speedref[i]) < 0.1 && fabs(pC->Mot.speed[i]) < 0.1)
				pC->Mot.etotal[i] = 0.0;

			pC->Mot.out[i] = pC->Mot.e[i]*kp + pC->Mot.etotal[i]*ki*PIDTIME;
			if(pC->Mot.out[i] > pC->Mot.outmax[i])
				pC->Mot.out[i] = pC->Mot.outmax[i];
			else if(pC->Mot.out[i] < -pC->Mot.outmax[i])
				pC->Mot.out[i] = -pC->Mot.outmax[i];
			
			pC->Mot.pwm[i] = pC->Mot.out[i] * (1000.0 / 150.0);
		}
	}
}
static void Mot_PidSpeedOff(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
		if(pC->Mot.pid[i] == Off)
			pC->Mot.pwm[i] = pC->Mot.speedref[i] * (1000.0 / 150.0);
}
static void Mot_DirPwmAct(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(abs(pC->Mot.pwm[i]) >= pC->Mot.pwmdeath[i])
			pC->Mot.pwmout[i] = abs(pC->Mot.pwm[i]) - pC->Mot.pwmdeath[i];
		else
			pC->Mot.pwmout[i] = 0;
		
		if(pC->Mot.pwmout[i] > pC->Mot.pwmmax[i])
			pC->Mot.pwmout[i] = pC->Mot.pwmmax[i];
	}
	
	PWM0_REG = pC->Mot.pwmout[0];
	PWM1_REG = pC->Mot.pwmout[1];
	PWM2_REG = pC->Mot.pwmout[2];
	PWM3_REG = pC->Mot.pwmout[3];
	
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(pC->Mot.pwm[i] > 0)				pC->Mot.dir[i] = Forward;
		else if(pC->Mot.pwm[i] < 0)		pC->Mot.dir[i] = Backward;
		else 													pC->Mot.dir[i] = Stop;
	}
	
	if(pC->Mot.dir[0] == Forward)				MOT0_FOR
	else if(pC->Mot.dir[0] == Backward)	MOT0_BACK
	else if(pC->Mot.dir[0] == Stop)			MOT0_STOP
		
	if(pC->Mot.dir[1] == Forward)				MOT1_FOR
	else if(pC->Mot.dir[1] == Backward)	MOT1_BACK
	else if(pC->Mot.dir[1] == Stop)			MOT1_STOP
		
	if(pC->Mot.dir[2] == Forward)				MOT2_FOR
	else if(pC->Mot.dir[2] == Backward)	MOT2_BACK
	else if(pC->Mot.dir[2] == Stop)			MOT2_STOP
		
	if(pC->Mot.dir[3] == Forward)				MOT3_FOR
	else if(pC->Mot.dir[3] == Backward)	MOT3_BACK
	else if(pC->Mot.dir[3] == Stop)			MOT3_STOP
}
static void Mot_Stop(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Mot.speedref[i] = 0;
		pC->Mot.out[i] = 0;
		pC->Mot.pwm[i] = 0;
		pC->Mot.pwmout[i] = 0;
		pC->Mot.dir[i] = Stop;
	}
}
static void Mot_ReadCurrent(void)
{
	uint32_t suma[] = {0,0,0,0};
	uint32_t adcval[MOTMAX];
	for(uint32_t i=0;i<1000;i++)
	{
		suma[0] += pC->Mot.adcval[MOTMAX*i+0];
		suma[1] += pC->Mot.adcval[MOTMAX*i+1];
		suma[2] += pC->Mot.adcval[MOTMAX*i+2];
		suma[3] += pC->Mot.adcval[MOTMAX*i+3];
	}
	for(uint32_t i=0;i<MOTMAX;i++)
	{
		adcval[i] = suma[i] / 1000;
		pC->Mot.current[i] = 4.0 * (double)adcval[i] / 191.146667;
		pC->Mot.currenterror[i] = pC->Mot.current[i] - pC->Mot.currentmax[i];
	}
}
static void Mot_ReadStatus(void)
{
	if(pC->Status.hostcomtick >= COMTICKMAX)
	{
		pC->Status.hostcomtick = COMTICKMAX;
		pC->Status.hostcom = On;
	}
	else
		pC->Status.hostcom = Off;
	
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(pC->Mot.currenterror[i] <= 0.0)
			pC->Mot.curenttime[i] = 0;
		
		if(pC->Mot.curenttime[i] > 1000)		pC->Status.overcurrent[i] = On;
		else																pC->Status.overcurrent[i] = Off;
		
		if(fabs(pC->Mot.e[i]) > pC->Mot.emax[i])
			pC->Status.overspeed[i] = On;
		else
			pC->Status.overspeed[i] = Off;
	}
	
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(pC->Status.overcurrent[i] == On || pC->Status.overspeed[i] == On)
			pC->Status.error[i] = On;
		else
			pC->Status.error[i] = Off;
	}
	
	if(pC->Status.hostcom == On)
		pC->Status.work = Off;
	else
		pC->Status.work = On;
}
static void Mot_Act(void)
{
	Mot_ReadCurrent();
	Mot_ReadPos();
	Mot_ReadStatus();
	if(pC->Status.work == On)
	{
		LED5_OFF
		Mot_PidSpeedOn();
		Mot_PidSpeedOff();
	}
	else
	{
		LED5_ON;
		Mot_Stop();
	}
	Mot_DirPwmAct();
}
//----------------Przerwania--------------------------------------
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		Mot_Act();
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
