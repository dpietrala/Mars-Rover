#include "Drive.h"
extern sControl* pC;
static void Drive_GpioConf(void)
{
	//-------------------- Silniki DC ------------------------
	//Silnik DC 0
	GPIOA->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0;
	//Silnik DC 1
	GPIOC->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0;
	//Silnik DC 2
	GPIOB->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0;
	//Silnik DC 3
	GPIOB->MODER |= GPIO_MODER_MODE2_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD2_0;
	GPIOE->MODER |= GPIO_MODER_MODE7_0;
	GPIOE->PUPDR |= GPIO_PUPDR_PUPD7_0;
	//Silnik DC 4
	GPIOB->MODER |= GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD12_0 | GPIO_PUPDR_PUPD13_0;
	MOT4FOR;
	
	//------------------- Krancówki ----------------------------
	GPIOE->MODER &= ~GPIO_PUPDR_PUPD8 & ~GPIO_PUPDR_PUPD9 & ~GPIO_PUPDR_PUPD10 & ~GPIO_PUPDR_PUPD11 & ~GPIO_PUPDR_PUPD12 & ~GPIO_PUPDR_PUPD13 & ~GPIO_PUPDR_PUPD14 & ~GPIO_PUPDR_PUPD15;
	GPIOE->PUPDR |= GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD12_0 | GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;
	
	//------------------- Przyciski ----------------------------
	GPIOD->MODER &= ~GPIO_PUPDR_PUPD8 & ~GPIO_PUPDR_PUPD9 & ~GPIO_PUPDR_PUPD10 & ~GPIO_PUPDR_PUPD11 & ~GPIO_PUPDR_PUPD12 & ~GPIO_PUPDR_PUPD13 & ~GPIO_PUPDR_PUPD14 & ~GPIO_PUPDR_PUPD15;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD12_0 | GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;
}
static void Drive_TimConf(void)
{
	//Encoder 0 TIM2 CH1, CH2, PA15, PB3
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
	pC->Drive.mottimsenc[0] = TIM2;

	//Encoder 1 TIM3 CH1, CH2, PB4, PB5
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0;					
	GPIOB->AFR[0] |= 0x00220000;
	TIM3->ARR = TIMMAX;
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;
	TIM3->CR1 |= TIM_CR1_CEN;
	pC->Drive.mottimsenc[1] = TIM3;
	
	//Encoder 2 TIM4 CH1, CH2, PB6, PB7
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
	GPIOB->AFR[0] |= 0x22000000;
	TIM4->ARR = TIMMAX;
	TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;
	TIM4->CR1 |= TIM_CR1_CEN;
	pC->Drive.mottimsenc[2] = TIM4;
	
	//Encoder 3 TIM5 CH1, CH2, PA0, PA1
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOA->AFR[0] |= 0x00000022;
	TIM5->ARR = TIMMAX;
	TIM5->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM5->CCMR1 |= TIM_CCMR1_CC1S_1 | TIM_CCMR1_CC2S_1;
	TIM5->CNT = 0;
	TIM5->CR1 |= TIM_CR1_CEN;
	pC->Drive.mottimsenc[3] = TIM5;
		
	//Silnik DC 0-1 PWM, PE5, PE6, TIM9 CH1 CH2
	GPIOA->MODER	|= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->PUPDR	|= GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1;
	GPIOA->AFR[0]	|= 0x00003300;
	TIM9->PSC	= 12-1;
	TIM9->ARR	= 1000-1;
	TIM9->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM9->CCR1 = 0;
	TIM9->CCR2 = 0;
	TIM9->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM9->CR1	|= TIM_CR1_CEN;
	
	//Silnik DC 2 PWM, PA6 TIM13 CH1
	GPIOA->MODER	|= GPIO_MODER_MODER6_1;
	GPIOA->PUPDR	|= GPIO_PUPDR_PUPDR6_1;
	GPIOA->AFR[0]	|= 0x09000000;
	TIM13->PSC	= 6-1;
	TIM13->ARR	= 1000-1;
	TIM13->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM13->CCR1 = 0;
	TIM13->CCER	|= TIM_CCER_CC1E;
	TIM13->CR1	|= TIM_CR1_CEN;
	
	//Silnik DC 3 PWM, PA7 TIM14 CH1
	GPIOA->MODER	|= GPIO_MODER_MODER7_1;
	GPIOA->PUPDR	|= GPIO_PUPDR_PUPDR7_1;
	GPIOA->AFR[0]	|= 0x90000000;
	TIM14->PSC	= 6-1;
	TIM14->ARR	= 1000-1;
	TIM14->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM14->CCR1 = 0;
	TIM14->CCER	|= TIM_CCER_CC1E;
	TIM14->CR1	|= TIM_CR1_CEN;
	
	//Silnik DC 4 PWM, PB14, TIM12 CH1
	GPIOB->MODER	|= GPIO_MODER_MODER14_1;
	GPIOB->PUPDR	|= GPIO_PUPDR_PUPDR14_1;
	GPIOB->AFR[1]	|= 0x99000000;
	TIM12->PSC	= 6-1;
	TIM12->ARR	= 1000-1;
	TIM12->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM12->CCR1 = 0;
	TIM12->CCER	|= TIM_CCER_CC1E;
	TIM12->CR1	|= TIM_CR1_CEN;

	//PWM serownapedy 0-3 PC6, PC7, PC8, PC9  TIM8 CH1-4
	GPIOC->MODER	|= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOC->PUPDR	|= GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR7_1 | GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR9_1;
	GPIOC->AFR[0]	|= 0x33000000;
	GPIOC->AFR[1]	|= 0x00000033;
	TIM8->PSC	= 168-1;
	TIM8->ARR	= 20000-1;
	TIM8->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM8->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM8->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM8->BDTR |= TIM_BDTR_MOE;
	SERW0PWM_REG = pC->Serw.griprefpwm;
	SERW1PWM_REG = pC->Serw.openrefpwm;
	SERW2PWM_REG = pC->Serw.turnrefpwm;
	SERW3PWM_REG = 1500;
	TIM8->CR1	|= TIM_CR1_CEN;
	
	//PWM serownapedy 4-7 PE9, PE11, PE13, PE14,  TIM1 CH1-4
	GPIOA->MODER	|= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOA->PUPDR	|= GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR9_1 | GPIO_PUPDR_PUPDR10_1 | GPIO_PUPDR_PUPDR11_1;
	GPIOA->AFR[1]	|= 0x00001111;
	TIM1->PSC	= 168-1;
	TIM1->ARR	= 20000-1;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM1->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM1->BDTR |= TIM_BDTR_MOE;
	SERW4PWM_REG = 1500;
	SERW5PWM_REG = 1500;
	SERW6PWM_REG = 1500;
	SERW7PWM_REG = 1500;
	TIM1->CR1	|= TIM_CR1_CEN;

	//TIM7 Interrupt
	TIM7->PSC = 84-1;
	TIM7->ARR = 10000-1;	//Przerwanie co 10 ms
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
}
static void Drive_AdcConf(void)
{
	GPIOC->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3;
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)pC->Drive.motadcval;
	DMA2_Stream0->NDTR = (uint16_t)(MOTMAX*1000);
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	ADC->CCR 	|= ADC_CCR_ADCPRE;
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->SMPR1 |= ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP13 | ADC_SMPR1_SMP13;
	ADC1->SQR1 |= ADC_SQR1_L_0 | ADC_SQR1_L_1;
	ADC1->SQR3 |= (10<<0) | (11<<5) | (12<<10) | (13<<15);
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
static void Drive_BackupConf(void)
{
	PWR->CR = PWR_CR_DBP;
	PWR->CSR = PWR_CSR_BRE;
	while (!( PWR->CSR & PWR_CSR_BRR));
}
static void Drive_WriteBackup(void)
{
	volatile int32_t * const p2 = (int32_t *)BACKUPADDRESS;
	int32_t offset = 0;
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		*(p2+offset) = (int32_t)(pC->Drive.motposabs[i] * 10000.0);
		offset++;
	}
}
static void Drive_ReadBackup(void)
{
	volatile int32_t * const p2 = (int32_t *)BACKUPADDRESS;
	int32_t offset = 0;
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Drive.motposstart[i] = (double)(*(p2+offset)) / 10000.0;
		pC->Drive.motposabsref[i] = pC->Drive.motposstart[i];
		offset++;
	}
}
void Drive_Conf(void)
{
	Drive_GpioConf();
	Drive_BackupConf();
	Drive_ReadBackup();
	Drive_TimConf();
	Drive_AdcConf();
}

static void Drive_MotStopOneMot(uint8_t num)
{
	pC->Drive.motposabsref[num] = pC->Drive.motposabs[num];
	pC->Drive.motetotal[num] = 0.0;
	pC->Drive.motspeedref[num] = 0;
	pC->Drive.motspeedreftemp[num] = 0;
}
static void Drive_MotSetHomePosition(uint8_t num)
{
	pC->Drive.motposabsref[num] = 0.0;
	pC->Drive.motposabs[num] = 0.0;
	pC->Drive.motposstart[num] = 0.0;
	pC->Drive.motimp[num] = 0.0;
	pC->Drive.motimplast[num] = 0.0;
	pC->Drive.motimptotal[num] = 0.0;
	pC->Drive.motfullturn[num] = 0.0;
	pC->Drive.mottimsenc[num]->CR1 &= ~TIM_CR1_CEN;
	pC->Drive.mottimsenc[num]->CNT = 0;
	pC->Drive.mottimsenc[num]->CR1 |= TIM_CR1_CEN;
}
static void Drive_ReadButtons(void)
{
	if((GPIOD->IDR & GPIO_IDR_ID9) == RESET)			pC->Drive.buttons[0] = On;
	else																					pC->Drive.buttons[0] = Off;
	if((GPIOD->IDR & GPIO_IDR_ID8) == RESET)			pC->Drive.buttons[1] = On;
	else																					pC->Drive.buttons[1] = Off;
	if((GPIOD->IDR & GPIO_IDR_ID10) == RESET)			pC->Drive.buttons[2] = On;
	else																					pC->Drive.buttons[2] = Off;
	if((GPIOD->IDR & GPIO_IDR_ID11) == RESET)			pC->Drive.buttons[3] = On;
	else																					pC->Drive.buttons[3] = Off;
	if((GPIOD->IDR & GPIO_IDR_ID12) == RESET)			pC->Drive.buttons[4] = On;
	else																					pC->Drive.buttons[4] = Off;
	if((GPIOD->IDR & GPIO_IDR_ID13) == RESET)			pC->Drive.buttons[5] = On;
	else																					pC->Drive.buttons[5] = Off;
	if((GPIOD->IDR & GPIO_IDR_ID14) == RESET)			pC->Drive.buttons[6] = On;
	else																					pC->Drive.buttons[6] = Off;
	if((GPIOD->IDR & GPIO_IDR_ID15) == RESET)			pC->Drive.buttons[7] = On;
	else																					pC->Drive.buttons[7] = Off;
	
	pC->Mode.manual = Off;
	for(uint8_t i=0;i<BUTTONMAX;i++)
	{
		if(pC->Drive.buttons[i] == On)
			pC->Mode.manual = On;
	}
}
static void Drive_ReadHardwareLimits(void)
{
	if((GPIOE->IDR & GPIO_IDR_ID8) == RESET)			pC->Drive.limitswitches[0] = On;
	else																					pC->Drive.limitswitches[0] = Off;
	if((GPIOE->IDR & GPIO_IDR_ID9) == RESET)			pC->Drive.limitswitches[1] = On;
	else																					pC->Drive.limitswitches[1] = Off;
	if((GPIOE->IDR & GPIO_IDR_ID10) == RESET)			pC->Drive.limitswitches[2] = On;
	else																					pC->Drive.limitswitches[2] = Off;
	if((GPIOE->IDR & GPIO_IDR_ID11) == RESET)			pC->Drive.limitswitches[3] = On;
	else																					pC->Drive.limitswitches[3] = Off;
	if((GPIOE->IDR & GPIO_IDR_ID12) == RESET)			pC->Drive.limitswitches[4] = On;
	else																					pC->Drive.limitswitches[4] = Off;
	if((GPIOE->IDR & GPIO_IDR_ID13) == RESET)			pC->Drive.limitswitches[5] = On;
	else																					pC->Drive.limitswitches[5] = Off;
	if((GPIOE->IDR & GPIO_IDR_ID14) == RESET)			pC->Drive.limitswitches[6] = On;
	else																					pC->Drive.limitswitches[6] = Off;
	if((GPIOE->IDR & GPIO_IDR_ID15) == RESET)			pC->Drive.limitswitches[7] = On;
	else																					pC->Drive.limitswitches[7] = Off;
	
	if(pC->Drive.limitswitches[0] == On)
		Drive_MotSetHomePosition(0);
	if(pC->Drive.limitswitches[2] == On)
		Drive_MotSetHomePosition(1);
	if(pC->Drive.limitswitches[4] == On)
		Drive_MotSetHomePosition(2);
	if(pC->Drive.limitswitches[6] == On)
		Drive_MotSetHomePosition(3);
	
	if(pC->Drive.limitswitches[0] == On && pC->Drive.motspeedref[0] < 0.0)
		Drive_MotStopOneMot(0);
	if(pC->Drive.limitswitches[2] == On && pC->Drive.motspeedref[1] < 0.0)
		Drive_MotStopOneMot(1);
	if(pC->Drive.limitswitches[4] == On && pC->Drive.motspeedref[2] < 0.0)
		Drive_MotStopOneMot(2);
	if(pC->Drive.limitswitches[6] == On && pC->Drive.motspeedref[3] < 0.0)
		Drive_MotStopOneMot(3);
}
static void Drive_ReadSoftwareLimits(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(pC->Drive.motlimits[i] == On && pC->Drive.motposabs[i] <= pC->Drive.motposabsmin[i] && pC->Drive.motspeedref[i] < 0.0)
			Drive_MotStopOneMot(i);
		if(pC->Drive.motlimits[i] == On && pC->Drive.motposabs[i] >= pC->Drive.motposabsmax[i] && pC->Drive.motspeedref[i] > 0.0)
			Drive_MotStopOneMot(i);
	}
}
static void Drive_MotReadPos(void)
{
	int32_t r=0;
	for(uint8_t i=0;i<3;i++)
	{
		pC->Drive.motimplast[i] = pC->Drive.motimp[i];
		pC->Drive.motimp[i] = (int16_t)pC->Drive.mottimsenc[i]->CNT;
		r = pC->Drive.motimp[i] - pC->Drive.motimplast[i];
		if(r < (-TIMMAX/2))
			pC->Drive.motfullturn[i]++;
		else if(r > (TIMMAX/2))
			pC->Drive.motfullturn[i]--;
		pC->Drive.motimptotal[i] = pC->Drive.motfullturn[i] * TIMMAX + pC->Drive.motimp[i];
		pC->Drive.motposabslast[i] = pC->Drive.motposabs[i];
		pC->Drive.motposabs[i] = pC->Drive.motposstart[i] + (double)pC->Drive.motimptotal[i] * 4.0 / 64.0 / 19.0;
		pC->Drive.motspeed[i] = (pC->Drive.motposabs[i] - pC->Drive.motposabslast[i]) / PIDTIME;
	}
	//Dla silnika obracajacego luminotester
	for(uint8_t i=3;i<MOTMAX;i++)
	{
		pC->Drive.motimplast[i] = pC->Drive.motimp[i];
		pC->Drive.motimp[i] = (int16_t)pC->Drive.mottimsenc[i]->CNT;
		r = pC->Drive.motimp[i] - pC->Drive.motimplast[i];
		if(r < (-TIMMAX/2))
			pC->Drive.motfullturn[i]++;
		else if(r > (TIMMAX/2))
			pC->Drive.motfullturn[i]--;
		pC->Drive.motimptotal[i] = pC->Drive.motfullturn[i] * TIMMAX + pC->Drive.motimp[i];
		pC->Drive.motposabslast[i] = pC->Drive.motposabs[i];
		pC->Drive.motposabs[i] = pC->Drive.motposstart[i] + (double)pC->Drive.motimptotal[i] * 360.0 / 64.0 / 131.0 / 3.0;
		pC->Drive.motspeed[i] = (pC->Drive.motposabs[i] - pC->Drive.motposabslast[i]) / PIDTIME;
	}
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if((pC->Drive.motposabs[i] - pC->Drive.motposabsmin[i]) < 2)
			pC->Drive.moterror[i] = On;
		else if((pC->Drive.motposabs[i] - pC->Drive.motposabsmax[i]) > -2)
			pC->Drive.moterror[i] = On;
		else
			pC->Drive.moterror[i] = Off;
	}
}
static void Drive_MotReadCurrent(void)
{
	uint32_t suma[MOTMAX]={0,0,0,0};
	for(uint32_t i=0;i<1000;i++)
	{
		suma[0] += pC->Drive.motadcval[4*i+0];
		suma[1] += pC->Drive.motadcval[4*i+1];
		suma[2] += pC->Drive.motadcval[4*i+2];
		suma[3] += pC->Drive.motadcval[4*i+3];
	}
	for(uint8_t i=0;i<MOTMAX;i++)
		pC->Drive.motcurrent[i] = 3.0*(double)(suma[i] / 1000.0) / 4096.0 / 0.140;
}
static void Drive_MotSetPosref(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Drive.motposabsref[i] += pC->Drive.motspeedref[i] * pC->Drive.motstep[i];
	}
}
static void Drive_AutoNextPoint(void)
{
	if(pC->Stats.seq[pC->Stats.seqnum].snum < (pC->Stats.seq[pC->Stats.seqnum].smax - 1))
	{
		pC->Stats.seq[pC->Stats.seqnum].snum++;
		pC->Stats.time = 0;
	}
	else
	{
		pC->Mode.work = Workremote;
	}
}
static void Drive_AutoPreviousPoint(void)
{
	if(pC->Stats.seq[pC->Stats.seqnum].snum > 0)
	{
		pC->Stats.seq[pC->Stats.seqnum].snum--;
		pC->Stats.time = 0;
	}
}
static void Drive_AutoSetPosref(void)
{
	eOnOff	inpos[MOTMAX] = {Off,Off,Off,Off};
	uint8_t seqnum = pC->Stats.seqnum;
	uint8_t snum = pC->Stats.seq[seqnum].snum;
	if(seqnum >= pC->Stats.seqmax || snum >= pC->Stats.seq[seqnum].smax)
	{
		pC->Mode.work = Workremote;
		return;
	}
	double hyst = pC->Stats.posrefhyst;
	sSingleStatus s = pC->Stats.seq[seqnum].s[snum];
	
	pC->Serw.griprefpwm = s.pwmrefg;
	pC->Serw.turnrefpwm = s.pwmreft;
	pC->Serw.openrefpwm = s.pwmrefo;
	
	double speed[MOTMAX];
	double time[MOTMAX];
	for(uint8_t i=0;i<MOTMAX;i++)
		time[i] = fabs(s.posref[i] - pC->Drive.motposabsref[i]) / (pC->Drive.motstep[i] * s.speed);
	double maxtime = time[0];
	for(uint8_t i=0;i<MOTMAX;i++)
		if(time[i] > maxtime)
			maxtime = time[i];
	for(uint8_t i=0;i<MOTMAX;i++)
		speed[i] = (time[i] / maxtime) * s.speed;
	
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if((s.posref[i] - pC->Drive.motposabsref[i]) > hyst)
			pC->Drive.motposabsref[i] += speed[i] * pC->Drive.motstep[i];
		else if((s.posref[i] - pC->Drive.motposabsref[i]) < -hyst)
			pC->Drive.motposabsref[i] -= speed[i] * pC->Drive.motstep[i];
		else
			inpos[i] = On;
	}
	
	if(inpos[0]==On && inpos[1]==On && inpos[2]==On && inpos[3]==On && pC->Stats.time >= s.delay)
		Drive_AutoNextPoint();
}
static void Drive_MotRegulator(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(pC->Drive.motpid[i] == On)
		{
			double kp=pC->Drive.motkp[i], ki=pC->Drive.motki[i], kd=pC->Drive.motkd[i];
			pC->Drive.motelast[i] = pC->Drive.mote[i];
			pC->Drive.mote[i] = pC->Drive.motposabsref[i] - pC->Drive.motposabs[i];
			
			pC->Drive.motetotal[i] += pC->Drive.mote[i];
			if(pC->Drive.motetotal[i] > pC->Drive.motetotalmax[i])
				pC->Drive.motetotal[i] = pC->Drive.motetotalmax[i];
			else if(pC->Drive.motetotal[i] < -pC->Drive.motetotalmax[i])
				pC->Drive.motetotal[i] = -pC->Drive.motetotalmax[i];
			
			pC->Drive.motout[i] = pC->Drive.mote[i]*kp + pC->Drive.motetotal[i]*ki*PIDTIME + (pC->Drive.mote[i]-pC->Drive.motelast[i])*kd/PIDTIME;
			if(pC->Drive.motout[i] > pC->Drive.motoutmax[i])
				pC->Drive.motout[i] = pC->Drive.motoutmax[i];
			else if(pC->Drive.motout[i] < -pC->Drive.motoutmax[i])
				pC->Drive.motout[i] = -pC->Drive.motoutmax[i];
		}
		else if(pC->Drive.motpid[i] == Off)
			pC->Drive.motout[i] = pC->Drive.motspeedref[i] / 100.0;
	}
}
static void Drive_MotPwmDirAct(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Drive.motpwm[i] = pC->Drive.motout[i] * (double)pC->Drive.motpwmmax[i];
		
		if(pC->Drive.motpwm[i] > pC->Drive.motpwmmax[i])
			pC->Drive.motpwm[i] = pC->Drive.motpwmmax[i];
		else if(pC->Drive.motpwm[i] < -pC->Drive.motpwmmax[i])
			pC->Drive.motpwm[i] = -pC->Drive.motpwmmax[i];
		
		if(abs(pC->Drive.motpwm[i]) >= pC->Drive.motpwmdeath[i])
			pC->Drive.motpwmout[i] = abs(pC->Drive.motpwm[i]) - pC->Drive.motpwmdeath[i];
		else
			pC->Drive.motpwmout[i] = 0;
	}
	MOT0PWM_REG = pC->Drive.motpwmout[0];
	MOT1PWM_REG = pC->Drive.motpwmout[1];
	MOT2PWM_REG = pC->Drive.motpwmout[2];
	MOT3PWM_REG = pC->Drive.motpwmout[3];
	
	
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		if(pC->Drive.motout[i] > 0)
			pC->Drive.motdir[i] = Forward;
		else if(pC->Drive.motout[i] < 0)
			pC->Drive.motdir[i] = Backward;
		else pC->Drive.motdir[i] = Stop;
	}
	
	if(pC->Drive.motdir[0] == Forward)				MOT0BACK
	else if(pC->Drive.motdir[0] == Backward)	MOT0FOR
	else if(pC->Drive.motdir[0] == Stop)			MOT0STOP

	if(pC->Drive.motdir[1] == Forward)				MOT1FOR
	else if(pC->Drive.motdir[1] == Backward)	MOT1BACK
	else if(pC->Drive.motdir[1] == Stop)			MOT1STOP
		
	if(pC->Drive.motdir[2] == Forward)				MOT2BACK
	else if(pC->Drive.motdir[2] == Backward)	MOT2FOR
	else if(pC->Drive.motdir[2] == Stop)			MOT2STOP

	if(pC->Drive.motdir[3] == Forward)				MOT3FOR
	else if(pC->Drive.motdir[3] == Backward)	MOT3BACK
	else if(pC->Drive.motdir[3] == Stop)			MOT3STOP
}
static void Drive_ReadMode(void)
{
	if(pC->Mode.hostcomtick >= COMTICKMAX)
	{
		pC->Mode.hostcomtick = COMTICKMAX;
		pC->Mode.hostcom = Off;
	}
	else
	{
		pC->Mode.hostcom = On;
	}
	
	if(pC->Mode.manual == On)
	{
		pC->Mode.work = Workmanual;
	}
	else if(pC->Mode.hostcom == Off)
	{
		pC->Mode.work = Workoff;
	}
	else if(pC->Mode.work == Workauto)
	{
	}
	else if(pC->Mode.hostcom == On)
	{
		pC->Mode.work = Workremote;
	}
	else
	{
		pC->Mode.work = Workoff;
	}
}
static void Drive_MotManualAct(void)
{
	double speed = 55, step = 0.25;

	if(pC->Drive.buttons[7] == On && pC->Drive.buttons[6] == Off && pC->Drive.motspeedreftemp[0] < speed)
		pC->Drive.motspeedreftemp[0] += step;
	else if(pC->Drive.buttons[7] == Off && pC->Drive.buttons[6] == On && pC->Drive.motspeedreftemp[0] > -speed)
		pC->Drive.motspeedreftemp[0] -= step;
	else if(pC->Drive.buttons[7] == Off && pC->Drive.buttons[6] == Off)
		pC->Drive.motspeedreftemp[0] = 0.0;
	
	if(pC->Drive.buttons[5] == On && pC->Drive.buttons[4] == Off && pC->Drive.motspeedreftemp[1] < speed)
		pC->Drive.motspeedreftemp[1] += step;
	else if(pC->Drive.buttons[5] == Off && pC->Drive.buttons[4] == On && pC->Drive.motspeedreftemp[1] > -speed)
		pC->Drive.motspeedreftemp[1] -= step;
	else if(pC->Drive.buttons[5] == Off && pC->Drive.buttons[4] == Off)
		pC->Drive.motspeedreftemp[1] = 0.0;
	
	if(pC->Drive.buttons[2] == On && pC->Drive.buttons[3] == Off && pC->Drive.motspeedref[2] < speed)
		pC->Drive.motspeedreftemp[2] += step;
	else if(pC->Drive.buttons[2] == Off && pC->Drive.buttons[3] == On && pC->Drive.motspeedref[2] > -speed)
		pC->Drive.motspeedreftemp[2] -= step;
	else if(pC->Drive.buttons[2] == Off && pC->Drive.buttons[3] == Off)
		pC->Drive.motspeedreftemp[2] = 0.0;
	
	if(pC->Drive.buttons[1] == On && pC->Drive.buttons[0] == Off && pC->Drive.motspeedreftemp[3] < speed)
		pC->Drive.motspeedreftemp[3] += step;
	else if(pC->Drive.buttons[1] == Off && pC->Drive.buttons[0] == On && pC->Drive.motspeedreftemp[3] > -speed)
		pC->Drive.motspeedreftemp[3] -= step;
	else if(pC->Drive.buttons[1] == Off && pC->Drive.buttons[0] == Off)
		pC->Drive.motspeedreftemp[3] = 0.0;
	
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Drive.motspeedref[i] = pC->Drive.motspeedreftemp[i];
		pC->Drive.motpid[i] = Off;
	}
}
static void Drive_MotStop(void)
{
	for(uint8_t i=0;i<MOTMAX;i++)
		Drive_MotStopOneMot(i);
}
static void Drive_SerwAct(void)
{
	SERW0PWM_REG = pC->Serw.griprefpwm;
	SERW1PWM_REG = pC->Serw.openrefpwm;
	SERW2PWM_REG = pC->Serw.turnrefpwm;
}
static void Drive_ReadCmd(void)
{
	switch(pC->Com.cmd)
	{
		case cmdLab_null:																														break;
		case cmdLab_Remote:							pC->Mode.work = Workremote;									break;
		case cmdLab_MotSetHomePos:			Drive_MotSetHomePosition(pC->Com.cmdval);		break;
		case cmdLab_MotEnableLimits:		pC->Drive.motlimits[pC->Com.cmdval] = On;		break;
		case cmdLab_MotDisableLimits:		pC->Drive.motlimits[pC->Com.cmdval] = Off;	break;
		case cmdLab_MotEnablePID:				pC->Drive.motpid[pC->Com.cmdval] = On;			break;
		case cmdLab_MotDisablePID:			pC->Drive.motpid[pC->Com.cmdval] = Off;			break;
		case cmdLab_WibOn:							pC->Drive.wibstate = On; MOT4PWM_REG = 700; break;
		case cmdLab_WibOff:							pC->Drive.wibstate = Off; MOT4PWM_REG = 0;  break;
		case cmdLab_GripperOpen:
			pC->Serw.gripstate = Off;
			pC->Serw.griprefpwm = pC->Serw.grippwm[1];	
			break;
		case cmdLab_GripperClose:
			pC->Serw.gripstate = On;
			pC->Serw.griprefpwm = pC->Serw.grippwm[0];
			break;
		case cmdLab_LumiOpen:
			pC->Serw.openstate = Off;
			pC->Serw.openrefpwm = pC->Serw.openpwm[0];	
			break;
		case cmdLab_LumiClose:
			pC->Serw.openstate = On;
			pC->Serw.openrefpwm = pC->Serw.openpwm[1];
			break;
		case cmdLab_LumiStartMeas:
			pC->Serw.turnstate = LSMeas;
			pC->Mode.lumiserwtick = 0;
			pC->Serw.turnrefpwm = pC->Serw.turnpwm[2];
			break;
		case cmdLab_LumiNeutralPos:
			pC->Serw.turnstate = LSNeutral;
			pC->Serw.turnrefpwm = pC->Serw.turnpwm[0];
			break;
		case cmdLab_LumiTurnOnOff:
			pC->Serw.turnstate = LSOn;
			pC->Mode.lumiserwtick = 0;
			pC->Serw.turnrefpwm = pC->Serw.turnpwm[1];
			break;
		
		//20 to numer sekwencji dla sterowania semi auto
		case cmdLab_LumiHorizontal:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.poslumihoriz;
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_LumiVertical:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.poslumivert;
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;	
		break;
		
		case cmdLab_MoveZPosUp:
			pC->Stats.seq[20].s[0].speed = 90.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.poszup;
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;	
		break;
		
		case cmdLab_MoveZPosSwab:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.poszswab - 40.0;
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
		
			pC->Stats.seq[20].s[1].speed = 30.0;
			pC->Stats.seq[20].s[1].delay = 0;
			pC->Stats.seq[20].s[1].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[1].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[1].posref[2] = pC->Drive.poszswab;
			pC->Stats.seq[20].s[1].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[1].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[1].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[1].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 2;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;	
		break;
		
		case cmdLab_MoveZPosSwab2:
			pC->Stats.seq[20].s[0].speed = 30.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.poszswab2;
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_MoveZPosLumi:
			pC->Stats.seq[20].s[0].speed = 30.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.poszlumi;
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_MoveZPosMeas:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.poszmeas;
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;

		case cmdLab_MoveXYPosSwab:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.posxyswab[pC->Com.cmdval][0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.posxyswab[pC->Com.cmdval][1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
		
//			pC->Stats.seq[20].s[1].speed = 100.0;
//			pC->Stats.seq[20].s[1].delay = 1500;
//			pC->Stats.seq[20].s[1].posref[0] = pC->Drive.posxyswab[pC->Com.cmdval][0];
//			pC->Stats.seq[20].s[1].posref[1] = pC->Drive.posxyswab[pC->Com.cmdval][1];
//			pC->Stats.seq[20].s[1].posref[2] = pC->Drive.motposabs[2];
//			pC->Stats.seq[20].s[1].posref[3] = pC->Drive.motposabs[3];
//			pC->Stats.seq[20].s[1].pwmrefg = pC->Serw.griprefpwm;
//			pC->Stats.seq[20].s[1].pwmreft = pC->Serw.turnrefpwm;
//			pC->Stats.seq[20].s[1].pwmrefo = pC->Serw.openrefpwm;
		
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;			
		break;
		
		case cmdLab_MoveXYPosLumi:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.posxylumi[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.posxylumi[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
		
//			pC->Stats.seq[20].s[1].speed = 100.0;
//			pC->Stats.seq[20].s[1].delay = 1500;
//			pC->Stats.seq[20].s[1].posref[0] = pC->Drive.posxylumi[0];
//			pC->Stats.seq[20].s[1].posref[1] = pC->Drive.posxylumi[1];
//			pC->Stats.seq[20].s[1].posref[2] = pC->Drive.motposabs[2];
//			pC->Stats.seq[20].s[1].posref[3] = pC->Drive.motposabs[3];
//			pC->Stats.seq[20].s[1].pwmrefg = pC->Serw.griprefpwm;
//			pC->Stats.seq[20].s[1].pwmreft = pC->Serw.turnrefpwm;
//			pC->Stats.seq[20].s[1].pwmrefo = pC->Serw.openrefpwm;
		
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_MoveXYPosMeas:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.posxymeas[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.posxymeas[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_MoveXYWib:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.posxywib[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.posxywib[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
				
		case cmdLab_MoveZWib:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.poszwib;
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_MoveFotopos:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.motposabs[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.poslumifoto;
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 1;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_MoveXYTrow:
			pC->Stats.seq[20].s[0].speed = 100.0;
			pC->Stats.seq[20].s[0].delay = 0;
			pC->Stats.seq[20].s[0].posref[0] = pC->Drive.motposabs[0];
			pC->Stats.seq[20].s[0].posref[1] = pC->Drive.posxyswabtrow[1];
			pC->Stats.seq[20].s[0].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[0].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[0].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[0].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[0].pwmrefo = pC->Serw.openrefpwm;
		
			pC->Stats.seq[20].s[1].speed = 100.0;
			pC->Stats.seq[20].s[1].delay = 0;
			pC->Stats.seq[20].s[1].posref[0] = pC->Drive.posxyswabtrow[0];
			pC->Stats.seq[20].s[1].posref[1] = pC->Drive.posxyswabtrow[1];
			pC->Stats.seq[20].s[1].posref[2] = pC->Drive.motposabs[2];
			pC->Stats.seq[20].s[1].posref[3] = pC->Drive.motposabs[3];
			pC->Stats.seq[20].s[1].pwmrefg = pC->Serw.griprefpwm;
			pC->Stats.seq[20].s[1].pwmreft = pC->Serw.turnrefpwm;
			pC->Stats.seq[20].s[1].pwmrefo = pC->Serw.openrefpwm;
			pC->Stats.seq[20].smax = 2;
			pC->Stats.seq[20].snum = 0;
			pC->Stats.seqnum = 20;
			pC->Mode.work = Workauto;
		break;
		
		case cmdLab_AutoRunSeq:
			pC->Mode.work = Workauto;
			pC->Stats.seqnum = pC->Com.cmdval;
			pC->Stats.seq[pC->Stats.seqnum].snum = 0;
		break;
		
		case cmdLab_AutoNextPoint: 
			Drive_AutoNextPoint(); 
		break;
		
		case cmdLab_AutoPrevPoint: 
			Drive_AutoPreviousPoint(); 
		break;
	}
	pC->Com.cmd = cmdLab_null;
	pC->Com.cmdval = 0;
}
static void Drive_Act(void)
{
	Drive_ReadCmd();
	Drive_ReadMode();
	Drive_ReadButtons();
	Drive_MotReadPos();
	Drive_WriteBackup();
	Drive_MotReadCurrent();
	Led_OnOff(0, LedOff);
	if(pC->Mode.work == Workmanual)
	{
		Drive_MotManualAct();
		Drive_MotSetPosref();
		Drive_MotRegulator();
		Drive_ReadHardwareLimits();
		Drive_SerwAct();
		Led_OnOff(2, LedOn);
	}
	else if(pC->Mode.work == Workremote)
	{
		Drive_MotSetPosref();
		Drive_MotRegulator();
		Drive_SerwAct();
		Drive_ReadHardwareLimits();
		Drive_ReadSoftwareLimits();
		Led_OnOff(3, LedOn);
	}
	else if(pC->Mode.work == Workauto)
	{
		Drive_AutoSetPosref();
		Drive_MotRegulator();
		Drive_SerwAct();
		Drive_ReadHardwareLimits();
		Drive_ReadSoftwareLimits();
		Led_OnOff(4, LedOn);
	}
	else if(pC->Mode.work == Workoff)
	{
		Drive_MotStop();
		Drive_MotSetPosref();
		Drive_MotRegulator();
		Led_OnOff(5, LedOn);
	}
	Drive_MotPwmDirAct();
}
//----------------Przerwania--------------------------------------
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		Drive_Act();
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
