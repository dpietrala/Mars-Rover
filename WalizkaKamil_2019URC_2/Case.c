#include "Case.h"
extern sControl* pC;
static void Case_GpioConf(void)
{
	// nozki PG11-14
	GPIOG->MODER 	|= GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0;
	GPIOG->OTYPER |= GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14;
	
	//diody  D_SPI
	GPIOI->MODER 	|= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0;
	GPIOI->OTYPER |= GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3;
	GPIOH->MODER 	|= GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0;
	GPIOH->OTYPER |= GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15;
	
	//Diody w pudelku obok minutnika
	GPIOE->MODER |= GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOE->OTYPER |= GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5;
	GPIOD->MODER	|= GPIO_MODER_MODE7_0;
	GPIOD->OTYPER |= GPIO_OTYPER_OT7;
	GPIOG->MODER 	|= GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0;
	GPIOG->OTYPER |= GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10;
	
	LEDBase_OFF;
	LEDTool_OFF;
	LEDJoint_OFF;
	LEDMotor1_OFF;
	LEDMotor2_OFF;
	LEDMotor3_OFF;
	LEDMotor4_OFF;
	LEDMotor5_OFF;
	LEDMotor6_OFF;
	LEDMotor7_OFF;
	LEDRedButton_OFF;
	LEDAuto1_OFF;
	LEDAuto2_OFF;
	LEDWheel1_OFF;
	LEDWheel2_OFF;
	LEDWheel3_OFF;
	LEDWheel4_OFF;
	LEDImu_OFF;
	
	//buzzer
	GPIOD->MODER |= GPIO_MODER_MODE12_0;
	GPIOD->OTYPER |= GPIO_OTYPER_OT12;
	GPIOD->ODR |= GPIO_ODR_OD12;
	
	//Wszystkie przyciski na wejscia z pull up 52 kanalów
	GPIOB->MODER &= ~GPIO_MODER_MODE3 & ~GPIO_MODER_MODE4 & ~GPIO_MODER_MODE5 & ~GPIO_MODER_MODE6 & ~GPIO_MODER_MODE7 & ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE9;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0;
	GPIOC->MODER &= ~GPIO_MODER_MODE13 & ~GPIO_MODER_MODE14 & ~GPIO_MODER_MODE15;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;
	GPIOD->MODER &= ~GPIO_MODER_MODE15;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD15_0;
	
//	//Te nózki pracuja jako diody w pudelku obok minutnika
//	GPIOE->MODER &= ~GPIO_MODER_MODE2 & ~GPIO_MODER_MODE3 & ~GPIO_MODER_MODE4 & ~GPIO_MODER_MODE5;
//	GPIOE->PUPDR |= GPIO_PUPDR_PUPD2_0 | GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0;

	GPIOE->MODER &= ~GPIO_MODER_MODE0 & ~GPIO_MODER_MODE1 & ~GPIO_MODER_MODE6 &
									~GPIO_MODER_MODE7 & ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE9 & ~GPIO_MODER_MODE10 & ~GPIO_MODER_MODE11 & ~GPIO_MODER_MODE12 & ~GPIO_MODER_MODE13 & 
									~GPIO_MODER_MODE14 & ~GPIO_MODER_MODE15;
	GPIOE->PUPDR |= GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0 | GPIO_PUPDR_PUPD6_0 | 
									GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0  | GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD12_0 | GPIO_PUPDR_PUPD13_0 | 
									GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;
									
	GPIOF->MODER &= ~GPIO_MODER_MODE13 & ~GPIO_MODER_MODE14 & ~GPIO_MODER_MODE15;
	GPIOF->PUPDR |= GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;
	GPIOG->MODER &= ~GPIO_MODER_MODE0 & ~GPIO_MODER_MODE1 & ~GPIO_MODER_MODE2 & ~GPIO_MODER_MODE3 & ~GPIO_MODER_MODE4 & ~GPIO_MODER_MODE5 &
									~GPIO_MODER_MODE6 & ~GPIO_MODER_MODE7 & ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE15;
	GPIOG->PUPDR |= GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0 | GPIO_PUPDR_PUPD2_0 | GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0 | 
									GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD15_0;
	GPIOH->MODER &= ~GPIO_MODER_MODE6 & ~GPIO_MODER_MODE7 & ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE9 & ~GPIO_MODER_MODE10 & ~GPIO_MODER_MODE11;
	GPIOH->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
	GPIOI->MODER &= ~GPIO_MODER_MODE4 & ~GPIO_MODER_MODE5 & ~GPIO_MODER_MODE8 & ~GPIO_MODER_MODE9 & ~GPIO_MODER_MODE10 & ~GPIO_MODER_MODE11;
	GPIOI->PUPDR |= GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
}
static void Case_TimConf(void)
{
	//TIM3 Interrupt ********************************************************************************
	TIM3->PSC = 840-1;
	TIM3->ARR = 1000-1;	//Przerwanie co 10 ms
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM3_IRQn);
	
	//TIM7 Interrupt ********************************************************************************
	TIM7->PSC = 840-1;
	TIM7->ARR = 5000-1;	//Przerwanie co 50 ms
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
	
	//TIM4 Interrupt ********************************************************************************
	TIM4->PSC = 840-1;
	TIM4->ARR = 3000-1;	//Przerwanie co 300 ms
	TIM4->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM4_IRQn);
}
static void Case_AdcConf_1(void)
{
	//8 kanalów analogowych DMA2-Stream0
	GPIOA->MODER |= GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7; 
	GPIOC->MODER |= GPIO_MODER_MODER4 | GPIO_MODER_MODER5;
	GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;
	
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)pC->Case.adc1val;
	DMA2_Stream0->NDTR = (uint16_t)8*ADCMAX;
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	
	//ADC->CCR 	|= ADC_CCR_ADCPRE;
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->SMPR2 |= ADC_SMPR2_SMP9 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP7 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP4;
	ADC1->SMPR1 |= ADC_SMPR1_SMP15 | ADC_SMPR1_SMP14;
	ADC1->SQR1 |= 7<<20;
	ADC1->SQR3 |= (9<<0) | (8<<5) | (15<<10) | (14<<15) | (7<<20) | (6<<25);
	ADC1->SQR2 |= (5<<0) | (4<<5);
	
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
static void Case_AdcConf_2(void)
{
	//8 kanalów analogowych DMA2-Stream3
	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3; 
	GPIOC->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3;
		
	DMA2_Stream3->PAR = (uint32_t)&ADC2->DR; 
	DMA2_Stream3->M0AR = (uint32_t)pC->Case.adc2val;
	DMA2_Stream3->NDTR = (uint16_t)8*ADCMAX;
	DMA2_Stream3->CR |= DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN | DMA_SxCR_CHSEL_0;
	
	//ADC->CCR 	|= ADC_CCR_ADCPRE;
	ADC2->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC2->CR1 |= ADC_CR1_SCAN;
	ADC2->SMPR2 |= ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3;
	ADC2->SMPR1 |= ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13;
	ADC2->SQR1 |= 7<<20;
	ADC2->SQR3 |= (3<<0) | (2<<5) | (1<<10) | (0<<15) | (13<<20) | (12<<25);
	ADC2->SQR2 |= (11<<0) | (10<<5);
	
	ADC2->CR2 |= ADC_CR2_SWSTART;
}
static void Case_AdcConf_3(void)
{
	//8 kanalów analogowych DMA2-Stream1
	GPIOF->MODER |= GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10; 
		
	DMA2_Stream1->PAR = (uint32_t)&ADC3->DR;
	DMA2_Stream1->M0AR = (uint32_t)pC->Case.adc3val;
	DMA2_Stream1->NDTR = (uint16_t)8*ADCMAX;
	DMA2_Stream1->CR |= DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN | DMA_SxCR_CHSEL_1;
	
	//ADC->CCR 	|= ADC_CCR_ADCPRE;
	ADC3->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC3->CR1 |= ADC_CR1_SCAN;
	ADC3->SMPR2 |= ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;
	ADC3->SMPR1 |= ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15;
	ADC3->SQR1 |= 7<<20;
	ADC3->SQR3 |= (9<<0) | (14<<5) | (15<<10) | (4<<15) | (5<<20) | (6<<25);
	ADC3->SQR2 |= (7<<0) | (8<<5);
	
	ADC3->CR2 |= ADC_CR2_SWSTART;
}
static void Case_ADC_value(void)
{
	uint32_t suma_adc1[AI_CHMAX], suma_adc2[AI_CHMAX], suma_adc3[AI_CHMAX];
	for(uint8_t i=0;i<8;i++)
	{
		suma_adc1[i] = 0;
		suma_adc2[i] = 0;
		suma_adc3[i] = 0;
	}
	
	uint32_t index = 0;
	//ADC1
	for(uint16_t i=0;i<ADCMAX;i++)
		for(uint8_t j=0;j<AI_CHMAX;j++)
			suma_adc1[j] += pC->Case.adc1val[index++];
	
	for(uint8_t i=0;i<AI_CHMAX;i++)
		pC->Case.val_adc1[i] = (double)suma_adc1[i] / ADCMAX;
	
	//ADC2
	index = 0;
	for(uint16_t i=0;i<ADCMAX;i++)
		for(uint8_t j=0;j<AI_CHMAX;j++)
			suma_adc2[j] += pC->Case.adc2val[index++];
	
	for(uint8_t i=0;i<AI_CHMAX;i++)
		pC->Case.val_adc2[i] = (double)suma_adc2[i] / ADCMAX;
	
	//ADC3
	index = 0;
	for(uint16_t i=0;i<ADCMAX;i++)
		for(uint8_t j=0;j<AI_CHMAX;j++)
			suma_adc3[j] += pC->Case.adc3val[index++];
	
	for(uint8_t i=0;i<AI_CHMAX;i++)
		pC->Case.val_adc3[i] = (double)suma_adc3[i] / ADCMAX;
		
	uint8_t num = 0;
	for(uint8_t i=0;i<AI_CHMAX;i++)
		pC->Case.val_adc[num++] = pC->Case.val_adc1[i];
	for(uint8_t i=0;i<AI_CHMAX;i++)
		pC->Case.val_adc[num++] = pC->Case.val_adc2[i];
	for(uint8_t i=0;i<AI_CHMAX;i++)
		pC->Case.val_adc[num++] = pC->Case.val_adc3[i];
		
	for(uint8_t i=0;i<AI_ALLCHMAX;i++)
	{
		double rangeup = pC->Case.val_adc_max[i] - pC->Case.val_adc_mean[i];
		double rangedown = pC->Case.val_adc_mean[i] - pC->Case.val_adc_min[i];
		
		double val = pC->Case.val_adc[i] - pC->Case.val_adc_mean[i];
		if(pC->Case.val_adc[i] > pC->Case.val_adc_mean[i])
		{
			pC->Case.val_adc[i] = val / rangeup;
		}
		else if(pC->Case.val_adc[i] < pC->Case.val_adc_mean[i])
		{
			pC->Case.val_adc[i] = val / rangedown;
		}
		
		if(pC->Case.val_adc[i] > 1.0)
			pC->Case.val_adc[i] = 1.0;
		else if(pC->Case.val_adc[i] < -1.0)
			pC->Case.val_adc[i] = -1.0;
	}
	
	pC->Case.manip_joy_x = ((100.0 + pC->Case.hyst[AI_MJ_X]) * pC->Case.val_adc[AI_MJ_X]);
	if(pC->Case.manip_joy_x > pC->Case.hyst[AI_MJ_X])
		pC->Case.manip_joy_x -= pC->Case.hyst[AI_MJ_X];
	else if(pC->Case.manip_joy_x < -pC->Case.hyst[AI_MJ_X])
		pC->Case.manip_joy_x += pC->Case.hyst[AI_MJ_X];
	else
		pC->Case.manip_joy_x = 0.0;
	
	pC->Case.manip_joy_y = ((100.0 + pC->Case.hyst[AI_MJ_Y]) * pC->Case.val_adc[AI_MJ_Y]);
	if(pC->Case.manip_joy_y > pC->Case.hyst[AI_MJ_Y])
		pC->Case.manip_joy_y -= pC->Case.hyst[AI_MJ_Y];
	else if(pC->Case.manip_joy_y < -pC->Case.hyst[AI_MJ_Y])
		pC->Case.manip_joy_y += pC->Case.hyst[AI_MJ_Y];
	else
		pC->Case.manip_joy_y = 0.0;
	
	pC->Case.manip_joy_z = ((100.0 + pC->Case.hyst[AI_MJ_Z]) * pC->Case.val_adc[AI_MJ_Z]);
	if(pC->Case.manip_joy_z > pC->Case.hyst[AI_MJ_Z])
		pC->Case.manip_joy_z -= pC->Case.hyst[AI_MJ_Z];
	else if(pC->Case.manip_joy_z < -pC->Case.hyst[AI_MJ_Z])
		pC->Case.manip_joy_z += pC->Case.hyst[AI_MJ_Z];
	else
		pC->Case.manip_joy_z = 0.0;
	
	pC->Case.manip_joy_Rx = ((100.0 + pC->Case.hyst[AI_MJ_RX]) * pC->Case.val_adc[AI_MJ_RX]);
	if(pC->Case.manip_joy_Rx > pC->Case.hyst[AI_MJ_RX])
		pC->Case.manip_joy_Rx -= pC->Case.hyst[AI_MJ_RX];
	else if(pC->Case.manip_joy_Rx < -pC->Case.hyst[AI_MJ_RX])
		pC->Case.manip_joy_Rx += pC->Case.hyst[AI_MJ_RX];
	else
		pC->Case.manip_joy_Rx = 0.0;
	
	pC->Case.manip_joy_Ry = ((100.0 + pC->Case.hyst[AI_MJ_RY]) * pC->Case.val_adc[AI_MJ_RY]);
	if(pC->Case.manip_joy_Ry > pC->Case.hyst[AI_MJ_RY])
		pC->Case.manip_joy_Ry -= pC->Case.hyst[AI_MJ_RY];
	else if(pC->Case.manip_joy_Ry < -pC->Case.hyst[AI_MJ_RY])
		pC->Case.manip_joy_Ry += pC->Case.hyst[AI_MJ_RY];
	else
		pC->Case.manip_joy_Ry = 0.0;
	
	pC->Case.manip_joy_Rz = ((100.0 + pC->Case.hyst[AI_MJ_RZ]) * pC->Case.val_adc[AI_MJ_RZ]);
	if(pC->Case.manip_joy_Rz > pC->Case.hyst[AI_MJ_RZ])
		pC->Case.manip_joy_Rz -= pC->Case.hyst[AI_MJ_RZ];
	else if(pC->Case.manip_joy_Rz < -pC->Case.hyst[AI_MJ_RZ])
		pC->Case.manip_joy_Rz += pC->Case.hyst[AI_MJ_RZ];
	else
		pC->Case.manip_joy_Rz = 0.0;

	pC->Case.manip_x_speed = (int8_t)(50.0 * (1.0 + pC->Case.val_adc[AI_MP_X]));
	pC->Case.manip_y_speed = (int8_t)(50.0 * (1.0 + pC->Case.val_adc[AI_MP_Y]));
	pC->Case.manip_z_speed = (int8_t)(50.0 * (1.0 + pC->Case.val_adc[AI_MP_Z]));
	pC->Case.manip_Rx_speed = (int8_t)(50.0 * (1.0 + pC->Case.val_adc[AI_MP_RX]));
	pC->Case.manip_Ry_speed = (int8_t)(50.0 * (1.0 + pC->Case.val_adc[AI_MP_RY]));
	pC->Case.manip_Rz_speed = (int8_t)(50.0 * (1.0 + pC->Case.val_adc[AI_MP_RZ]));
	
	pC->Case.gripper_speed = 50.0 * (1.0 + pC->Case.val_adc[AI_M_GS]);
	
	pC->Case.dirspeed_L = (100.0 + pC->Case.hyst[AI_D_DSL]) * pC->Case.val_adc[AI_D_DSL];
	if(pC->Case.dirspeed_L > pC->Case.hyst[AI_D_DSL])						pC->Case.dirspeed_L -= pC->Case.hyst[AI_D_DSL];
	else if(pC->Case.dirspeed_L < -pC->Case.hyst[AI_D_DSL])			pC->Case.dirspeed_L += pC->Case.hyst[AI_D_DSL];
	else																												pC->Case.dirspeed_L = 0.0;
	
	pC->Case.frontspeed_R = (100.0 + pC->Case.hyst[AI_D_FSR]) * pC->Case.val_adc[AI_D_FSR];
	if(pC->Case.frontspeed_R > pC->Case.hyst[AI_D_FSR])					pC->Case.frontspeed_R -= pC->Case.hyst[AI_D_FSR];
	else if(pC->Case.frontspeed_R < -pC->Case.hyst[AI_D_FSR])		pC->Case.frontspeed_R += pC->Case.hyst[AI_D_FSR];
	else																												pC->Case.frontspeed_R = 0.0;
	
	
	double gup1 = 100.0 * pC->Case.val_adc[AI_GUP_0] - pC->Case.frontspeed_R;
	if(gup1 > 100.)				gup1 = 100.;
	else if(gup1 < -100.)	gup1 = -100.;
	double gup0 = 100.0 * pC->Case.val_adc[AI_GUP_1] - pC->Case.dirspeed_L;
	if(gup0 > 100.)				gup0 = 100.;
	else if(gup0 < -100.)	gup0 = -100.;
	double gdown0 = 100.0 * pC->Case.val_adc[AI_GDW_0];
	double gdown1 = 100.0 * pC->Case.val_adc[AI_GDW_1];
	double spos = 100.0 * pC->Case.val_adc[AI_D_SP];
	double range = 2.;
	for(int i=-95;i<=95;i+=10)
	{
		if(gup0 >= (i - range) && gup0 <= (i + range))					pC->Case.gimbal_up[0] = (double)i;
		if(gup1 >= (i - range) && gup1 <= (i + range))					pC->Case.gimbal_up[1] = (double)i;
		if(gdown0 >= (i - range) && gdown0 <= (i + range))			pC->Case.gimbal_down[0] = (double)i;
		if(gdown1 >= (i - range) && gdown1 <= (i + range))			pC->Case.gimbal_down[1] = (double)i;
		
		pC->Case.station_pos_prev = pC->Case.station_pos;
		if(spos >= (i - range) && spos <= (i + range))					pC->Case.station_pos = (double)i;
		if(pC->Case.station_pos != pC->Case.station_pos_prev)		pC->Com.frames[Fnum_SP].rfull = On;
	}
	
	pC->Case.gimbal_up[0] = gup1;
	pC->Case.gimbal_up[1] = gup0;
	pC->Case.gimbal_down[0] = gdown0;
	pC->Case.gimbal_down[1] = gdown1;
	
	pC->Case.speed_L = 0.5 * (1.0 + pC->Case.val_adc[AI_D_SL]);
	pC->Case.speed_R = 0.5 * (1.0 + pC->Case.val_adc[AI_D_SR]);
	
	pC->Case.frontspeed_L = ((100.0 + pC->Case.hyst[AI_D_FSL]) * pC->Case.val_adc[AI_D_FSL]);
	if(pC->Case.frontspeed_L > pC->Case.hyst[AI_D_FSL])
		pC->Case.frontspeed_L -= pC->Case.hyst[AI_D_FSL];
	else if(pC->Case.frontspeed_L < -pC->Case.hyst[AI_D_FSL])
		pC->Case.frontspeed_L += pC->Case.hyst[AI_D_FSL];
	else
		pC->Case.frontspeed_L = 0.0;
	pC->Case.frontspeed_L *= pC->Case.speed_L;
	
	pC->Case.dirspeed_R = ((100.0 + pC->Case.hyst[AI_D_DSR]) * pC->Case.val_adc[AI_D_DSR]);
	if(pC->Case.dirspeed_R > pC->Case.hyst[AI_D_DSR])
		pC->Case.dirspeed_R -= pC->Case.hyst[AI_D_DSR];
	else if(pC->Case.dirspeed_R < -pC->Case.hyst[AI_D_DSR])
		pC->Case.dirspeed_R += pC->Case.hyst[AI_D_DSR];
	else
		pC->Case.dirspeed_R = 0.0;
	 pC->Case.dirspeed_R *= pC->Case.speed_R;
	
	//Dostosowanie kierunków
	pC->Case.manip_joy_x *= -1;
	pC->Case.manip_joy_Rx *= -1;
	pC->Case.frontspeed_L *= -1;
	pC->Case.manip_y_speed = 100 - pC->Case.manip_y_speed;
	pC->Case.manip_z_speed = 100 - pC->Case.manip_z_speed;
	pC->Case.manip_Rx_speed = 100 - pC->Case.manip_Rx_speed;
	pC->Case.manip_Ry_speed = 100 - pC->Case.manip_Ry_speed;
	pC->Case.gimbal_up[0] *= -1.;
	pC->Case.gimbal_up[1] *= -1.;
	pC->Case.gimbal_down[0] *= -1.;
	pC->Case.gimbal_down[1] *= -1.;
}
void Case_Conf(void)
{
	Case_GpioConf();
	Case_TimConf();
	Case_AdcConf_1();
	Case_AdcConf_2();
	Case_AdcConf_3();
}
void buzzer_on(void)
{
	Buzzer_ON;
	TIM4->CR1 |= TIM_CR1_CEN;
}
void buzzer_off(void)
{
	Buzzer_OFF;
	TIM4->CR1 &= ~TIM_CR1_CEN;
	TIM4->CNT = 0;
}
static void Button_Set(void)
{
	//przyciski manipulator
	if((ManipX_LIdr) == RESET)		pC->Case.manip_x_L = On;
	else													pC->Case.manip_x_L = Off;	
	if((ManipX_RIdr) == RESET)		pC->Case.manip_x_R = On;
	else													pC->Case.manip_x_R = Off;
	if((ManipY_LIdr) == RESET)		pC->Case.manip_y_L = On;
	else													pC->Case.manip_y_L = Off;
	if((ManipY_RIdr) == RESET)		pC->Case.manip_y_R = On;
	else													pC->Case.manip_y_R = Off;
	if((ManipZ_LIdr) == RESET)		pC->Case.manip_z_L = On;
	else													pC->Case.manip_z_L = Off;
	if((ManipZ_RIdr) == RESET)		pC->Case.manip_z_R = On;
	else													pC->Case.manip_z_R = Off;
	if((ManipRX_LIdr) == RESET)		pC->Case.manip_Rx_L = On;
	else													pC->Case.manip_Rx_L = Off;
	if((ManipRX_RIdr) == RESET)		pC->Case.manip_Rx_R = On;
	else													pC->Case.manip_Rx_R = Off;
	if((ManipRY_LIdr) == RESET)		pC->Case.manip_Ry_L = On;
	else													pC->Case.manip_Ry_L = Off;
	if((ManipRY_RIdr) == RESET)		pC->Case.manip_Ry_R = On;
	else													pC->Case.manip_Ry_R = Off;
	if((ManipRZ_LIdr) == RESET)		pC->Case.manip_Rz_L = On;
	else													pC->Case.manip_Rz_L = Off;
	if((ManipRZ_RIdr) == RESET)		pC->Case.manip_Rz_R = On;
	else													pC->Case.manip_Rz_R = Off;

	//pudelko z ukladami 
	if((ManipHomeIdr) == RESET)
	{
		pC->Case.home = On;
		buzzer_on();
	}
	else
		pC->Case.home = Off;
	
	if((ManipParkIdr) == RESET)
	{
		pC->Case.park = On;
		buzzer_on();
	}
	else
		pC->Case.park = Off;
	
	if((ManipTakeIdr) == RESET)
	{
		pC->Case.take = On;
		buzzer_on();
	}
	else
		pC->Case.take = Off;
	
	if((ManipWorkIdr) == RESET)
	{
		pC->Case.work = On;
		buzzer_on();
	}
	else
		pC->Case.work = Off;
	
	if((ManipJointIdr) == RESET)															
	{
		pC->Case.joint = On;
		pC->Tele.manipcs = ManipJoin;
		buzzer_on();
	}
	else																																		
		pC->Case.joint = Off;	
	
	if((ManipToolIdr) == RESET)															
	{
		pC->Case.tool = On;
		pC->Tele.manipcs = ManipTool;
		buzzer_on();
	}
	else																																		
		pC->Case.tool = Off;
	
	if((ManipBaseIdr) == RESET)															
	{
		pC->Case.base = On;
		pC->Tele.manipcs = ManipBase;
		buzzer_on();
	}
	else																																		
		pC->Case.base = Off;
	
	if((ManipCalibIdr) == RESET)															
	{
		pC->Case.calibration = On;
		buzzer_on();
	}
	else																																		
		pC->Case.calibration = Off;

	//glowne pudelko
	if((ManipLaserOnIdr) == RESET)															
	{
		pC->Case.laseron = On;
		buzzer_on();
	}
	else																																		
		pC->Case.laseron = Off;
	
	if((ManipLaserOffIdr) == RESET)															
	{
		pC->Case.laseroff = On;
		buzzer_on();
	}
	else																																		
		pC->Case.laseroff = Off;
	
	if((GripperOpenIdr) == RESET)															
	{
		pC->Case.gripper_open = On;
		buzzer_on();
	}
	else																																		
		pC->Case.gripper_open = Off;
	
	if((GripperCloseIdr) == RESET)															
	{
		pC->Case.gripper_close = On;
		buzzer_on();
	}
	else																																		
		pC->Case.gripper_close = Off;
	
	if((ManipMagnesOnIdr) == RESET)															
	{
		pC->Case.magneson = On;
		buzzer_on();
	}
	else																																		
		pC->Case.magneson = Off;
	
	if((ManipMagnesOffIdr) == RESET)															
	{
		pC->Case.magnesoff = On;
		buzzer_on();
	}
	else																																		
		pC->Case.magnesoff = Off;
	
	//komunikacja
	if((WifiIdr) == RESET)																											
	{
		pC->Case.select_wifi = On;
		buzzer_on();
	}
	else																																		
		pC->Case.select_wifi = Off;
	
	if((SatelIdr) == RESET)																										
	{
		pC->Case.select_satel = On;
		buzzer_on();
	}
	else																																		
		pC->Case.select_satel = Off;
	
	if((AutostmIdr) == RESET)																									
	{
		pC->Case.autostm = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autostm = Off;
	
	if((AutonvidiaIdr) == RESET)																							  
	{
		pC->Case.autonvidia = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autonvidia = Off;

	if((AutopauseIdr) == RESET)
	{
		pC->Case.autopause = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autopause = Off;
	
	if((AutocontinueIdr) == RESET)
	{
		pC->Case.autocontinue = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autocontinue = Off;
		
	if((AutonextIdr) == RESET && pC->Case.autonext_prev == Off)						
	{
		pC->Case.autonext = On;
		pC->Case.autonext_prev = On;
		buzzer_on();
	}
	else if((AutonextIdr) != RESET)																		
	{
		pC->Case.autonext = Off;
		pC->Case.autonext_prev = Off;
	}
	else
		pC->Case.autonext = Off;
	
	if((DriveTelemAllEnIdr) == RESET)																					
	{
		pC->Case.driveTelemAllEn = On;
		buzzer_on();
	}
	else																																		
		pC->Case.driveTelemAllEn = Off;
	
	if((DriveTelemAllDisIdr) == RESET)																					
	{
		pC->Case.driveTelemAllDis = On;
		buzzer_on();
	}
	else																																		
		pC->Case.driveTelemAllDis = Off;
	
	if((ManipTelemAllEnIdr) == RESET)																					
	{
		pC->Case.manipTelemAllEn = On;
		buzzer_on();
	}
	else																																		
		pC->Case.manipTelemAllEn = Off;
	
	if((ManipTelemAllDisIdr) == RESET)																					
	{
		pC->Case.manipTelemAllDis = On;
		buzzer_on();
	}
	else																																		
		pC->Case.manipTelemAllDis = Off;
	
	if((Resetdev1Idr) == RESET)																								
	{
		pC->Case.resetdev1 = On;
		buzzer_on();
	}
	else																																		
		pC->Case.resetdev1 = Off;
	
	if((Resetdev2Idr) == RESET)																							  
	{
		pC->Case.resetdev2 = On;
		buzzer_on();
	}
	else																																		
		pC->Case.resetdev2 = Off;

	//grzyb
	if((RedButtonOnIdr) == RESET)
	{
		pC->Case.EmergencySwitchEn = On;
	}
	else
		pC->Case.EmergencySwitchEn = Off;
	
	if((RedButtonOffIdr) == RESET)																							  
	{
		pC->Case.EmergencySwitchDis = On;
	}
	else
		pC->Case.EmergencySwitchDis = Off;
		

	//kamery
	if((Cam1NextIdr) == RESET && pC->Case.select_cam1_next_prev == Off)						
	{
		pC->Case.select_cam1_next = On;
		pC->Case.select_cam1_next_prev = On;
		buzzer_on();
	}
	else if((Cam1NextIdr) != RESET)																		
	{
		pC->Case.select_cam1_next = Off;
		pC->Case.select_cam1_next_prev = Off;
	}
	else
		pC->Case.select_cam1_next = Off;
	
	if((Cam1BackIdr) == RESET && pC->Case.select_cam1_back_prev == Off)						
	{
		pC->Case.select_cam1_back = On;
		pC->Case.select_cam1_back_prev = On;
		buzzer_on();
	}
	else if((Cam1BackIdr) != RESET)																		
	{
		pC->Case.select_cam1_back = Off;
		pC->Case.select_cam1_back_prev = Off;
	}
	else
		pC->Case.select_cam1_back = Off;
	
	if((Cam2NextIdr) == RESET && pC->Case.select_cam2_next_prev == Off)						
	{
		pC->Case.select_cam2_next = On;
		pC->Case.select_cam2_next_prev = pC->Case.select_cam2_next;
		buzzer_on();
	}
	else if((Cam2NextIdr) != RESET)																		
	{
		pC->Case.select_cam2_next = Off;
		pC->Case.select_cam2_next_prev = pC->Case.select_cam2_next;
	}
	else
		pC->Case.select_cam2_next = Off;
	
	if((Cam2BackIdr) == RESET && pC->Case.select_cam2_back_prev == Off)						
	{
		pC->Case.select_cam2_back = On;
		pC->Case.select_cam2_back_prev = pC->Case.select_cam2_back;
		buzzer_on();
	}
	else if((Cam2BackIdr) != RESET)																		
	{
		pC->Case.select_cam2_back = Off;
		pC->Case.select_cam2_back_prev = pC->Case.select_cam2_back;
	}
	else
		pC->Case.select_cam2_back = Off;
}
static void Button_ReadDebouncing(void)
{
	//przyciski manipulator
	if((ManipX_LIdr) == RESET)		pC->Case.manip_x_L_num++;
	else													pC->Case.manip_x_L_num = 0;	
	if((ManipX_RIdr) == RESET)		pC->Case.manip_x_R_num++;
	else													pC->Case.manip_x_R_num = 0;
	if((ManipY_LIdr) == RESET)		pC->Case.manip_y_L_num++;
	else													pC->Case.manip_y_L_num = 0;
	if((ManipY_RIdr) == RESET)		pC->Case.manip_y_R_num++;
	else													pC->Case.manip_y_R_num = 0;
	if((ManipZ_LIdr) == RESET)		pC->Case.manip_z_L_num++;
	else													pC->Case.manip_z_L_num = 0;
	if((ManipZ_RIdr) == RESET)		pC->Case.manip_z_R_num++;
	else													pC->Case.manip_z_R_num = 0;
	if((ManipRX_LIdr) == RESET)		pC->Case.manip_Rx_L_num++;
	else													pC->Case.manip_Rx_L_num = 0;
	if((ManipRX_RIdr) == RESET)		pC->Case.manip_Rx_R_num++;
	else													pC->Case.manip_Rx_R_num = 0;
	if((ManipRY_LIdr) == RESET)		pC->Case.manip_Ry_L_num++;
	else													pC->Case.manip_Ry_L_num = 0;
	if((ManipRY_RIdr) == RESET)		pC->Case.manip_Ry_R_num++;
	else													pC->Case.manip_Ry_R_num = 0;
	if((ManipRZ_LIdr) == RESET)		pC->Case.manip_Rz_L_num++;
	else													pC->Case.manip_Rz_L_num = 0;
	if((ManipRZ_RIdr) == RESET)		pC->Case.manip_Rz_R_num++;
	else													pC->Case.manip_Rz_R_num = 0;

	//pudelko z ukladami 
	if((ManipHomeIdr) == RESET)
	{
		pC->Case.home_num++;
		buzzer_on();
	}
	else
		pC->Case.home_num = 0;
	
	if((ManipParkIdr) == RESET)
	{
		pC->Case.park_num++;
		buzzer_on();
	}
	else
		pC->Case.park_num = 0;
	
	if((ManipTakeIdr) == RESET)
	{
		pC->Case.take_num++;
		buzzer_on();
	}
	else
		pC->Case.take_num = 0;
	
	if((ManipWorkIdr) == RESET)
	{
		pC->Case.work_num++;
		buzzer_on();
	}
	else
		pC->Case.work_num = 0;
	
	if((ManipJointIdr) == RESET)															
	{
		pC->Case.joint_num++;
		pC->Tele.manipcs = ManipJoin;
		buzzer_on();
	}
	else																																		
		pC->Case.joint_num = 0;	
	
	if((ManipToolIdr) == RESET)															
	{
		pC->Case.tool_num++;
		pC->Tele.manipcs = ManipTool;
		buzzer_on();
	}
	else																																		
		pC->Case.tool_num = 0;
	
	if((ManipBaseIdr) == RESET)															
	{
		pC->Case.base_num++;
		pC->Tele.manipcs = ManipBase;
		buzzer_on();
	}
	else																																		
		pC->Case.base_num = 0;
	
	if((ManipCalibIdr) == RESET)															
	{
		pC->Case.calibration_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.calibration_num = 0;

	//glowne pudelko
	if((ManipLaserOnIdr) == RESET)															
	{
		pC->Case.laseron_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.laseron_num = 0;
	
	if((ManipLaserOffIdr) == RESET)															
	{
		pC->Case.laseroff_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.laseroff_num = 0;
	
	if((GripperOpenIdr) == RESET)															
	{
		pC->Case.gripper_open_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.gripper_open_num = 0;
	
	if((GripperCloseIdr) == RESET)															
	{
		pC->Case.gripper_close_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.gripper_close_num = 0;
	
	if((ManipMagnesOnIdr) == RESET)															
	{
		pC->Case.magneson_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.magneson_num = 0;
	
	if((ManipMagnesOffIdr) == RESET)															
	{
		pC->Case.magnesoff_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.magnesoff_num = 0;
	
	//komunikacja
	if((WifiIdr) == RESET)																											
	{
		pC->Case.select_wifi_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.select_wifi_num = 0;
	
	if((SatelIdr) == RESET)																										
	{
		pC->Case.select_satel_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.select_satel_num = 0;
	
	if((AutostmIdr) == RESET)																									
	{
		pC->Case.autostm_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.autostm_num = 0;
	
	if((AutonvidiaIdr) == RESET)																							  
	{
		pC->Case.autonvidia_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.autonvidia_num = 0;

	if((AutopauseIdr) == RESET)
	{
		pC->Case.autopause_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.autopause_num = 0;
	
	if((AutocontinueIdr) == RESET)
	{
		pC->Case.autocontinue_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.autocontinue_num = 0;
	
	if((DriveTelemAllEnIdr) == RESET)																					
	{
		pC->Case.driveTelemAllEn_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.driveTelemAllEn_num = 0;
	
	if((DriveTelemAllDisIdr) == RESET)																					
	{
		pC->Case.driveTelemAllDis_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.driveTelemAllDis_num = 0;
	
	if((ManipTelemAllEnIdr) == RESET)																					
	{
		pC->Case.manipTelemAllEn_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.manipTelemAllEn_num = 0;
	
	if((ManipTelemAllDisIdr) == RESET)																					
	{
		pC->Case.manipTelemAllDis_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.manipTelemAllDis_num = 0;
	
	if((Resetdev1Idr) == RESET)																								
	{
		pC->Case.resetdev1_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.resetdev1_num = 0;
	
	if((Resetdev2Idr) == RESET)																							  
	{
		pC->Case.resetdev2_num++;
		buzzer_on();
	}
	else																																		
		pC->Case.resetdev2_num = 0;

	//grzyb
	if((RedButtonOnIdr) == RESET)
	{
		pC->Case.EmergencySwitchEn_num++;
	}
	else
		pC->Case.EmergencySwitchEn_num = 0;
	
	if((RedButtonOffIdr) == RESET)																							  
	{
		pC->Case.EmergencySwitchDis_num++;
	}
	else
		pC->Case.EmergencySwitchDis_num = 0;
}
static void Button_SetDebouncing(void)
{
	if(pC->Case.manip_x_L_num >= BUTDEBMAX)	{pC->Case.manip_x_L = On;pC->Case.manip_x_L_num = 0;}
	else																		pC->Case.manip_x_L = Off;
	if(pC->Case.manip_x_R_num >= BUTDEBMAX)	{pC->Case.manip_x_R = On;pC->Case.manip_x_R_num = 0;}
	else																		pC->Case.manip_x_R = Off;
	if(pC->Case.manip_y_L_num >= BUTDEBMAX)	{pC->Case.manip_y_L = On;pC->Case.manip_y_L_num = 0;}
	else																		pC->Case.manip_y_L = Off;
	if(pC->Case.manip_y_R_num >= BUTDEBMAX)	{pC->Case.manip_y_R = On;pC->Case.manip_y_R_num = 0;}
	else																		pC->Case.manip_y_R = Off;
	if(pC->Case.manip_z_L_num >= BUTDEBMAX)	{pC->Case.manip_z_L = On;pC->Case.manip_z_L_num = 0;}
	else																		pC->Case.manip_z_L = Off;
	if(pC->Case.manip_z_R_num >= BUTDEBMAX)	{pC->Case.manip_z_R = On;pC->Case.manip_z_R_num = 0;}
	else																		pC->Case.manip_z_R = Off;
	if(pC->Case.manip_Rx_L_num >= BUTDEBMAX){pC->Case.manip_Rx_L = On;pC->Case.manip_Rx_L_num = 0;}
	else																		pC->Case.manip_Rx_L = Off;
	if(pC->Case.manip_Rx_R_num >= BUTDEBMAX){pC->Case.manip_Rx_R = On;pC->Case.manip_Rx_R_num = 0;}
	else																		pC->Case.manip_Rx_R = Off;
	if(pC->Case.manip_Ry_L_num >= BUTDEBMAX){pC->Case.manip_Ry_L = On;pC->Case.manip_Ry_L_num = 0;}
	else																		pC->Case.manip_Ry_L = Off;
	if(pC->Case.manip_Ry_R_num >= BUTDEBMAX){pC->Case.manip_Ry_R = On;pC->Case.manip_Ry_R_num = 0;}
	else																		pC->Case.manip_Ry_R = Off;
	if(pC->Case.manip_Rz_L_num >= BUTDEBMAX){pC->Case.manip_Rz_L = On;pC->Case.manip_Rz_L_num = 0;}
	else																		pC->Case.manip_Rz_L = Off;
	if(pC->Case.manip_Rz_R_num >= BUTDEBMAX){pC->Case.manip_Rz_R = On;pC->Case.manip_Rz_R_num = 0;}
	else																		pC->Case.manip_Rz_R = Off;
	
	//pudelko z ukladami 
	if(pC->Case.home_num >= BUTDEBMAX)			
	{
		pC->Case.home_num = 0;
		pC->Case.home = On;
		buzzer_on();
	}
	else
		pC->Case.home = Off;
	
	if(pC->Case.park_num >= BUTDEBMAX)
	{
		pC->Case.park_num = 0;
		pC->Case.park = On;
		buzzer_on();
	}
	else
		pC->Case.park = Off;
	
	if(pC->Case.take_num >= BUTDEBMAX)
	{
		pC->Case.take_num = 0;
		pC->Case.take = On;
		buzzer_on();
	}
	else
		pC->Case.take = Off;
	
	if(pC->Case.work_num >= BUTDEBMAX)
	{
		pC->Case.work_num = 0;
		pC->Case.work = On;
		buzzer_on();
	}
	else
		pC->Case.work = Off;
	
	if(pC->Case.joint_num >= BUTDEBMAX)															
	{
		pC->Case.joint_num = 0;
		pC->Case.joint = On;
		pC->Tele.manipcs = ManipJoin;
		buzzer_on();
	}
	else																																		
		pC->Case.joint = Off;	
	
	if(pC->Case.tool_num >= BUTDEBMAX)															
	{
		pC->Case.tool_num = 0;
		pC->Case.tool = On;
		pC->Tele.manipcs = ManipTool;
		buzzer_on();
	}
	else																																		
		pC->Case.tool = Off;
	
	if(pC->Case.base_num >= BUTDEBMAX)															
	{
		pC->Case.base_num = 0;
		pC->Case.base = On;
		pC->Tele.manipcs = ManipBase;
		buzzer_on();
	}
	else																																		
		pC->Case.base = Off;
	
	if(pC->Case.calibration_num >= BUTDEBMAX)															
	{
		pC->Case.calibration_num = 0;
		pC->Case.calibration = On;
		buzzer_on();
	}
	else																																		
		pC->Case.calibration = Off;

	//glowne pudelko
	if(pC->Case.laseron_num >= BUTDEBMAX)															
	{
		pC->Case.laseron_num = 0;
		pC->Case.laseron = On;
		buzzer_on();
	}
	else																																		
		pC->Case.laseron = Off;
	
	if(pC->Case.laseroff_num >= BUTDEBMAX)															
	{
		pC->Case.laseroff_num = 0;
		pC->Case.laseroff = On;
		buzzer_on();
	}
	else																																		
		pC->Case.laseroff = Off;
	
	if(pC->Case.gripper_open_num >= BUTDEBMAX)															
	{
		pC->Case.gripper_open_num = 0;
		pC->Case.gripper_open = Off;
		buzzer_on();
	}
	else																																		
		pC->Case.gripper_open = Off;
	
	if(pC->Case.gripper_close_num >= BUTDEBMAX)															
	{
		pC->Case.gripper_close_num = 0;
		pC->Case.gripper_close = On;
		buzzer_on();
	}
	else																																		
		pC->Case.gripper_close = Off;
	
	if(pC->Case.magneson_num >= BUTDEBMAX)															
	{
		pC->Case.magneson_num = 0;
		pC->Case.magneson = On;
		buzzer_on();
	}
	else																																		
		pC->Case.magneson = Off;
	
	if(pC->Case.magnesoff_num >= BUTDEBMAX)															
	{
		pC->Case.magnesoff_num = 0;
		pC->Case.magnesoff = On;
		buzzer_on();
	}
	else																																		
		pC->Case.magnesoff = Off;
	
	//komunikacja
	if(pC->Case.select_wifi_num >= BUTDEBMAX)																											
	{
		pC->Case.select_wifi_num = 0;
		pC->Case.select_wifi = On;
		buzzer_on();
	}
	else																																		
		pC->Case.select_wifi = Off;
	
	if(pC->Case.select_satel_num >= BUTDEBMAX)																										
	{
		pC->Case.select_satel_num = 0;
		pC->Case.select_satel = On;
		buzzer_on();
	}
	else																																		
		pC->Case.select_satel = Off;
	
	if(pC->Case.autostm_num >= BUTDEBMAX)																									
	{
		pC->Case.autostm_num = 0;
		pC->Case.autostm = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autostm = Off;
	
	if(pC->Case.autonvidia_num >= BUTDEBMAX)																							  
	{
		pC->Case.autonvidia_num = 0;
		pC->Case.autonvidia = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autonvidia = Off;

	if(pC->Case.autopause_num >= BUTDEBMAX)
	{
		pC->Case.autopause_num = 0;
		pC->Case.autopause = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autopause = Off;
	
	if(pC->Case.autocontinue_num >= BUTDEBMAX)
	{
		pC->Case.autocontinue_num = 0;
		pC->Case.autocontinue = On;
		buzzer_on();
	}
	else																																		
		pC->Case.autocontinue = Off;
	
	if(pC->Case.driveTelemAllEn_num >= BUTDEBMAX)																					
	{
		pC->Case.driveTelemAllEn_num = 0;
		pC->Case.driveTelemAllEn = On;
		buzzer_on();
	}
	else																																		
		pC->Case.driveTelemAllEn = On;
	
	if(pC->Case.driveTelemAllDis_num >= BUTDEBMAX)																					
	{
		pC->Case.driveTelemAllDis_num = 0;
		pC->Case.driveTelemAllDis = On;
		buzzer_on();
	}
	else																																		
		pC->Case.driveTelemAllDis = Off;
	
	if(pC->Case.manipTelemAllEn_num >= BUTDEBMAX)																					
	{
		pC->Case.manipTelemAllEn_num = 0;
		pC->Case.manipTelemAllEn = On;
		buzzer_on();
	}
	else																																		
		pC->Case.manipTelemAllEn = Off;
	
	if(pC->Case.manipTelemAllDis_num >= BUTDEBMAX)																					
	{
		pC->Case.manipTelemAllDis_num = 0;
		pC->Case.manipTelemAllDis = On;
		buzzer_on();
	}
	else																																		
		pC->Case.manipTelemAllDis = Off;
	
	if(pC->Case.resetdev1_num >= BUTDEBMAX)																								
	{
		pC->Case.resetdev1_num = 0;
		pC->Case.resetdev1 = On;
		buzzer_on();
	}
	else																																		
		pC->Case.resetdev1 = Off;
	
	if(pC->Case.resetdev2_num >= BUTDEBMAX)																							  
	{
		pC->Case.resetdev2_num = 0;
		pC->Case.resetdev2 = On;
		buzzer_on();
	}
	else																																		
		pC->Case.resetdev2 = Off;

	//grzyb
	if(pC->Case.EmergencySwitchEn_num >= BUTDEBMAX)
	{
		pC->Case.EmergencySwitchEn_num = 0;
		pC->Case.EmergencySwitchEn = On;
	}
	else
		pC->Case.EmergencySwitchEn = Off;
	
	if(pC->Case.EmergencySwitchDis_num >= BUTDEBMAX)																							  
	{
		pC->Case.EmergencySwitchDis_num = 0;
		pC->Case.EmergencySwitchDis = On;
	}
	else
		pC->Case.EmergencySwitchDis = Off;
		

	if((AutonextIdr) == RESET && pC->Case.autonext_prev == Off)						
	{
		pC->Case.autonext = On;
		pC->Case.autonext_prev = On;
		buzzer_on();
	}
	else if((AutonextIdr) != RESET)																		
	{
		pC->Case.autonext = Off;
		pC->Case.autonext_prev = Off;
	}
	else
		pC->Case.autonext = Off;
	//kamery
	if((Cam1NextIdr) == RESET && pC->Case.select_cam1_next_prev == Off)						
	{
		pC->Case.select_cam1_next = On;
		pC->Case.select_cam1_next_prev = On;
		buzzer_on();
	}
	else if((Cam1NextIdr) != RESET)																		
	{
		pC->Case.select_cam1_next = Off;
		pC->Case.select_cam1_next_prev = Off;
	}
	else
		pC->Case.select_cam1_next = Off;
	
	if((Cam1BackIdr) == RESET && pC->Case.select_cam1_back_prev == Off)						
	{
		pC->Case.select_cam1_back = On;
		pC->Case.select_cam1_back_prev = On;
		buzzer_on();
	}
	else if((Cam1BackIdr) != RESET)																		
	{
		pC->Case.select_cam1_back = Off;
		pC->Case.select_cam1_back_prev = Off;
	}
	else
		pC->Case.select_cam1_back = Off;
	
	if((Cam2NextIdr) == RESET && pC->Case.select_cam2_next_prev == Off)						
	{
		pC->Case.select_cam2_next = On;
		pC->Case.select_cam2_next_prev = pC->Case.select_cam2_next;
		buzzer_on();
	}
	else if((Cam2NextIdr) != RESET)																		
	{
		pC->Case.select_cam2_next = Off;
		pC->Case.select_cam2_next_prev = pC->Case.select_cam2_next;
	}
	else
		pC->Case.select_cam2_next = Off;
	
	if((Cam2BackIdr) == RESET && pC->Case.select_cam2_back_prev == Off)						
	{
		pC->Case.select_cam2_back = On;
		pC->Case.select_cam2_back_prev = pC->Case.select_cam2_back;
		buzzer_on();
	}
	else if((Cam2BackIdr) != RESET)																		
	{
		pC->Case.select_cam2_back = Off;
		pC->Case.select_cam2_back_prev = pC->Case.select_cam2_back;
	}
	else
		pC->Case.select_cam2_back = Off;
}
static void direction_motor(void)
{
	//chwytak
	if(pC->Case.gripper_open == On && pC->Case.gripper_close == Off)
		pC->Case.gripper_speed = -pC->Case.gripper_speed;
	else if(pC->Case.gripper_open == Off && pC->Case.gripper_close == On)
		pC->Case.gripper_speed = pC->Case.gripper_speed;
	else
		pC->Case.gripper_speed = 0;
	
	//manipulator
	if(pC->Case.manip_x_L == On && pC->Case.manip_x_R == Off)
		pC->Case.manip_x_speed = -pC->Case.manip_x_speed;
	else if(pC->Case.manip_x_L == Off && pC->Case.manip_x_R == On)
		pC->Case.manip_x_speed = pC->Case.manip_x_speed;
	else
		pC->Case.manip_x_speed = 0;
	
	if(pC->Case.manip_y_L == On && pC->Case.manip_y_R == Off)
		pC->Case.manip_y_speed = -pC->Case.manip_y_speed;
	else if(pC->Case.manip_y_L == Off && pC->Case.manip_y_R == On)
		pC->Case.manip_y_speed = pC->Case.manip_y_speed;
	else
		pC->Case.manip_y_speed = 0;
	
	if(pC->Case.manip_z_L == On && pC->Case.manip_z_R == Off)
		pC->Case.manip_z_speed = -pC->Case.manip_z_speed;
	else if(pC->Case.manip_z_L == Off && pC->Case.manip_z_R == On)
		pC->Case.manip_z_speed = pC->Case.manip_z_speed;
	else
		pC->Case.manip_z_speed = 0;
	
	if(pC->Case.manip_Rx_L == On && pC->Case.manip_Rx_R == Off)
		pC->Case.manip_Rx_speed = -pC->Case.manip_Rx_speed;
	else if(pC->Case.manip_Rx_L == Off && pC->Case.manip_Rx_R == On)
		pC->Case.manip_Rx_speed = pC->Case.manip_Rx_speed;
	else
		pC->Case.manip_Rx_speed = 0;
	
	if(pC->Case.manip_Ry_L == On && pC->Case.manip_Ry_R == Off)
		pC->Case.manip_Ry_speed = -pC->Case.manip_Ry_speed;
	else if(pC->Case.manip_Ry_L == Off && pC->Case.manip_Ry_R == On)
		pC->Case.manip_Ry_speed = pC->Case.manip_Ry_speed;
	else
		pC->Case.manip_Ry_speed = 0;
	
	if(pC->Case.manip_Rz_L == On && pC->Case.manip_Rz_R == Off)
		pC->Case.manip_Rz_speed = -pC->Case.manip_Rz_speed;
	else if(pC->Case.manip_Rz_L == Off && pC->Case.manip_Rz_R == On)
		pC->Case.manip_Rz_speed = pC->Case.manip_Rz_speed;
	else
		pC->Case.manip_Rz_speed = 0;
}

static void CmdManip_Set(void)
{
	sFrame *f = &pC->Com.frames[Fnum_Manip];
	if(pC->Case.joint == On)										f->cmd = cmdmanip_join;
	else if(pC->Case.tool == On)								f->cmd = cmdmanip_tool;
	else if(pC->Case.base == On)								f->cmd = cmdmanip_base;
	else if(pC->Case.calibration == On)					f->cmd = cmdmanip_calibrate;
	else if(pC->Case.laseron == On)							f->cmd = cmdmanip_laseron;
	else if(pC->Case.laseroff == On)						f->cmd = cmdmanip_laseroff;
	else if(pC->Case.magneson == On)						f->cmd = cmdmanip_magneson;
	else if(pC->Case.magnesoff == On)						f->cmd = cmdmanip_magnesoff;
	//else if(pC->Case.automove == On)						f->cmd = cmdmanip_automove;
	else if(pC->Case.clearseq == On)						f->cmd = cmdmanip_clearseq;
	else if(pC->Case.manipTelemAllEn == On)			f->cmd = cmdmanip_telemallen;
	else if(pC->Case.manipTelemAllDis == On)		f->cmd = cmdmanip_telemalldis;
	else if(pC->Case.home == On)
	{
		f->cmd = cmdmanip_automove;
		f->cmdval = 0;
	}
	else if(pC->Case.park == On)
	{
		f->cmd = cmdmanip_automove;
		f->cmdval = 1;
	}
	else if(pC->Case.work == On)
	{
		f->cmd = cmdmanip_automove;
		f->cmdval = 2;
	}
	else if(pC->Case.take == On)
	{
		f->cmd = cmdmanip_automove;
		f->cmdval = 3;
	}
}
static void CmdLab_Set(void)
{
	sFrame *f = &pC->Com.frames[Fnum_Lab];
	if(pC->Case.manip_Ry_L == On)										f->cmd = cmdLab_LumiOpen;
	else if(pC->Case.manip_Ry_R == On)							f->cmd = cmdLab_LumiClose;
	else if(pC->Case.manip_Rz_L == On)							f->cmd = cmdLab_LumiStartMeas;
	else if(pC->Case.manip_Rz_R == On)							f->cmd = cmdLab_LumiTurnOnOff;
	else if(pC->Case.gripper_open == On)						f->cmd = cmdLab_GripperOpen;
	else if(pC->Case.gripper_close == On)						f->cmd = cmdLab_GripperClose;
}
static void CmdDrive_Set(void)
{
	sFrame *f = &pC->Com.frames[Fnum_Drive];
	if(	pC->Case.select_satel == On)							f->cmd = cmddrive_satel;
	else if(pC->Case.select_wifi == On)						f->cmd = cmddrive_wifi;
	else if(pC->Case.select_cam1_next == On)			f->cmd = cmddrive_select_cam1_next;
	else if(pC->Case.select_cam1_back == On)			f->cmd = cmddrive_select_cam1_back;
	else if(pC->Case.select_cam2_next == On)			f->cmd = cmddrive_select_cam2_next;
	else if(pC->Case.select_cam2_back == On)			f->cmd = cmddrive_select_cam2_back;
	else if(pC->Case.autostm == On)								f->cmd = cmddrive_autostm;
	else if(pC->Case.autonvidia == On)						f->cmd = cmddrive_autonvidia;
	else if(pC->Case.autopause == On)							f->cmd = cmddrive_autopause;
	else if(pC->Case.autocontinue == On)					f->cmd = cmddrive_autocontinue;
	else if(pC->Case.autonext == On)							f->cmd = cmddrive_autonext;
	else if(pC->Case.resetdev1 == On)							f->cmd = cmddrive_resetdev1;
	else if(pC->Case.resetdev2 == On)							f->cmd = cmddrive_resetdev2;
	else if(pC->Case.gimdigiton == On)						f->cmd = cmddrive_gimdigiton;
	else if(pC->Case.gimdigitoff == On)						f->cmd = cmddrive_gimdigitoff;
	else if(pC->Case.driveTelemAllEn == On)				f->cmd = cmddrive_TelemAllEn;
	else if(pC->Case.driveTelemAllDis == On)			f->cmd = cmddrive_TelemAllDis;
	else if(pC->Case.EmergencySwitchEn == On)			f->cmd = cmddrive_EmerSoftEn;
	else if(pC->Case.EmergencySwitchDis == On)		f->cmd = cmddrive_EmerSoftDis;
}
static void PrepareToFrame(void)
{
	Button_Set();
	Case_ADC_value();
	direction_motor();
	CmdManip_Set();
	CmdDrive_Set();
	CmdLab_Set();
}
//----------------Przerwania--------------------------------------
void TIM3_IRQHandler(void)
{
	if((TIM3->SR & TIM_SR_UIF) != RESET)
	{
//		Button_ReadDebouncing();
		TIM3->SR &= ~TIM_SR_UIF;
	}
}
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		PrepareToFrame();
	}
	TIM7->SR &= ~TIM_SR_UIF;
}

void TIM4_IRQHandler(void)
{
	if((TIM4->SR & TIM_SR_UIF) != RESET)
	{
		buzzer_off();
		TIM4->SR &= ~TIM_SR_UIF;
	}
}
