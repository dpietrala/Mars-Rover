#include "Inputs.h"
extern sControl* pC;
void AI_Conf(void)
{
	GPIOA->MODER |= GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7;
	GPIOB->MODER |= GPIO_MODER_MODE0 | GPIO_MODER_MODE1;
	GPIOC->MODER |= GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5;
	
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)pC->AIRawvalue;
	DMA2_Stream0->NDTR = (uint16_t)1600;
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	
	ADC->CCR 	|= ADC_CCR_ADCPRE;
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->SMPR2 |= ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;
	ADC1->SMPR1 |= ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13 | ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15;
	
	ADC1->SQR3 |= (0<<0) | (1<<5) | (2<<10) | (3<<15) | (4<<20) | (5<<25);
	ADC1->SQR2 |= (6<<0) | (7<<5) | (8<<10) | (9<<15) | (10<<20) | (11<<25);
	ADC1->SQR1 |= (12<<0) | (13<<5) | (14<<10) | (15<<15) | (16<<20);
	ADC1->CR2 |= ADC_CR2_SWSTART;
	
	for(uint8_t i=0;i<AI_MAX;i++)
	{
		pC->AI[i].min = -100;
		pC->AI[i].max = 100;
		pC->AI[i].mean = (pC->AI[i].max + pC->AI[i].min) / 2;
		pC->AI[i].val = pC->AI[i].mean;
		pC->AI[i].hyst = 5;
	}
}
void DI_Conf(void)
{
	//EXTI 0-7 
	pC->DI[0].port = GPIOF;
	pC->DI[0].moder = GPIO_MODER_MODE0;
	pC->DI[0].pupdr = GPIO_PUPDR_PUPD0_0;
	pC->DI[0].idr = GPIO_IDR_ID0;
	pC->DI[1].port = GPIOF;
	pC->DI[1].moder = GPIO_MODER_MODE1;
	pC->DI[1].pupdr = GPIO_PUPDR_PUPD1_0;
	pC->DI[1].idr = GPIO_IDR_ID1;
	pC->DI[2].port = GPIOF;
	pC->DI[2].moder = GPIO_MODER_MODE2;
	pC->DI[2].pupdr = GPIO_PUPDR_PUPD2_0;
	pC->DI[2].idr = GPIO_IDR_ID2;
	pC->DI[3].port = GPIOF;
	pC->DI[3].moder = GPIO_MODER_MODE3;
	pC->DI[3].pupdr = GPIO_PUPDR_PUPD3_0;
	pC->DI[3].idr = GPIO_IDR_ID3;
	pC->DI[4].port = GPIOF;
	pC->DI[4].moder = GPIO_MODER_MODE4;
	pC->DI[4].pupdr = GPIO_PUPDR_PUPD4_0;
	pC->DI[4].idr = GPIO_IDR_ID4;
	pC->DI[5].port = GPIOF;
	pC->DI[5].moder = GPIO_MODER_MODE5;
	pC->DI[5].pupdr = GPIO_PUPDR_PUPD5_0;
	pC->DI[5].idr = GPIO_IDR_ID5;
	pC->DI[6].port = GPIOF;
	pC->DI[6].moder = GPIO_MODER_MODE6;
	pC->DI[6].pupdr = GPIO_PUPDR_PUPD6_0;
	pC->DI[6].idr = GPIO_IDR_ID6;
	pC->DI[7].port = GPIOF;
	pC->DI[7].moder = GPIO_MODER_MODE7;
	pC->DI[7].pupdr = GPIO_PUPDR_PUPD7_0;
	pC->DI[7].idr = GPIO_IDR_ID7;
	
	//EXTI 8-15
	pC->DI[8].port = GPIOE;
	pC->DI[8].moder = GPIO_MODER_MODE8;
	pC->DI[8].pupdr = GPIO_PUPDR_PUPD8_0;
	pC->DI[8].idr = GPIO_IDR_ID8;
	pC->DI[9].port = GPIOE;
	pC->DI[9].moder = GPIO_MODER_MODE9;
	pC->DI[9].pupdr = GPIO_PUPDR_PUPD9_0;
	pC->DI[9].idr = GPIO_IDR_ID9;
	pC->DI[10].port = GPIOE;
	pC->DI[10].moder = GPIO_MODER_MODE10;
	pC->DI[10].pupdr = GPIO_PUPDR_PUPD10_0;
	pC->DI[10].idr = GPIO_IDR_ID10;
	pC->DI[11].port = GPIOE;
	pC->DI[11].moder = GPIO_MODER_MODE11;
	pC->DI[11].pupdr = GPIO_PUPDR_PUPD11_0;
	pC->DI[11].idr = GPIO_IDR_ID11;
	pC->DI[12].port = GPIOE;
	pC->DI[12].moder = GPIO_MODER_MODE12;
	pC->DI[12].pupdr = GPIO_PUPDR_PUPD12_0;
	pC->DI[12].idr = GPIO_IDR_ID12;
	pC->DI[13].port = GPIOE;
	pC->DI[13].moder = GPIO_MODER_MODE13;
	pC->DI[13].pupdr = GPIO_PUPDR_PUPD13_0;
	pC->DI[13].idr = GPIO_IDR_ID13;
	pC->DI[14].port = GPIOE;
	pC->DI[14].moder = GPIO_MODER_MODE14;
	pC->DI[14].pupdr = GPIO_PUPDR_PUPD14_0;
	pC->DI[14].idr = GPIO_IDR_ID14;
	pC->DI[15].port = GPIOE;
	pC->DI[15].moder = GPIO_MODER_MODE15;
	pC->DI[15].pupdr = GPIO_PUPDR_PUPD15_0;
	pC->DI[15].idr = GPIO_IDR_ID15;
	
	//IO00 - IO07
	pC->DI[16].port = GPIOE;
	pC->DI[16].moder = GPIO_MODER_MODE8;
	pC->DI[16].pupdr = GPIO_PUPDR_PUPD8_0;
	pC->DI[16].idr = GPIO_IDR_ID8;
	pC->DI[17].port = GPIOE;
	pC->DI[17].moder = GPIO_MODER_MODE9;
	pC->DI[17].pupdr = GPIO_PUPDR_PUPD9_0;
	pC->DI[17].idr = GPIO_IDR_ID9;
	pC->DI[18].port = GPIOE;
	pC->DI[18].moder = GPIO_MODER_MODE10;
	pC->DI[18].pupdr = GPIO_PUPDR_PUPD10_0;
	pC->DI[18].idr = GPIO_IDR_ID10;
	pC->DI[19].port = GPIOE;
	pC->DI[19].moder = GPIO_MODER_MODE11;
	pC->DI[19].pupdr = GPIO_PUPDR_PUPD11_0;
	pC->DI[19].idr = GPIO_IDR_ID11;
	pC->DI[20].port = GPIOE;
	pC->DI[20].moder = GPIO_MODER_MODE12;
	pC->DI[20].pupdr = GPIO_PUPDR_PUPD12_0;
	pC->DI[20].idr = GPIO_IDR_ID12;
	pC->DI[21].port = GPIOE;
	pC->DI[21].moder = GPIO_MODER_MODE13;
	pC->DI[21].pupdr = GPIO_PUPDR_PUPD13_0;
	pC->DI[21].idr = GPIO_IDR_ID13;
	pC->DI[22].port = GPIOE;
	pC->DI[22].moder = GPIO_MODER_MODE14;
	pC->DI[22].pupdr = GPIO_PUPDR_PUPD14_0;
	pC->DI[22].idr = GPIO_IDR_ID14;
	pC->DI[23].port = GPIOE;
	pC->DI[23].moder = GPIO_MODER_MODE15;
	pC->DI[23].pupdr = GPIO_PUPDR_PUPD15_0;
	pC->DI[23].idr = GPIO_IDR_ID15;
	
	//IO20 - IO27
	pC->DI[24].port = GPIOD;
	pC->DI[24].moder = GPIO_MODER_MODE8;
	pC->DI[24].pupdr = GPIO_PUPDR_PUPD8_0;
	pC->DI[24].idr = GPIO_IDR_ID8;
	pC->DI[25].port = GPIOD;
	pC->DI[25].moder = GPIO_MODER_MODE9;
	pC->DI[25].pupdr = GPIO_PUPDR_PUPD9_0;
	pC->DI[25].idr = GPIO_IDR_ID9;
	pC->DI[26].port = GPIOD;
	pC->DI[26].moder = GPIO_MODER_MODE10;
	pC->DI[26].pupdr = GPIO_PUPDR_PUPD10_0;
	pC->DI[26].idr = GPIO_IDR_ID10;
	pC->DI[27].port = GPIOD;
	pC->DI[27].moder = GPIO_MODER_MODE11;
	pC->DI[27].pupdr = GPIO_PUPDR_PUPD11_0;
	pC->DI[27].idr = GPIO_IDR_ID11;
	pC->DI[28].port = GPIOD;
	pC->DI[28].moder = GPIO_MODER_MODE12;
	pC->DI[28].pupdr = GPIO_PUPDR_PUPD12_0;
	pC->DI[28].idr = GPIO_IDR_ID12;
	pC->DI[29].port = GPIOD;
	pC->DI[29].moder = GPIO_MODER_MODE13;
	pC->DI[29].pupdr = GPIO_PUPDR_PUPD13_0;
	pC->DI[29].idr = GPIO_IDR_ID13;
	pC->DI[30].port = GPIOD;
	pC->DI[30].moder = GPIO_MODER_MODE14;
	pC->DI[30].pupdr = GPIO_PUPDR_PUPD14_0;
	pC->DI[30].idr = GPIO_IDR_ID14;
	pC->DI[31].port = GPIOD;
	pC->DI[31].moder = GPIO_MODER_MODE15;
	pC->DI[31].pupdr = GPIO_PUPDR_PUPD15_0;
	pC->DI[31].idr = GPIO_IDR_ID15;
	
	for(uint8_t i=0;i<DI_MAX;i++)
	{
		pC->DI[i].port->MODER &= ~pC->DI[i].moder;
		pC->DI[i].port->PUPDR |= pC->DI[i].pupdr;
	}
	
	pC->DI[0].cmd = Cmdrover_satel;
	pC->DI[1].cmd = Cmdrover_wifi;
	pC->DI[2].cmd = Cmdrover_auto;
	pC->DI[3].cmd = Cmdrover_manipon;
	pC->DI[4].cmd = Cmdrover_manipoff;
	pC->DI[5].cmd = Cmdrover_manipdefpos;
	pC->DI[6].cmd = Cmdrover_manipjoin;
	pC->DI[7].cmd = Cmdrover_manipbase;
	pC->DI[8].cmd = Cmdrover_maniptool;
	pC->DI[9].cmd = Cmdrover_gimdigiton;
	pC->DI[10].cmd = Cmdrover_gimdigitoff;
	pC->DI[11].cmd = Cmdrover_laser_on;
	pC->DI[12].cmd = Cmdrover_laser_off;
	pC->DI[13].cmd = Cmdrover_cam2inc;
	pC->DI[14].cmd = Cmdrover_cam2dec;
	pC->DI[15].cmd = Cmdrover_cam1inc;
	pC->DI[16].cmd = Cmdrover_cam1dec;
	pC->DI[17].cmd = Cmdrover_manipptp0;
	pC->DI[18].cmd = Cmdrover_manipptp1;
	pC->DI[19].cmd = Cmdrover_manipptp2;
	pC->DI[20].cmd = Cmdrover_manipptp3;
	pC->DI[21].cmd = Cmdrover_manipptp4;
	pC->DI[22].cmd = Cmdrover_manipptp5;
	pC->DI[23].cmd = Cmdrover_manipptp6;
	pC->DI[24].cmd = Cmdrover_manipptp7;
	pC->DI[25].cmd = Cmdrover_autopause;
	pC->DI[26].cmd = Cmdrover_autocontinue;
	pC->DI[27].cmd = Cmdrover_autostage0;
	pC->DI[28].cmd = Cmdrover_autostage1;
	pC->DI[29].cmd = Cmdrover_autostage2;
	pC->DI[30].cmd = Cmdrover_autostage3;
	pC->DI[31].cmd = Cmdrover_autostage4;
}
static void AI_Read(void)
{
	for(uint8_t i=0;i<AI_MAX;i++)
	{
		uint32_t sum = 0;
		for(uint16_t j=0;j<100;j++)
		{
			sum += pC->AIRawvalue[100*i+j];
		}
		pC->AI[i].adcval = sum / 100;
		pC->AI[i].val = pC->AI[i].min + (int16_t)(((double)pC->AI[i].adcval / 4096.0) * (double)(pC->AI[i].max - pC->AI[i].min));
		if((pC->AI[i].val > (pC->AI[i].mean - pC->AI[i].hyst)) && (pC->AI[i].val < (pC->AI[i].mean + pC->AI[i].hyst)))
		{
			if(pC->AI[i].val > pC->AI[i].mean)
				pC->AI[i].val -= pC->AI[i].hyst;
			else if(pC->AI[i].val < pC->AI[i].mean)
				pC->AI[i].val += pC->AI[i].hyst;
		}
	}
}
static void DI_Read(void)
{
	for(uint8_t i=0;i<DI_MAX;i++)
	{
		if((pC->DI[i].port->IDR & pC->DI[i].idr) == RESET)
			pC->DI[i].cnt++;
		
		if(pC->DI[i].cnt >= DITICK_MAX)
		{
			pC->DI[i].state = DIStateOn;
			pC->DI[i].cnt = 0;
		}
	}
}
void Inputs_Read(void)
{
	AI_Read();
	DI_Read();
}
