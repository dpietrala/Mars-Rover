#include "AddDevice.h"
extern sControl* pC;
static void Dev_LedConf(void)
{
	LED_PORT->MODER 	|= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0;
	LED_PORT->PUPDR 	|= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0;
}
static void Dev_SignalConf(void)
{
	GPIOA->MODER |= GPIO_MODER_MODE12_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE9_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD12_1 | GPIO_PUPDR_PUPD11_1 | GPIO_PUPDR_PUPD9_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT9;
}
static void Dev_CamConf(void)
{
	GPIOB->MODER &= ~GPIO_MODER_MODE3_0 & ~GPIO_MODER_MODE3_1 & ~GPIO_MODER_MODE4_0 & ~GPIO_MODER_MODE4_1 & ~GPIO_MODER_MODE5_0 & ~GPIO_MODER_MODE5_1;
	GPIOB->MODER |= GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD3_1 | GPIO_PUPDR_PUPD4_1 | GPIO_PUPDR_PUPD5_1;
	COMMUXA_LOW;
	COMMUXB_LOW;
	COMMUXC_LOW;
	
	GPIOE->MODER |= GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0;
	GPIOE->PUPDR |= GPIO_PUPDR_PUPD7_1 | GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD9_1 | GPIO_PUPDR_PUPD10_1 | GPIO_PUPDR_PUPD11_1 | GPIO_PUPDR_PUPD12_1 | GPIO_PUPDR_PUPD13_1 | GPIO_PUPDR_PUPD14_1;
	
	GPIOD->MODER	|= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOD->PUPDR	|= GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR13_0 | GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR15_0;
	GPIOD->AFR[1]	|= 0x22220000;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM4->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM4->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM4->PSC	= 84-1;
	TIM4->ARR	= 20000-1;
	CAM_SERW_PWM0_REG = 1500;
	CAM_SERW_PWM1_REG = 1500;
	CAM_SERW_PWM2_REG = 1500;
	CAM_SERW_PWM3_REG = 1500;
	TIM4->CR1	|= TIM_CR1_CEN;
}
static void Dev_ComConf(void)
{
//---------- Usart1 GPS and other	-------------------------------------	
	DMA2_Stream5->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream5->M0AR 	= (uint32_t)pC->Dev.gpsbufread;
  DMA2_Stream5->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream5->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA2_Stream7->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream7->M0AR 	= (uint32_t)pC->Dev.gpsbufwrite;
  DMA2_Stream7->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream7->CR 		|= DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
  GPIOB->AFR[0] |= 0x77000000;
  USART1->BRR = 84000000/115200;
  USART1->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn);
}
static void Dev_emergencyhardConf(void)
{
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
}
static void Dev_CamSelect(uint8_t Trans, uint8_t Cam)
{
	switch (Trans)
	{
		case 1:
			switch (Cam)
			{
				case 1:	{MUX1A_LOW;		MUX1B_LOW;	MUX1C_LOW;	MUX1D_HIGH;	break;}
				case 2:	{MUX1A_HIGH;	MUX1B_LOW;	MUX1C_LOW;	MUX1D_HIGH;	break;}
				case 3:	{MUX1A_LOW;		MUX1B_HIGH;	MUX1C_LOW;	MUX1D_HIGH;	break;}
				case 4:	{MUX1A_HIGH;	MUX1B_LOW;	MUX1C_HIGH;	MUX1D_HIGH;	break;}
				case 5:	{MUX1A_LOW;		MUX1B_LOW;	MUX1C_HIGH;	MUX1D_HIGH;	break;}
				case 6:	{MUX1A_HIGH;	MUX1B_HIGH;	MUX1C_LOW;	MUX1D_HIGH;	break;}
				default:	break;
			}
			break;
		case 2:
			switch (Cam)
			{
				case 1:	{MUX2A_LOW;		MUX2B_LOW;	MUX2C_LOW;	MUX2D_HIGH;	break;}
				case 2:	{MUX2A_HIGH;	MUX2B_LOW;	MUX2C_LOW;	MUX2D_HIGH;	break;}
				case 3:	{MUX2A_LOW;		MUX2B_HIGH;	MUX2C_LOW;	MUX2D_HIGH;	break;}
				case 4:	{MUX2A_HIGH;	MUX2B_LOW;	MUX2C_HIGH;	MUX2D_HIGH;	break;}
				case 5:	{MUX2A_LOW;		MUX2B_LOW;	MUX2C_HIGH;	MUX2D_HIGH;	break;}
				case 6:	{MUX2A_HIGH;	MUX2B_HIGH;	MUX2C_LOW;	MUX2D_HIGH;	break;}
				default:	break;
			}
			break;
		default:
			break;
	}
}
void Dev_Conf(void)
{
	Dev_LedConf();
	Dev_CamConf();
	Dev_ComConf();
	Dev_emergencyhardConf();
	Dev_SignalConf();
	Dev_CamSelect(1,pC->Dev.Trans1);
	Dev_CamSelect(2,pC->Dev.Trans2);
}
void Dev_ResetDevice1(void)
{
	pC->Dev.resetdev1 = On;
	RESETDEV1_ON;
	pC->Dev.resetdev1tick = 0;
}
void Dev_UnresetDevice1(void)
{
	if(pC->Dev.resetdev1tick >= RESETDEVTICKMAX)
	{
		pC->Dev.resetdev1 = Off;
		RESETDEV1_OFF;
		pC->Dev.resetdev1tick = RESETDEVTICKMAX;
	}
}
void Dev_ResetDevice2(void)
{
	pC->Dev.resetdev2 = On;
	RESETDEV2_ON;
	pC->Dev.resetdev2tick = 0;
}
static void Dev_UnresetDevice2(void)
{
	if(pC->Dev.resetdev1tick >= RESETDEVTICKMAX)
	{
		pC->Dev.resetdev2 = Off;
		RESETDEV2_OFF;
		pC->Dev.resetdev2tick = RESETDEVTICKMAX;
	}
}
static void Dev_CamAct(void)
{
	for(uint8_t i=0;i<4;i++)
	{
		pC->Dev.SerwPwm[i] = pC->Dev.SerwPos[i] * 9 + 1500;
		if(pC->Dev.SerwPwm[i] > 2400)
			pC->Dev.SerwPwm[i] = 2400;
		else if(pC->Dev.SerwPwm[i] < 600)
			pC->Dev.SerwPwm[i] = 600;
	}
	CAM_SERW_PWM0_REG = pC->Dev.SerwPwm[0];
	CAM_SERW_PWM1_REG = pC->Dev.SerwPwm[1];
	CAM_SERW_PWM2_REG = pC->Dev.SerwPwm[2];
	CAM_SERW_PWM3_REG = pC->Dev.SerwPwm[3];
	Dev_CamSelect(1, pC->Dev.Trans1);
	Dev_CamSelect(2, pC->Dev.Trans2);
}
static void Dev_EmergencyhardAct(void)
{
	if((GPIOA->IDR & GPIO_IDR_IDR_10) != RESET)
		pC->Error.emergencyhard = On;
	else
		pC->Error.emergencyhard = Off;
}
void Dev_Act(void)
{
	Dev_UnresetDevice1();
	Dev_UnresetDevice2();
	Dev_CamAct();
	Dev_EmergencyhardAct();
}
union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};
union conv64
{
    uint64_t u64; // here_write_bits
    double   d64; // here_read_double
};
static uint16_t Dev_GPSCrc(uint8_t data[], uint32_t length) 
{
	uint32_t i;
	uint16_t crc = 0;
	for(i=0; i<length; i++)
	{
		crc = (uint8_t)(crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (uint8_t)(crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0x00ff) << 5;
	}
	return crc;
}
static void Dev_ReadFromGPS(void)
{
	uint8_t* buf = pC->Dev.gpsbufread;
	uint8_t buf2[BUF_LEN];
	for(uint8_t i=0;i<41;i++)
		buf2[i] = buf[i+1];
	union conv32 x;
	union conv64 y;
	uint16_t crc1 = Dev_GPSCrc(buf2, 41);
	uint16_t crc2 = ((uint16_t)buf[42]<<8) + ((uint16_t)buf[43]<<0);
	if(buf[0] == 0xfa && crc1 == crc2)
	{
		x.u32 = ((uint32_t)buf[9]<<24) + ((uint32_t)buf[8]<<16) + ((uint32_t)buf[7]<<8) + ((uint32_t)buf[6]<<0);
		pC->Dev.imuyawraw = x.f32;
		pC->Dev.imuyaw = pC->Dev.imuyawraw - pC->Dev.imuyawoffset;
		if(pC->Dev.imuyaw < -180.0)
		{
			pC->Dev.imuyaw = 360.0 - fabs(pC->Dev.imuyaw);
		}
		else if(pC->Dev.imuyaw > 180.0)
		{
			pC->Dev.imuyaw -= 360.0;
		}
		
		x.u32 = ((uint32_t)buf[13]<<24) + ((uint32_t)buf[12]<<16) + ((uint32_t)buf[11]<<8) + ((uint32_t)buf[10]<<0);
		pC->Dev.imupitch = x.f32;
		x.u32 = ((uint32_t)buf[17]<<24) + ((uint32_t)buf[16]<<16) + ((uint32_t)buf[15]<<8) + ((uint32_t)buf[14]<<0);
		pC->Dev.imuroll = x.f32;
		
		y.u64 = ((uint64_t)buf[25]<<56) + ((uint64_t)buf[24]<<48) + ((uint64_t)buf[23]<<40) + ((uint64_t)buf[22]<<32) + ((uint64_t)buf[21]<<24) + ((uint64_t)buf[20]<<16) + ((uint64_t)buf[19]<<8) + ((uint64_t)buf[18]<<0);
		pC->Dev.gpslat = y.d64;
		y.u64 = ((uint64_t)buf[33]<<56) + ((uint64_t)buf[32]<<48) + ((uint64_t)buf[31]<<40) + ((uint64_t)buf[30]<<32) + ((uint64_t)buf[29]<<24) + ((uint64_t)buf[28]<<16) + ((uint64_t)buf[27]<<8) + ((uint64_t)buf[26]<<0);
		pC->Dev.gpslon = y.d64;
		y.u64 = ((uint64_t)buf[41]<<56) + ((uint64_t)buf[40]<<48) + ((uint64_t)buf[39]<<40) + ((uint64_t)buf[38]<<32) + ((uint64_t)buf[37]<<24) + ((uint64_t)buf[36]<<16) + ((uint64_t)buf[35]<<8) + ((uint64_t)buf[34]<<0);
		pC->Dev.gpsalt = y.d64;
	}
	
	ClearStr(pC->Dev.gpsbufread, BUF_LEN);
	DMA2_Stream5->CR 		&= ~DMA_SxCR_EN;
	DMA2->HIFCR 				|= DMA_HIFCR_CTCIF5;
  DMA2_Stream5->CR 		|= DMA_SxCR_EN;
}
//----------- Przerwania --------------------------------------------
void USART1_IRQHandler(void)
{
	if((USART1->SR & USART_SR_IDLE) != RESET)
	{
		Dev_ReadFromGPS();
		char c = USART1->DR;
	}
}
