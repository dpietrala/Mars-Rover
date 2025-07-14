#include "Comunication.h"
extern sControl* pC;
void COM_Conf(void)
{
	DMA2_Stream2->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream2->M0AR 	= (uint32_t)pC->Com.bufread;
  DMA2_Stream2->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA2_Stream7->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream7->M0AR 	= (uint32_t)pC->Com.bufwrite;
  DMA2_Stream7->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream7->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0;
  GPIOA->OTYPER|= GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10;
  GPIOA->AFR[1] |= 0x00000770;
  USART1->BRR = 84000000/115200;
  USART1->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn);
	
	TIM6->PSC = 84-1;
	TIM6->ARR = 60000-1;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
void ClearStr(uint8_t* str, uint32_t l)
{
	for(uint32_t i=0;i<l;i++)
		str[i] = 0;
}
uint16_t crc16(uint8_t* packet, uint32_t nBytes)
{
	uint16_t crc = 0;
	for(uint32_t byte = 0; byte < nBytes; byte++)
	{
		crc = crc ^ ((uint16_t)packet[byte] << 8);
		for (uint8_t bit = 0; bit < 8; bit++)
			if(crc & 0x8000) 	crc = (crc << 1) ^ 0x1021;
			else							crc = crc << 1;
	}
	return crc;
}
static void COM_ReadFromHostGenerallFrame(uint8_t *buf)
{
	uint16_t crc1 = crc16(buf, 14);
	uint16_t crc2 = (uint16_t)(buf[14]<<8) + (uint16_t)buf[15];
	if(crc1 == crc2)
	{
		for(uint8_t i=0;i<MOTMAX;i++)
		{
			pC->Mot.pid[i] = (eOnOff)buf[3*i+0+2];
			int16_t speedref = (int16_t)((uint16_t)(buf[3*i+1+2]<<8) + (uint16_t)(buf[3*i+2+2]));
			if(speedref > pC->Mot.speedrefmax[i])
				pC->Mot.speedref[i] = pC->Mot.speedrefmax[i];
			else if(speedref < -pC->Mot.speedrefmax[i])
				pC->Mot.speedref[i] = -pC->Mot.speedrefmax[i];
			else
				pC->Mot.speedref[i] = speedref;
		}
		LED2_OFF;
		LED3_TOG;
		pC->Status.hostcomtick = 0;
	}
}
static void COM_ReadFromHostEmergencyFrame(uint8_t *buf)
{
	uint16_t crc1 = crc16(buf, 3);
	uint16_t crc2 = (uint16_t)(buf[3]<<8) + (uint16_t)buf[4];
	if(crc1 == crc2)
	{
		for(uint8_t i=0;i<MOTMAX;i++)
		{
			pC->Mot.pid[i] = Off;
			pC->Mot.speedref[i] = 0;
		}
		LED3_OFF;
		LED2_TOG;
		pC->Status.hostcomtick = 0;
	}
}
static void COM_ReadFromHost(void)
{
	uint8_t* buf = pC->Com.bufread;
	if(buf[0] == Frametype_Header && buf[1] == Frametype_MainboardToDriveGenerallSpeed)
		COM_ReadFromHostGenerallFrame(buf);
	else if(buf[0] == Frametype_Header && buf[1] == Frametype_EmergencyStop)
		COM_ReadFromHostEmergencyFrame(buf);
	
	ClearStr(buf, BUF_LEN);
	DMA2_Stream2->CR 		&= ~DMA_SxCR_EN;
	DMA2->LIFCR 				|= DMA_LIFCR_CTCIF2;
  DMA2_Stream2->CR 		|= DMA_SxCR_EN;
}
static void COM_SendToHost(void)
{
	int32_t speed = pC->Mot.roverspeed * 1000.0;
	int32_t angle = pC->Mot.azimuth * 1000.0;
	int32_t posx = pC->Mot.posx * 1000.0;
	int32_t posy = pC->Mot.posy * 1000.0;
	uint8_t errors = 0x00;
	if(pC->Status.error[0] == On)
		errors |= 0x01;
	if(pC->Status.error[1] == On)
		errors |= 0x02;
	if(pC->Status.error[2] == On)
		errors |= 0x04;
	if(pC->Status.error[3] == On)
		errors |= 0x08;
	
	uint8_t* buf = pC->Com.bufwrite;
	ClearStr(buf, BUF_LEN);
	buf[0] = speed>>24;
	buf[1] = speed>>16;
	buf[2] = speed>>8;
	buf[3] = speed>>0;
	buf[4] = angle>>24;
	buf[5] = angle>>16;
	buf[6] = angle>>8;
	buf[7] = angle>>0;
	buf[8] = posx>>24;
	buf[9] = posx>>16;
	buf[10] = posx>>8;
	buf[11] = posx>>0;
	buf[12] = posy>>24;
	buf[13] = posy>>16;
	buf[14] = posy>>8;
	buf[15] = posy>>0;
	buf[16] = errors;
	uint16_t crc = crc16(buf, 17);
	buf[17] = crc>>8;
	buf[18] = crc>>0;
	
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;
	DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
	DMA2_Stream7->NDTR 	= (uint16_t)19;
	DMA2_Stream7->PAR 	= (uint32_t)&USART1->DR;
	DMA2_Stream7->M0AR 	= (uint32_t)pC->Com.bufwrite;
	DMA2_Stream7->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
}
//----------- Przerwania --------------------------------------------
void USART1_IRQHandler(void)
{
	if((USART1->SR & USART_SR_IDLE) != RESET) 
	{
		char c = USART1->DR;
		COM_ReadFromHost();
	}
}
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_SR_UIF) != RESET)
	{
		COM_SendToHost();
		TIM6->SR &= ~TIM_SR_UIF;
	}
}
