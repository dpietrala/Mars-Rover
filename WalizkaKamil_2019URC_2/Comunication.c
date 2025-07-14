#include "Comunication.h"
extern sControl* pC;
void COM_Conf(void)
{
	//TIM6 Interrupt ********************************************************************************
	TIM6->PSC = 840-1;
	TIM6->ARR = 7000-1;	//Przerwanie co 70 ms
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
	//TIM2 Interrupt ********************************************************************************
	TIM2->PSC = 840-1;
	TIM2->ARR = 10000-1;	//Przerwanie co 100 ms
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM2_IRQn);
	
	//USART1 DMA2-Stream5,7 PA9, PA10 CHANNEL 4
	DMA2_Stream5->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream5->M0AR 	= (uint32_t)pC->Com.frames[Fnum_Us1].rbuf;
  DMA2_Stream5->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream5->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA2_Stream7->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream7->M0AR 	= (uint32_t)pC->Com.frames[Fnum_Us1].wbuf;
  DMA2_Stream7->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream7->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0;
  GPIOA->AFR[1] |= 0x00000770;
  USART1->BRR = 84000000/115200;
  USART1->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn);
	
	//USART2 DMA1-Stream5,6 PD5, PD6 CHANNEL 4
	DMA1_Stream5->PAR 	= (uint32_t)&USART2->DR;
  DMA1_Stream5->M0AR 	= (uint32_t)pC->Com.frames[Fnum_Us2].rbuf;
  DMA1_Stream5->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream5->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA1_Stream6->PAR 	= (uint32_t)&USART2->DR;
  DMA1_Stream6->M0AR 	= (uint32_t)pC->Com.frames[Fnum_Us2].wbuf;
  DMA1_Stream6->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream6->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;
  GPIOD->PUPDR |= GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0;
  GPIOD->AFR[0] |= 0x07700000;
  USART2->BRR = 42000000/115200;
  USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(USART2_IRQn);
	
	//UART4 DMA1-Stream2,4 PC10, PC11 CHANNEL 4 SATEL
	DMA1_Stream2->PAR 	= (uint32_t)&UART4->DR;
  DMA1_Stream2->M0AR 	= (uint32_t)pC->Com.bufsatelread;
  DMA1_Stream2->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA1_Stream4->PAR 	= (uint32_t)&UART4->DR;
  DMA1_Stream4->M0AR 	= (uint32_t)pC->Com.bufsatelwrite;
  DMA1_Stream4->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream4->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOC->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
  GPIOC->AFR[1] |= 0x00008800;
  UART4->BRR = 42000000/19200;
  UART4->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  UART4->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(UART4_IRQn);
	
	//UART5 DMA1-Stream0,7 PD2, PC12 CHANNEL 4
	DMA1_Stream0->PAR 	= (uint32_t)&UART5->DR;
  DMA1_Stream0->M0AR 	= (uint32_t)pC->Com.bufethread;
  DMA1_Stream0->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream0->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA1_Stream7->PAR 	= (uint32_t)&UART5->DR;
  DMA1_Stream7->M0AR 	= (uint32_t)pC->Com.bufethwrite;
  DMA1_Stream7->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream7->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOD->MODER |= GPIO_MODER_MODE2_1;
  GPIOD->PUPDR |= GPIO_PUPDR_PUPD2_0;
	GPIOC->MODER |= GPIO_MODER_MODE12_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD12_0;
  GPIOD->AFR[0] |= 0x00000800;
	GPIOC->AFR[1] |= 0x00080000;
  UART5->BRR = 42000000/57600;
  UART5->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  UART5->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(UART5_IRQn);
	
	//USART6 DMA2-Stream2,6 PC6, PC7 CHANNEL 5
	DMA2_Stream2->PAR 	= (uint32_t)&USART6->DR;
  DMA2_Stream2->M0AR 	= (uint32_t)pC->Com.frames[Fnum_Us6].rbuf;
  DMA2_Stream2->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0 | DMA_SxCR_EN;
  	
  DMA2_Stream6->PAR 	= (uint32_t)&USART6->DR;
  DMA2_Stream6->M0AR 	= (uint32_t)pC->Com.frames[Fnum_Us6].wbuf;
  DMA2_Stream6->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream6->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0;
  	
  GPIOC->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
  GPIOC->AFR[0] |= 0x88000000;
  USART6->BRR = 84000000/115200;
  USART6->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  USART6->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(USART6_IRQn);
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
///////////////////////////////////////////////////////////////////////
static void COM_ReadFromSatel(void)
{
	ClearStr(pC->Com.frames[Fnum_Us1].wbuf, BUF_LEN);
	ClearStr(pC->Com.frames[Fnum_Us2].wbuf, BUF_LEN);
	ClearStr(pC->Com.frames[Fnum_Us6].wbuf, BUF_LEN);
	ClearStr(pC->Com.buftoanalysis, BUF_LEN);
	uint8_t len = BUF_LEN - DMA1_Stream2->NDTR;
	for(uint8_t i=0;i<len;i++)
	{
		pC->Com.frames[Fnum_Us1].wbuf[i] = pC->Com.bufsatelread[i];
		pC->Com.frames[Fnum_Us2].wbuf[i] = pC->Com.bufsatelread[i];
		pC->Com.frames[Fnum_Us6].wbuf[i] = pC->Com.bufsatelread[i];
		pC->Com.buftoanalysis[i] = pC->Com.bufsatelread[i];
	}
	//umozliwienie kolejnego odbioru przez satel UART4
	DMA1_Stream2->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF2;
  DMA1_Stream2->CR 		|= DMA_SxCR_EN;
	
	//rozpoczecie wysylania do komuterów USART1
  DMA2_Stream7->CR 		&= ~DMA_SxCR_EN;
	DMA2_Stream7->NDTR 	= (uint16_t)len;
	DMA2->HIFCR 				|= DMA_HIFCR_CTCIF7;
  DMA2_Stream7->CR 		|= DMA_SxCR_EN;
	
	//rozpoczecie wysylania do komuterów USART2
	DMA1_Stream6->CR 		&= ~DMA_SxCR_EN;
  DMA1_Stream6->NDTR 	= (uint16_t)len;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF6;
  DMA1_Stream6->CR 		|= DMA_SxCR_EN;
	
	//rozpoczecie wysylania do komuterów USART6
	DMA2_Stream6->CR 		&= ~DMA_SxCR_EN;
  DMA2_Stream6->NDTR 	= (uint16_t)len;
	DMA2->HIFCR 				|= DMA_HIFCR_CTCIF6;
  DMA2_Stream6->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromEth(void)
{
	ClearStr(pC->Com.frames[Fnum_Us1].wbuf, BUF_LEN);
	ClearStr(pC->Com.frames[Fnum_Us2].wbuf, BUF_LEN);
	ClearStr(pC->Com.frames[Fnum_Us6].wbuf, BUF_LEN);
	ClearStr(pC->Com.buftoanalysis, BUF_LEN);
	uint8_t len = BUF_LEN - DMA1_Stream0->NDTR;
	for(uint8_t i=0;i<len;i++)
	{
		pC->Com.frames[Fnum_Us1].wbuf[i] = pC->Com.bufethread[i];
		pC->Com.frames[Fnum_Us2].wbuf[i] = pC->Com.bufethread[i];
		pC->Com.frames[Fnum_Us6].wbuf[i] = pC->Com.bufethread[i];
		pC->Com.buftoanalysis[i] = pC->Com.bufethread[i];
	}
	//umozliwienie kolejnego odbioru przez satel UART4
	DMA1_Stream0->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF0; 
  DMA1_Stream0->CR 		|= DMA_SxCR_EN;
	
	//rozpoczecie wysylania do komuterów USART1
  DMA2_Stream7->CR 		&= ~DMA_SxCR_EN;
	DMA2_Stream7->NDTR 	= (uint16_t)len;
	DMA2->HIFCR 				|= DMA_HIFCR_CTCIF7;
  DMA2_Stream7->CR 		|= DMA_SxCR_EN;
	
	//rozpoczecie wysylania do komuterów USART2
	DMA1_Stream6->CR 		&= ~DMA_SxCR_EN;
  DMA1_Stream6->NDTR 	= (uint16_t)len;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF6;
  DMA1_Stream6->CR 		|= DMA_SxCR_EN;
	
	//rozpoczecie wysylania do komuterów USART6
	DMA2_Stream6->CR 		&= ~DMA_SxCR_EN;
  DMA2_Stream6->NDTR 	= (uint16_t)len;
	DMA2->HIFCR 				|= DMA_HIFCR_CTCIF6;
  DMA2_Stream6->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromUSART1(void)
{
	pC->Com.frames[Fnum_Us1].rfull = On;
	pC->Com.frames[Fnum_Us1].rlen = (BUF_LEN - DMA2_Stream5->NDTR);
	DMA2_Stream5->CR 		&= ~DMA_SxCR_EN;
	DMA2->HIFCR 				|= DMA_HIFCR_CTCIF5;
  DMA2_Stream5->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromUSART2(void)
{
	pC->Com.frames[Fnum_Us2].rfull = On;
	pC->Com.frames[Fnum_Us2].rlen = (BUF_LEN - DMA1_Stream5->NDTR);
	DMA1_Stream5->CR 		&= ~DMA_SxCR_EN;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF5; 
  DMA1_Stream5->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromUSART6(void)
{
	pC->Com.frames[Fnum_Us6].rfull = On;
	pC->Com.frames[Fnum_Us6].rlen = (BUF_LEN - DMA2_Stream2->NDTR);
	DMA2_Stream2->CR 		&= ~DMA_SxCR_EN;
	DMA2->LIFCR 				|= DMA_LIFCR_CTCIF2; 
  DMA2_Stream2->CR 		|= DMA_SxCR_EN;
}
void COM_PrepareFrameToDrive(void)
{
	sFrame *f = &pC->Com.frames[Fnum_Drive];
	uint8_t *buf = f->rbuf;
	ClearStr(buf, BUF_LEN);
	buf[0] = (uint8_t)Frametype_Header;
	buf[1] = (uint8_t)Frametype_DriveGeneral;
	buf[2] = (int8_t)pC->Case.frontspeed_L;
	buf[3] = (int8_t)pC->Case.dirspeed_R;
	buf[4] = (int8_t)-pC->Case.gimbal_up[0];
	buf[5] = (int8_t)pC->Case.gimbal_up[1];
	buf[6] = (int8_t)pC->Case.gimbal_down[0];
	buf[7] = (int8_t)pC->Case.gimbal_down[1];
	buf[8] = (uint8_t)f->cmd;
	uint16_t crc = crc16(buf, 9);
	buf[9] = crc>>8;
	buf[10] = crc>>0;
	f->rlen = 11;
	f->rfull = On;
}
void COM_PrepareFrameToManip(void)
{
	sFrame *f = &pC->Com.frames[Fnum_Manip];
	uint8_t *buf = pC->Com.frames[Fnum_Manip].rbuf;
	ClearStr(buf, BUF_LEN);
	buf[0] = (uint8_t)Frametype_Header;
	buf[1] = (uint8_t)Frametype_ManipGeneral;
	if(pC->Case.manip_x_L == On 	||  pC->Case.manip_x_R == On 	|| pC->Case.manip_y_L == On 	|| 	pC->Case.manip_y_R == On 	|| 
		 pC->Case.manip_z_L == On 	||	pC->Case.manip_z_R == On 	|| pC->Case.manip_Rx_L == On 	||  pC->Case.manip_Rx_R == On ||
		 pC->Case.manip_Ry_L == On ||  pC->Case.manip_Ry_R == On || pC->Case.manip_Rz_L == On 	||  pC->Case.manip_Rz_R == On)
	{
		buf[2] = (uint8_t)-pC->Case.manip_x_speed;
		buf[3] = (uint8_t)-pC->Case.manip_y_speed;
		buf[4] = (uint8_t)-pC->Case.manip_z_speed;
		buf[5] = (uint8_t)pC->Case.manip_Rx_speed;
		buf[6] = (uint8_t)-pC->Case.manip_Ry_speed;
		buf[7] = (uint8_t)pC->Case.manip_Rz_speed;
	}
	else
	{
		if(pC->Tele.manipcs == ManipJoin)
		{
			buf[2] = (uint8_t)-pC->Case.manip_joy_y;
			buf[3] = (uint8_t)-pC->Case.manip_joy_x;
			buf[4] = (uint8_t)-pC->Case.manip_joy_z;
			buf[5] = (uint8_t)pC->Case.manip_joy_Ry;
			buf[6] = (uint8_t)pC->Case.manip_joy_Rx;
			buf[7] = (uint8_t)pC->Case.manip_joy_Rz;
		}
		else if(pC->Tele.manipcs == ManipTool)
		{
			buf[2] = (uint8_t)-pC->Case.manip_joy_y;
			buf[3] = (uint8_t)pC->Case.manip_joy_x;
			buf[4] = (uint8_t)pC->Case.manip_joy_z;
			buf[5] = (uint8_t)pC->Case.manip_joy_Ry;
			buf[6] = (uint8_t)pC->Case.manip_joy_Rx;
			buf[7] = (uint8_t)pC->Case.manip_joy_Rz;
		}
		else if(pC->Tele.manipcs == ManipBase)
		{
			buf[2] = (uint8_t)-pC->Case.manip_joy_y;
			buf[3] = (uint8_t)pC->Case.manip_joy_x;
			buf[4] = (uint8_t)pC->Case.manip_joy_z;
			buf[5] = (uint8_t)pC->Case.manip_joy_Ry;
			buf[6] = (uint8_t)pC->Case.manip_joy_Rx;
			buf[7] = (uint8_t)pC->Case.manip_joy_Rz;
		}
	}
	buf[8] = (int8_t)-pC->Case.gripper_speed;
	buf[9] = (uint8_t)f->cmd;
	buf[10] = (uint8_t)f->cmdval;
	uint16_t crc = crc16(buf, 11);
	buf[11] = crc>>8;
	buf[12] = crc>>0;
	f->rlen = 13;
	f->rfull = On;
}
void COM_PrepareFrameToLab(void)
{
	sFrame *f = &pC->Com.frames[Fnum_Lab];
	uint8_t *buf = pC->Com.frames[Fnum_Lab].rbuf;
	ClearStr(buf, BUF_LEN);
	buf[0] = (uint8_t)Frametype_Header;
	buf[1] = (uint8_t)Frametype_LabGeneral;
	if(pC->Case.manip_x_L == On	|| pC->Case.manip_x_R == On	|| pC->Case.manip_y_L == On	|| 	pC->Case.manip_y_R == On 	|| pC->Case.manip_z_L == On	|| pC->Case.manip_z_R == On	|| pC->Case.manip_Rx_L == On || pC->Case.manip_Rx_R == On)
	{
		buf[2] = (uint8_t)pC->Case.manip_x_speed;
		buf[3] = (uint8_t)pC->Case.manip_y_speed;
		buf[4] = (uint8_t)pC->Case.manip_z_speed;
		buf[5] = (uint8_t)pC->Case.manip_Rx_speed;
	}
	buf[6] = (uint8_t)f->cmd;
	buf[7] = (uint8_t)f->cmdval;
	uint16_t crc = crc16(buf, 8);
	buf[8] = crc>>8;
	buf[9] = crc>>0;
	f->rlen = 10;
	f->rfull = On;
}
static void COM_PreapareFrameStationPos(void)
{
	int32_t pos = 	-45.0 * pC->Case.station_pos;
	int32_t speed = 2000;
	int32_t accel = 4000;
	int32_t deccel = 4000;
	
	uint8_t *buf = pC->Com.frames[Fnum_SP].rbuf;
	ClearStr(buf, BUF_LEN);
	
	buf[0] = 128;
	buf[1] = 65;
	buf[2] = accel >> 24;
	buf[3] = accel >> 16;
	buf[4] = accel >> 8;
	buf[5] = accel >> 0;
	buf[6] = speed >> 24;
	buf[7] = speed >> 16;
	buf[8] = speed >> 8;
	buf[9] = speed >> 0;
	buf[10] = deccel >> 24;
	buf[11] = deccel >> 16;
	buf[12] = deccel >> 8;
	buf[13] = deccel >> 0;
	buf[14] = pos >> 24;
	buf[15] = pos >> 16;
	buf[16] = pos >> 8;
	buf[17] = pos >> 0;
	buf[18] = 1;
	uint16_t crc = crc16(buf, 19);
	buf[19] = crc>>8;
	buf[20] = crc>>0;
	
	pC->Com.frames[Fnum_SP].rlen = 21;
}
static void Com_SendToSatel(sFrame *f)
{
	ClearStr(pC->Com.bufsatelwrite, BUF_LEN);
	for(uint8_t i=0;i<BUF_LEN;i++)
		pC->Com.bufsatelwrite[i] = f->rbuf[i];
	
	DMA1_Stream4->CR 		&= ~DMA_SxCR_EN;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF4;
	DMA1_Stream4->NDTR 	= (uint16_t)f->rlen;
	DMA1_Stream4->CR 		|= DMA_SxCR_EN;
}
static void Com_SendToEth(sFrame *f)
{
	ClearStr(pC->Com.bufethwrite, BUF_LEN);
	for(uint8_t i=0;i<BUF_LEN;i++)
		pC->Com.bufethwrite[i] = f->rbuf[i];
	
	DMA1_Stream7->CR 		&= ~DMA_SxCR_EN;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF7;
	DMA1_Stream7->NDTR 	= (uint16_t)f->rlen;
	DMA1_Stream7->CR 		|= DMA_SxCR_EN;
}
static void COM_SendToHost(void)
{
	COM_PrepareFrameToDrive();
	#ifdef MANIPULATOR
	COM_PrepareFrameToManip();
	#endif
	#ifdef LABORATORIUM
	COM_PrepareFrameToLab();
	#endif
	COM_PreapareFrameStationPos();
	for(uint8_t i=0;i<FRAMEMAX;i++)
	{
		if(pC->Com.framenum++ >= FRAMEMAX)
			pC->Com.framenum = 0;
		if(pC->Com.frames[pC->Com.framenum].rfull == On)
		{
			Com_SendToSatel(&pC->Com.frames[pC->Com.framenum]);
			Com_SendToEth(&pC->Com.frames[pC->Com.framenum]);
			pC->Com.frames[pC->Com.framenum].rfull = Off;
			pC->Com.frames[pC->Com.framenum].cmd = 0;
			pC->Com.frames[pC->Com.framenum].cmdval = 0;
			break;
		}
		else
		{
			continue;
		}
	}
}
static void COM_ReadTelemetryFrame(void)
{
	uint8_t* buf = pC->Com.buftoanalysis;
	uint16_t crc1 = 0, crc2 = 0;
	
	if(buf[0] == Frametype_Header) 
	{
		if(buf[1] == FrameType_DriveTelemetry) 
		{
			crc1 = crc16(buf, 5);
			crc2 = (uint16_t)(buf[5]<<8) + (uint16_t)buf[6];
			if(crc1 == crc2)
			{
				if((buf[2] & 0x01) == 0x01)					pC->Tele.emer_hard = On;
				else 																pC->Tele.emer_hard = Off;
				if((buf[2] & 0x02) == 0x02)					pC->Tele.emer_soft = On;
				else 																pC->Tele.emer_soft = Off;
				
				if((buf[2] & 0x0c) == 0x00)					pC->Tele.hostname = Satel;
				else if((buf[2] & 0x0c) == 0x04)		pC->Tele.hostname = Wifi;
				else if((buf[2] & 0x0c) == 0x08)		pC->Tele.hostname = AutoStm;
				else if((buf[2] & 0x0c) == 0x0c)		pC->Tele.hostname = AutoNV;
				
				if((buf[2] & 0x10) == 0x10)					pC->Tele.drivemoter[0] = On;
				else																pC->Tele.drivemoter[0] = Off;
				if((buf[2] & 0x20) == 0x20)					pC->Tele.drivemoter[1] = On;
				else																pC->Tele.drivemoter[1] = Off;
				if((buf[2] & 0x40) == 0x40)					pC->Tele.drivemoter[2] = On;
				else																pC->Tele.drivemoter[2] = Off;
				if((buf[2] & 0x80) == 0x80)					pC->Tele.drivemoter[3] = On;
				else																pC->Tele.drivemoter[3] = Off;
				
				pC->Tele.roverspeed = (double)buf[3] / 256.0 * 8.482300164;
			}
		}
		else if(buf[1] == Frametype_DriveFeadbackGPS)
		{
			crc1 = crc16(buf, 19);
			crc2 = (uint16_t)(buf[19]<<8) + (uint16_t)buf[20];
			if(crc1 == crc2)
			{
				pC->Tele.imuyaw =    (double)((int16_t)(((uint16_t)buf[2]<<8) + ((uint16_t)buf[3]<<0))) / 10.0;
				pC->Tele.imupitch =  (double)((int16_t)(((uint16_t)buf[4]<<8) + ((uint16_t)buf[5]<<0))) / 10.0;
				pC->Tele.imuroll =   (double)((int16_t)(((uint16_t)buf[6]<<8) + ((uint16_t)buf[7]<<0))) / 10.0;
				pC->Tele.gpslon =    (double)((int64_t)(((uint64_t)buf[8]<<32) + ((uint64_t)buf[9]<<24) + ((uint64_t)buf[10]<<16) + ((uint64_t)buf[11]<<8) + ((uint64_t)buf[12]<<0))) / 10000000.0;
				pC->Tele.gpslat =    (double)(((int64_t)((uint64_t)buf[13]<<32) + ((uint64_t)buf[14]<<24) + ((uint64_t)buf[15]<<16) + ((uint64_t)buf[16]<<8) + ((uint64_t)buf[17]<<0))) / 10000000.0;
				pC->Tele.autostate = (eAutoState)(buf[18]);
				if(fabs(pC->Tele.imuroll) > fabs(pC->Tele.imurollmax))			pC->Tele.imuroller = On;
				else																												pC->Tele.imuroller = Off;
				if(fabs(pC->Tele.imupitch) > fabs(pC->Tele.imupitchmax))		pC->Tele.imupitcher = On;
				else																												pC->Tele.imupitcher = Off;
					
			}
		}
	  else if(buf[1] == FrameType_ManipTelemetry)
		{
			crc1 = crc16(buf, 5);
			crc2 = (uint16_t)(buf[5]<<8) + (uint16_t)buf[6];
			if(crc1 == crc2)
			{
				if((buf[2] & 0x03) == 0x00 && pC->Tele.manipcs != ManipJoin)
					buzzer_on();
				else if((buf[2] & 0x03) == 0x01 && pC->Tele.manipcs != ManipTool)
					buzzer_on();
				else if((buf[2] & 0x03) == 0x02 && pC->Tele.manipcs != ManipBase)
					buzzer_on();
				else if((buf[2] & 0x03) == 0x03 && pC->Tele.manipcs != ManipAutpPtp)
					buzzer_on();
				
				if((buf[2] & 0x03) == 0x00)					pC->Tele.manipcs = ManipJoin;
				else if((buf[2] & 0x03) == 0x01)		pC->Tele.manipcs = ManipTool;
				else if((buf[2] & 0x03) == 0x02)		pC->Tele.manipcs = ManipBase;
				else if((buf[2] & 0x03) == 0x03)		pC->Tele.manipcs = ManipAutpPtp;
				
				if((buf[2] & 0x10)== 0x10)					pC->Tele.manipcalibrated = On;
				else																pC->Tele.manipcalibrated = Off;
				
				if ((buf[3] & 0x01) != RESET)				pC->Tele.manipmoter[0] = On;
				else																pC->Tele.manipmoter[0] = Off;
				if ((buf[3] & 0x02) != RESET)				pC->Tele.manipmoter[1] = On;
				else																pC->Tele.manipmoter[1] = Off;
				if ((buf[3] & 0x04) != RESET)				pC->Tele.manipmoter[2] = On;
				else																pC->Tele.manipmoter[2] = Off;
				if ((buf[3] & 0x08) != RESET)				pC->Tele.manipmoter[3] = On;
				else																pC->Tele.manipmoter[3] = Off;
				if ((buf[3] & 0x10) != RESET)				pC->Tele.manipmoter[4] = On;
				else																pC->Tele.manipmoter[4] = Off;
				if ((buf[3] & 0x20) != RESET)				pC->Tele.manipmoter[5] = On;
				else																pC->Tele.manipmoter[5] = Off;
				if ((buf[3] & 0x40) != RESET)				pC->Tele.manipmoter[6] = On;
				else																pC->Tele.manipmoter[6] = Off;
			}
		}
		else if(buf[1] == FrameType_LabTelemetry)
		{
			crc1 = crc16(buf, 5);
			crc2 = (uint16_t)(buf[5]<<8) + (uint16_t)buf[6];
			if(crc1 == crc2)
			{
				if((buf[2] & 0x01) == 0x01)					pC->Tele.labmoter[0] = On;
				else																pC->Tele.labmoter[0] = Off;
				if((buf[2] & 0x02) == 0x02)					pC->Tele.labmoter[1] = On;
				else																pC->Tele.labmoter[1] = Off;
				if((buf[2] & 0x04) == 0x04)					pC->Tele.labmoter[2] = On;
				else																pC->Tele.labmoter[2] = Off;
				if((buf[2] & 0x08) == 0x08)					pC->Tele.labmoter[3] = On;
				else																pC->Tele.labmoter[3] = Off;
				
				if((buf[2] & 0x10) == 0x10)					pC->Tele.labgripstate = On;
				else																pC->Tele.labgripstate = Off;
				if((buf[2] & 0x20) == 0x20)					pC->Tele.lablumiopenstate = On;
				else																pC->Tele.lablumiopenstate = Off;
				if((buf[2] & 0x40) == 0x40)					pC->Tele.lablumiturnstate = LSOn;
				else if((buf[2] & 0x80) == 0x80)		pC->Tele.lablumiturnstate = LSMeas;
				else																pC->Tele.lablumiturnstate = LSNeutral;
				
				if((buf[3] & 0x01) == 0x01)					pC->Tele.labwibstate = On;
				else																pC->Tele.labwibstate = Off;
				if((buf[3] & 0x06) == 0x00)					pC->Tele.labwork = LabWorkoff;
				else if((buf[3] & 0x06) == 0x02)		pC->Tele.labwork = LabWorkmanual;
				else if((buf[3] & 0x06) == 0x04)		pC->Tele.labwork = LabWorkremote;
				else if((buf[3] & 0x06) == 0x06)		pC->Tele.labwork = LabWorkauto;
				
				if((buf[3] & 0x10) == 0x10)					pC->Tele.labmotlimit[0] = On;
				else																pC->Tele.labmotlimit[0] = Off;	
				if((buf[3] & 0x20) == 0x20)					pC->Tele.labmotlimit[1] = On;
				else																pC->Tele.labmotlimit[1] = Off;	
				if((buf[3] & 0x40) == 0x40)					pC->Tele.labmotlimit[2] = On;
				else																pC->Tele.labmotlimit[2] = Off;	
				if((buf[3] & 0x80) == 0x80)					pC->Tele.labmotlimit[3] = On;
				else																pC->Tele.labmotlimit[3] = Off;
			}
		}
	}
}
static void COM_LEDRefresh(void)
{
	//-----------Diody od jazdy lub ogólnego przeznaczenia---------------------------------
	if(pC->Tele.emer_hard == On)								LEDRedButton_TOG;
	else if(pC->Tele.emer_soft == On)						LEDRedButton_ON;
	else																				LEDRedButton_OFF;
	
	if(pC->Tele.autostate == ASManual)			
	{
		LEDAuto1_OFF;
		LEDAuto2_OFF;
	}
	else if(pC->Tele.autostate == ASWait)			
	{
		LEDAuto1_ON;
		LEDAuto2_OFF;
	}
	else if(pC->Tele.autostate == ASDrivetopoint)
	{
		LEDAuto1_TOG;
		LEDAuto2_OFF;
	}
	else if(pC->Tele.autostate == ASLookball)
	{
		LEDAuto1_TOG;
		LEDAuto2_TOG;
	}
	else if(pC->Tele.autostate == ASDrivetoball)
	{
		LEDAuto1_TOG;
		LEDAuto2_ON;
	}
	else if(pC->Tele.autostate == ASFoundball)
	{
		LEDAuto1_ON;
		LEDAuto2_ON;
	}
	
	if(pC->Tele.imuroller == On || pC->Tele.imupitcher == On)		LEDImu_TOG;
	else																												LEDImu_OFF;
	if(pC->Tele.drivemoter[0] == On)			LEDWheel1_TOG;
	else																	LEDWheel1_OFF;
	if(pC->Tele.drivemoter[1] == On)			LEDWheel2_TOG;
	else																	LEDWheel2_OFF;
	if(pC->Tele.drivemoter[2] == On)			LEDWheel3_TOG;
	else																	LEDWheel3_OFF;
	if(pC->Tele.drivemoter[3] == On)			LEDWheel4_TOG;
	else																	LEDWheel4_OFF;
	
	//--------------Diody od manipulatora---------------------------------------------
	#ifdef MANIPULATOR
	if(pC->Tele.manipcalibrated == On)
	{
		LEDJoint_ON;
		LEDTool_ON;
		LEDBase_ON;
	}
	else if(pC->Tele.manipcs == ManipJoin)
	{
		LEDJoint_ON;
		LEDTool_OFF;
		LEDBase_OFF;
	}
	else if(pC->Tele.manipcs == ManipTool)
	{
		LEDJoint_OFF;
		LEDTool_ON;
		LEDBase_OFF;
	}
	else if(pC->Tele.manipcs == ManipBase)
	{
		LEDJoint_OFF;
		LEDTool_OFF;
		LEDBase_ON;
	}
	else if(pC->Tele.manipcs == ManipAutpPtp)
	{
		LEDJoint_TOG;
		LEDTool_TOG;
		LEDBase_TOG;
	}
	if(pC->Tele.manipmoter[0] == On)			LEDMotor1_TOG;
	else																	LEDMotor1_OFF;
	if(pC->Tele.manipmoter[1] == On)			LEDMotor2_TOG;
	else																	LEDMotor2_OFF;
	if(pC->Tele.manipmoter[2] == On)			LEDMotor3_TOG;
	else																	LEDMotor3_OFF;
	if(pC->Tele.manipmoter[3] == On)			LEDMotor4_TOG;
	else																	LEDMotor4_OFF;
	if(pC->Tele.manipmoter[4] == On)			LEDMotor5_TOG;
	else																	LEDMotor5_OFF;
	if(pC->Tele.manipmoter[5] == On)			LEDMotor6_TOG;
	else																	LEDMotor6_OFF;
	if(pC->Tele.manipmoter[6] == On)			LEDMotor7_TOG;
	else																	LEDMotor7_OFF;
	#endif
	
	//---------------Diody od laboratorium-----------------------------
	#ifdef LABORATORIUM
	if(pC->Tele.labmoter[0] == On)								LEDMotor1_TOG;
	else																					LEDMotor1_OFF;
	if(pC->Tele.labmoter[1] == On)								LEDMotor2_TOG;
	else																					LEDMotor2_OFF;
	if(pC->Tele.labmoter[2] == On)								LEDMotor3_TOG;
	else																					LEDMotor3_OFF;
	if(pC->Tele.labmoter[3] == On)								LEDMotor4_TOG;
	else																					LEDMotor4_OFF;
	if(pC->Tele.labgripstate == On)								LEDMotor7_TOG;
	else																					LEDMotor7_OFF;
	if(pC->Tele.lablumiopenstate == On)						LEDMotor5_TOG;
	else																					LEDMotor5_OFF;
	if(pC->Tele.lablumiturnstate == LSOn)					LEDMotor6_TOG;
	else if(pC->Tele.lablumiturnstate == LSMeas)	LEDMotor6_ON;
	else																					LEDMotor6_OFF;
	#endif
}
//----------- Przerwania --------------------------------------------
void USART1_IRQHandler(void)
{
	if((USART1->SR & USART_SR_IDLE) != RESET) 
	{
		char c = USART1->DR;
		COM_ReadFromUSART1();
	}
}
void USART2_IRQHandler(void)
{
	if((USART2->SR & USART_SR_IDLE) != RESET) 
	{
		char c = USART2->DR;
		COM_ReadFromUSART2();
	} 
}
//Satel
void UART4_IRQHandler(void)
{
	if((UART4->SR & USART_SR_IDLE) != RESET) 
	{
		char c = UART4->DR;
		COM_ReadFromSatel();
		COM_ReadTelemetryFrame();
	}
}
void UART5_IRQHandler(void)
{
	if((UART5->SR & USART_SR_IDLE) != RESET) 
	{
		char c = UART5->DR;
		COM_ReadFromEth();
		COM_ReadTelemetryFrame();
	}
}
void USART6_IRQHandler(void)
{
	if((USART6->SR & USART_SR_IDLE) != RESET) 
	{
		char c = USART6->DR;
		COM_ReadFromUSART6();
	}
}
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_SR_UIF) != RESET)
	{
		COM_SendToHost();
	}
	TIM6->SR &= ~TIM_SR_UIF;
}
void TIM2_IRQHandler(void)
{
	if((TIM2->SR & TIM_SR_UIF) != RESET)
	{
		COM_LEDRefresh();
		LCD_Dwin_SendToLcd();
		TIM2->SR &= ~TIM_SR_UIF;
	}
}
