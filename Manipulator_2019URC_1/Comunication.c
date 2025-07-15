#include "Comunication.h"
extern sControl* pC;
void Com_Conf(void)
{	
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0;
	GPIOA->AFR[1] |= 0x00000770;
	
	DMA2_Stream2->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream2->M0AR 	= (uint32_t)pC->Com.hostbufread;
  DMA2_Stream2->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA2_Stream7->PAR 	= (uint32_t)&USART1->DR;
  DMA2_Stream7->M0AR 	= (uint32_t)pC->Com.hostbufwrite;
  DMA2_Stream7->NDTR 	= (uint16_t)BUF_LEN;
  DMA2_Stream7->CR 		|= DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
	
	USART1->BRR = 84000000/230400;
	USART1->CR3	|= USART_CR3_DMAR | USART_CR3_DMAT;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART1_IRQn);
}
void ClearStr(uint8_t* str, uint32_t l)
{
	for(uint32_t i=0;i<l;i++)
		str[i] = 0x00;
}
static uint16_t crc16(uint8_t* packet, uint32_t nBytes)
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
static void Com_ReadFrameGeneral(void)
{
	uint8_t* buf = pC->Com.hostbufread;
	uint16_t crc1 = crc16(buf, 11);
	uint16_t crc2 = (uint16_t)(buf[11]<<8) + (uint16_t)(buf[12]);
	if(crc1 == crc2)
	{
		LED3_TOG;
		LED4_OFF;
		pC->Axes.speedref[0] = (int8_t)buf[2];
		pC->Axes.speedref[1] = (int8_t)buf[3];
		pC->Axes.speedref[2] = (int8_t)buf[4];
		pC->Axes.speedref[3] = (int8_t)buf[5];
		pC->Axes.speedref[4] = (int8_t)buf[6];
		pC->Axes.speedref[5] = (int8_t)buf[7];
		pC->Axes.speedref[6] = (int8_t)buf[8];
		pC->Com.cmdin = (eHostCmd)buf[9];
		pC->Com.cmdinval = (eHostCmd)buf[10];
		pC->Status.hostcomtime = 0;
	}
}
static void Com_ReadFrameOnlyCmd(void)
{
	uint8_t* buf = pC->Com.hostbufread;
	uint16_t crc1 = crc16(buf, 4);
	uint16_t crc2 = (uint16_t)(buf[4]<<8) + (uint16_t)(buf[5]);
	if(crc1 == crc2)
	{
		pC->Com.cmdin = (eHostCmd)buf[2];
		pC->Com.cmdinval = (eHostCmd)buf[3];
		pC->Status.hostcomtime = 0;
	}
}
static void Com_ReadFrameSavePoint(void)
{
	uint8_t* buf = pC->Com.hostbufread;
	uint16_t crc1 = crc16(buf, 8);
	uint16_t crc2 = (uint16_t)(buf[8]<<8) + (uint16_t)(buf[9]);
	if(crc1 == crc2)
	{
		uint8_t seq = buf[2];
		uint8_t point = buf[3];
		uint8_t speed = buf[4];
		eAutoType type = (eAutoType)buf[5];
		int8_t gripperspeed = (int8_t)buf[6];
		uint8_t grippertime = buf[7];
		Axes_AutoSavePoint(seq, point, speed, type, gripperspeed, grippertime);
		Axes_AutoSavePointToBackup();
		pC->Status.hostcomtime = 0;
	}
}
static void Com_ReadFrameEmergencyStop(void)
{
	uint8_t* buf = pC->Com.hostbufread;
	uint16_t crc1 = crc16(buf, 3);
	uint16_t crc2 = (uint16_t)(buf[3]<<8) + (uint16_t)(buf[4]);
	if(crc1 == crc2)
	{
		LED4_TOG;
		LED3_OFF;
		pC->Axes.speedref[0] = 0;
		pC->Axes.speedref[1] = 0;
		pC->Axes.speedref[2] = 0;
		pC->Axes.speedref[3] = 0;
		pC->Axes.speedref[4] = 0;
		pC->Axes.speedref[5] = 0;
		pC->Axes.speedref[6] = 0;
		if(pC->Status.coordinate != Join)		pC->Com.cmdin = Hostcmd_join;
		else																pC->Com.cmdin = Hostcmd_null;
		pC->Com.cmdinval = 0;
		pC->Status.hostcomtime = 0;
	}
}
static void Com_ReadFromHost(void)
{
	uint8_t* buf = pC->Com.hostbufread;
	if(buf[0] == Frametype_Header)
	{
		if(buf[1] == Frametype_ManipGeneral)
		{
			Com_ReadFrameGeneral();
		}
		else if(buf[1] == Frametype_ManipCmd)
		{
			Com_ReadFrameOnlyCmd();
		}
		else if(buf[1] == Frametype_ManipSave)
		{
			Com_ReadFrameSavePoint();
		}
		else if(buf[1] == Frametype_EmergencyStop)
		{
			Com_ReadFrameEmergencyStop();
		}
	}
	ClearStr(buf, BUF_LEN);
	DMA2_Stream2->CR 		&= ~DMA_SxCR_EN;
	DMA2->LIFCR 				|= DMA_LIFCR_CTCIF2;
  DMA2_Stream2->CR 		|= DMA_SxCR_EN;
}
static void Com_PrepareFrameTelemetry(void)
{
	uint8_t value1 = 0x00;
	//bity 0. i 1. sygnalizuje stan aktualny uklad wsp.: join = 0x00, tool = 0x01, base = 0x02, autoptp = 0x03
	if(pC->Status.coordinate == Join)
		value1 |= 0x00;
	else if(pC->Status.coordinate == Tool)
		value1 |= 0x01;
	else if(pC->Status.coordinate == Base)
		value1 |= 0x02;
	else if(pC->Status.coordinate == Ptp)
		value1 |= 0x03;
	//bit 4. sygnalizuje czy skalibrowano manipulator
	if(pC->Status.calibrated == On)
		value1 |= 0x10;
	
	uint8_t value2 = 0x00;
	//bity 0. do 7. sygnalizuja blad napedu: wyjscie poza zakres lub przeciazenie
	for(uint8_t i=0;i<AXESMAX;i++)
		if(pC->Mot.error[i] == On)
			value2 |= (1<<i);
	
	uint8_t *buf = pC->Com.frames[Framenum_Telemetry].frame;
	buf[0] = Frametype_Header;
	buf[1] = (uint8_t)FrameType_ManipTelemetry;
	buf[2] = value1;
	buf[3] = value2;
	buf[4] = 0;
	uint16_t crc = crc16(buf, 5);
	buf[5] = (uint8_t)(crc>>8);
	buf[6] = (uint8_t)crc;
	pC->Com.frames[Framenum_Telemetry].full = On;
	pC->Com.frames[Framenum_Telemetry].len = 7;
}
static void Com_PrepareFrameMotPos(void)
{
	int16_t posmot[6];
	for(uint8_t i=0;i<6;i++)
		posmot[i] = (pC->Mot.pos[i] * 1000.0);
	
	uint8_t *buf = pC->Com.frames[Framenum_MotPos].frame;
	ClearStr(buf, BUF_LEN);
	uint8_t index=0;
	buf[index++] = Frametype_Header;
	buf[index++] = FrameType_ManipFeedbackMotpos;
	for(uint8_t i=0;i<6;i++)
	{
		buf[index++] = (uint8_t)((uint16_t)posmot[i] >> 8);
		buf[index++] = (uint8_t)((uint16_t)posmot[i] >> 0);
	}
	uint16_t crc = crc16(buf, 14);
	buf[index++] = (uint8_t)(crc >> 8);
	buf[index++] = (uint8_t)(crc >> 0);
	
	pC->Com.frames[Framenum_MotPos].full = On;
	pC->Com.frames[Framenum_MotPos].len = index;
}
void Com_SendToHost(void)
{
	Com_PrepareFrameTelemetry();
	Com_PrepareFrameMotPos();
	for(uint8_t i=0;i<FRAMEMAX;i++)
	{
		if(pC->Com.framenum++ >= FRAMEMAX)
			pC->Com.framenum = 0;
		uint8_t num = pC->Com.framenum;
		if(pC->Com.frames[num].active == On && pC->Com.frames[num].full == On)
		{
			ClearStr(pC->Com.hostbufwrite, BUF_LEN);
			for(uint16_t j=0;j<pC->Com.frames[num].len;j++)
				pC->Com.hostbufwrite[j] = pC->Com.frames[num].frame[j];
			DMA2_Stream7->CR 		&= ~DMA_SxCR_EN;
			DMA2->HIFCR 				|= DMA_HIFCR_CTCIF7;
			DMA2_Stream7->NDTR 	= pC->Com.frames[num].len;
			DMA2_Stream7->CR 		|= DMA_SxCR_EN;
			pC->Com.frames[num].full = Off;
			break;
		}
		else
		{
			continue;
		}
	}
}
void USART1_IRQHandler(void)
{
	if((USART1->SR & USART_SR_IDLE) != RESET)
	{
		char c = USART1->DR;
		Com_ReadFromHost();
	}
}
