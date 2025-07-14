#include "Comunication.h"
extern sControl* pC;
void COM_Conf(void)
{
	// ----------------- Timer do wysylania -------------------------------------------
	TIM6->PSC = 840-1;
	TIM6->ARR = 12500-1;	//Przerwanie co 125 ms
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
	// ---------------- USART5 RS422 Host, do plyty glównej --------------------------------------------
	DMA1_Stream0->PAR 	= (uint32_t)&UART5->DR;
  DMA1_Stream0->M0AR 	= (uint32_t)pC->Com.bufread;
  DMA1_Stream0->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream0->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA1_Stream7->PAR 	= (uint32_t)&UART5->DR;
  DMA1_Stream7->M0AR 	= (uint32_t)pC->Com.bufwrite;
  DMA1_Stream7->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream7->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOC->MODER |= GPIO_MODER_MODE12_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD12_0;
  GPIOC->AFR[1] |= 0x00080000;
	GPIOD->MODER |= GPIO_MODER_MODE2_1;
  GPIOD->PUPDR |= GPIO_PUPDR_PUPD2_0;
  GPIOD->AFR[0] |= 0x00000800;
  UART5->BRR = 42000000/230400;
  UART5->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  UART5->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(UART5_IRQn);
	
	// ---------------- USART2 RS422 Slave, do czujników --------------------------------------------
	DMA1_Stream5->PAR 	= (uint32_t)&USART2->DR;
  DMA1_Stream5->M0AR 	= (uint32_t)pC->Com.bufread;
  DMA1_Stream5->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream5->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA1_Stream6->PAR 	= (uint32_t)&USART2->DR;
  DMA1_Stream6->M0AR 	= (uint32_t)pC->Com.bufwrite;
  DMA1_Stream6->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream6->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;
  GPIOD->PUPDR |= GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0;
  GPIOD->AFR[0] |= 0x07700000;
  USART2->BRR = 42000000/230400;
  USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(USART2_IRQn);
	
// ---------------- Uart4 to USB --------------------------------------------------------
	DMA1_Stream2->PAR 	= (uint32_t)&UART4->DR;
  DMA1_Stream2->M0AR 	= (uint32_t)pC->Com.bufread;
  DMA1_Stream2->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  	
  DMA1_Stream4->PAR 	= (uint32_t)&UART4->DR;
  DMA1_Stream4->M0AR 	= (uint32_t)pC->Com.bufwrite;
  DMA1_Stream4->NDTR 	= (uint16_t)BUF_LEN;
  DMA1_Stream4->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2;
  	
  GPIOC->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
  GPIOC->AFR[1] |= 0x00008800;
  UART4->BRR = 42000000/230400;
  UART4->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  UART4->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(UART4_IRQn);
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
static void COM_ReadLabFrameGeneral(uint8_t* buf)
{
	uint16_t crc1 = crc16(buf, 8);
	uint16_t crc2 = (uint16_t)(buf[8]<<8) + (uint16_t)buf[9];
	if(crc1 == crc2)
	{
		pC->Drive.motspeedref[0] = (int8_t)buf[2];
		pC->Drive.motspeedref[1] = (int8_t)buf[3];
		pC->Drive.motspeedref[2] = (int8_t)buf[4];
		pC->Drive.motspeedref[3] = (int8_t)buf[5];
		pC->Com.cmd = (eCmd)buf[6];
		pC->Com.cmdval = buf[7];
		pC->Mode.hostcomtick = 0;
		
		for(uint8_t i=0;i<MOTMAX;i++)
			pC->Drive.motpid[i] = On;
	}
	ClearStr(buf, BUF_LEN);
}
static void COM_ReadLabFrameOnlyCmd(uint8_t* buf)
{
	uint16_t crc1 = crc16(buf, 4);
	uint16_t crc2 = (uint16_t)(buf[4]<<8) + (uint16_t)buf[5];
	if(crc1 == crc2)
	{
		pC->Com.cmd = (eCmd)buf[2];
		pC->Com.cmdval = buf[3];
		pC->Mode.hostcomtick = 0;
	}
	ClearStr(buf, BUF_LEN);
}
static void COM_ReadEmergencyStopFrame(uint8_t* buf)
{
	uint16_t crc1 = crc16(buf, 3);
	uint16_t crc2 = (uint16_t)(buf[3]<<8) + (uint16_t)buf[4];
	if(crc1 == crc2)
	{
		pC->Mode.work = Workoff;
		for(uint8_t i=0;i<MOTMAX;i++)
		{
			pC->Drive.motpid[i] = Off;
			pC->Drive.motspeedref[i] = 0;
			pC->Drive.motout[i] = 0;
			pC->Drive.motposabsref[i] = pC->Drive.motposabs[i];
			pC->Drive.motpwm[i] = 0;
			pC->Drive.motpwmout[i] = 0;
		}
		pC->Drive.wibstate = Off;
		MOT4PWM_REG = 0;
	}
	ClearStr(buf, BUF_LEN);
}
static void COM_ReadFromHost(void)
{
	if(pC->Mode.manual == On)
		return;
	uint8_t* buf = pC->Com.bufread;
	if(buf[0] == Frametype_Header && buf[1] == Frametype_LabGeneral)
	{
		COM_ReadLabFrameGeneral(buf);
	}
	else if(buf[0] == Frametype_Header && buf[1] == FrameType_LabCmd)
	{
		COM_ReadLabFrameOnlyCmd(buf);
	}
	else if(buf[0] == Frametype_Header && buf[1] == Frametype_EmergencyStop)
	{
		COM_ReadEmergencyStopFrame(buf);
	}
	DMA1_Stream0->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF0;
  DMA1_Stream0->CR 		|= DMA_SxCR_EN;
	
	DMA1_Stream2->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF2;
  DMA1_Stream2->CR 		|= DMA_SxCR_EN;
	
	DMA1_Stream5->CR 		&= ~DMA_SxCR_EN;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF5;
  DMA1_Stream5->CR 		|= DMA_SxCR_EN;
}
static void COM_PrepareFrameTelemetry(void)
{
	uint8_t value1 = 0x00;
	//bity 0. do 3. sygnalizuja blad napedu: wyjscie poza zakres lub przeciazenie
	for(uint8_t i=0;i<MOTMAX;i++)
		if(pC->Drive.moterror[i] == On)
			value1 |= (1<<i);
	//bit 4. sygnalizuja stan serwa chwytaka 0 - otwarty, 1 - zamkniety
		if(pC->Serw.gripstate == On)
			value1 |= 0x10;
	//bit 5. sygnalizuja stan serwa klapki luminotestera 0 - otwarta, 1 - zamknieta
		if(pC->Serw.openstate == On)
			value1 |= 0x20;
	//bity 6. i 7. sygnalizuja stan serwa wlaczenia luminotestera 0 - neutral, 1 - wlaczenie, 2 - pomiar
		if(pC->Serw.turnstate == LSOn)
			value1 |= 0x40;
		else if(pC->Serw.turnstate == LSMeas)
			value1 |= 0x80;
		
	uint8_t value2 = 0x00;
	//bit 0. sygnalizuje stan wibratora. 0 - wylaczony, 1 - wlaczony
	if(pC->Drive.wibstate == On)
			value2 |= 0x01;
	//bity 1. i 2. sygnalizuja stan laboratorium 0x00 - Workmanual, 0x02 - Workremote, 0x04 - Workauto, 0x06 - Workoff
	if(pC->Mode.work == Workoff)
		value2 |= 0x00;
	else if(pC->Mode.work == Workmanual)
		value2 |= 0x02;
	else if(pC->Mode.work == Workremote)
		value2 |= 0x04;
	else if(pC->Mode.work == Workauto)
		value2 |= 0x06;
	
	//bity 4. do 7. sygnalizuja czy sa wlaczone limity programowe 0 - wylaczone, 1 - wlaczone
	if(pC->Drive.motlimits[0] == On)
		value2 |= 0x10;
	if(pC->Drive.motlimits[1] == On)
		value2 |= 0x20;
	if(pC->Drive.motlimits[2] == On)
		value2 |= 0x40;
	if(pC->Drive.motlimits[3] == On)
		value2 |= 0x80;
	
	uint8_t *buf = pC->Com.frames[Framenum_Telemetry].frame;
	buf[0] = Frametype_Header;
	buf[1] = (uint8_t)FrameType_LabTelemetry;
	buf[2] = value1;
	buf[3] = value2;
	buf[4] = 0;
	uint16_t crc = crc16(buf, 5);
	buf[5] = (uint8_t)(crc>>8);
	buf[6] = (uint8_t)crc;
	pC->Com.frames[Framenum_Telemetry].full = On;
	pC->Com.frames[Framenum_Telemetry].len = 7;
}
static void COM_PrepareFrameMotpos(void)
{
	uint8_t* buf = pC->Com.frames[Framenum_MotPos].frame;
	ClearStr(buf, BUF_LEN);
	int16_t pos[MOTMAX];
	uint8_t	cur[MOTMAX];
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pos[i] = (int16_t)(pC->Drive.motposabs[i] * 10.0);
		cur[i] = (uint8_t)(pC->Drive.motcurrent[i] * 10.0);
	}
	
	buf[0] = Frametype_Header;
	buf[1] = Frametype_LabFeadbackMotPos;
	uint8_t index = 2;
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		buf[index++] = (uint8_t)(pos[i] >> 8);
		buf[index++] = (uint8_t)(pos[i] >> 0);
		buf[index++] = (uint8_t)cur[i];
	}
	uint16_t crc = crc16(buf, index);
	buf[index++] = crc>>8;
	buf[index++] = crc>>0;
	
	pC->Com.frames[Framenum_MotPos].full = On;
	pC->Com.frames[Framenum_MotPos].len = index;
}
static void COM_SendToHost(void)
{
	COM_PrepareFrameTelemetry();
	COM_PrepareFrameMotpos();
	for(uint16_t i=0;i<FRAMEMAX;i++)
	{
		if(pC->Com.framenum++ >= FRAMEMAX)
			pC->Com.framenum = 0;
		uint8_t num = pC->Com.framenum;
		if(pC->Com.frames[num].active == On && pC->Com.frames[num].full == On)
		{
			ClearStr(pC->Com.bufwrite, BUF_LEN);
			for(uint16_t j=0;j<pC->Com.frames[num].len;j++)
				pC->Com.bufwrite[j] = pC->Com.frames[num].frame[j];

			DMA1_Stream7->CR 		&= ~DMA_SxCR_EN;
			DMA1->HIFCR 				|= DMA_HIFCR_CTCIF7;
			DMA1_Stream7->NDTR 	= pC->Com.frames[num].len;
			DMA1_Stream7->CR 		|= DMA_SxCR_EN;
			
			DMA1_Stream6->CR 		&= ~DMA_SxCR_EN;
			DMA1->HIFCR 				|= DMA_HIFCR_CTCIF6;
			DMA1_Stream6->NDTR 	= pC->Com.frames[num].len;
			DMA1_Stream6->CR 		|= DMA_SxCR_EN;
			
			DMA1_Stream4->CR 		&= ~DMA_SxCR_EN;
			DMA1->HIFCR 				|= DMA_HIFCR_CTCIF4;
			DMA1_Stream4->NDTR 	= pC->Com.frames[num].len;
			DMA1_Stream4->CR 		|= DMA_SxCR_EN;	
			
			pC->Com.frames[num].full = Off;
			break;
		}
		else
		{
			continue;
		}
	}
}
//----------- Przerwania --------------------------------------------
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_SR_UIF) != RESET)
	{
		COM_SendToHost();
		TIM6->SR &= ~TIM_SR_UIF;
	}
}
//RS422 Slave
void UART5_IRQHandler(void)
{
	if((UART5->SR & USART_SR_IDLE) != RESET)
	{
		char c = UART5->DR;
		COM_ReadFromHost();
		
	}
}
//RS422 Host
void USART2_IRQHandler(void)
{
	if((USART2->SR & USART_SR_IDLE) != RESET)
	{
		char c = USART2->DR;
		COM_ReadFromHost();
		
	}
}
//USB
void UART4_IRQHandler(void)
{
	if((UART4->SR & USART_SR_IDLE) != RESET) 
	{
		char c = UART4->DR;
		COM_ReadFromHost();
	}
}
