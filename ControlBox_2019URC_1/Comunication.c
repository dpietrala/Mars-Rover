#include "Comunication.h"
extern sControl* pC;
void COM_Conf(void)
{
	//-------------- Przypisanie strumieni DMA do Com ---------------------------------
	//USART1 Com0 - Pierwsze USB
	pC->Com[0].com = USART1;
	pC->Com[0].comirq = USART1_IRQn;
	pC->Com[0].dma = DMA2;
	pC->Com[0].readstream = DMA2_Stream2;
	pC->Com[0].readclearflag = DMA_LIFCR_CTCIF2;
	pC->Com[0].writestream = DMA2_Stream7;
	pC->Com[0].writeclearflag = DMA_HIFCR_CTCIF7;
	pC->Com[0].writedmairq = DMA2_Stream7_IRQn;
	
	//USART2 Com1 - USART ogólnego zastosowania 
	pC->Com[1].com = USART2;
	pC->Com[1].comirq = USART2_IRQn;
	pC->Com[1].dma = DMA1;
	pC->Com[1].readstream = DMA1_Stream5;
	pC->Com[1].readclearflag = DMA_HIFCR_CTCIF5;
	pC->Com[1].writestream = DMA1_Stream6;
	pC->Com[1].writeclearflag = DMA_HIFCR_CTCIF6;
	pC->Com[1].writedmairq = DMA1_Stream6_IRQn;
	
	//USART3 Com2 - WiFi
	pC->Com[2].com = USART3;
	pC->Com[2].comirq = USART3_IRQn;
	pC->Com[2].dma = DMA1;
	pC->Com[2].readstream = DMA1_Stream1;
	pC->Com[2].readclearflag = DMA_LIFCR_CTCIF1;
	pC->Com[2].writestream = DMA1_Stream3;
	pC->Com[2].writeclearflag = DMA_LIFCR_CTCIF3;
	pC->Com[2].writedmairq = DMA1_Stream3_IRQn;
	
	//UART4 Com3 - RS232 Satel
	pC->Com[3].com = UART4;
	pC->Com[3].comirq = UART4_IRQn;
	pC->Com[3].dma = DMA1;
	pC->Com[3].readstream = DMA1_Stream2;
	pC->Com[3].readclearflag = DMA_LIFCR_CTCIF2;
	pC->Com[3].writestream = DMA1_Stream4;
	pC->Com[3].writeclearflag = DMA_HIFCR_CTCIF4;
	pC->Com[3].writedmairq = DMA1_Stream4_IRQn;
	
	//UART5 Com4 - RS232 ogólnego zastosowania
	pC->Com[4].com = UART5;
	pC->Com[4].comirq = UART5_IRQn;
	pC->Com[4].dma = DMA1;
	pC->Com[4].readstream = DMA1_Stream0;
	pC->Com[4].readclearflag = DMA_LIFCR_CTCIF0;
	pC->Com[4].writestream = DMA1_Stream7;
	pC->Com[4].writeclearflag = DMA_HIFCR_CTCIF7;
	pC->Com[4].writedmairq = DMA1_Stream7_IRQn;
	
	//USART6 Com5 - Drugie USB
	pC->Com[5].com = USART6;
	pC->Com[5].comirq = USART6_IRQn;
	pC->Com[5].dma = DMA2;
	pC->Com[5].readstream = DMA2_Stream1;
	pC->Com[5].readclearflag = DMA_LIFCR_CTCIF1;
	pC->Com[5].writestream = DMA2_Stream6;
	pC->Com[5].writeclearflag = DMA_HIFCR_CTCIF6;
	pC->Com[5].writedmairq = DMA2_Stream6_IRQn;
	
	//-------------- Konfiguracja poszczególnych usartów ---------------------------------
	//USART1 Com0 - Pierwsze USB
	pC->Com[0].readstream->PAR 	= (uint32_t)&USART1->DR;
  pC->Com[0].readstream->M0AR 	= (uint32_t)pC->Com[2].bufread;
  pC->Com[0].readstream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[0].readstream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  pC->Com[0].writestream->PAR 	= (uint32_t)&USART1->DR;
  pC->Com[0].writestream->M0AR 	= (uint32_t)pC->Com[2].bufwrite[0];
  pC->Com[0].writestream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[0].writestream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_TCIE;
  GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0;
  GPIOA->AFR[1] |= 0x00000770;
  pC->Com[0].com->BRR = 84000000/115200;
  pC->Com[0].com->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  pC->Com[0].com->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(pC->Com[0].comirq);
	NVIC_EnableIRQ(pC->Com[0].writedmairq);
	
	//USART2 Com1 - USART ogólnego zastosowania 
	pC->Com[1].readstream->PAR 	= (uint32_t)&USART2->DR;
  pC->Com[1].readstream->M0AR 	= (uint32_t)pC->Com[5].bufread;
  pC->Com[1].readstream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[1].readstream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  pC->Com[1].writestream->PAR 	= (uint32_t)&USART2->DR;
  pC->Com[1].writestream->M0AR 	= (uint32_t)pC->Com[5].bufwrite[0];
  pC->Com[1].writestream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[1].writestream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_TCIE;
  GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;
  GPIOD->PUPDR |= GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0;
  GPIOD->AFR[0] |= 0x07700000;
  pC->Com[1].com->BRR = 42000000/115200;
  pC->Com[1].com->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  pC->Com[1].com->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(pC->Com[1].comirq);
	NVIC_EnableIRQ(pC->Com[1].writedmairq);
	
	//USART3 Com2 - WiFi
	pC->Com[2].readstream->PAR 	= (uint32_t)&USART3->DR;
  pC->Com[2].readstream->M0AR 	= (uint32_t)pC->Com[4].bufread;
  pC->Com[2].readstream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[2].readstream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  pC->Com[2].writestream->PAR 	= (uint32_t)&USART3->DR;
  pC->Com[2].writestream->M0AR 	= (uint32_t)pC->Com[4].bufwrite[0];
  pC->Com[2].writestream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[2].writestream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_TCIE;
  GPIOB->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
  GPIOB->AFR[1] |= 0x00007700;
  pC->Com[2].com->BRR = 42000000/115200;
  pC->Com[2].com->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  pC->Com[2].com->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(pC->Com[2].comirq);
	NVIC_EnableIRQ(pC->Com[2].writedmairq);
	
	//UART4 Com3 - RS232 Satel
	pC->Com[3].readstream->PAR 	= (uint32_t)&UART4->DR;
  pC->Com[3].readstream->M0AR 	= (uint32_t)pC->Com[0].bufread;
  pC->Com[3].readstream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[3].readstream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  pC->Com[3].writestream->PAR 	= (uint32_t)&UART4->DR;
  pC->Com[3].writestream->M0AR 	= (uint32_t)pC->Com[0].bufwrite[0];
  pC->Com[3].writestream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[3].writestream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_TCIE;
  GPIOC->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
  GPIOC->AFR[1] |= 0x00008800;
  pC->Com[3].com->BRR = 42000000/19200;
  pC->Com[3].com->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  pC->Com[3].com->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(pC->Com[3].comirq);
	NVIC_EnableIRQ(pC->Com[3].writedmairq);
	
	//UART5 Com4 - RS232 ogólnego zastosowania
	pC->Com[4].readstream->PAR 	= (uint32_t)&UART5->DR;
  pC->Com[4].readstream->M0AR 	= (uint32_t)pC->Com[1].bufread;
  pC->Com[4].readstream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[4].readstream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
  pC->Com[4].writestream->PAR 	= (uint32_t)&UART5->DR;
  pC->Com[4].writestream->M0AR 	= (uint32_t)pC->Com[1].bufwrite[0];
  pC->Com[4].writestream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[4].writestream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_TCIE;
  GPIOC->MODER |= GPIO_MODER_MODE12_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD12_0;
  GPIOC->AFR[1] |= 0x00080000;
	GPIOD->MODER |= GPIO_MODER_MODE2_1;
  GPIOD->PUPDR |= GPIO_PUPDR_PUPD2_0;
  GPIOD->AFR[0] |= 0x00000800;
  pC->Com[4].com->BRR = 42000000/115200;
  pC->Com[4].com->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  pC->Com[4].com->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(pC->Com[4].comirq);
	NVIC_EnableIRQ(pC->Com[4].writedmairq);
	
	//USART6 Com5 - Drugie USB
	pC->Com[5].readstream->PAR 	= (uint32_t)&USART6->DR;
  pC->Com[5].readstream->M0AR 	= (uint32_t)pC->Com[3].bufread;
  pC->Com[5].readstream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[5].readstream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0 | DMA_SxCR_EN;
  pC->Com[5].writestream->PAR 	= (uint32_t)&USART6->DR;
  pC->Com[5].writestream->M0AR 	= (uint32_t)pC->Com[3].bufwrite[0];
  pC->Com[5].writestream->NDTR 	= (uint16_t)BUF_LEN;
  pC->Com[5].writestream->CR 		|= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0 | DMA_SxCR_TCIE;
  GPIOC->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
  GPIOC->AFR[0] |= 0x88000000;
  pC->Com[5].com->BRR = 84000000/115200;
  pC->Com[5].com->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
  pC->Com[5].com->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
	NVIC_EnableIRQ(pC->Com[5].comirq);
	NVIC_EnableIRQ(pC->Com[5].writedmairq);
	
	TIM6->PSC = 84-1;
	TIM6->ARR = 20000-1;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
	TIM7->PSC = 84-1;
	TIM7->ARR = 50000-1;
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
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
static void Com_AddToFifo(uint8_t num, uint8_t *buf, uint8_t len)
{
	for(uint8_t i=0;i<len;i++)
		pC->Com[num].bufwrite[pC->Com[num].fifowp][i] = buf[i];
	pC->Com[num].bufwritelen[pC->Com[num].fifowp] = len;
	if(pC->Com[num].fifowp++ >= BUF_MAX)
		pC->Com[num].fifowp = 0;
	if(pC->Com[num].fifonum++ >= BUF_MAX)
		pC->Com[num].fifonum = BUF_MAX;
}
static void COM_GetFromFifo(uint8_t num)
{
	if(pC->Com[num].busy == Off)
	{
		if(pC->Com[num].fifonum > 0)
		{
			pC->Com[num].writestream->CR &= ~DMA_SxCR_EN;
			if(num == 2)
				pC->Com[num].dma->LIFCR |= pC->Com[num].writeclearflag;
			else
				pC->Com[num].dma->HIFCR |= pC->Com[num].writeclearflag;
			pC->Com[num].writestream->M0AR 	= (uint32_t)pC->Com[num].bufwrite[pC->Com[num].fiforp];
			pC->Com[num].writestream->NDTR 	= (uint16_t)pC->Com[num].bufwritelen[pC->Com[num].fiforp];
			pC->Com[num].writestream->CR 		|= DMA_SxCR_EN;
			pC->Com[num].busy = On;
			if(pC->Com[num].fiforp++ >= BUF_MAX)
				pC->Com[num].fiforp = 0;
			if(pC->Com[num].fifonum-- <= 0)
				pC->Com[num].fifonum = 0;
		}
		else
		{
			DMA1_Stream4->CR &= ~DMA_SxCR_EN;
		}
	}
}
static void COM_ReadFromCom0(void)
{
	uint8_t len = BUF_LEN - pC->Com[0].readstream->NDTR;
	Com_AddToFifo(3, pC->Com[0].bufread, len);
	
	pC->Com[0].readstream->CR 		&= ~DMA_SxCR_EN;
	pC->Com[0].dma->LIFCR 				|= DMA_LIFCR_CTCIF2;
  pC->Com[0].readstream->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromCom1(void)
{
	uint8_t len = BUF_LEN - pC->Com[1].readstream->NDTR;
	Com_AddToFifo(3, pC->Com[1].bufread, len);
	
	pC->Com[1].readstream->CR 		&= ~DMA_SxCR_EN;
	pC->Com[1].dma->HIFCR 				|= DMA_HIFCR_CTCIF5;
  pC->Com[1].readstream->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromCom2(void)
{
	uint8_t len = BUF_LEN - pC->Com[2].readstream->NDTR;
	Com_AddToFifo(3, pC->Com[2].bufread, len);
	
	pC->Com[2].readstream->CR 		&= ~DMA_SxCR_EN;
	pC->Com[2].dma->LIFCR 				|= DMA_LIFCR_CTCIF1;
  pC->Com[2].readstream->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromCom3(void)
{
	uint8_t len = BUF_LEN - pC->Com[3].readstream->NDTR;
	Com_AddToFifo(0, pC->Com[3].bufread, len);
	Com_AddToFifo(1, pC->Com[3].bufread, len);
	Com_AddToFifo(2, pC->Com[3].bufread, len);
	Com_AddToFifo(4, pC->Com[3].bufread, len);
	Com_AddToFifo(5, pC->Com[3].bufread, len);
	
	pC->Com[3].readstream->CR 		&= ~DMA_SxCR_EN;
	pC->Com[3].dma->LIFCR 				|= DMA_LIFCR_CTCIF2;
  pC->Com[3].readstream->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromCom4(void)
{
	uint8_t len = BUF_LEN - pC->Com[4].readstream->NDTR;
	Com_AddToFifo(3, pC->Com[4].bufread, len);
	
	pC->Com[4].readstream->CR 		&= ~DMA_SxCR_EN;
	pC->Com[4].dma->LIFCR 				|= DMA_LIFCR_CTCIF0;
  pC->Com[4].readstream->CR 		|= DMA_SxCR_EN;
}
static void COM_ReadFromCom5(void)
{
	uint8_t len = BUF_LEN - pC->Com[5].readstream->NDTR;
	Com_AddToFifo(3, pC->Com[5].bufread, len);
	
	pC->Com[5].readstream->CR 		&= ~DMA_SxCR_EN;
	pC->Com[5].dma->LIFCR 				|= DMA_LIFCR_CTCIF1;
  pC->Com[5].readstream->CR 		|= DMA_SxCR_EN;
}
static void COM_FrameCmd(void)
{
	for(uint8_t i=0;i<DI_MAX;i++)
	{
		if(pC->DI[i].state == DIStateOn)
		{
			pC->Frame.cmd = pC->DI[i].cmd;
			pC->DI[i].state = DIStateOff;
			break;
		}
	}
}
static void COM_FrameValue(void)
{
	Inputs_Read();
	COM_FrameCmd();
	
	uint8_t buf[BUF_LEN];
	pC->Frame.frontspeed = pC->AI[0].val;
	pC->Frame.dirspeed = pC->AI[1].val;
	pC->Frame.manipspeed[0] = pC->AI[2].val;
	pC->Frame.manipspeed[1] = pC->AI[3].val;
	pC->Frame.manipspeed[2] = pC->AI[4].val;
	pC->Frame.manipspeed[3] = pC->AI[5].val;
	pC->Frame.manipspeed[4] = pC->AI[6].val;
	pC->Frame.manipspeed[5] = pC->AI[7].val;
	pC->Frame.manipspeed[6] = pC->AI[8].val;
	pC->Frame.serwpos[0] = pC->AI[9].val;
	pC->Frame.serwpos[1] = pC->AI[10].val;
	pC->Frame.serwpos[2] = pC->AI[11].val;
	pC->Frame.serwpos[3] = pC->AI[12].val;
	
	buf[0] = 1;
	buf[1] = pC->Frame.frontspeed;
	buf[2] = pC->Frame.dirspeed;
	buf[3] = pC->Frame.manipspeed[0];
	buf[4] = pC->Frame.manipspeed[1];
	buf[5] = pC->Frame.manipspeed[2];
	buf[6] = pC->Frame.manipspeed[3];
	buf[7] = pC->Frame.manipspeed[4];
	buf[8] = pC->Frame.manipspeed[5];
	buf[9] = pC->Frame.manipspeed[6];
	buf[10] = pC->Frame.serwpos[0];
	buf[11] = pC->Frame.serwpos[1];
	buf[12] = pC->Frame.serwpos[2];
	buf[13] = pC->Frame.serwpos[3];
	buf[14] = pC->Frame.cmd;
	uint16_t crc = crc16(buf, 15);
	buf[15] = (uint8_t)(crc>>8);
	buf[16] = (uint8_t)crc;
	Com_AddToFifo(0, buf, 17);
}
//----------- Przerwania --------------------
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		COM_FrameValue();
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR & TIM_SR_UIF) != RESET)
	{
		COM_GetFromFifo(0);
		COM_GetFromFifo(1);
		COM_GetFromFifo(2);
		COM_GetFromFifo(3);
		COM_GetFromFifo(4);
		COM_GetFromFifo(5);
		TIM6->SR &= ~TIM_SR_UIF;
	}
}
void USART1_IRQHandler(void)
{
	if((USART1->SR & USART_SR_IDLE) != RESET)
	{
		char c = USART1->DR;
		COM_ReadFromCom0();
	}
}
void USART2_IRQHandler(void)
{
	if((USART2->SR & USART_SR_IDLE) != RESET) 
	{
		char c = USART2->DR;
		COM_ReadFromCom1();
	}
}
void USART3_IRQHandler(void)
{
	if((USART3->SR & USART_SR_IDLE) != RESET) 
	{
		char c = USART3->DR;
		COM_ReadFromCom2();
	}
}
void UART4_IRQHandler(void)
{
	if((UART4->SR & USART_SR_IDLE) != RESET) 
	{
		char c = UART4->DR;
		COM_ReadFromCom3();
	}
}
void UART5_IRQHandler(void)
{
	if((UART5->SR & USART_SR_IDLE) != RESET) 
	{
		char c = UART5->DR;
		COM_ReadFromCom4();
	}
}
void USART6_IRQHandler(void)
{
	if((USART6->SR & USART_SR_IDLE) != RESET) 
	{
		char c = USART6->DR;
		COM_ReadFromCom5();
	}
}
// Com[0]
void DMA2_Stream7_IRQHandler(void)
{
	if((DMA2->HISR & DMA_HISR_TCIF7) != RESET)
	{
		pC->Com[0].busy = Off;
		COM_GetFromFifo(0);
		DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
	}
}
// Com[1]
void DMA1_Stream6_IRQHandler(void)
{
	if((DMA1->HISR & DMA_HISR_TCIF6) != RESET)
	{
		pC->Com[1].busy = Off;
		COM_GetFromFifo(1);
		DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
	}
}
// Com[2]
void DMA1_Stream3_IRQHandler(void)
{
	if((DMA1->LISR & DMA_LISR_TCIF3) != RESET)
	{
		pC->Com[2].busy = Off;
		COM_GetFromFifo(2);
		DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
	}
}
// Com[3]
void DMA1_Stream4_IRQHandler(void)
{
	if((DMA1->HISR & DMA_HISR_TCIF4) != RESET)
	{
		pC->Com[3].busy = Off;
		COM_GetFromFifo(3);
		DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
	}
}
// Com[4]
void DMA1_Stream7_IRQHandler(void)
{
	if((DMA1->HISR & DMA_HISR_TCIF7) != RESET)
	{
		pC->Com[4].busy = Off;
		COM_GetFromFifo(4);
		DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
	}
}
// Com[5]
void DMA2_Stream6_IRQHandler(void)
{
	if((DMA2->HISR & DMA_HISR_TCIF6) != RESET)
	{
		pC->Com[5].busy = Off;
		COM_GetFromFifo(5);
		DMA2->HIFCR |= DMA_HIFCR_CTCIF6;
	}
}
