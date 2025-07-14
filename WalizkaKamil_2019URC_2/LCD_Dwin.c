#include "LCD_Dwin.h"
extern sControl* pC;
uint8_t bufread[BUF_LEN];
uint8_t bufwrite[BUF_LEN];
char bufchar[BUF_LEN];
uint8_t tick;
static void LCD_Dwin_ClearStr(uint8_t* str, uint32_t l)
{
	for(uint32_t i=0;i<l;i++)
		str[i] = 0;
}
void LCD_Dwin_Conf(void)
{
  GPIOB->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0;
  GPIOB->AFR[1] |= 0x00007700;
  USART3->BRR = 42000000/115200;
  USART3->CR3 |= USART_CR3_DMAT;
  USART3->CR1 |= USART_CR1_TE | USART_CR1_UE;
}
void LCD_Dwin_SendToLcd(void)
{
//	pC->Tele.manipcs = ManipJoin;
//	pC->Tele.imuyaw += 1;
//	pC->Tele.imupitch += 2;
//	pC->Tele.imuroll -= 1;
//	pC->Tele.gpslat = 38.456789;
//	pC->Tele.gpslon = -110.123456;
	tick+=2;
	uint8_t* buf = bufwrite;
	char* buff = bufchar;
	LCD_Dwin_ClearStr(buf, BUF_LEN);

	uint8_t pitch_graph_dwin;
	uint8_t pitch_value1_dwin;
	int8_t pitch_value2_dwin;
	uint16_t roll_graph_dwin;
	int16_t roll_value;
	uint16_t yaw_graph_dwin;
	int16_t yaw_value;
	int32_t gpsx = pC->Tele. gpslon * 1000000.0;
	int32_t gpsy = pC->Tele. gpslat * 1000000.0;
	int16_t velocity = pC->Tele.roverspeed * 10;

	uint8_t emer_soft_tick;
	uint8_t emer_hard_tick;
	emer_soft_tick = pC->Tele.emer_soft;	
	
	pitch_graph_dwin = 50 - (pC->Tele.imupitch);
	pitch_value2_dwin = (pC->Tele.imupitch);

	if((pC->Tele.imupitch)<0)
		pitch_value1_dwin=0xff;
	else
		pitch_value1_dwin=0;
	
	if((pC->Tele.imuroll)<0)
		roll_graph_dwin=(2*(pC->Tele.imuroll))+720;
	else
		roll_graph_dwin=(2*(pC->Tele.imuroll));
	
	roll_value=pC->Tele.imuroll;

	if((pC->Tele.imuyaw)<0)
		yaw_graph_dwin=(2*(pC->Tele.imuyaw))+720;
	else
		yaw_graph_dwin=(2*(pC->Tele.imuyaw));
	
	yaw_value= pC->Tele.imuyaw;


	if(pC->Tele.emer_soft==1)
	{
		if(tick<10)
			emer_soft_tick=1;
		if(tick>10&&tick<20)
			emer_soft_tick=0;
		if(tick>=20)
			tick=0;
	}
	else
	{
		emer_soft_tick=0;
	}
	
	if(pC->Tele.emer_hard==1)
	{
		if(tick<10)
			emer_hard_tick=1;
		if(tick>10&&tick<20)
			emer_hard_tick=0;
		if(tick>=20)
			tick=0;
	}
	else
	{
		emer_hard_tick=0;
	}
	
	
	buff[0] = 170;
	buff[1] = 187;
	buff[2] = 36;
	buff[3] = 130;
	buff[4] = 0;
	buff[5] = 0;
	buff[6] = pitch_graph_dwin;			//#0
	buff[7] = 0;										//#0
	buff[8] = pitch_value1_dwin;		//#1
	buff[9] = pitch_value2_dwin;		//#1
	buff[10] = roll_graph_dwin>>8;	//#2
	buff[11] = roll_graph_dwin>>0;	//#2
	buff[12] = roll_value>>8;				//#3
	buff[13] = roll_value>>0;				//#3
	buff[14] = yaw_graph_dwin>>8;		//#4
	buff[15] = yaw_graph_dwin>>0;		//#4
	buff[16] = yaw_value>>8;				//#5
	buff[17] = yaw_value>>0;				//#5
	buff[18] = 0;										//#6
	buff[19] = pC->Tele.manipcs;		//#6
	buff[20] = 0;										//#7
	buff[21] = pC->Tele.hostname;		//#7
	buff[22] = 0;										//#8
	buff[23] = emer_soft_tick;			//#8
	buff[24] = 0;										//#9
	buff[25] = emer_hard_tick;			//#9
	buff[26] = gpsx>>24;						//#A
	buff[27] = gpsx>>16;						//#A
	buff[28] = gpsx>>8;							//#B
	buff[29] = gpsx>>0;							//#B
	buff[30] = gpsy>>24;						//#C
	buff[31] = gpsy>>16;						//#C
	buff[32] = gpsy>>8;							//#D
	buff[33] = gpsy>>0;							//#D
	buff[34] = 0;										//#E
	buff[35] = (pC->Tele.autostate)-245;	//#E
	buff[36] = velocity>>8;					//#F
	buff[37] = velocity>>0;					//#F
	

	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
	DMA1_Stream3->NDTR 	= (uint16_t)38;
	DMA1_Stream3->M0AR = (uint32_t)bufchar;
	DMA1_Stream3->PAR = (uint32_t)&USART3->DR;
	DMA1_Stream3->CR |= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
}
