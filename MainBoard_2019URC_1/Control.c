#include "Control.h"
extern sControl* pC;
volatile uint32_t tick=0;
static void RCC_SystemInit(void)
{
	uint32_t PLL_M=8, PLL_N=336, PLL_P=2, PLL_Q=7;
	RCC->CR |= RCC_CR_HSEON;								//W³¹czenie HSE
	while(!(RCC->CR & RCC_CR_HSERDY));						//czekamy a¿ HSE bêdzie gotowy
	RCC->CFGR = RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_HPRE_DIV1;
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1)-1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
	FLASH->ACR |= FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
	RCC->CR |= RCC_CR_PLLON;								//w³¹czenie PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));						//czekanie a¿ PLL gotowa
	RCC->CFGR |= RCC_CFGR_SW_PLL;							//PLL jako Ÿród³o dla SYSCLK
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); //czekanie a¿ PLL bedzie gotowe jako SYSCLK
}
static void RCC_Conf(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
									RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
									RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN |
									RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_SPI3EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN |
									RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | 
									RCC_APB1ENR_UART4EN | RCC_APB1ENR_UART5EN;
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM9EN;
}
void SystemStart(void)
{
	RCC_SystemInit();
	SysTick_Config(168000);
	RCC_Conf();
}
void delay_ms(uint32_t ms)
{
	tick = 0;
	while(tick < ms);
}
void ClearStr(uint8_t* str, uint32_t l)
{
	for(uint32_t i=0;i<l;i++)
		str[i] = 0x00;
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
static void Control_StructConf(void)
{
//-------- Mode ------------------------------
	pC->timems = 0;
	pC->Mode.SelectHost = Satel;
	
	//-------- Com ------------------------------
	pC->Com.framenum = 0;
	pC->Com.frames[Framenum_Telemetry].active = On;
	pC->Com.frames[Framenum_GpsPos].active = Off;
	pC->Com.frames[Framenum_BallPos].active = Off;
	pC->Com.frames[Framenum_Manip].active = On;
	
//-------- Erors ------------------------------
	pC->Error.emergencyhard = Off;
	pC->Error.drivecomerror = Off;
	pC->Error.drivecomtick = COMTICKMAX;
	pC->Error.hostcomerror = Off;
	pC->Error.hostcomtick = COMTICKMAX;
	
//--------Drives------------------------------
	for(uint16_t i=0;i<MOTMAX;i++)
	{
		pC->Drive.pid[i] = On;
		pC->Drive.speedref[i] = 0.0;
	}
	pC->Drive.autonprec = Off;
	pC->Drive.slope = Off;
	pC->Drive.frontvalue = 0;
	pC->Drive.dirvalue = 0;
	pC->Drive.motyaw = 0;
	pC->Drive.motspeed = 0;
	pC->Drive.motposx = 0;
	pC->Drive.motposy = 0;
	ClearStr(pC->Drive.bufread, BUF_LEN);
	ClearStr(pC->Drive.bufwrite, BUF_LEN);

	for(uint8_t i=0;i<POINTMAX;i++)
	{
		pC->Drive.points[i].activated = Off;
		pC->Drive.points[i].lon = -1.0;
		pC->Drive.points[i].lat = -1.0;
		pC->Drive.points[i].yawref = 0.0;
		pC->Drive.points[i].yawer = 0.0;
		pC->Drive.points[i].disttopoint = 0.0;
		pC->Drive.points[i].type = Temppoint;
		pC->Drive.points[i].status = Waitpoint;
		pC->Drive.points[i].statustemp = Waitpoint;
	}
	pC->Drive.numpoints = 0;
	pC->Drive.nrp = 0;
	pC->Drive.refpoint = pC->Drive.points[pC->Drive.nrp];
	
//--------Add Devices------------------------------
	ClearStr(pC->Com.nvbufread, BUF_LEN);
	ClearStr(pC->Com.nvbufwrite, BUF_LEN);
	ClearStr(pC->Dev.gpsbufread, BUF_LEN);
	ClearStr(pC->Dev.gpsbufwrite, BUF_LEN);
	for(uint8_t i=0;i<4;i++)
	{
		pC->Dev.SerwPos[i] = 0;
		pC->Dev.SerwPwm[i] = 1500;
	}
	pC->Dev.Gimdigit = Off;
	pC->Dev.Trans1 = 1;
	pC->Dev.Trans2 = 4;
	pC->Dev.imuyawoffset = 0;
	pC->Dev.imuyawraw = 0;
	pC->Dev.imuyaw = 0;
	pC->Dev.imupitch = 0;
	pC->Dev.imuroll = 0;
	pC->Dev.gpslat = 0;
	pC->Dev.gpslon = 0;
	pC->Dev.gpsalt = 0;
	pC->Dev.resetdev1 = Off;
	pC->Dev.resetdev1tick = RESETDEVTICKMAX;
	pC->Dev.resetdev2 = Off;
	pC->Dev.resetdev2tick = RESETDEVTICKMAX;
}
static void Control_TimConf(void)
{
	TIM7->PSC = 84-1;
	TIM7->ARR = 50000-1;
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
}
static void Control_IWDGConf(void)
{
	RCC->CSR |= RCC_CSR_LSION;
	while((RCC->CSR & RCC_CSR_LSIRDY) == 0){;}
  //Zezwolenie na zapis do rejestrów IWDG_PR oraz IWDG_RLR
  IWDG->KR = 0x5555;
  //Ustawienie taktowania, 32kHz/32 = 1024Hz
	IWDG->PR = 0x03;
  //Ustawienie wartosci poczatkowej
  IWDG->RLR = 100;
  //Przeladowanie wybranej konfiguracji
  IWDG->KR = 0xaaaa;
  //Wlaczenie watchdoga oraz sygnalu taktujacego LSI
  IWDG->KR = 0xcccc;
}
void Control_Conf(void)
{
	Control_StructConf();
	Control_TimConf();
	Control_IWDGConf();
}
static void Control_ErrorScan(void)
{
//---------------Emergency---------------------
	if(pC->Error.emergencyhard == On || pC->Error.emergencysoft == On)
	{
		SIG0_ON;
		pC->Error.emergency = On;
	}
	else if(pC->Error.emergencyhard == Off && pC->Error.emergencysoft == Off)
	{
		SIG0_OFF;
		pC->Error.emergency = Off;
	}
//---------------COM---------------------------
	if(pC->Error.hostcomtick >= COMTICKMAX)
	{
		LED3_OFF;
		LED4_ON;
		pC->Error.hostcomerror = On;
		pC->Error.hostcomtick = COMTICKMAX;
	}
	else
	{
		LED3_TOG;
		LED4_OFF;
		pC->Error.hostcomerror = Off;
	}
//---------------NVidia---------------------------
	if(pC->Error.nvcomtick >= NVCOMTICKMAX)
	{
		pC->Error.nvcomerror = On;
		pC->Error.nvcomtick = NVCOMTICKMAX;
	}
	else
	{
		pC->Error.nvcomerror = Off;
	}
}
static void Control_Act(void)
{
	Dev_Act();
	Control_ErrorScan();
	Drive_Act();
}
//----------------Interrupts--------------------------------------
void SysTick_Handler(void)
{
	IWDG->KR = 0xaaaa;
	tick++;
	pC->timems++;
	pC->Error.hostcomtick++;
	pC->Error.drivecomtick++;
	pC->Error.gpscomtick++;
	pC->Error.nvcomtick++;
	pC->Dev.resetdev1tick++;
	pC->Dev.resetdev2tick++;
	if(pC->Drive.autonprectime++ >= AUTONPRECMAX)
		pC->Drive.autonprec = Off;
}
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		Control_Act();
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
