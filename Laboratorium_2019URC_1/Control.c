#include "Control.h"
extern sControl* pC;
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
									RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_BKPSRAMEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN |
									RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | 
									RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM13EN | RCC_APB1ENR_TIM14EN | 
									RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_UART4EN | 
									RCC_APB1ENR_UART5EN | RCC_APB1ENR_PWREN;
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN | 
									RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM9EN | 
									RCC_APB2ENR_TIM10EN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
}
static void Led_Conf(void)
{
		GPIOE->MODER 	|= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0;
		GPIOE->PUPDR 	|= GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR6_0;
}
static void Control_StructConf(void)
{
	pC->Mode.work = Workremote;
	pC->Mode.manual = Off;
	pC->Mode.times = 0;
	pC->Mode.hostcomtick = 0;
	pC->Mode.hostcom = Off;

	pC->Com.framenum = 0;
	pC->Com.frames[Framenum_Telemetry].active = On;
	pC->Com.frames[Framenum_MotPos].active = On;
	pC->Com.cmd = cmdLab_null;
	pC->Com.cmdval = 0;
	ClearStr(pC->Com.bufread, BUF_LEN);
	ClearStr(pC->Com.bufwrite, BUF_LEN);
	
	for(uint8_t i=0;i<BUTTONMAX;i++)
		pC->Drive.buttons[i] = Off;
	for(uint8_t i=0;i<LIMITSWITCHMAX;i++)
		pC->Drive.limitswitches[i] = Off;
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Drive.motimp[i] = 0;
		pC->Drive.motimplast[i] = 0;
		pC->Drive.motimptotal[i] = 0;
		pC->Drive.motfullturn[i] = 0;
		pC->Drive.motposabs[i] = 0;
		pC->Drive.motposabsmin[i] = 0;
		pC->Drive.motposabsref[i] = 0;
		pC->Drive.motposstart[i] = 0;
		pC->Drive.motpid[i] = On;
		pC->Drive.motlimits[i] = On;
		pC->Drive.motdir[i] = Stop;
		pC->Drive.mote[i] = 0;
		pC->Drive.motelast[i] = 0;
		pC->Drive.motetotal[i] = 0;
		pC->Drive.motemax[i] = 30;
		pC->Drive.motetotalmax[i] = 200;
		pC->Drive.motcurrent[i] = 0;
		pC->Drive.motkp[i] = 1.0;
		pC->Drive.motki[i] = 1.3;
		pC->Drive.motkd[i] = 0.0046;
		pC->Drive.motpwm[i] = 0;
		pC->Drive.motpwmout[i] = 0;
		pC->Drive.motpwmmax[i] = 550;
		pC->Drive.motpwmdeath[i] = 10;
		pC->Drive.motspeedref[i] = 0;
		pC->Drive.motstep[i] = 0.0026;
		pC->Drive.motout[i] = 0;
		pC->Drive.motoutmax[i] = 1.0;
	}
	pC->Drive.wibstate = Off;
	pC->Drive.motposabsmax[0] = 660.0;
	pC->Drive.motposabsmax[1] = 255.0;
	pC->Drive.motposabsmax[2] = 652.0;
	pC->Drive.motposabsmax[3] = 150.0;
	
	pC->Drive.motkp[3] = 0.3;
	pC->Drive.motki[3] = 0.5;
	pC->Drive.motkd[3] = 0.005;
	pC->Drive.motstep[3] = 0.003;
	pC->Drive.motposabsmax[3] = 125.0;
	pC->Drive.motposabsmin[3] = 0.;
	
	//Offset xy przy odwróceniu chwytaka
	
	pC->Drive.poslumivert = 0.0;
	pC->Drive.poslumihoriz = 120.0;
	pC->Drive.poszup = 3.0;
	pC->Drive.poszlumi = 49.0;
	pC->Drive.poszswab = 190.0;
	pC->Drive.poszswab2 = 226.0;
	pC->Drive.poszmeas = 500.0;
	pC->Drive.posxylumi[0] = 76.2;
	pC->Drive.posxylumi[1] = 72.9;
	pC->Drive.posxymeas[0] = 650.0;
	pC->Drive.posxymeas[1] = 90.0;
	pC->Drive.posxywib[0] = 133;
	pC->Drive.posxywib[1] = 259;
	pC->Drive.poszwib = 105;
	pC->Drive.poslumifoto = 30.0;
	pC->Drive.posxyswabtrow[0] = 73;
	pC->Drive.posxyswabtrow[1] = 200;

	pC->Drive.posxyswab[0][0] = 145.4;
	pC->Drive.posxyswab[0][1] = 224.9;
	pC->Drive.posxyswab[1][0] = 145.6;
	pC->Drive.posxyswab[1][1] = 171.0;
	pC->Drive.posxyswab[2][0] = 145.9;
	pC->Drive.posxyswab[2][1] = 116.7;
	pC->Drive.posxyswab[3][0] = 145.9;
	pC->Drive.posxyswab[3][1] = 62.3;
	pC->Drive.posxyswab[4][0] = 145.6;
	pC->Drive.posxyswab[4][1] = 8.0;
	
	pC->Drive.posxyswab[5][0] = 210.4;
	pC->Drive.posxyswab[5][1] = 224.6;
	pC->Drive.posxyswab[6][0] = 210.4;
	pC->Drive.posxyswab[6][1] = 170.0;
	pC->Drive.posxyswab[7][0] = 211.2;
	pC->Drive.posxyswab[7][1] = 115.6;
	pC->Drive.posxyswab[8][0] = 210.3;
	pC->Drive.posxyswab[8][1] = 61.7;
	pC->Drive.posxyswab[9][0] = 209.9;
	pC->Drive.posxyswab[9][1] = 7.2;
	
	pC->Serw.gripstate = On;
	pC->Serw.grippwm[0] = 1500;	//zamkniety
	pC->Serw.grippwm[1] = 1700;	//otwarty
	pC->Serw.griprefpwm = pC->Serw.grippwm[0]; //domyslnie zamkniety
	pC->Serw.openstate = On;
	pC->Serw.openpwm[0] = 650; //otwarty
	pC->Serw.openpwm[1] = 2070;	//zamkniety
	pC->Serw.openrefpwm = pC->Serw.openpwm[1];	//domyslnie zamkniety
	pC->Serw.turnstate = LSNeutral;
	pC->Serw.turnpwm[0] = 1900;	//srodek
	pC->Serw.turnpwm[1] = 2130;	//wlaczenie
	pC->Serw.turnpwm[2] = 1650;	//pomiar
	pC->Serw.turnrefpwm = pC->Serw.turnpwm[0];	//domyslnie srodek
	
// ------------------- Sekwencje do pracy automatycznej ------------------------
	pC->Stats.posrefhyst = 0.3;
	
	pC->Stats.seq[0].s[0].speed = 50.0;
	pC->Stats.seq[0].s[0].delay = 0;
	pC->Stats.seq[0].s[0].posref[0] = 100.0;
	pC->Stats.seq[0].s[0].posref[1] = 120.0;
	pC->Stats.seq[0].s[0].posref[2] = 50.0;
	pC->Stats.seq[0].s[0].posref[3] = 5.0;
	pC->Stats.seq[0].s[0].pwmrefg = pC->Serw.grippwm[0];
	pC->Stats.seq[0].s[0].pwmreft = pC->Serw.turnpwm[0];
	pC->Stats.seq[0].s[0].pwmrefo = pC->Serw.openpwm[0];
	
	pC->Stats.seq[0].s[1].speed = 50.0;
	pC->Stats.seq[0].s[1].delay = 0;
	pC->Stats.seq[0].s[1].posref[0] = 150.0;
	pC->Stats.seq[0].s[1].posref[1] = 100.0;
	pC->Stats.seq[0].s[1].posref[2] = 150.0;
	pC->Stats.seq[0].s[1].posref[3] = 5.0;
	pC->Stats.seq[0].s[1].pwmrefg = pC->Serw.grippwm[1];
	pC->Stats.seq[0].s[1].pwmreft = pC->Serw.turnpwm[0];
	pC->Stats.seq[0].s[1].pwmrefo = pC->Serw.openpwm[1];
	
	pC->Stats.seq[0].smax = 2;
	pC->Stats.seq[0].snum = 0;
	
	pC->Stats.seq[1].smax = 3;
	pC->Stats.seq[1].snum = 0;
	
	pC->Stats.seqmax = 30;
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
	RCC_SystemInit();
	RCC_Conf();
	SysTick_Config(168000);
	Led_Conf();
	Control_StructConf();
	Control_IWDGConf();
}
void Led_OnOff(uint8_t num, eLedState newstate)
{
	switch(num)
	{
		case 0:
			if(newstate == LedOff)
				GPIOE->ODR &= ~GPIO_ODR_ODR_2 & ~GPIO_ODR_ODR_3 & ~GPIO_ODR_ODR_4 & ~GPIO_ODR_ODR_5 & ~GPIO_ODR_ODR_6;
			else if(newstate == LedOn)
				GPIOE->ODR |= GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6;
			else if(newstate == LedTog)
				GPIOE->ODR ^= GPIO_ODR_ODR_2 | GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6;
			break;
		case 1:
			if(newstate == LedOff)
				GPIOE->ODR &= ~GPIO_ODR_ODR_2;
			else if(newstate == LedOn)
				GPIOE->ODR |= GPIO_ODR_ODR_2;
			else if(newstate == LedTog)
				GPIOE->ODR ^= GPIO_ODR_ODR_2;
			break;
		case 2:
			if(newstate == LedOff)
				GPIOE->ODR &= ~GPIO_ODR_ODR_3;
			else if(newstate == LedOn)
				GPIOE->ODR |= GPIO_ODR_ODR_3;
			else if(newstate == LedTog)
				GPIOE->ODR ^= GPIO_ODR_ODR_3;
			break;
		case 3:
			if(newstate == LedOff)
				GPIOE->ODR &= ~GPIO_ODR_ODR_4;
			else if(newstate == LedOn)
				GPIOE->ODR |= GPIO_ODR_ODR_4;
			else if(newstate == LedTog)
				GPIOE->ODR ^= GPIO_ODR_ODR_4;
			break;
		case 4:
			if(newstate == LedOff)
				GPIOE->ODR &= ~GPIO_ODR_ODR_5;
			else if(newstate == LedOn)
				GPIOE->ODR |= GPIO_ODR_ODR_5;
			else if(newstate == LedTog)
				GPIOE->ODR ^= GPIO_ODR_ODR_5;
			break;
		case 5:
			if(newstate == LedOff)
				GPIOE->ODR &= ~GPIO_ODR_ODR_6;
			else if(newstate == LedOn)
				GPIOE->ODR |= GPIO_ODR_ODR_6;
			else if(newstate == LedTog)
				GPIOE->ODR ^= GPIO_ODR_ODR_6;
			break;
		default:
			break;
	}
}
void delay_ms(uint32_t ms)
{
	pC->Mode.times = 0;
	while(pC->Mode.times < ms);
}
void SysTick_Handler(void)
{
	IWDG->KR = 0xaaaa;
	pC->Mode.hostcomtick++;
	pC->Mode.times++;
	pC->Stats.time++;
	if(pC->Serw.turnstate != LSNeutral)
		pC->Mode.lumiserwtick++;
	if(pC->Mode.lumiserwtick > LUMISERWTICKMAX)
	{
		pC->Serw.turnstate = LSNeutral;
		pC->Serw.turnrefpwm = pC->Serw.turnpwm[0];
		SERW2PWM_REG = pC->Serw.turnrefpwm;
	}
}
