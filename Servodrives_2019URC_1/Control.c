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
									RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN |
									RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN | RCC_APB2ENR_ADC1EN | 
									RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN;
}
static void Led_Conf(void)
{
	GPIOE->MODER 	|= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0;
	GPIOE->PUPDR 	|= GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR6_0;
}
static void Control_StructConf(void)
{
	pC->Status.times = 0;
	pC->Status.work = On;
	pC->Status.hostcomtick = 0;
	pC->Status.hostcom = Off;
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Status.overcurrent[i] = Off;
		pC->Status.overspeed[i] = Off;
	}
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		pC->Mot.pid[i] = Off;
		pC->Mot.pos[i] = 0;
		pC->Mot.poslast[i] = 0;
		pC->Mot.fullturn[i] = 0;
		pC->Mot.postotal[i] = 0;
		pC->Mot.postotallast[i] = 0;
		pC->Mot.speed[i] = 0;
		pC->Mot.speedref[i] = 0;
		pC->Mot.speedrefmax[i] = 150.0;
		pC->Mot.distance[i]= 0;
		pC->Mot.kp[i] = 0.3;
		pC->Mot.ki[i] = 5;
		pC->Mot.kd[i] = 0;
		pC->Mot.e[i] = 0;
		pC->Mot.emax[i] = 30.0;
		pC->Mot.elast[i] = 0;
		pC->Mot.etotal[i] = 0;
		pC->Mot.etotalmax[i] = 1500.0;
		pC->Mot.out[i] = 0;
		pC->Mot.outmax[i] = 150.0;
		pC->Mot.pwm[i] = 0;
		pC->Mot.pwmout[i] = 0;
		pC->Mot.pwmdeath[i] = 10;
		pC->Mot.pwmmax[i] = 999;
		pC->Mot.dir[i] = Stop;
		pC->Mot.current[i] = 0;
		pC->Mot.currentmax[i] = 20.0;
		pC->Mot.currenterror[i] = 0.0;
	}
	pC->Mot.azimuth = 0.0;
	pC->Mot.roverspeed = 0.0;
	pC->Mot.posx = 0.0;
	pC->Mot.posy = 0.0;
	pC->Mot.tims[0] = TIM2;
	pC->Mot.tims[1] = TIM3;
	pC->Mot.tims[2] = TIM4;
	pC->Mot.tims[3] = TIM5;
	
	for(uint32_t j=0;j<1000;j++)
		pC->Mot.adcval[j] = 0;
	
	ClearStr(pC->Com.bufread, BUF_LEN);
	ClearStr(pC->Com.bufwrite, BUF_LEN);
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
void delay_ms(uint32_t ms)
{
	pC->Status.times = 0;
	while(pC->Status.times < ms);
}
void SysTick_Handler(void)
{
	IWDG->KR = 0xaaaa;
	pC->Status.hostcomtick++;
	pC->Status.times++;
	for(uint8_t i=0;i<MOTMAX;i++)
		pC->Mot.curenttime[i]++;
}
