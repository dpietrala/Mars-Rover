#include "Control.h"
extern sControl* pC;
static void RCC_SystemInit(void)
{
	//Zewnetrzny kwarc 8MHz
//	uint32_t PLL_M=8, PLL_N=336, PLL_P=2, PLL_Q=7;
//	RCC->CR |= RCC_CR_HSEON;								//Wlaczenie HSE
//	while(!(RCC->CR & RCC_CR_HSERDY));			//czekamy az HSE bedzie gotowy
//	RCC->CFGR = RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_HPRE_DIV1;
//	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1)-1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
//	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
//	RCC->CR |= RCC_CR_PLLON;								//wlaczenie PLL
//	while(!(RCC->CR & RCC_CR_PLLRDY));			//czekanie az PLL gotowa
//	RCC->CFGR |= RCC_CFGR_SW_PLL;						//PLL jako zródlo dla SYSCLK
//	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); //czekanie a¿ PLL bedzie gotowe jako SYSCLK
	
	//Wewnetrzny kwarc 16 MHz
	uint32_t PLL_M=16, PLL_N=336, PLL_P=2, PLL_Q=7;
//	RCC->CR |= RCC_CR_HSEON;							//Wlaczenie HSE
//	while(!(RCC->CR & RCC_CR_HSERDY));		//czekamy az HSE bedzie gotowy
	RCC->CFGR = RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_HPRE_DIV1;
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1)-1) << 16) | (RCC_PLLCFGR_PLLSRC_HSI) | (PLL_Q << 24);
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
	RCC->CR |= RCC_CR_PLLON;								//wlaczenie PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));			//czekanie az PLL gotowa
	RCC->CFGR |= RCC_CFGR_SW_PLL;						//PLL jako zródlo dla SYSCLK
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); //czekanie az PLL bedzie gotowe jako SYSCLK
}
static void RCC_Conf(void)
{
	//Porty A,B,C,D,E,F,G,H,I
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
									RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
									RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN;
	
	//USART
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_UART4EN |
									RCC_APB1ENR_UART5EN;
	
	
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_BKPSRAMEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | 
									RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | 
									RCC_APB1ENR_PWREN | RCC_APB1ENR_TIM7EN;
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC3EN | 
									RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_SYSCFGEN;
}

static void Control_StructConf(void)
{
	pC->Com.framenum = 0;
	pC->Status.times = 0;
	pC->Status.hostcomtick = COMTICKMAX;
	pC->Status.hostcom = On;
	pC->Com.frames[Fnum_Drive].rfull = On;
	pC->Com.frames[Fnum_Manip].rfull = On;
	
	for(uint8_t i=0;i<AI_CHMAX;i++)
	{
		pC->Case.val_adc1[i] = 0.;
		pC->Case.val_adc2[i] = 0.;
		pC->Case.val_adc3[i] = 0.;
	}
	for(uint8_t i=0;i<AI_ALLCHMAX;i++)
	{
		pC->Case.val_adc_min[i] = 0.;
		pC->Case.val_adc_max[i] = 4095.;
		pC->Case.hyst[i] = 0.0;
	}
	pC->Case.hyst[AI_MJ_X] = 5.;
	pC->Case.hyst[AI_MJ_Y] = 5.;
	pC->Case.hyst[AI_MJ_Z] = 5.;
	pC->Case.hyst[AI_MJ_RX] = 5.;
	pC->Case.hyst[AI_MJ_RY] = 5.;
	pC->Case.hyst[AI_MJ_RZ] = 5.;
	pC->Case.hyst[AI_D_DSR] = 5.;
	pC->Case.hyst[AI_D_FSR] = 5.;
	pC->Case.hyst[AI_D_DSL] = 5.;
	pC->Case.hyst[AI_D_FSL] = 5.;
	
	pC->Case.val_adc_min[AI_MJ_X] = 660.;
	pC->Case.val_adc_max[AI_MJ_X] = 3290.;
	pC->Case.val_adc_min[AI_MJ_Y] = 660.;
	pC->Case.val_adc_max[AI_MJ_Y] = 3200.;
	pC->Case.val_adc_min[AI_MJ_Z] = 900.;
	pC->Case.val_adc_max[AI_MJ_Z] = 3050.;
	pC->Case.val_adc_min[AI_MJ_RX] = 620.;
	pC->Case.val_adc_max[AI_MJ_RX] = 3290.;
	pC->Case.val_adc_min[AI_MJ_RY] = 750.;
	pC->Case.val_adc_max[AI_MJ_RY] = 3270.;
	pC->Case.val_adc_min[AI_MJ_RZ] = 920.;
	pC->Case.val_adc_max[AI_MJ_RZ] = 3000.;
	pC->Case.val_adc_min[AI_M_GS] = 12.;
	pC->Case.val_adc_max[AI_M_GS] = 3770.;
	pC->Case.val_adc_min[AI_D_SP] = 12.;
	pC->Case.val_adc_max[AI_D_SP] = 3780.;
	
	pC->Case.val_adc_min[AI_MP_X] = 5.;
	pC->Case.val_adc_max[AI_MP_X] = 3770.;
	pC->Case.val_adc_min[AI_MP_Y] = 5.;
	pC->Case.val_adc_max[AI_MP_Y] = 3770.;
	pC->Case.val_adc_min[AI_MP_Z] = 5.;
	pC->Case.val_adc_max[AI_MP_Z] = 3770.;
	pC->Case.val_adc_min[AI_MP_RX] = 5.;
	pC->Case.val_adc_max[AI_MP_RX] = 3770.;
	pC->Case.val_adc_min[AI_MP_RY] = 5.;
	pC->Case.val_adc_max[AI_MP_RY] = 3770.;
	pC->Case.val_adc_min[AI_MP_RZ] = 5.;
	pC->Case.val_adc_max[AI_MP_RZ] = 3770.;
	pC->Case.val_adc_min[AI_GUP_0] = 5.;
	pC->Case.val_adc_max[AI_GUP_0] = 3770.;
	pC->Case.val_adc_min[AI_GUP_1] = 5.;
	pC->Case.val_adc_max[AI_GUP_1] = 3770.;
	
	pC->Case.val_adc_min[AI_D_SL] = 5.;
	pC->Case.val_adc_max[AI_D_SL] = 3780.;
	pC->Case.val_adc_min[AI_D_SR] = 5.;
	pC->Case.val_adc_max[AI_D_SR] = 3770.;
	pC->Case.val_adc_min[AI_D_DSR] = 770.;
	pC->Case.val_adc_max[AI_D_DSR] = 3290.;
	pC->Case.val_adc_min[AI_D_FSR] = 720.;
	pC->Case.val_adc_max[AI_D_FSR] = 3300.;
	pC->Case.val_adc_min[AI_D_DSL] = 710.;
	pC->Case.val_adc_max[AI_D_DSL] = 3200.;
	pC->Case.val_adc_min[AI_D_FSL] = 700.;
	pC->Case.val_adc_max[AI_D_FSL] = 3350.;
	pC->Case.val_adc_min[AI_GDW_0] = 7.;
	pC->Case.val_adc_max[AI_GDW_0] = 3770.;
	pC->Case.val_adc_min[AI_GDW_1] = 11.;
	pC->Case.val_adc_max[AI_GDW_1] = 3770.;
	
	for(uint8_t i=0;i<AI_ALLCHMAX;i++)
	{
		pC->Case.val_adc_mean[i] = (pC->Case.val_adc_max[i] - pC->Case.val_adc_min[i]) / 2.0;
		pC->Case.val_adc[i] = pC->Case.val_adc_mean[i];
	}
	pC->Case.val_adc_mean[AI_MJ_X] = 1895.;
	pC->Case.val_adc_mean[AI_MJ_Y] = 1930.;
	pC->Case.val_adc_mean[AI_MJ_Z] = 1910.;
	pC->Case.val_adc_mean[AI_MJ_RX] = 1890.;
	pC->Case.val_adc_mean[AI_MJ_RY] = 1920.;
	pC->Case.val_adc_mean[AI_MJ_RZ] = 1895.;
	
	pC->Case.val_adc_mean[AI_D_FSL] = 1900.;
	pC->Case.val_adc_mean[AI_D_DSL] = 1900.;
	pC->Case.val_adc_mean[AI_D_FSR] = 1895.;
	pC->Case.val_adc_mean[AI_D_DSR] = 1930.;
	
	
	pC->Case.joint = Off; 
	pC->Case.tool = Off;
	pC->Case.base = Off;
	pC->Case.home = Off;
	pC->Case.park = Off;
	pC->Case.work = Off;
	pC->Case.take = Off;
	pC->Case.calibration = Off;
	
	pC->Case.joint_prev = Off; 
	pC->Case.tool_prev = Off;
	pC->Case.base_prev = Off;
	pC->Case.home_prev = Off;
	pC->Case.park_prev = Off;
	pC->Case.work_prev = Off;
	pC->Case.take_prev = Off;
	pC->Case.calibration_prev = Off;
	
	pC->Case.select_cam1_next=Off;
	pC->Case.select_cam1_back=Off;
	pC->Case.select_cam2_next=Off;
	pC->Case.select_cam2_back=Off;
	
	pC->Case.select_cam1_next_prev=Off;
	pC->Case.select_cam1_back_prev=Off;
	pC->Case.select_cam2_next_prev=Off;
	pC->Case.select_cam2_back_prev=Off;
	
	pC->Case.gimbal_up[0] = 0;
	pC->Case.gimbal_up[1] = 0;
	pC->Case.gimbal_down[0] = 0;
	pC->Case.gimbal_down[1] = 0;
	
	pC->Case.station_pos = 0;
	pC->Case.gripper_speed = 0;
	pC->Case.gripper_open = Off;
	pC->Case.gripper_close = Off;
	pC->Case.laseron = Off;
	pC->Case.laseroff = Off;
	pC->Case.magneson = Off;
	pC->Case.magnesoff = Off;
	
	pC->Case.gripper_open_prev = Off;
	pC->Case.gripper_close_prev = Off;
	pC->Case.laseron_prev = Off;
	pC->Case.laseroff_prev = Off;
	pC->Case.magneson_prev = Off;
	pC->Case.magnesoff_prev = Off;
	
	pC->Case.autostm = Off;
	pC->Case.autonvidia = Off;
	pC->Case.autopause = Off;
	pC->Case.autocontinue = Off;
	pC->Case.autonext = Off;
	pC->Case.resetdev1 = Off;
	pC->Case.resetdev2 = Off;
	pC->Case.gimdigiton = Off;
	pC->Case.gimdigitoff = Off;
	pC->Case.automove = Off;
	pC->Case.clearseq = Off;
	pC->Case.driveTelemAllEn = Off;
	pC->Case.driveTelemAllDis = Off;
	pC->Case.manipTelemAllEn = Off;
	pC->Case.manipTelemAllDis = Off;
	
	pC->Case.autostm_prev = Off;
	pC->Case.autonvidia_prev = Off;
	pC->Case.autopause_prev = Off;
	pC->Case.autocontinue_prev = Off;
	pC->Case.autonext_prev = Off;
	pC->Case.resetdev1_prev = Off;
	pC->Case.resetdev2_prev = Off;
	pC->Case.gimdigiton_prev = Off;
	pC->Case.gimdigitoff_prev = Off;
	pC->Case.automove_prev = Off;
	pC->Case.clearseq_prev = Off;
	pC->Case.driveTelemAllEn_prev = Off;
	pC->Case.driveTelemAllDis_prev = Off;
	pC->Case.manipTelemAllEn_prev = Off;
	pC->Case.manipTelemAllDis_prev = Off;
	
	
	pC->Case.select_satel = Off;
	pC->Case.select_wifi = Off;
	
	pC->Case.select_satel_prev = Off;
	pC->Case.select_wifi_prev = Off;
	
	pC->Case.frontspeed_L = 0;
	pC->Case.dirspeed_R = 0;
	pC->Case.speed_L = 0;
	pC->Case.speed_R = 0;
	
	//manipulator_joy
	pC->Case.manip_joy_x = 0;
	pC->Case.manip_joy_y = 0;
	pC->Case.manip_joy_z = 0;
	pC->Case.manip_joy_Rx = 0;
	pC->Case.manip_joy_Ry = 0;
	pC->Case.manip_joy_Rz = 0;
	
	//manipulator_przyciski
	pC->Case.manip_x_speed = 0;
	pC->Case.manip_y_speed = 0;
	pC->Case.manip_z_speed = 0;
	pC->Case.manip_Rx_speed = 0;
	pC->Case.manip_Ry_speed = 0;
	pC->Case.manip_Rz_speed = 0;
	pC->Case.manip_x_L = Off;
	pC->Case.manip_x_R = Off;
	pC->Case.manip_y_L = Off;
	pC->Case.manip_y_R = Off;
	pC->Case.manip_z_L = Off;
	pC->Case.manip_z_R = Off;
	pC->Case.manip_Rx_L = Off;
	pC->Case.manip_Rx_R = Off;
	pC->Case.manip_Ry_L = Off;
	pC->Case.manip_Ry_R = Off;
	pC->Case.manip_Rz_L = Off;
	pC->Case.manip_Rz_R = Off;
	
	pC->Case.EmergencySwitchEn = Off;
	pC->Case.EmergencySwitchDis = Off;
	
	//Inicjalizacja zmiennych z telemetrii
	pC->Tele.imurollmax = 25.;
	pC->Tele.imupitchmax = 30.;
	
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
}
