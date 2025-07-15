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
									RCC_APB1ENR_PWREN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_UART4EN;
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM9EN | 
									RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN | RCC_APB2ENR_USART1EN | 
									RCC_APB2ENR_ADC1EN;
}
static void Control_StructConf(void)
{
//----Status------------------------------------
	pC->Status.time = 0;
	pC->Status.work = On;
	pC->Status.coordinate = Join;
	pC->Status.hostcomtime = 0;
	pC->Status.hostcomer = Off;
	pC->Status.backuper = Off;
//----Com------------------------------------
	pC->Com.framenum = 0;
	pC->Com.frames[Framenum_Telemetry].active = On;
	pC->Com.frames[Framenum_MotPos].active = Off;
	pC->Com.cmdin = Hostcmd_null;
	pC->Com.cmdinval = 0;
//----Mot---------------------------------
	pC->Mot.tims[0] = TIM1;
	pC->Mot.tims[1] = TIM2;
	pC->Mot.tims[2] = TIM3;
	pC->Mot.tims[3] = TIM4;
	pC->Mot.tims[4] = TIM5;
	pC->Mot.tims[5] = TIM8;
	pC->Mot.tims[6] = TIM8;
	pC->Mot.tims[7] = TIM8;	
	
	pC->Mot.impperrad[0] = 64.0 * 30.0 * 160.0 / M_2_PI;
	pC->Mot.impperrad[1] = 64.0 * 50.0 * 160.0 / M_2_PI;
	pC->Mot.impperrad[2] = 64.0 * 30.0 * 160.0 / M_2_PI;
	pC->Mot.impperrad[3] = 64.0 * 19.0 * 160.0 / M_2_PI;
	pC->Mot.impperrad[4] = 64.0 * 30.0 * 160.0 / M_2_PI;
	pC->Mot.impperrad[5] = 64.0 * 19.0 * 160.0 / M_2_PI;
	pC->Mot.impperrad[6] = 64.0 * 19.0 / M_2_PI;
	pC->Mot.impperrad[7] = 64.0 * 19.0 / M_2_PI;
	
	pC->Mot.speedmax[0] = 8000.0 / pC->Mot.impperrad[0];
	pC->Mot.speedmax[1] = 8000.0 / pC->Mot.impperrad[1];
	pC->Mot.speedmax[2] = 8000.0 / pC->Mot.impperrad[2];
	pC->Mot.speedmax[3] = 8000.0 / pC->Mot.impperrad[3];
	pC->Mot.speedmax[4] = 8000.0 / pC->Mot.impperrad[4];
	pC->Mot.speedmax[5] = 8000.0 / pC->Mot.impperrad[5];
	pC->Mot.speedmax[6] = 0.0;
	pC->Mot.speedmax[7] = 0.0;
	
	pC->Mot.posabsmin[0] = -1.4*M_PI;
	pC->Mot.posabsmin[1] = -M_PI;
	pC->Mot.posabsmin[2] = -2.98;
	pC->Mot.posabsmin[3] = -1.2*M_PI;
	pC->Mot.posabsmin[4] = -1.4*M_PI_2;
	pC->Mot.posabsmin[5] = -30000.0*M_PI;
	pC->Mot.posabsmin[6] = -1.0;
	pC->Mot.posabsmin[7] = 0.0;
	
	pC->Mot.posabsmax[0] = 1.4*M_PI;
	pC->Mot.posabsmax[1] = 0.01;
	pC->Mot.posabsmax[2] = 2.98;
	pC->Mot.posabsmax[3] = 1.3*M_PI;
	pC->Mot.posabsmax[4] = 1.4*M_PI_2;
	pC->Mot.posabsmax[5] = 30000.0*M_PI;
	pC->Mot.posabsmax[6] = 1.0;
	pC->Mot.posabsmax[7] = 0.0;
	
	pC->Mot.posdef[0] = 0.01;
	pC->Mot.posdef[1] = -M_PI_2+0.01;
	pC->Mot.posdef[2] = M_PI_2+0.01;
	pC->Mot.posdef[3] = 0.01;
	pC->Mot.posdef[4] = 0.05;
	pC->Mot.posdef[5] = 0.01;
	pC->Mot.posdef[6] = 0.0;
	pC->Mot.posdef[7] = 0.0;
	
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Mot.pid[i] = On;
		pC->Mot.imp[i] = 0;
		pC->Mot.implast[i] = 0;
		pC->Mot.impfullturn[i] = 0;
		pC->Mot.speedref[i] = 0;
		pC->Mot.pos[i] = 0;
		pC->Mot.posabs[i] = 0;
		pC->Mot.posfullturn[i] = 0;
		pC->Mot.posref[i] = 0;
		pC->Mot.posstart[i] = 0;
		pC->Mot.step[i] = pC->Mot.speedmax[i] * BASETIME / 100.0;
		pC->Mot.kp[i] = 55.0;
		pC->Mot.ki[i] = 0.6;
		pC->Mot.kd[i] = 0.4;
		pC->Mot.e[i] = 0;
		pC->Mot.emax[i] = 2.0;
		pC->Mot.elast[i] = 0;
		pC->Mot.etotal[i] = 0;
		pC->Mot.etotalmax[i] = 200.0;
		pC->Mot.out[i] = 0;
		pC->Mot.outmax[i] = 1.0;
		pC->Mot.pwm[i] = 0;
		pC->Mot.pwmout[i] = 0;
		pC->Mot.pwmmax[i] = 990;
		pC->Mot.pwmdeath[i] = 20;
		pC->Mot.dir[i] = Stop;
		pC->Mot.current[i] = 0;
		pC->Mot.currentmax[i] = 4;
	}
	pC->Mot.pwmmax[6] = 700;
	
	pC->Mot.pid[6] = Off;
	pC->Mot.pid[7] = Off;
	for(uint16_t i=0;i<AXESMAX*1000;i++)
		pC->Mot.adcval[i] = 0;
//----Axes---------------------------------
	
	pC->Axes.kartposstep[0] = 70.0 * 0.05 / 100.0;
	pC->Axes.kartposstep[1] = 70.0 * 0.05 / 100.0;
	pC->Axes.kartposstep[2] = 70.0 * 0.05 / 100.0;
	pC->Axes.kartposstep[3] = 0.15 * 0.05 / 100.0;
	pC->Axes.kartposstep[4] = 0.15 * 0.05 / 100.0;
	pC->Axes.kartposstep[5] = 0.25 * 0.05 / 100.0;
	pC->Axes.kartposstep[6] = 0.0;
	pC->Axes.kartposstep[7] = 0.0;

//********************* Sekwencje wartosci domyslne przed odczytem z backapu *****************************
	for(uint8_t i=0;i<AUTOSEQMAX;i++)
	{
		for(uint8_t j=0;j<AUTOPOINTMAX;j++)
		{
			pC->Axes.autoseq[i].points[j].active = Off;
			pC->Axes.autoseq[i].points[j].type = AutoPtp;
			pC->Axes.autoseq[i].points[j].speed = 100;
			for(uint8_t k=0;k<AXESMAX;k++)
			{
				pC->Axes.autoseq[i].points[j].pos[k] = pC->Mot.posdef[k];
			}
		}
	}
	//Zapisane cztery pierwsze sekwencje dla Home Park Take i Work. Po jedym punkcie w kazdej sekwencji
	//Pozycja Home - kalibracja
	pC->Axes.autoseq[0].points[0].active = On;
	pC->Axes.autoseq[0].points[0].type = AutoPtp;
	pC->Axes.autoseq[0].points[0].speed = 100;
	for(uint8_t i=0;i<AXESMAX;i++)
		pC->Axes.autoseq[0].points[0].pos[i] = pC->Mot.posdef[i];
	
	//Pozycja Park - zlozony manipulator
	pC->Axes.autoseq[1].points[0].active = On;
	pC->Axes.autoseq[1].points[0].type = AutoPtp;
	pC->Axes.autoseq[1].points[0].speed = 100;
	for(uint8_t i=0;i<AXESMAX;i++)
		pC->Axes.autoseq[1].points[0].pos[i] = pC->Mot.posdef[i];
	pC->Axes.autoseq[1].points[0].pos[1] = -2.95;
	pC->Axes.autoseq[1].points[0].pos[2] = 2.95;
	
	//Pozycja Take
	pC->Axes.autoseq[2].points[0].active = On;
	pC->Axes.autoseq[2].points[0].type = AutoPtp;
	pC->Axes.autoseq[2].points[0].speed = 100;
	pC->Axes.autoseq[2].points[0].pos[0] = 0.93;
	pC->Axes.autoseq[2].points[0].pos[1] = -2.57;
	pC->Axes.autoseq[2].points[0].pos[2] = 2.23;
	pC->Axes.autoseq[2].points[0].pos[3] = 1.08;
	pC->Axes.autoseq[2].points[0].pos[4] = 0.80;
	pC->Axes.autoseq[2].points[0].pos[5] = -0.90;
	pC->Axes.autoseq[2].points[0].pos[6] = pC->Mot.posdef[6];
	pC->Axes.autoseq[2].points[0].pos[7] = pC->Mot.posdef[7];

	//Pozycja Work - do podnoszenia
	pC->Axes.autoseq[3].points[0].active = On;
	pC->Axes.autoseq[3].points[0].type = AutoPtp;
	pC->Axes.autoseq[3].points[0].speed = 100;
	pC->Axes.autoseq[3].points[0].pos[0] = 1.56;
	pC->Axes.autoseq[3].points[0].pos[1] = -1.43;
	pC->Axes.autoseq[3].points[0].pos[2] = 1.57;
	pC->Axes.autoseq[3].points[0].pos[3] = 0.03;
	pC->Axes.autoseq[3].points[0].pos[4] = 1.51;
	pC->Axes.autoseq[3].points[0].pos[5] = 0.05;
	pC->Axes.autoseq[3].points[0].pos[6] = pC->Mot.posdef[6];
	pC->Axes.autoseq[3].points[0].pos[7] = pC->Mot.posdef[7];
	
	Axes_AutoSavePointToBackup();

	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Axes.speedref[i] = 0.0;
		
		pC->Axes.kartpos[i] = 0.0;
		pC->Axes.kartposref[i] = 0.0;
		pC->Axes.kartangle[i] = 0.0;
		pC->Axes.kartangleabs[i] = 0.0;
		pC->Axes.kartanglerefabs[i] = 0.0;
		pC->Axes.kartangleref[i] = 0.0;
		
		pC->Axes.autoglobalinpos = On;
		pC->Axes.autoglobalspeed = 100.;
		pC->Axes.autoposref[i] = pC->Mot.posdef[i];
		pC->Axes.autoposrefabs[i] = pC->Mot.posabs[i];
		pC->Axes.autoposrefabstemp[i] = pC->Mot.posabs[i];
		pC->Axes.autohyst[i] = 250.0 * pC->Mot.step[i];
		pC->Axes.autospeed[i] = 0.;
		pC->Axes.autoinpos[i] = On;
	}
}
static void Control_TimConf(void)
{
	TIM7->PSC = 84-1;
	TIM7->ARR = 50000-1;	//Przerwanie co 50 ms
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
}
static void Control_BackupConf(void)
{
	PWR->CR = PWR_CR_DBP;
	PWR->CSR = PWR_CSR_BRE;
	while (!( PWR->CSR & PWR_CSR_BRR));
}
static void Control_WriteBackup(void)
{
	volatile int32_t * const p2 = (int32_t *)BACKUPADDRESS;
	for(uint32_t i=0;i<AXESMAX;i++)
	{
		*(p2 + i) = (int32_t)(pC->Mot.posabs[i] * 10000);
	}
}
static void Control_ReadBackup(void)
{
//	volatile int32_t * const p1 = (int32_t *)0x40024000;
	volatile int32_t * const p2 = (int32_t *)BACKUPADDRESS;
	double pos[AXESMAX];
//	if(*p1 == 1001)
	{
		for(uint8_t i=0;i<AXESMAX;i++)
		{
			pos[i] = (double)*(p2 + i) / 10000;
			if(pos[i] > pC->Mot.posabsmax[i] || pos[i] < pC->Mot.posabsmin[i])
				pos[i] = pC->Mot.posdef[i];
			pC->Mot.posabs[i] = pos[i];
			pC->Mot.posstart[i] = pos[i];
			pC->Mot.posref[i] = pos[i];
			pC->Axes.kartangleabs[i] = pos[i];
			pC->Axes.kartanglerefabs[i] = pos[i];
		}
	}
//	else
//	{
//		pC->Status.backuper = On;
//		LED4_ON;
//	}
		
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
	SysTick_Config(168000);
	RCC_Conf();
	Control_BackupConf();
	Control_StructConf();
	Control_TimConf();
	Control_ReadBackup();
	Axes_AutoReadPointFromBackup();
	Control_IWDGConf();
}
static void Control_Status(void)
{
	if(pC->Status.hostcomtime >= COMTICKMAX)
	{
		pC->Status.hostcomer = On;
		pC->Status.hostcomtime = COMTICKMAX;
	}
	else
	{
		pC->Status.hostcomer = Off;
	}
	
	if(pC->Status.hostcomer)
	{
		pC->Status.work = Off;
	}
	else
	{
		pC->Status.work = On;
	}
}
static void Control_ReadCmdFromHost(void)
{
	switch(pC->Com.cmdin)
	{
		case Hostcmd_enable:					Axes_ManipEnable();									break;
		case Hostcmd_disable:					Axes_ManipDisable();								break;
		case Hostcmd_calibrate:				Axes_Defpos();											break;
		case Hostcmd_join:						Axes_CoordinateAct(Hostcmd_join);		break;
		case Hostcmd_base:						Axes_CoordinateAct(Hostcmd_base);		break;
		case Hostcmd_tool:						Axes_CoordinateAct(Hostcmd_tool);		break;
		case Hostcmd_laseron:					LASER_ON;														break;
		case Hostcmd_laseroff:				LASER_OFF;													break;
		case Hostcmd_solenoidon:			SOLENOID_ON;												break;
		case Hostcmd_solenoidoff:			SOLENOID_OFF;												break;
		case Hostcmd_telemerrorsen:		pC->Com.frames[Framenum_Telemetry].active = On;			break;
		case Hostcmd_telemerrorsdis:	pC->Com.frames[Framenum_Telemetry].active = Off;		break;
		case Hostcmd_telemposen:			pC->Com.frames[Framenum_MotPos].active = On;				break;
		case Hostcmd_telemposdis:			pC->Com.frames[Framenum_MotPos].active = Off;				break;
		case Hostcmd_telemallen:
			pC->Com.frames[Framenum_Telemetry].active = On;
			pC->Com.frames[Framenum_MotPos].active = On;
		break;
		case Hostcmd_telemalldis:
			pC->Com.frames[Framenum_Telemetry].active = Off;
			pC->Com.frames[Framenum_MotPos].active = Off;
		break;
		
		case Hostcmd_automove:
			pC->Axes.autoseqnum = pC->Com.cmdinval;
			pC->Axes.autopointnum = 0;
			Axes_AutoSystemOn(pC->Axes.autoseqnum, pC->Axes.autopointnum);
			break;
		
		case Hostcmd_clearseq:
			Axes_AutoClearSeq(pC->Com.cmdinval);
			Axes_AutoSavePointToBackup();
			break;
		
		default:	return;
	}
	pC->Com.cmdin = Hostcmd_null;
	pC->Com.cmdinval = 0;
}
void Control_Act(void)
{
	Control_Status();
	Control_ReadCmdFromHost();
	Axes_Act();
	Control_WriteBackup();
}
void delay_ms(uint32_t ms)
{
	pC->Status.time = 0;
	while(pC->Status.time < ms);
}
//----------------Przerwania--------------------------------------
void SysTick_Handler(void)
{
	IWDG->KR = 0xaaaa;
	pC->Status.time++;
	pC->Status.hostcomtime++;
	pC->Axes.autotimetick++;
	if(pC->Status.calibratedflagtime++ > FLAGTIME)
		pC->Status.calibrated = Off;
}
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		Control_Act();
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
