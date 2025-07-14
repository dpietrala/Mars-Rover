#include "Comunication.h"
extern sControl* pC;
void Com_Conf(void)
{
	TIM6->PSC = 840-1;
	TIM6->ARR = 12000-1; //120ms
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
//---------- Usart6 Satel	-------------------------------------
	DMA2_Stream1->PAR 	= (uint32_t)&USART6->DR;
	DMA2_Stream1->M0AR 	= (uint32_t)pC->Com.satelbufread;
	DMA2_Stream1->NDTR 	= (uint16_t)BUF_LEN;
	DMA2_Stream1->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0 | DMA_SxCR_EN;
	
	DMA2_Stream6->PAR 	= (uint32_t)&USART6->DR;
	DMA2_Stream6->M0AR 	= (uint32_t)pC->Com.satelbufwrite;
	DMA2_Stream6->NDTR 	= (uint16_t)BUF_LEN;
	DMA2_Stream6->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0 | DMA_SxCR_DIR_0;
	
	GPIOC->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
	GPIOC->AFR[0] |= 0x88000000;
	USART6->BRR = 84000000/19200;
	USART6->CR3	|= USART_CR3_DMAR | USART_CR3_DMAT;
	USART6->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART6_IRQn);
	
//---------- Usart3 Wifi	-------------------------------------
	DMA1_Stream1->PAR 	= (uint32_t)&USART3->DR;
	DMA1_Stream1->M0AR 	= (uint32_t)pC->Com.wifibufread;
	DMA1_Stream1->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream1->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
	
	DMA1_Stream3->PAR 	= (uint32_t)&USART3->DR;
	DMA1_Stream3->M0AR 	= (uint32_t)pC->Com.wifibufwrite;
	DMA1_Stream3->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream3->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0;
	
	GPIOD->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;
	GPIOD->AFR[1] |= 0x00000077;
	USART3->BRR = 42000000/230400;
	USART3->CR3	|= USART_CR3_DMAR | USART_CR3_DMAT;
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART3_IRQn);
	
//---------- Usart4 Manipulator / Labaratorium	-------------------------------------
	DMA1_Stream2->PAR 	= (uint32_t)&UART4->DR;
	DMA1_Stream2->M0AR 	= (uint32_t)pC->Com.manipbufread;
	DMA1_Stream2->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream2->CR 		|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
	
	DMA1_Stream4->PAR 	= (uint32_t)&UART4->DR;
	DMA1_Stream4->M0AR 	= (uint32_t)pC->Com.manipbufwrite;
	DMA1_Stream4->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream4->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0;
	
	GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
	GPIOC->AFR[1] |= 0x00008800;
	UART4->BRR = 42000000/230400;
	UART4->CR3	|= USART_CR3_DMAR | USART_CR3_DMAT;
	UART4->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(UART4_IRQn);
	
//---------- Usart5 NVidia Jetson	-------------------------------------
	DMA1_Stream0->PAR 	= (uint32_t)&UART5->DR;
	DMA1_Stream0->M0AR 	= (uint32_t)pC->Com.nvbufread;
	DMA1_Stream0->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream0->CR 		|= DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
	
	DMA1_Stream7->PAR 	= (uint32_t)&UART5->DR;
	DMA1_Stream7->M0AR 	= (uint32_t)pC->Com.nvbufwrite;
	DMA1_Stream7->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream7->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0;
	
	GPIOC->MODER |= GPIO_MODER_MODER12_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR12_0;
	GPIOC->AFR[1] |= 0x00080000;
	GPIOD->MODER |= GPIO_MODER_MODER2_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPDR2_0;
	GPIOD->AFR[0] |= 0x00000800;
	UART5->BRR = 42000000/115200;
	UART5->CR3	|= USART_CR3_DMAR | USART_CR3_DMAT;
	UART5->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(UART5_IRQn);
}
static void Com_ReadCmd(void)
{
	switch(pC->Com.cmdin)
	{
		case Hostcmd_satel:			pC->Mode.SelectHost = Satel;	break;
		case Hostcmd_wifi:			pC->Mode.SelectHost = Wifi;		break;
		case Hostcmd_autostm:
			pC->Mode.SelectHost = AutoStm;
			pC->Drive.nrp = 0;
			pC->Drive.refpoint = pC->Drive.points[pC->Drive.nrp];
		break;
		case Hostcmd_autopause:
			if(pC->Drive.refpoint.status != Waitpoint)
			{
				pC->Drive.refpoint.statustemp = pC->Drive.refpoint.status;
				pC->Drive.refpoint.status = Waitpoint;
			}
		break;
		case Hostcmd_autocontinue:		pC->Drive.refpoint.status = pC->Drive.refpoint.statustemp;break;
		case Hostcmd_autonext:				Drive_AutoNextPoint();																		break;
		case Hostcmd_clearyawoffset: 	pC->Dev.imuyawoffset = 0;																	break;
		case Hostcmd_setyawoffset:		pC->Dev.imuyawoffset = pC->Dev.imuyawraw;									break;
		case Hostcmd_resetdev1: 			Dev_ResetDevice1(); 																			break;
		case Hostcmd_resetdev2: 			Dev_ResetDevice2(); 																			break;
//		case Hostcmd_gimdigiton: 			pC->Dev.Gimdigit = On; 																		break;
//		case Hostcmd_gimdigitoff: 		pC->Dev.Gimdigit = Off; 																	break;
		case Hostcmd_cam1inc:					if(pC->Dev.Trans1++ >= CAMMAX) pC->Dev.Trans1 = CAMMAX; 	break;
		case Hostcmd_cam1dec:					if(pC->Dev.Trans1-- <= 1) pC->Dev.Trans1 = 1;							break;
		case Hostcmd_cam2inc:					if(pC->Dev.Trans2++ >= CAMMAX) pC->Dev.Trans2 = CAMMAX;		break;
		case Hostcmd_cam2dec:					if(pC->Dev.Trans2-- <= 1) pC->Dev.Trans2 = 1;							break;
		case Hostcmd_TelemGpsEn: 			pC->Com.frames[Framenum_GpsPos].active = On;							break;
		case Hostcmd_TelemGpsDis: 		pC->Com.frames[Framenum_GpsPos].active = Off;							break;
		case Hostcmd_TelemBallEn: 		pC->Com.frames[Framenum_BallPos].active = On;							break;
		case Hostcmd_TelemBallDis: 		pC->Com.frames[Framenum_BallPos].active = Off;						break;
		case Hostcmd_TelemErrorsEn: 	pC->Com.frames[Framenum_Telemetry].active = On;						break;
		case Hostcmd_TelemErrorsDis: 	pC->Com.frames[Framenum_Telemetry].active = Off;					break;
		case Hostcmd_TelemAllEn:
			pC->Com.frames[Framenum_GpsPos].active = On;
			pC->Com.frames[Framenum_BallPos].active = On;
			pC->Com.frames[Framenum_Telemetry].active = On;
		break;
		case Hostcmd_TelemAllDis:
			pC->Com.frames[Framenum_GpsPos].active = Off;
			pC->Com.frames[Framenum_BallPos].active = Off;
			pC->Com.frames[Framenum_Telemetry].active = Off;
		break;
		case Hostcmd_EmerSoftEn: 			pC->Error.emergencysoft = On; 														break;
		case Hostcmd_EmerSoftDis: 		pC->Error.emergencysoft = Off; 														break;
		default:																																								break;
	}
	pC->Com.cmdin = Hostcmd_null;
}
static void Com_ReadData(uint8_t* buf)
{
	pC->Drive.frontvalue = (int8_t)buf[2];
	pC->Drive.dirvalue = (int8_t)buf[3];
	pC->Dev.SerwPos[0] = buf[4];
	pC->Dev.SerwPos[1] = buf[5];
	pC->Dev.SerwPos[2] = buf[6];
	pC->Dev.SerwPos[3] = buf[7];
	pC->Error.hostcomtick = 0;
}
static void Com_AutoAddHeliPoint(void)
{
	uint8_t offset = pC->Drive.numrecpoints;
	double lonstart = pC->Drive.points[offset - 1].lon;
	double latstart = pC->Drive.points[offset - 1].lat;
	double thetamax = 3.0*M_2_PI;
	double rmax = 0.00008;
	for(uint8_t i=0;i<HELIPOINTMAX;i++)
	{
		double theta = (double)i * thetamax / (double)HELIPOINTMAX;
		double r = (double)i * rmax / (double)HELIPOINTMAX;
		double lon, lat;
		lon = r*cos(theta);
		lat = r*sin(theta);
		
		pC->Drive.points[offset+i].activated = On;
		pC->Drive.points[offset+i].lon = -lon + lonstart;
		pC->Drive.points[offset+i].lat = lat + latstart;
		pC->Drive.points[offset+i].type = Lookpoint;
		pC->Drive.points[offset+i].status = Lookball;
		pC->Drive.points[offset+i].statustemp = Lookball;
	}
}
static void Com_AutoNewPoints(uint8_t* buf)
{
	uint16_t np = buf[3];
	if(np > (POINTMAX - HELIPOINTMAX))
		return;
	uint16_t nd = 10 * np + 4;
	uint16_t crc1 = crc16(buf, nd);
	uint16_t crc2 = ((uint16_t)buf[nd]<<8) + ((uint16_t)buf[nd+1]<<0);
	if(crc1 == crc2)
	{
		for(uint16_t i=0;i<POINTMAX;i++)
		{
			pC->Drive.points[i].activated = Off;
			pC->Drive.points[i].lon = -1.0;
			pC->Drive.points[i].lat = -1.0;
			pC->Drive.points[i].yawref = 0.0;
			pC->Drive.points[i].yawer = 0.0;
			pC->Drive.points[i].disttopoint = 5.0;
			pC->Drive.points[i].type = Temppoint;
			pC->Drive.points[i].status = Waitpoint;
			pC->Drive.points[i].statustemp = Waitpoint;
		}
		pC->Drive.numpoints = 0;
		pC->Drive.numrecpoints = 0;
		for(uint16_t i=0;i<np;i++)
		{
			double lon = (double)((int64_t)(((uint64_t)buf[10*i+4+0]<<32) + ((uint64_t)buf[10*i+4+1]<<24) + ((uint64_t)buf[10*i+4+2]<<16) + ((uint64_t)buf[10*i+4+3]<<8) + ((uint64_t)buf[10*i+4+4]<<0))) / 10000000.0;
			double lat = (double)((int64_t)(((uint64_t)buf[10*i+4+5]<<32) + ((uint64_t)buf[10*i+4+6]<<24) + ((uint64_t)buf[10*i+4+7]<<16) + ((uint64_t)buf[10*i+4+8]<<8) + ((uint64_t)buf[10*i+4+9]<<0))) / 10000000.0;
			pC->Drive.points[i].activated = On;
			pC->Drive.points[i].lon = -lon;
			pC->Drive.points[i].lat = lat;
			pC->Drive.points[i].type = Temppoint;
			pC->Drive.points[i].status = Drivetopoint;
			pC->Drive.points[i].statustemp = Drivetopoint;
		}
		pC->Drive.nrp = 0;
		pC->Drive.refpoint = pC->Drive.points[pC->Drive.nrp];
		pC->Drive.autonprec = On;
		pC->Drive.autonprectime = 0;
		pC->Drive.numrecpoints = np;
		if(buf[2] == 1)
		{
			pC->Drive.numpoints = pC->Drive.numrecpoints;
			pC->Drive.points[pC->Drive.numpoints - 1].type = Checkpoint;
		}
		else if(buf[2] == 0)
		{
			Com_AutoAddHeliPoint();
			pC->Drive.numpoints = pC->Drive.numrecpoints + HELIPOINTMAX;
		}
	}
}
static void Com_ReadOnlyCmdDrive(uint8_t *buf)
{
	uint16_t crc1 = crc16(buf, 3);
	uint16_t crc2 = ((uint16_t)buf[3]<<8) + ((uint16_t)buf[4]<<0);
	if(crc1 == crc2)
	{
		pC->Com.cmdin = (eHostCmd)buf[2];
		Com_ReadCmd();
	}
}
static void Com_SendToSatel(uint8_t* buf, uint8_t len)
{
	ClearStr(pC->Com.satelbufwrite, BUF_LEN);
	for(uint8_t i=0;i<BUF_LEN;i++)
		pC->Com.satelbufwrite[i] = buf[i];
	
	DMA2_Stream6->CR 		&= ~DMA_SxCR_EN;
	DMA2->HIFCR 				|= DMA_HIFCR_CTCIF6;
	DMA2_Stream6->NDTR = len;
	DMA2_Stream6->CR 		|= DMA_SxCR_EN;
}
static void Com_SendToWifi(uint8_t* buf, uint8_t len)
{
	ClearStr(pC->Com.wifibufwrite, BUF_LEN);
	for(uint8_t i=0;i<BUF_LEN;i++)
		pC->Com.wifibufwrite[i] = buf[i];
	
	DMA1_Stream3->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF3;
	DMA1_Stream3->NDTR = len;
	DMA1_Stream3->CR 		|= DMA_SxCR_EN;
}
static void Com_SendToManip(uint8_t* buf, uint8_t len)
{
	ClearStr(pC->Com.manipbufwrite, BUF_LEN);
	if(pC->Error.emergency == On)
	{
		uint8_t index = 0;
		pC->Com.manipbufwrite[index++] = (uint8_t)Frametype_Header;
		pC->Com.manipbufwrite[index++] = (uint8_t)Frametype_emergencyhardStop;
		pC->Com.manipbufwrite[index++] = Hostcmd_null;
		uint16_t crc = crc16(pC->Com.manipbufwrite, index);
		pC->Com.manipbufwrite[index++] = (uint8_t)(crc>>8);
		pC->Com.manipbufwrite[index++] = (uint8_t)crc;
		DMA1_Stream4->CR 		&= ~DMA_SxCR_EN;
		DMA1->HIFCR 				|= DMA_HIFCR_CTCIF4;
		DMA1_Stream4->NDTR = index;
		DMA1_Stream4->CR 		|= DMA_SxCR_EN;
	}
	else if(pC->Error.emergency == Off)
	{
		for(uint8_t i=0;i<BUF_LEN;i++)
			pC->Com.manipbufwrite[i] = buf[i];
		DMA1_Stream4->CR 		&= ~DMA_SxCR_EN;
		DMA1->HIFCR 				|= DMA_HIFCR_CTCIF4;
		DMA1_Stream4->NDTR = len;
		DMA1_Stream4->CR 		|= DMA_SxCR_EN;
	}
}
static void Com_ReadFromSatel(void)
{
	uint8_t* buf = pC->Com.satelbufread;
	if(pC->Mode.SelectHost == Satel || pC->Mode.SelectHost == AutoStm || pC->Mode.SelectHost == AutoNvidia)
	{
		Com_SendToManip(buf, (BUF_LEN - DMA2_Stream1->NDTR));
	}
	if(buf[0] == Frametype_Header)
	{
		if(buf[1] == Frametype_DriveGeneral)
		{
			uint16_t crc1 = crc16(buf, 9);
			uint16_t crc2 = ((uint16_t)buf[9]<<8) + ((uint16_t)buf[10]<<0);
			if(crc1 == crc2)
			{
				pC->Com.cmdin = (eHostCmd)buf[8];
				Com_ReadCmd();
				if(pC->Mode.SelectHost == Satel || pC->Mode.SelectHost == AutoStm || pC->Mode.SelectHost == AutoNvidia)
				{
					Com_ReadData(buf);
				}
			}
		}
		else if(buf[1] == Frametype_Newpoints)
		{
			Com_AutoNewPoints(buf);
		}
		else if(buf[1] == Frametype_DriveCmd)
		{
			Com_ReadOnlyCmdDrive(buf);
		}
	}
	ClearStr(buf, BUF_LEN);
	DMA2_Stream1->CR 		&= ~DMA_SxCR_EN;
	DMA2->LIFCR 				|= DMA_LIFCR_CTCIF1;
	DMA2_Stream1->CR 		|= DMA_SxCR_EN;
}
static void Com_ReadFromWifi(void)
{
	uint8_t* buf = pC->Com.wifibufread;
	if(pC->Mode.SelectHost == Wifi || pC->Mode.SelectHost == AutoStm || pC->Mode.SelectHost == AutoNvidia)
	{
		Com_SendToManip(buf, (BUF_LEN - DMA1_Stream1->NDTR));
	}
	if(buf[0] == Frametype_Header)
	{
		if(buf[1] == Frametype_DriveGeneral)
		{
			uint16_t crc1 = crc16(buf, 9);
			uint16_t crc2 = ((uint16_t)buf[9]<<8) + ((uint16_t)buf[10]<<0);
			if(crc1 == crc2)
			{
				pC->Com.cmdin = (eHostCmd)buf[8];
				Com_ReadCmd();
				if(pC->Mode.SelectHost == Wifi || pC->Mode.SelectHost == AutoStm || pC->Mode.SelectHost == AutoNvidia)
				{
					Com_ReadData(buf);
				}
			}
		}
		else if(buf[1] == Frametype_Newpoints)
		{
			Com_AutoNewPoints(buf);
		}
		else if(buf[1] == Frametype_DriveCmd)
		{
			Com_ReadOnlyCmdDrive(buf);
		}
	}
	ClearStr(buf, BUF_LEN);
	DMA1_Stream1->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF1;
	DMA1_Stream1->CR 		|= DMA_SxCR_EN;
}
static void Com_ReadFromManip(void)
{
	ClearStr(pC->Com.frames[Framenum_Manip].frame, BUF_LEN);
	pC->Com.frames[Framenum_Manip].len = BUF_LEN - DMA1_Stream2->NDTR;
	pC->Com.frames[Framenum_Manip].full = On;
	for(uint8_t i=0;i<pC->Com.frames[Framenum_Manip].len;i++)
		pC->Com.frames[Framenum_Manip].frame[i] = pC->Com.manipbufread[i];
	DMA1_Stream2->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF2;
  DMA1_Stream2->CR 		|= DMA_SxCR_EN;
}
static void Com_ReadFromNVidia(void)
{
	uint8_t* buf = pC->Com.nvbufread;
	if(buf[0] == Frametype_Header)
	{
		if(buf[1] == Frametype_NvidiaGenaral)
		{
			uint16_t crc1 = crc16(buf, 6);
			uint16_t crc2 = ((uint16_t)buf[6]<<8) + ((uint16_t)buf[7]<<0);
			if(crc1 == crc2)
			{
				LED2_TOG;
				pC->Error.nvcomtick = 0;
				double ang = (double)((int16_t)(((uint16_t)buf[2]<<8) + ((uint16_t)buf[3]<<0)));
				double dist = (double)((int16_t)(((uint16_t)buf[4]<<8) + ((uint16_t)buf[5]<<0))) / 100.0;
				if(dist > 0.0)
				{
					pC->Drive.ball.activated = On;
					double yaw = pC->Dev.imuyaw + ang;
					double lonB, latB;
					
					Drive_FindGeoFromDist(pC->Dev.gpslon, pC->Dev.gpslat, yaw, dist, &lonB, &latB);
					pC->Drive.ball.disttopoint = dist;
					pC->Drive.ball.yawref = yaw;
					pC->Drive.ball.lon = lonB;
					pC->Drive.ball.lat = latB;
					pC->Drive.ball.status = Drivetoball;
					pC->Drive.ball.statustemp = Drivetoball;
					pC->Drive.ball.type = Checkpoint;
					
					ClearStr(pC->Com.frames[Framenum_BallPos].frame, BUF_LEN);
					uint8_t *ballbuff = pC->Com.frames[Framenum_BallPos].frame;
					int64_t lon = (int64_t)(pC->Drive.ball.lon * 10000000.0);
					int64_t lat = (int64_t)(pC->Drive.ball.lat * 10000000.0);
					ballbuff[0] = Frametype_Header;
					ballbuff[1] = FrameType_PosOfBall;
					ballbuff[2] = lon>>32;
					ballbuff[3] = lon>>24;
					ballbuff[4] = lon>>16;
					ballbuff[5] = lon>>8;
					ballbuff[6] = lon;
					ballbuff[7] = lat>>32;
					ballbuff[8] = lat>>24;
					ballbuff[9] = lat>>16;
					ballbuff[10] = lat>>8;
					ballbuff[11] = lat;
					uint16_t crc = crc16(ballbuff, 12);
					ballbuff[12] = (uint8_t)(crc>>8);
					ballbuff[13] = (uint8_t)crc;
					pC->Com.frames[Framenum_BallPos].full = On;
					pC->Com.frames[Framenum_BallPos].len = 14;
				}
				else
				{
					pC->Drive.ball.activated = Off;
				}
			}
		}
	}
	ClearStr(buf, BUF_LEN);
	DMA1_Stream0->CR 		&= ~DMA_SxCR_EN;
	DMA1->LIFCR 				|= DMA_LIFCR_CTCIF0;
	DMA1_Stream0->CR 		|= DMA_SxCR_EN;
}
static void Com_PrepareFrameTelemetry(void)
{
	uint8_t value1 = 0x00;
	//bit 0. sygnalizuje stan hardware safety button: On = 0x01
	if(pC->Error.emergencyhard == On)
		value1 |= 0x01;
	//bit 1. sygnalizuje stan software safety button: On = 0x02
	if(pC->Error.emergencysoft == On)
		value1 |= 0x02;
	//bity 2. i 3. sygnalizuje stan lazika: satel = 0x00, wifi = 0x04, autoStm = 0x06, autoNVidia = 0x0c
	if(pC->Mode.SelectHost == Satel)
		value1 |= 0x00;
	else if(pC->Mode.SelectHost == Wifi)
		value1 |= 0x04;
	else if(pC->Mode.SelectHost == AutoStm)
		value1 |= 0x08;
	else if(pC->Mode.SelectHost == AutoNvidia)
		value1 |= 0x0c;
	
	//bity 4., 5., 6. i 7. sygnalizuja bledy poszczególnych kól
	if(pC->Drive.error[0] == On)
		value1 |= 0x10;
	if(pC->Drive.error[1] == On)
		value1 |= 0x20;
	if(pC->Drive.error[2] == On)
		value1 |= 0x40;
	if(pC->Drive.error[3] == On)
		value1 |= 0x80;
	
	double speed = fabs(pC->Drive.motspeed);
	speed = speed * 60.0 * 0.94247779607 / 1000.0; //predkosc w km/h
	uint8_t value2 = (speed / 8.482300164 * 256.0);
	uint8_t value3 = 0x00;
	//bity 0. do 6. numer punktu do którego jedzie
	if(pC->Drive.nrp <= 63)
		value3 |= pC->Drive.nrp;
	//bit 6. potwierdza komunikacje z nvidia 0 - brak, 1 - jest
	if(pC->Error.nvcomerror == Off)
		value3 |= 0x40;
	//bit 7. potwierdza odebranie nowych punktów do autonomii
	if(pC->Drive.autonprec == On)
		value3 |= 0x80;
	
	uint8_t *buf = pC->Com.frames[Framenum_Telemetry].frame;
	buf[0] = Frametype_Header;
	buf[1] = (uint8_t)FrameType_DriveTelemetry;
	buf[2] = value1;
	buf[3] = value2;
	buf[4] = value3;
	uint16_t crc = crc16(buf, 5);
	buf[5] = (uint8_t)(crc>>8);
	buf[6] = (uint8_t)crc;
	pC->Com.frames[Framenum_Telemetry].full = On;
	pC->Com.frames[Framenum_Telemetry].len = 7;
}
static void Com_PrepareFrameGPS(void)
{
	int16_t yaw  = (int16_t)(pC->Dev.imuyaw * 10.0);
	int16_t pitch = (int16_t)(pC->Dev.imupitch * 10.0);
	int16_t roll = (int16_t)(pC->Dev.imuroll * 10.0);
	int64_t lon = (int64_t)(fabs(pC->Dev.gpslon) * 10000000.0);
	int64_t lat = (int64_t)(pC->Dev.gpslat * 10000000.0);
	
	uint8_t *buf = pC->Com.frames[Framenum_GpsPos].frame;
	buf[0] = Frametype_Header;
	buf[1] = (uint8_t)Frametype_DriveFeadbackGPS;
	buf[2] = yaw>>8;
	buf[3] = yaw;
	buf[4] = pitch>>8;
	buf[5] = pitch;
	buf[6] = roll>>8;
	buf[7] = roll;
	buf[8] = lon>>32;
	buf[9] = lon>>24;
	buf[10] = lon>>16;
	buf[11] = lon>>8;
	buf[12] = lon;
	buf[13] = lat>>32;
	buf[14] = lat>>24;
	buf[15] = lat>>16;
	buf[16] = lat>>8;
	buf[17] = lat;
	buf[18] = pC->Com.cmdout;
	uint16_t crc = crc16(buf, 19);
	buf[19] = (uint8_t)(crc>>8);
	buf[20] = (uint8_t)crc;
	pC->Com.frames[Framenum_GpsPos].full = On;
	pC->Com.frames[Framenum_GpsPos].len = 21;
}
static void Com_PrepareFrameBallPos(void)
{
	if(pC->Drive.ball.activated == On)
	{
		uint8_t *buff = pC->Com.frames[Framenum_BallPos].frame;
		ClearStr(buff, BUF_LEN);
		int64_t lon = (int64_t)(fabs(pC->Drive.ball.lon) * 10000000.0);
		int64_t lat = (int64_t)(fabs(pC->Drive.ball.lat) * 10000000.0);
		buff[0] = Frametype_Header;
		buff[1] = FrameType_PosOfBall;
		buff[2] = lon>>32;
		buff[3] = lon>>24;
		buff[4] = lon>>16;
		buff[5] = lon>>8;
		buff[6] = lon;
		buff[7] = lat>>32;
		buff[8] = lat>>24;
		buff[9] = lat>>16;
		buff[10] = lat>>8;
		buff[11] = lat;
		uint16_t crc = crc16(buff, 12);
		buff[12] = (uint8_t)(crc>>8);
		buff[13] = (uint8_t)crc;
		pC->Com.frames[Framenum_BallPos].full = On;
		pC->Com.frames[Framenum_BallPos].len = 14;
	}
}
static void Com_SendToHost(void)
{
	Com_PrepareFrameTelemetry();
	Com_PrepareFrameGPS();
	Com_PrepareFrameBallPos();
	for(uint8_t i=0;i<FRAMEMAX;i++)
	{
		if(pC->Com.framenum++ >= FRAMEMAX)
			pC->Com.framenum = 0;
		uint8_t num = pC->Com.framenum;
		if(pC->Com.frames[num].active == On && pC->Com.frames[num].full == On)
		{
			Com_SendToSatel(pC->Com.frames[num].frame, pC->Com.frames[num].len);
			Com_SendToWifi(pC->Com.frames[num].frame, pC->Com.frames[num].len);
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
		Com_SendToHost();
		TIM6->SR &= ~TIM_SR_UIF;
	}
}
void USART6_IRQHandler(void)
{
	if((USART6->SR & USART_SR_IDLE) != RESET)
	{
		char c = USART6->DR;
		Com_ReadFromSatel();
		Com_SendToHost();
		TIM6->CR1 &= ~TIM_CR1_CEN;
		TIM6->CNT = 0;
		TIM6->CR1 |= TIM_CR1_CEN;
	}
}
void USART3_IRQHandler(void)
{
	if((USART3->SR & USART_SR_IDLE) != RESET)
	{
		char c = USART3->DR;
		Com_ReadFromWifi();
	}
}
void UART4_IRQHandler(void)
{
	if((UART4->SR & USART_SR_IDLE) != RESET)
	{
		char c = UART4->DR;
		Com_ReadFromManip();
	}
}
void UART5_IRQHandler(void)
{
	if((UART5->SR & USART_SR_IDLE) != RESET)
	{
		char c = UART5->DR;
		Com_ReadFromNVidia();
	}
}
