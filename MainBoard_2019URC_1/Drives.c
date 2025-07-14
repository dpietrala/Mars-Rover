#include "Drives.h"
extern sControl* pC;
void Drive_Conf(void)
{
	DMA1_Stream5->PAR 	= (uint32_t)&USART2->DR;
	DMA1_Stream5->M0AR 	= (uint32_t)pC->Drive.bufread;
	DMA1_Stream5->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream5->CR 	|= DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_CIRC |  DMA_SxCR_CHSEL_2 | DMA_SxCR_EN;
	
	DMA1_Stream6->PAR 	= (uint32_t)&USART2->DR;
	DMA1_Stream6->M0AR 	= (uint32_t)pC->Drive.bufwrite;
	DMA1_Stream6->NDTR 	= (uint16_t)BUF_LEN;
	DMA1_Stream6->CR 		|= DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0;
	
	GPIOD->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0;
	GPIOD->AFR[0] |= 0x07700000;
	USART2->BRR = 42000000/115200;
	USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
	USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;
	NVIC_EnableIRQ(USART2_IRQn);
}
static void Drive_SendToMotorsGeneralFrame(void)
{
	uint8_t* buf = pC->Drive.bufwrite;
	ClearStr(buf, BUF_LEN);
	uint8_t index = 0;
	buf[index++] = (uint8_t)Frametype_Header;
	buf[index++] = (uint8_t)Frametype_MainboardToDriveGenerallSpeed;
	for(uint8_t i=0;i<MOTMAX;i++)
	{
		buf[index++] = (uint8_t)pC->Drive.pid[i];
		int16_t speed = pC->Drive.speedref[i];
		buf[index++] = speed>>8;
		buf[index++] = speed;
	}
	uint16_t crc = crc16(buf, index);
	buf[index++] = (uint8_t)(crc>>8);
	buf[index++] = (uint8_t)crc;
	
	DMA1_Stream6->CR 		&= ~DMA_SxCR_EN;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF6;
	DMA1_Stream6->NDTR 	= (uint16_t)index;
	DMA1_Stream6->CR 		|= DMA_SxCR_EN;
}
static void Drive_SendToMotorsemergencyhardFrame(void)
{
	uint8_t* buf = pC->Drive.bufwrite;
	ClearStr(buf, BUF_LEN);
	uint8_t index = 0;
	buf[index++] = (uint8_t)Frametype_Header;
	buf[index++] = (uint8_t)Frametype_emergencyhardStop;
	buf[index++] = Hostcmd_null;
	uint16_t crc = crc16(buf, index);
	buf[index++] = (uint8_t)(crc>>8);
	buf[index++] = (uint8_t)crc;
	
	DMA1_Stream6->CR 		&= ~DMA_SxCR_EN;
	DMA1->HIFCR 				|= DMA_HIFCR_CTCIF6;
	DMA1_Stream6->NDTR 	= (uint16_t)index;
	DMA1_Stream6->CR 		|= DMA_SxCR_EN;
}
static void Drive_SendToMotors(void)
{
	if(pC->Error.emergencyhard == On)
		Drive_SendToMotorsemergencyhardFrame();
	else
		Drive_SendToMotorsGeneralFrame();
}
static void Drive_ReadFromMotors(void)
{
	uint8_t* buf = pC->Drive.bufread;
	uint16_t crc1 = crc16(buf, 17);
	uint16_t crc2 = (uint16_t)(buf[17]<<8) + (uint16_t)buf[18];
	if(crc1 == crc2)
	{
		pC->Drive.motspeed = (double)((int32_t)(buf[0]<<24)+(int32_t)(buf[1]<<16)+(int32_t)(buf[2]<<8)+(int32_t)buf[3]) / 1000.0;
		pC->Drive.motyaw = (double)((int32_t)(buf[4]<<24)+(int32_t)(buf[5]<<16)+(int32_t)(buf[6]<<8)+(int32_t)buf[7]) / 1000.0;
		pC->Drive.motposx = (double)((int32_t)(buf[8]<<24)+(int32_t)(buf[9]<<16)+(int32_t)(buf[10]<<8)+(int32_t)buf[11]) / 1000.0;
		pC->Drive.motposy = (double)((int32_t)(buf[12]<<24)+(int32_t)(buf[13]<<16)+(int32_t)(buf[14]<<8)+(int32_t)buf[15]) / 1000.0;
		if((buf[16] & 0x01) == 0x01)		pC->Drive.error[0] = On;
		else														pC->Drive.error[0] = Off;
		if((buf[16] & 0x02) == 0x02)		pC->Drive.error[1] = On;
		else														pC->Drive.error[1] = Off;
		if((buf[16] & 0x04) == 0x04)		pC->Drive.error[2] = On;
		else														pC->Drive.error[2] = Off;
		if((buf[16] & 0x08) == 0x08)		pC->Drive.error[3] = On;
		else														pC->Drive.error[3] = Off;
		pC->Error.drivecomtick = 0;
	}
	ClearStr(buf, BUF_LEN);
	DMA1_Stream5->CR 		&= ~DMA_SxCR_EN;
	DMA1->HIFCR					|= DMA_HIFCR_CTCIF5;
	DMA1_Stream5->CR 		|= DMA_SxCR_EN;
}
static void Drive_Speedref(void)
{
	double right=0, left=0, death=3;
	double templ = 0, tempr = 0, mull = 1.2;
	left = pC->Drive.frontvalue + pC->Drive.dirvalue;
	right = pC->Drive.frontvalue - pC->Drive.dirvalue;

	if(left > death)
		templ = (double)(left - death) * mull;
	else if(left < -death)
		templ = (double)(left + death) * mull;
	else
		templ = 0;
	
	if(right > death)
		tempr = (double)(right - death) * mull;
	else if(right < -death)
		tempr = (double)(right + death) * mull;
	else
		tempr = 0;
	
	pC->Drive.speedref[0] = templ;
	pC->Drive.speedref[1] = tempr;
	
	pC->Drive.speedref[2] = templ;
	pC->Drive.speedref[3] = tempr;
}
static void Drive_Stop(void)
{
	pC->Drive.frontvalue = 0.0;
	pC->Drive.dirvalue = 0.0;
	for(uint8_t i=0;i<MOTMAX;i++)
		pC->Drive.speedref[i] = 0.0;
}
void Drive_FindGeoFromDist(double lo, double la, double az, double d, double* lonB, double* latB)
{
	double lonA = lo * M_PI / 180.0;
	double latA = la * M_PI / 180.0;
	double azimuth = az * M_PI / 180.0;
	double dx = sin(azimuth) * d;
	double dy = cos(azimuth) * d;
	*latB = (latA + dy / 6371008.8) * 180.0 / M_PI;
	double rR = 6371008.8 * cos(latA);
	*lonB = (lonA + dx / rR) * 180 / M_PI;
}
static double Drive_FindDistFromGeo(double lonact, double latact, double lonref, double latref)
{
	double dist = 6371008.8 * acos(sin(latact)*sin(latref) + cos(latact)*cos(latref)*cos(lonref-lonact));
	return dist;
}
static double Drive_FindYawFromGeo(double lonact, double latact, double lonref, double latref)
{
	double yawref = atan2(sin(lonref-lonact), cos(latact)*tan(latref) - sin(latact)*cos(lonref-lonact)) * 180.0 / M_PI;
	return yawref;
}
void Drive_AutoNextPoint(void)
{
	if(pC->Drive.refpoint.type == Temppoint)
	{
		pC->Drive.nrp++;
		pC->Drive.refpoint = pC->Drive.points[pC->Drive.nrp];
	}
	else if(pC->Drive.refpoint.type == Checkpoint)
	{
		pC->Drive.refpoint.status = Foundball; //koniec jazdy auto bez szukania z nvidii
	}
	else if(pC->Drive.refpoint.type == Lookpoint)
	{
		pC->Drive.nrp++;
		if(pC->Drive.nrp >= pC->Drive.numpoints)
		{
			pC->Drive.nrp -= HELIPOINTMAX;
			pC->Drive.refpoint = pC->Drive.points[pC->Drive.nrp];
		}
	}
}
static void Drive_AutoWait(void)
{
	SIG0_OFF;
	SIG1_OFF;
	Drive_Stop();
	pC->Com.cmdout = Hostcmd_out_wait;
}
static void Drive_AutoDriveToPoint(double frontmax, double dirmax)
{
	SIG0_OFF;
	SIG1_ON;
	pC->Com.cmdout = Hostcmd_out_drive;
	
	double yawkp = 2.2, yawhyst = 20.0, distkp = 10.0, disthyst = 1.0;
	double lonact = pC->Dev.gpslon * M_PI / 180.0;
	double latact = pC->Dev.gpslat * M_PI / 180.0;
	double lonref = pC->Drive.refpoint.lon * M_PI / 180.0;
	double latref = pC->Drive.refpoint.lat * M_PI / 180.0;
	double yawact = pC->Dev.imuyaw;
	pC->Drive.refpoint.disttopoint = Drive_FindDistFromGeo(lonact, latact, lonref, latref);
	
	if((lonref-lonact) == 0.0 && (latref-latact) == 0.0)
		pC->Drive.refpoint.yawref = 0.0;
	else
		pC->Drive.refpoint.yawref = Drive_FindYawFromGeo(lonact, latact, lonref, latref);
	
	if(fabs(pC->Drive.refpoint.yawref - yawact) < 180.0)
		pC->Drive.refpoint.yawer = pC->Drive.refpoint.yawref - yawact;
	else if((pC->Drive.refpoint.yawref - yawact) > 180.0)
		pC->Drive.refpoint.yawer = fabs(pC->Drive.refpoint.yawref - yawact) - 360.0;
	else if((pC->Drive.refpoint.yawref - yawact) < -180.0)
		pC->Drive.refpoint.yawer = 360.0 - fabs(pC->Drive.refpoint.yawref - yawact);
	
	if(pC->Drive.refpoint.disttopoint > disthyst)
		pC->Drive.frontvalue = pC->Drive.refpoint.disttopoint * distkp;
	else
		Drive_AutoNextPoint();

	if(pC->Drive.frontvalue > frontmax)
		pC->Drive.frontvalue = frontmax;
	else if(pC->Drive.frontvalue < -frontmax)
		pC->Drive.frontvalue = -frontmax;
	
	pC->Drive.dirvalue = pC->Drive.refpoint.yawer * yawkp;
	if(pC->Drive.dirvalue > dirmax)
		pC->Drive.dirvalue = dirmax;
	else if(pC->Drive.dirvalue < -dirmax)
		pC->Drive.dirvalue = -dirmax;
	
	if(fabs(pC->Drive.refpoint.yawer) > 2*yawhyst)
		pC->Drive.frontvalue = 0.0;
}
static void Drive_AutoLookBall(double frontmax, double dirmax)
{
	SIG0_ON;
	SIG1_OFF;
	pC->Com.cmdout = Hostcmd_out_lookball;
	
	double yawkp = 2.2, yawhyst = 20.0, distkp = 10.0, disthyst = 0.8;
	double lonact = pC->Dev.gpslon * M_PI / 180.0;
	double latact = pC->Dev.gpslat * M_PI / 180.0;
	double lonref = pC->Drive.refpoint.lon * M_PI / 180.0;
	double latref = pC->Drive.refpoint.lat * M_PI / 180.0;
	double yawact = pC->Dev.imuyaw;
	pC->Drive.refpoint.disttopoint = Drive_FindDistFromGeo(lonact,latact,lonref, latref);
	
	if((lonref-lonact) == 0.0 && (latref-latact) == 0.0)
		pC->Drive.refpoint.yawref = 0.0;
	else
		pC->Drive.refpoint.yawref = Drive_FindYawFromGeo(lonact,latact,lonref, latref);
	
	if(fabs(pC->Drive.refpoint.yawref - yawact) < 180.0)
		pC->Drive.refpoint.yawer = pC->Drive.refpoint.yawref - yawact;
	else if((pC->Drive.refpoint.yawref - yawact) > 180.0)
		pC->Drive.refpoint.yawer = fabs(pC->Drive.refpoint.yawref - yawact) - 360.0;
	else if((pC->Drive.refpoint.yawref - yawact) < -180.0)
		pC->Drive.refpoint.yawer = 360.0 - fabs(pC->Drive.refpoint.yawref - yawact);
	
//	if(pC->Drive.refpoint.type == Temppoint)
//	{
		if(pC->Drive.refpoint.disttopoint > disthyst)
			pC->Drive.frontvalue = pC->Drive.refpoint.disttopoint * distkp;
		else
			Drive_AutoNextPoint();
//	}

	if(pC->Drive.frontvalue > frontmax)
		pC->Drive.frontvalue = frontmax;
	else if(pC->Drive.frontvalue < -frontmax)
		pC->Drive.frontvalue = -frontmax;
	
	pC->Drive.dirvalue = pC->Drive.refpoint.yawer * yawkp;
	if(pC->Drive.dirvalue > dirmax)
		pC->Drive.dirvalue = dirmax;
	else if(pC->Drive.dirvalue < -dirmax)
		pC->Drive.dirvalue = -dirmax;
	
	if(fabs(pC->Drive.refpoint.yawer) > 2*yawhyst)
		pC->Drive.frontvalue = 0.0;
	
	if(pC->Drive.ball.activated == On)
	{
		pC->Drive.refpoint = pC->Drive.ball;
		pC->Drive.refpoint.status = Drivetoball;
	}
	else
	{
		pC->Drive.refpoint = pC->Drive.points[pC->Drive.nrp];
	}
}
static void Drive_AutoDriveToBall(double frontmax, double dirmax)
{
	SIG0_ON;
	SIG1_OFF;
	pC->Com.cmdout = Hostcmd_out_drivetoball;
	
	if(pC->Drive.ball.activated == On)
	{
		pC->Drive.refpoint = pC->Drive.ball;
		pC->Drive.refpoint.status = Drivetoball;
	}
	else
	{
		pC->Drive.refpoint.status = Lookball;
	}
	
	double yawkp = 1.0, yawhyst = 20.0, distkp = 10.0, disthyst = 1;
	double lonact = pC->Dev.gpslon * M_PI / 180.0;
	double latact = pC->Dev.gpslat * M_PI / 180.0;
	double lonref = pC->Drive.refpoint.lon * M_PI / 180.0;
	double latref = pC->Drive.refpoint.lat * M_PI / 180.0;
	double yawact = pC->Dev.imuyaw;
	pC->Drive.refpoint.disttopoint = Drive_FindDistFromGeo(lonact,latact,lonref, latref);
	
	if((lonref-lonact) == 0.0 && (latref-latact) == 0.0)
		pC->Drive.refpoint.yawref = 0.0;
	else
		pC->Drive.refpoint.yawref = Drive_FindYawFromGeo(lonact,latact,lonref, latref);
	
	if(fabs(pC->Drive.refpoint.yawref - yawact) < 180.0)
		pC->Drive.refpoint.yawer = pC->Drive.refpoint.yawref - yawact;
	else if((pC->Drive.refpoint.yawref - yawact) > 180.0)
		pC->Drive.refpoint.yawer = fabs(pC->Drive.refpoint.yawref - yawact) - 360.0;
	else if((pC->Drive.refpoint.yawref - yawact) < -180.0)
		pC->Drive.refpoint.yawer = 360.0 - fabs(pC->Drive.refpoint.yawref - yawact);
	

		if(pC->Drive.refpoint.disttopoint > disthyst)
			pC->Drive.frontvalue = pC->Drive.refpoint.disttopoint * distkp;
		else
		{
			Drive_Stop();
			pC->Drive.refpoint.status = Foundball;
		}

	if(pC->Drive.frontvalue > frontmax)
		pC->Drive.frontvalue = frontmax;
	else if(pC->Drive.frontvalue < -frontmax)
		pC->Drive.frontvalue = -frontmax;
	
	pC->Drive.dirvalue = pC->Drive.refpoint.yawer * yawkp;
	if(pC->Drive.dirvalue > dirmax)
		pC->Drive.dirvalue = dirmax;
	else if(pC->Drive.dirvalue < -dirmax)
		pC->Drive.dirvalue = -dirmax;
	
	if(fabs(pC->Drive.refpoint.yawer) > 2*yawhyst)
		pC->Drive.frontvalue = 0.0;
}
static void Drive_AutoFoundBall(void)
{
	SIG0_ON;
	SIG1_ON;
	Drive_Stop();
	pC->Com.cmdout = Hostcmd_out_foundball;
}
void Drive_Act(void)
{
	if(pC->Mode.SelectHost == Satel || pC->Mode.SelectHost == Wifi)
	{
		SIG0_OFF;
		SIG1_OFF;
		pC->Com.cmdout = Hostcmd_out_manual;
		if(pC->Error.hostcomerror == On)
			Drive_Stop();
		Drive_Speedref();
	}
	else if(pC->Mode.SelectHost == AutoStm || pC->Mode.SelectHost == AutoNvidia)
	{
		if(pC->Error.nvcomerror == On)
		{
			if(pC->Drive.nrp >= pC->Drive.numrecpoints)
			{
				pC->Drive.nrp = pC->Drive.numrecpoints-1;
				pC->Drive.refpoint = pC->Drive.points[pC->Drive.numrecpoints-1];
				pC->Drive.refpoint.type = Checkpoint;
			}
			else
			{
				pC->Drive.points[pC->Drive.numrecpoints-1].type = Checkpoint;
			}
		}
		
		if(pC->Drive.refpoint.status == Waitpoint)
			Drive_AutoWait();
		else if(pC->Drive.refpoint.status == Drivetopoint)
		{
			if(fabs(pC->Dev.imupitch) > 25.0 || fabs(pC->Dev.imuroll) > 25.0)
				Drive_AutoDriveToPoint(15.0, 10.0);
			else
				Drive_AutoDriveToPoint(40.0, 30.0);
		}
		else if(pC->Drive.refpoint.status == Lookball)
			Drive_AutoLookBall(18, 15);
		else if(pC->Drive.refpoint.status == Drivetoball)
			Drive_AutoDriveToBall(12.0, 8.0);
		else if(pC->Drive.refpoint.status == Foundball)
			Drive_AutoFoundBall();
		
		Drive_Speedref();
	}
	if(pC->Error.emergency == On)
	{
		pC->Mode.SelectHost = Satel;
		Drive_Stop();
		LEDALL_ON;
	}
	Drive_SendToMotors();
}
//---- Przerwania --------------------------------
void USART2_IRQHandler(void)
{
	if((USART2->SR & USART_SR_IDLE) != RESET)
	{
		char c = USART2->DR;
		Drive_ReadFromMotors();
	}
}
