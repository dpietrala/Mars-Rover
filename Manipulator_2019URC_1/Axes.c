#include "Axes.h"
extern sControl* pC;
void Axes_Defpos(void)
{
	LED4_TOG;
	pC->Status.coordinate = Join;
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Mot.imp[i] = 0;
		pC->Mot.implast[i] = 0;
		pC->Mot.tims[i]->CNT = 0;
		pC->Mot.impfullturn[i] = 0;
		pC->Mot.posabs[i] = pC->Mot.posdef[i];
		pC->Mot.posfullturn[i] = 0;
		pC->Mot.posstart[i] = pC->Mot.posdef[i];
		pC->Mot.posref[i] = pC->Mot.posdef[i];
		pC->Axes.kartangleabs[i] = pC->Mot.posdef[i];
		pC->Axes.kartanglerefabs[i] = pC->Mot.posdef[i];
	}
	pC->Status.calibrated = On;
	pC->Status.calibratedflagtime = 0;
}
static void Axes_JointSystem(void)
{
	//Obydwie walizki
	pC->Mot.speedref[0] = pC->Axes.speedref[0];
	pC->Mot.speedref[1] = pC->Axes.speedref[1];
	pC->Mot.speedref[2] = pC->Axes.speedref[2];
	pC->Mot.speedref[3] = pC->Axes.speedref[3];
	pC->Mot.speedref[4] = pC->Axes.speedref[4];
	pC->Mot.speedref[5] = pC->Axes.speedref[5];
	pC->Mot.speedref[6] = pC->Axes.speedref[6];
	pC->Mot.speedref[7] = pC->Axes.speedref[7];

	
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Mot.posref[i] += pC->Mot.speedref[i] * pC->Mot.step[i];
		if(pC->Mot.posref[i] > pC->Mot.posabsmax[i])
			pC->Mot.posref[i] = pC->Mot.posabsmax[i];
		else if(pC->Mot.posref[i] < pC->Mot.posabsmin[i])
			pC->Mot.posref[i] = pC->Mot.posabsmin[i];
	}
}
static void Axes_BaseSystem(void)
{
	double speedref[8];
	//Obydwie walizka
	speedref[0] = pC->Axes.speedref[2];
	speedref[1] = pC->Axes.speedref[0];
	speedref[2] = pC->Axes.speedref[1];
	speedref[3] = -pC->Axes.speedref[3];
	speedref[4] = -pC->Axes.speedref[4];
	speedref[5] = pC->Axes.speedref[5];
	speedref[6] = pC->Axes.speedref[6];
	speedref[7] = pC->Axes.speedref[7];
	
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Mot.speedref[i] = pC->Axes.speedref[i];
		pC->Axes.kartposref[i] = pC->Axes.kartpos[i] + speedref[i] * pC->Axes.kartposstep[i];
	}
  Kin_Reference(pC->Axes.matref, pC->Axes.kartposref);
}
static void Axes_ToolSystem(void)
{
	double speedref[8];
	//Obydwie walizka
	speedref[0] = pC->Axes.speedref[1];
	speedref[1] = -pC->Axes.speedref[0];
	speedref[2] = pC->Axes.speedref[2];
	speedref[3] = -pC->Axes.speedref[3];
	speedref[4] = -pC->Axes.speedref[4];
	speedref[5] = pC->Axes.speedref[5];
	speedref[6] = pC->Axes.speedref[6];
	speedref[7] = pC->Axes.speedref[7];
	
  double M1[4][4], M2[4][4], delta[AXESMAX]={0,0,0,0,0,0,0,0};
  for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Mot.speedref[i] = pC->Axes.speedref[i];
		delta[i] = speedref[i] * pC->Axes.kartposstep[i];
	}
	Kin_Forward(M1, pC->Mot.posref);
  Kin_Funrpy(M2, delta);
  Matxmat(M1, M2, pC->Axes.matref);
}
static void Axes_AutoSystem(void)
{
	for(uint8_t i=0;i<6;i++)
	{
		if(fabs(pC->Axes.autoposrefabstemp[i] - pC->Axes.autoposrefabs[i]) < pC->Axes.autohyst[i])
		{
			pC->Axes.autoinpos[i] = On;
			continue;
		}
		
		pC->Axes.autoposrefabstemp[i] += pC->Axes.autospeed[i] * pC->Mot.step[i];
		pC->Mot.posref[i] = pC->Axes.autoposrefabstemp[i];
		if(pC->Mot.posref[i] > pC->Mot.posabsmax[i])
			pC->Mot.posref[i] = pC->Mot.posabsmax[i];
		else if(pC->Mot.posref[i] < pC->Mot.posabsmin[i])
			pC->Mot.posref[i] = pC->Mot.posabsmin[i];
	}
	
	pC->Axes.autoglobalinpos = On;
	for(uint8_t i=0;i<6;i++)
		if(pC->Axes.autoinpos[i] == Off)
			pC->Axes.autoglobalinpos = Off;
		
	if(pC->Axes.autoposref[7] > 0 && pC->Axes.autotimetick < pC->Axes.autoposref[7])
		pC->Mot.speedref[6] = pC->Axes.autoposref[6];
	else
		pC->Mot.speedref[6] = pC->Axes.speedref[6];
	
	if(pC->Axes.autoglobalinpos == On && pC->Axes.autotimetick > pC->Axes.autoposref[7])
	{
		if((pC->Axes.autopointnum + 1) < AUTOPOINTMAX)
		{
			Axes_AutoSystemOn(pC->Axes.autoseqnum, pC->Axes.autopointnum + 1);
		}
		else
			Axes_CoordinateAct(Hostcmd_join);
	}
}
static uint8_t Axes_KartCheckResults(void)
{
	for(uint8_t i=0;i<6;i++)
		if(pC->Axes.kartangleref[i] == NAN)
			return 1;

	double x = 0, dist = 0;
	for(uint8_t i=0;i<6;i++)
	{
		dist = pC->Axes.kartangleref[i] - pC->Mot.pos[i];
		if(fabs(dist) < M_PI)
			x = dist;
		else if(dist > M_PI)
			x = dist - M_2_PI;
		else if(dist < -M_PI)
			x = dist + M_2_PI;
		pC->Axes.kartanglerefabs[i] = pC->Mot.posabs[i] + x;
	}
	
//	for(uint8_t i=0;i<6;i++)
//		if(fabs((pC->Axes.kartanglerefabs[i] - pC->Mot.posabs[i]) / BASETIME) > pC->Mot.speedmax[i])
//			return 2;
		
	for(uint8_t i=0;i<6;i++)
		if((pC->Axes.kartanglerefabs[i] > pC->Mot.posabsmax[i]) || (pC->Axes.kartanglerefabs[i] < pC->Mot.posabsmin[i]))
			return 3;

	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Axes.kartangle[i] = pC->Axes.kartangleref[i];
		pC->Mot.posref[i] = pC->Axes.kartanglerefabs[i];
		pC->Axes.kartpos[i] = pC->Axes.kartposref[i];
	}
	return 0;
}
static void Axes_Stop(void)
{
	pC->Status.coordinate = Join;
	for(uint8_t i=0;i<AXESMAX;i++)
	{
		pC->Axes.speedref[i] = 0.0;
//		pC->Axes.kartanglerefabs[i] = pC->Axes.kartangleabs[i];
		pC->Mot.speedref[i] = 0.0;
//		pC->Mot.posref[i] = pC->Mot.posabs[i];
//		pC->Mot.out[i] = 0;
//		pC->Mot.pwm[i] = 0;
//		pC->Mot.pwmout[i] = 0;
	}
}
void Axes_CoordinateAct(eHostCmd cmd)
{
	if(cmd == Hostcmd_join)
	{
		for(uint8_t i=0;i<AXESMAX;i++)
		{
			pC->Axes.kartposref[i] = pC->Axes.kartpos[i];
			pC->Mot.posref[i] = pC->Mot.posabs[i];
			pC->Axes.kartangleabs[i] = pC->Mot.posabs[i];
			pC->Axes.kartanglerefabs[i] = pC->Mot.posabs[i];
		}
		pC->Status.coordinate = Join;
	}
	else if(cmd == Hostcmd_base)
	{
		if(Kin_FindRpy(pC->Axes.matact, pC->Mot.pos, pC->Axes.kartpos) != 0)
			return;
		for(uint8_t i=0;i<AXESMAX;i++)
		{
			pC->Axes.kartposref[i] = pC->Axes.kartpos[i];
			pC->Mot.posref[i] = pC->Mot.posabs[i];
			pC->Axes.kartangleabs[i] = pC->Mot.posabs[i];
			pC->Axes.kartanglerefabs[i] = pC->Mot.posabs[i];
		}
		pC->Status.coordinate = Base;
	}
	else if(cmd == Hostcmd_tool)
	{
		if(Kin_FindRpy(pC->Axes.matact, pC->Mot.pos, pC->Axes.kartpos) != 0)
			return;
		for(uint8_t i=0;i<AXESMAX;i++)
		{
			pC->Axes.kartposref[i] = pC->Axes.kartpos[i];
			pC->Mot.posref[i] = pC->Mot.posabs[i];
			pC->Axes.kartangleabs[i] = pC->Mot.posabs[i];
			pC->Axes.kartanglerefabs[i] = pC->Mot.posabs[i];
		}
		pC->Status.coordinate = Tool;
	}
}
void Axes_AutoSystemOn(uint8_t seq, uint8_t point)
{
	if(seq >= AUTOSEQMAX)
		return;
	
	uint8_t num = point;
	for(num=point; num<AUTOPOINTMAX; num++)
	{
		if(pC->Axes.autoseq[seq].points[num].active == On)
		{
			pC->Axes.autoseqnum = seq;
			pC->Axes.autopointnum = num;
			pC->Axes.autoglobalspeed = pC->Axes.autoseq[seq].points[num].speed;
			pC->Axes.autotimetick = 0;
			
			for(uint8_t i=0;i<6;i++)
			{
				pC->Axes.kartposref[i] = pC->Axes.kartpos[i];
				pC->Mot.posref[i] = pC->Mot.posabs[i];
				pC->Axes.kartangleabs[i] = pC->Mot.posabs[i];
				pC->Axes.kartanglerefabs[i] = pC->Mot.posabs[i];
			}
			for(uint8_t i=0;i<AXESMAX;i++)
			{
				pC->Axes.autoposref[i] = pC->Axes.autoseq[pC->Axes.autoseqnum].points[pC->Axes.autopointnum].pos[i];
			}
			
			double x = 0;
			for(uint8_t i=0;i<6;i++)
			{
				pC->Axes.autodist[i] = pC->Axes.autoposref[i] - pC->Mot.pos[i];
				if(fabs(pC->Axes.autodist[i]) < M_PI)
					x = pC->Axes.autodist[i];
				else if(pC->Axes.autodist[i] > M_PI)
					x = pC->Axes.autodist[i] - M_2_PI;
				else if(pC->Axes.autodist[i] < -M_PI)
					x = pC->Axes.autodist[i] + M_2_PI;
				pC->Axes.autoposrefabs[i] = pC->Mot.posabs[i] + x;
				pC->Axes.autoposrefabstemp[i] = pC->Mot.posabs[i];
				
				if(pC->Axes.autoposrefabs[i] > pC->Mot.posabsmax[i] || pC->Axes.autoposrefabs[i] < pC->Mot.posabsmin[i])
					return;
			}
			for(uint8_t i=0;i<6;i++)
				pC->Axes.automovetimemax[i] = fabs(pC->Axes.autodist[i] / (pC->Axes.autoglobalspeed * pC->Mot.speedmax[i] / 100.0));

			double maxtime = pC->Axes.automovetimemax[0];
			for(uint8_t i=0;i<6;i++)
				if(fabs(pC->Axes.automovetimemax[i]) > maxtime)
					maxtime = fabs(pC->Axes.automovetimemax[i]);
			
			for(uint8_t i=0;i<6;i++)
			{
				pC->Axes.autospeed[i] = 100.0 * fabs(pC->Axes.autodist[i] / maxtime) / pC->Mot.speedmax[i];
				if(pC->Axes.autospeed[i] > 100)
					return;
				
				if((pC->Axes.autoposrefabs[i] - pC->Mot.posabs[i]) >= 0.0)
					pC->Axes.autospeed[i] = fabs(pC->Axes.autospeed[i]);
				else if((pC->Axes.autoposrefabs[i] - pC->Mot.posabs[i]) < 0.0)
					pC->Axes.autospeed[i] = -1.0*fabs(pC->Axes.autospeed[i]);
				
				pC->Axes.autoinpos[i] = Off;
			}
			pC->Status.coordinate = Ptp;
			
			break;
		}
		else
		{
			Axes_CoordinateAct(Hostcmd_join);
		}
	}
}
void Axes_AutoClearSeq(uint8_t seq)
{
	for(uint8_t i=0;i<AUTOPOINTMAX;i++)
	{
		for(uint8_t j=0;j<6;j++)
			pC->Axes.autoseq[seq].points[i].pos[j] = pC->Mot.posdef[j];
		pC->Axes.autoseq[seq].points[i].pos[6] = 0;
		pC->Axes.autoseq[seq].points[i].pos[7] = 0;
		pC->Axes.autoseq[seq].points[i].speed = 50;
		pC->Axes.autoseq[seq].points[i].type = AutoPtp;
		pC->Axes.autoseq[seq].points[i].active = Off;
	}
}
void Axes_AutoSavePoint(uint8_t seq, uint8_t point, uint8_t speed, eAutoType type, int8_t gripperspeed, uint8_t grippertime)
{
	for(uint8_t i=0;i<6;i++)
		pC->Axes.autoseq[seq].points[point].pos[i] = pC->Mot.pos[i];
	pC->Axes.autoseq[seq].points[point].pos[6] = gripperspeed;
	pC->Axes.autoseq[seq].points[point].pos[7] = 1000.0 * (double)grippertime;
	pC->Axes.autoseq[seq].points[point].speed = speed;
	pC->Axes.autoseq[seq].points[point].type = type;
	pC->Axes.autoseq[seq].points[point].active = On;
}
void Axes_AutoSavePointToBackup(void)
{
	volatile int32_t * const p2 = (int32_t *)0x40024400;
	uint32_t offset = 0;
	for(uint32_t i=0;i<AUTOSEQMAX;i++)
	{
		for(uint32_t j=0;j<AUTOPOINTMAX;j++)
		{
			for(uint32_t k=0;k<AXESMAX;k++)
			{
				*(p2 + offset) = (int32_t)(pC->Axes.autoseq[i].points[j].pos[k] * 10000);
				offset++;
			}
			*(p2 + offset) = (int32_t)(pC->Axes.autoseq[i].points[j].active);
			offset++;
			*(p2 + offset) = (int32_t)(pC->Axes.autoseq[i].points[j].speed);
			offset++;
			*(p2 + offset) = (int32_t)(pC->Axes.autoseq[i].points[j].type);
			offset++;
		}
	}
}
void Axes_AutoReadPointFromBackup(void)
{
	volatile int32_t * const p2 = (int32_t *)0x40024400;
	uint32_t offset = 0;
	for(uint32_t i=0;i<AUTOSEQMAX;i++)
	{
		for(uint32_t j=0;j<AUTOPOINTMAX;j++)
		{
			for(uint32_t k=0;k<AXESMAX;k++)
			{
				pC->Axes.autoseq[i].points[j].pos[k] = (double)*(p2 + offset) / 10000.0;
				offset++;
			}
			pC->Axes.autoseq[i].points[j].active = (eOnOff)(*(p2 + offset));
			offset++;
			pC->Axes.autoseq[i].points[j].speed = *(p2 + offset);
			offset++;
			pC->Axes.autoseq[i].points[j].type = (eAutoType)(*(p2 + offset));
			offset++;
		}
	}
}
void Axes_ManipEnable(void)
{
//	for(uint8_t i=0;i<AXESMAX;i++)
//	{
//		pC->Mot.pid[i] = On;
//	}
//	Axes_CoordinateAct(Hostcmd_join);
}
void Axes_ManipDisable(void)
{
//	for(uint8_t i=0;i<AXESMAX;i++)
//	{
//		pC->Mot.pid[i] = Off;
//	}
//	pC->Status.work = Off;
}
void Axes_Act(void)
{
	if(pC->Status.work == On)
	{
		Kin_Forward(pC->Axes.matact, pC->Mot.pos);
		LED1_OFF;
		if(pC->Status.coordinate == Join)
		{
			Axes_JointSystem();
		}
		else if(pC->Status.coordinate == Base)
		{
			Axes_BaseSystem();
			Kin_Calc(pC->Axes.matref, pC->Mot.pos, pC->Axes.kartangleref);
			Axes_KartCheckResults();
		}
		else if(pC->Status.coordinate == Tool)
		{
			Axes_ToolSystem();
			Kin_Calc(pC->Axes.matref, pC->Mot.pos, pC->Axes.kartangleref);
			Axes_KartCheckResults();
		}
		else if(pC->Status.coordinate == Ptp)
		{
			LED4_TOG;
			Axes_AutoSystem();
		}
	}
	else
	{
		LED1_ON;
		Axes_Stop();
	}
}
