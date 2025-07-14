#ifndef	_CONTROL
#define _CONTROL

typedef enum {Off =	0, On =	1} eOnOff;
typedef enum {LedOff =	0, LedOn =	1, LedTog = 2} eLedState;
typedef enum {Workoff = 0, Workmanual, Workremote, Workauto} eMode;
typedef enum {Stop =	0, Backward = 1, Forward = 2} eDir;
typedef enum
{
    cmdLab_null = 0,
    cmdLab_Remote = 1,
    cmdLab_MotSetHomePos = 2,
    cmdLab_MotEnableLimits = 3,
    cmdLab_MotDisableLimits = 4,
    cmdLab_MotEnablePID = 5,
    cmdLab_MotDisablePID = 6,
    cmdLab_GripperOpen = 7,
    cmdLab_GripperClose = 8,
    cmdLab_LumiOpen = 9,
    cmdLab_LumiClose = 10,
    cmdLab_LumiStartMeas = 11,
    cmdLab_LumiNeutralPos = 12,
    cmdLab_LumiTurnOnOff = 13,
		cmdLab_LumiHorizontal = 14,
		cmdLab_LumiVertical = 15,
		cmdLab_MoveZPosUp = 16,
		cmdLab_MoveZPosMeas = 17,
		cmdLab_MoveZPosSwab = 18,
		cmdLab_MoveZPosLumi = 19,
		cmdLab_MoveXYPosSwab = 20,
		cmdLab_MoveXYPosLumi = 21,
		cmdLab_MoveXYPosMeas = 22,
		cmdLab_AutoRunSeq = 23,
		cmdLab_AutoNextPoint = 24,
		cmdLab_AutoPrevPoint = 25,
		cmdLab_MoveZPosSwab2 = 26,
		cmdLab_WibOn = 27,
		cmdLab_WibOff = 28,
		cmdLab_MoveXYWib = 29,
		cmdLab_MoveZWib = 30,
		cmdLab_MoveFotopos = 31,
		cmdLab_MoveXYTrow = 32,
}eCmd;
typedef enum
{
	Frametype_null = 0,
	Frametype_DriveGeneral = 1,
	Frametype_ManipGeneral = 2,
	Frametype_LabGeneral = 3,
	Frametype_ManipCmd = 4,
	Frametype_ManipSave = 5,
	Frametype_NvidiaGenaral = 4,
	Frametype_Newpoints = 7,
	Frametype_LabFeadbackMotPos = 8,
  Frametype_DriveFeadbackGPS = 9,
	Frametype_DriveCmd = 10,
	Frametype_MainboardToDriveGenerallSpeed = 11,
	FrameType_ManipFeedbackMotpos = 12,
	FrameType_PosOfBall = 13,
	FrameType_DriveTelemetry = 14,
	FrameType_ManipTelemetry = 15,
	FrameType_LabTelemetry = 16,
	FrameType_LabCmd = 17,
	Frametype_Header = 155,
	Frametype_EmergencyStop = 201
}eFrameType;
typedef enum
{
	Framenum_Telemetry = 0,
	Framenum_MotPos,
}eFrameNum;
typedef enum {LSNeutral = 0, LSOn = 1, LSMeas = 2} eLS;

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "Drive.h"
#include "Comunication.h"

#define PIDTIME						0.01
#define BUTTONMAX					8
#define POINTSMAX					20
#define SEQMAX						21
#define LIMITSWITCHMAX		8
#define MOTMAX						4
#define SWABMAX						10
#define COMTICKMAX				600
#define BUF_LEN						100
#define TIMMAX						0xffff
#define BACKUPADDRESS			0x40024008
#define FRAMEMAX					2
#define LUMISERWTICKMAX		1000
typedef struct
{
	uint8_t	frame[BUF_LEN];
	eOnOff	full;
	uint8_t	len;
	eOnOff	active;
}sFrame;
typedef struct
{
	eCmd		cmd;
	uint8_t	cmdval;
	uint8_t bufread[BUF_LEN];
	uint8_t bufwrite[BUF_LEN];
	sFrame	frames[FRAMEMAX];
	uint8_t	tick;
	uint8_t framenum;
}sCom;
typedef struct
{
	eOnOff				manual;
	uint32_t			times;
	eMode					work;
	eOnOff				hostcom;
	uint32_t			hostcomtick;
	uint32_t			lumiserwtick;
}sMode;
typedef struct
{
	double		speed;
	double		posref[MOTMAX];
	uint16_t	pwmrefg;
	uint16_t	pwmreft;
	uint16_t	pwmrefo;
	uint32_t	delay;
}sSingleStatus;
typedef struct
{
	sSingleStatus s[POINTSMAX];
	uint8_t				snum;
	uint8_t				smax;
}sSeqStat;
typedef struct
{
	sSeqStat 	seq[SEQMAX];
	uint8_t		seqnum;
	uint8_t		seqmax;
	double 		posrefhyst;
	uint32_t	time;
}sDevStat;
typedef struct
{
	eOnOff				buttons[BUTTONMAX];
	eOnOff				limitswitches[LIMITSWITCHMAX];
	
	TIM_TypeDef		*mottimsenc[MOTMAX];
	eOnOff				motpid[MOTMAX];
	eOnOff				motlimits[MOTMAX];
	int32_t				motimp[MOTMAX];
	int32_t				motimplast[MOTMAX];
	int32_t				motfullturn[MOTMAX];
	int32_t				motimptotal[MOTMAX];
	double				motposabs[MOTMAX];
	double				motposabslast[MOTMAX];
	double				motposabsmin[MOTMAX];
	double				motposabsmax[MOTMAX];
	double				motposabsref[MOTMAX];
	double				motposstart[MOTMAX];
	int8_t				motspeedref[MOTMAX];
	double				motspeedreftemp[MOTMAX];
	double				motspeed[MOTMAX];
	double				motstep[MOTMAX];
	double				motkp[MOTMAX];
	double				motki[MOTMAX];
	double				motkd[MOTMAX];
	double				motout[MOTMAX];
	double				motoutmax[MOTMAX];
	double				mote[MOTMAX];
	double				motelast[MOTMAX];
	double				motemax[MOTMAX];
	double				motetotal[MOTMAX];
	double				motetotalmax[MOTMAX];
	eDir					motdir[MOTMAX];
	int16_t				motpwm[MOTMAX];
	int16_t				motpwmout[MOTMAX];
	int16_t				motpwmmax[MOTMAX];
	int16_t				motpwmdeath[MOTMAX];
	uint16_t			motadcval[MOTMAX*1000];
	double				motcurrent[MOTMAX];
	eOnOff				moterror[MOTMAX];
	
	double				poslumihoriz;
	double				poslumivert;
	double				poszup;
	double				poszswab;
	double				poszswab2;
	double				poszlumi;
	double				poszmeas;
	double				posxyswab[SWABMAX][2];
	double				posxylumi[2];
	double				posxymeas[2];
	double				posxywib[2];
	double				poszwib;
	double				poslumifoto;
	double				posxyswabtrow[2];
	
	eOnOff				wibstate;
}sDrive;
typedef struct
{
	eOnOff		gripstate;
	uint16_t	grippwm[2];
	uint16_t	griprefpwm;
	eLS				turnstate;
	uint16_t	turnpwm[3];
	uint16_t	turnrefpwm;
	eOnOff		openstate;
	uint16_t	openpwm[2];
	uint16_t	openrefpwm;
}sSerw;
typedef struct
{
	uint8_t	srnum;
	double	srposx;
	double	srposy;
	double	srposzup;
	double	srposzup1;
	double	srposzup2;
	double	srposzlumi;
	double	posx[SWABMAX];
	double	posy[SWABMAX];
	double	poszup[SWABMAX];
	double	poszdown1[SWABMAX];
	double	poszdown2[SWABMAX];
	double	poszlumi[SWABMAX];
}sSwab;
typedef struct
{
	sMode			Mode;
	sCom			Com;
	sDrive		Drive;
	sSerw			Serw;
	sSwab			Swab;
	sDevStat	Stats;
}sControl;

void Control_Conf(void);
void Led_OnOff(uint8_t num, eLedState newstate);
void delay_ms(uint32_t ms);

#endif
