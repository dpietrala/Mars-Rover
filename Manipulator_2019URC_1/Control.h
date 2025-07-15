#ifndef _CONTROL
#define _CONTROL

typedef enum {Off =	0, On =	1} eOnOff;
typedef enum {Forward =	0, Backward = 1, Stop = 2} eDir;
typedef enum {Join =	0, Base = 1, Tool = 2, Ptp = 3, Line = 4} eCoordinate;
typedef enum {AutoPtp =	0, AutoLine = 1} eAutoType;
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
	Hostcmd_null = 0,
	Hostcmd_enable = 1,
	Hostcmd_disable = 2,
	Hostcmd_calibrate = 3,
	Hostcmd_join = 4,
	Hostcmd_base = 5,
	Hostcmd_tool = 6,
	Hostcmd_laseron = 7,
	Hostcmd_laseroff = 8,
	Hostcmd_solenoidon = 9,
	Hostcmd_solenoidoff = 10,
	Hostcmd_automove = 11,
	Hostcmd_clearseq = 12,
	Hostcmd_telemerrorsen = 13,
	Hostcmd_telemerrorsdis = 14,
	Hostcmd_telemposen = 15,
	Hostcmd_telemposdis = 16,
	Hostcmd_telemallen = 17,
	Hostcmd_telemalldis = 18,
}eHostCmd;
typedef enum
{
	Framenum_Telemetry = 0,
	Framenum_MotPos,
}eFrameNum;

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "Comunication.h"
#include "AddDevice.h"
#include "Kin.h"
#include "Axes.h"
#include "Motors.h"

#define TIMMAX				0xffff
#define COMTICKMAX		800
#define AUTOSEQMAX		8
#define	AUTOPOINTMAX	8
#define AXESMAX				8
#define PIDTIME				(double)0.01
#define BASETIME			(double)0.05
#define BUF_LEN				100
#define BACKUPADDRESS	0x40024008
#define FRAMEMAX			2
#define FLAGTIME			1500

#define M_2_PI	6.28318530717958647
#define M_PI		3.14159265358979323
#define M_PI_2	1.57079632679489661
#define M_PI_4 	0.78539816339744830
#define M_PI_8 	0.39269908169872415

typedef struct
{
	uint8_t	frame[BUF_LEN];
	eOnOff	full;
	uint8_t	len;
	eOnOff	active;
}sFrame;
typedef struct
{
	eHostCmd	 		cmdin;
	uint8_t				cmdinval;
	uint8_t				hostbufread[BUF_LEN];
	uint8_t				hostbufwrite[BUF_LEN];
	sFrame				frames[FRAMEMAX];
	uint8_t				framenum;
}sCom;
typedef struct
{
	eOnOff				error[AXESMAX];
	eOnOff				pid[AXESMAX];
	TIM_TypeDef* 	tims[AXESMAX];
	int16_t 			imp[AXESMAX];
	int16_t 			implast[AXESMAX];
	int32_t				impfullturn[AXESMAX];
	int32_t				impperrad[AXESMAX];
	
	double				speedref[AXESMAX];
	double				pos[AXESMAX];
	double				posabs[AXESMAX];
	double				posfullturn[AXESMAX];
	double				posref[AXESMAX];
	double				posstart[AXESMAX];
	double				posabsmin[AXESMAX];
	double				posabsmax[AXESMAX];
	double				posdef[AXESMAX];
	double				speedmax[AXESMAX];
	double				step[AXESMAX];
	double				speed[AXESMAX];
	
	double				kp[AXESMAX];
	double				ki[AXESMAX];
	double 				kd[AXESMAX];
	double				e[AXESMAX];
	double				emax[AXESMAX];
	double 				elast[AXESMAX];
	double 				etotal[AXESMAX];
	double 				etotalmax[AXESMAX];
	double				out[AXESMAX];
	double				outmax[AXESMAX];
	int16_t				pwm[AXESMAX];
	uint16_t			pwmout[AXESMAX];
	int16_t				pwmmax[AXESMAX];
	int16_t				pwmdeath[AXESMAX];
	eDir					dir[AXESMAX];
	uint16_t			adcval[AXESMAX*1000];
	double				current[AXESMAX];
	double				currentmax[AXESMAX];
}sMot;
typedef struct
{
	eAutoType		type;
	eOnOff			active;
	uint8_t			speed;
	double			pos[AXESMAX];
}sAutoPoint;
typedef struct
{
	sAutoPoint 	points[AUTOPOINTMAX];
}sAutoSeq;
typedef struct
{
	double		speedref[AXESMAX];
	
	double		matact[4][4];
	double		matref[4][4];
	double		kartpos[AXESMAX];
	double		kartposref[AXESMAX];
	double		kartangle[AXESMAX];
	double		kartangleabs[AXESMAX];
	double		kartangleref[AXESMAX];
	double		kartanglerefabs[AXESMAX];
	double		kartposstep[AXESMAX];
	
	eOnOff		autoglobalinpos;
	double		autoglobalspeed;
	sAutoSeq	autoseq[AUTOSEQMAX];
	uint8_t		autoseqnum;
	uint8_t		autopointnum;
	uint32_t	autotimetick;
	double		autoposref[AXESMAX];
	double		autodist[AXESMAX];
	double		autoposrefabs[AXESMAX];
	double		autoposrefabstemp[AXESMAX];
	double		autospeed[AXESMAX];
	double		autohyst[AXESMAX];
	eOnOff		autoinpos[AXESMAX];
	double		automovetimemax[AXESMAX];
}sAxes;
typedef struct
{
	uint32_t			time;
	eOnOff				work;
	eCoordinate		coordinate;
	eOnOff				calibrated;
	uint32_t			calibratedflagtime;
	uint32_t			hostcomtime;
	eOnOff				hostcomer;
	eOnOff				backuper;
}sStatus;
typedef struct
{
	sStatus		Status;
	sCom			Com;
	sMot			Mot;
	sAxes			Axes;
}sControl;
void delay_ms(uint32_t ms);
void Control_Conf(void);

#endif
