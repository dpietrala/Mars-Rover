#ifndef _CONTROL
#define _CONTROL
#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

typedef enum {Off =	0, On =	1} eOnOff;
typedef enum {Wifi = 0, Satel = 1, AutoStm = 2, AutoNvidia = 3} eHostName;
typedef enum {Waitpoint = 0, Drivetopoint = 1, Lookball = 2, Drivetoball = 3, Foundball = 4, DriveLook = 5} ePointStatus;
typedef enum {Checkpoint = 0, Temppoint = 1, Lookpoint = 2} ePointType;
typedef enum {Nvidiacmd_null = 0, Nvidiacmd_seeball = 1, Nvidiacmd_notseeball = 2, Nvidiacmd_foundball = 3} eNvidiaCmd;
typedef enum
{
	Hostcmd_null = 0,
	Hostcmd_satel = 1,
	Hostcmd_wifi = 2,
	Hostcmd_autostm = 3,
	Hostcmd_autonvidia = 4,
	Hostcmd_cam1inc = 5,
	Hostcmd_cam1dec = 6,
	Hostcmd_cam2inc = 7,
	Hostcmd_cam2dec = 8,
	Hostcmd_autopause = 9,
	Hostcmd_autocontinue = 10,
	Hostcmd_autonext = 11,
	Hostcmd_resetdev1 = 16,
	Hostcmd_resetdev2 = 17,
	Hostcmd_gimdigiton = 18,
	Hostcmd_gimdigitoff = 19,
	Hostcmd_TelemGpsEn = 20,
	Hostcmd_TelemGpsDis = 21,
	Hostcmd_TelemBallEn = 23,
	Hostcmd_TelemBallDis = 24,
	Hostcmd_TelemErrorsEn = 25,
	Hostcmd_TelemErrorsDis = 26,
	Hostcmd_TelemAllEn = 27,
	Hostcmd_TelemAllDis = 28,
	Hostcmd_EmerSoftEn = 29,
	Hostcmd_EmerSoftDis = 30,

	Hostcmd_clearyawoffset = 243,
	Hostcmd_setyawoffset = 244,

	Hostcmd_out_manual = 245,
	Hostcmd_out_wait = 246,
	Hostcmd_out_drive = 247,
	Hostcmd_out_lookball = 248,
	Hostcmd_out_drivetoball = 249,
	Hostcmd_out_foundball = 250,
	Hostcmd_out_endlook = 251,
}eHostCmd;
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
	Frametype_Header = 155,
	Frametype_emergencyhardStop = 201
}eFrameType;
typedef enum
{
	Framenum_Telemetry = 0,
	Framenum_GpsPos,
	Framenum_BallPos,
	Framenum_Manip
}eFrameNum;

#include "Drives.h"
#include "AddDevice.h"
#include "Comunication.h"

#define NVCOMTICKMAX		6000
#define COMTICKMAX			500
#define RESETDEVTICKMAX	1000
#define MOTMAX					4
#define BUF_LEN					250
#define FRAMEMAX				4
#define CAMMAX					6
#define POINTMAX				100
#define HELIPOINTMAX		20
#define AUTONPRECMAX		1500

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
	eHostCmd				cmdout;
	eHostCmd				cmdin;
	uint8_t 				satelbufread[BUF_LEN];
	uint8_t 				satelbufwrite[BUF_LEN];
	uint8_t 				wifibufread[BUF_LEN];
	uint8_t 				wifibufwrite[BUF_LEN];
	uint8_t 				manipbufread[BUF_LEN];
	uint8_t 				manipbufwrite[BUF_LEN];
	uint8_t					nvbufread[BUF_LEN];
	uint8_t					nvbufwrite[BUF_LEN];
	sFrame					frames[FRAMEMAX];
	uint8_t					framenum;
}sCom;
typedef struct
{
	eHostName	SelectHost;
}sMode;
typedef struct
{
	eOnOff		emergency;
	eOnOff		emergencyhard;
	eOnOff		emergencysoft;
	eOnOff		drivecomerror;
	uint32_t	drivecomtick;
	eOnOff		hostcomerror;
	uint32_t	hostcomtick;
	eOnOff		gpscomerror;
	uint32_t	gpscomtick;
	eOnOff		nvcomerror;
	uint32_t	nvcomtick;
}sError;
typedef struct
{
	eOnOff				activated;
	double				lon;
	double				lat;
	double				disttopoint;
	double				yawer;
	double				yawref;
	ePointType		type;
	ePointStatus	status;
	ePointStatus	statustemp;
}sPoint;
typedef struct
{
	eOnOff		slope;
	double		frontvalue;
	double 		dirvalue;
	double		motspeed;
	double 		motyaw;
	double		motposx;
	double		motposy;
	uint8_t		bufread[BUF_LEN];
	uint8_t		bufwrite[BUF_LEN];
	eOnOff		pid[MOTMAX];
	eOnOff		error[MOTMAX];
	double		speedref[MOTMAX];
	
	sPoint		points[POINTMAX];
	uint8_t		numpoints;
	uint8_t		numrecpoints;
	uint8_t		nrp;
	sPoint		refpoint;
	sPoint		ball;
	eOnOff		autonprec; // potwierdzenie odebrania punktów
	uint32_t	autonprectime;
}sDrive;
typedef struct
{
	eOnOff		Gimdigit;
	int8_t		Trans1;
	int8_t 		Trans2;
	int8_t 		SerwPos[4];
	uint16_t	SerwPwm[4];
	
	uint8_t		gpsbufread[BUF_LEN];
	uint8_t		gpsbufwrite[BUF_LEN];
	double		gpslat;
	double		gpslon;
	double		gpsalt;
	double		imuyawoffset;
	double		imuyawraw;
	double		imuyaw;
	double		imupitch;
	double		imuroll;
	eOnOff		resetdev1;
	uint32_t	resetdev1tick;
	eOnOff		resetdev2;
	uint32_t	resetdev2tick;
}sAddDev;
typedef struct
{
	uint32_t	timems;
	sMode			Mode;
	sDrive		Drive;
	sCom			Com;
	sError		Error;
	sAddDev		Dev;
}sControl;
void SystemStart(void);
void delay_ms(uint32_t ms);
void ClearStr(uint8_t* str, uint32_t l);
void Control_Conf(void);
uint16_t crc16(uint8_t* packet, uint32_t nBytes);

#endif
