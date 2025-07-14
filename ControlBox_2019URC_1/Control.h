#ifndef	_CONTROL
#define _CONTROL

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Comunication.h"
#include "Inputs.h"

#define COMTICKMAX	200
#define BUF_LEN			200
#define BUF_MAX			8
#define COM_MAX			6
#define AI_MAX			16
#define DI_MAX			32
#define DITICK_MAX	3
typedef enum {Off =	0, On =	1} eOnOff;
typedef enum {DIStateOff =	0, DIStateOn =	1} eDIState;
typedef enum
{
	Cmdrover_null = 0,
	Cmdrover_satel = 1,
	Cmdrover_wifi = 2,
	Cmdrover_auto = 3,
	Cmdrover_manipon = 4,
	Cmdrover_manipoff = 5,
	Cmdrover_manipdefpos = 6,
	Cmdrover_manipjoin = 7,
	Cmdrover_manipbase = 8,
	Cmdrover_maniptool = 9,
	Cmdrover_gimdigiton = 10,
	Cmdrover_gimdigitoff = 11,
	Cmdrover_laser_on = 12,
	Cmdrover_laser_off = 13,
	Cmdrover_cam2inc = 14,
	Cmdrover_cam2dec = 15,
	Cmdrover_cam1inc = 16,
	Cmdrover_cam1dec = 17,
	Cmdrover_manipptp0 = 20,
	Cmdrover_manipptp1 = 21,
	Cmdrover_manipptp2 = 22,
	Cmdrover_manipptp3 = 23,
	Cmdrover_manipptp4 = 24,
	Cmdrover_manipptp5 = 25,
	Cmdrover_manipptp6 = 26,
	Cmdrover_manipptp7 = 27,
	Cmdrover_autopause = 28,
	Cmdrover_autocontinue = 29,
	Cmdrover_autostage0 = 30,
	Cmdrover_autostage1 = 31,
	Cmdrover_autostage2 = 32,
	Cmdrover_autostage3 = 33,
	Cmdrover_autostage4 = 34,
	Cmdrover_autonext = 35,
	Cmdrover_autostage5 = 36,
	Cmdrover_autozeropos = 37,
	Cmdrover_manipptp8 = 40,
	Cmdrover_manipptp9 = 41,
	Cmdrover_manipptp10 = 42,
	Cmdrover_manipptp11 = 43,
	Cmdrover_manipptp12 = 44,
	Cmdrover_manipptp13 = 45,
	Cmdrover_manipptp14 = 46,
	Cmdrover_manipptp15 = 47,
	Cmdrover_resetdev = 222,
	Cmdrover_resetdev2 = 223,
	Cmdrover_solenoid_on = 224,
	Cmdrover_solenoid_off = 225,
	Cmdrover_out_manual = 245,
	Cmdrover_out_wait = 246,
	Cmdrover_out_drive = 247,
	Cmdrover_out_lookball = 248,
	Cmdrover_out_drivetoball = 249,
	Cmdrover_out_foundpoint = 250,
	Cmdrover_out_endlook = 251,
}eCmdRover;
typedef struct
{
	USART_TypeDef				*com;
	IRQn_Type						comirq;
	DMA_TypeDef					*dma;
	
	DMA_Stream_TypeDef	*readstream;
	IRQn_Type						writedmairq;
	uint32_t						readclearflag;
	DMA_Stream_TypeDef	*writestream;
	IRQn_Type						readdmairq;
	uint32_t						writeclearflag;
	
	eOnOff							busy;
	uint8_t 						bufread[BUF_LEN];
	uint8_t 						bufwrite[BUF_MAX][BUF_LEN];
	uint8_t 						bufwritelen[BUF_MAX];
	int16_t 						fifowp;
	int16_t 						fiforp;
	int16_t 						fifonum;
}sCom;
typedef struct
{
	int8_t	frontspeed;
	int8_t	dirspeed;
	int8_t	manipspeed[7];
	int8_t	serwpos[4];
	int8_t	cmd;
}sFrame;
typedef struct
{
	uint16_t 	adcval;
	int16_t		max;
	int16_t		min;
	int16_t		val;
	int16_t		mean;
	uint8_t		hyst;
}sAI;
typedef struct
{
	GPIO_TypeDef 	*port;
	uint32_t			moder;
	uint32_t			pupdr;
	uint32_t			idr;
	eCmdRover			cmd;
	uint8_t				cnt;
	eDIState			state;
}sDI;
typedef struct
{
	uint32_t	times;
}sStatus;
typedef struct
{
	sStatus		Status;
	sCom			Com[COM_MAX];
	sAI				AI[AI_MAX];
	sDI				DI[DI_MAX];
	sFrame		Frame;
	uint16_t	AIRawvalue[100*AI_MAX];
}sControl;


void Control_Conf(void);
void delay_ms(uint32_t ms);

#endif
