#ifndef	_CONTROL
#define _CONTROL

typedef enum {Off =	0, On =	1} eOnOff;
typedef enum {Forward =	0, Backward = 1, Stop = 2} eDir;
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
	Frametype_EmergencyStop = 201
}eFrameType;

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "Motors.h"
#include "Comunication.h"

#define MOTMAX			4
#define COMTICKMAX	200
#define BUF_LEN			100
#define PIDTIME			0.01
#define TIMMAX			0xffff
typedef struct
{
	uint8_t bufread[BUF_LEN];
	uint8_t bufwrite[BUF_LEN];
}sCom;
typedef struct
{
	uint32_t	times;
	eOnOff		work;
	eOnOff		hostcom;
	uint32_t	hostcomtick;
	eOnOff		overcurrent[MOTMAX];
	eOnOff		overspeed[MOTMAX];
	eOnOff		error[MOTMAX];
}sStatus;
typedef struct
{
	eOnOff				pid[MOTMAX];
	TIM_TypeDef* 	tims[MOTMAX];
	int16_t 			pos[MOTMAX];
	int16_t 			poslast[MOTMAX];
	int32_t				fullturn[MOTMAX];
	int32_t				postotal[MOTMAX];
	int32_t				postotallast[MOTMAX];
	double				roverspeed;
	double				azimuth;
	double				posx;
	double				posy;
	double 				speed[MOTMAX];
	double 				speedref[MOTMAX];
	double 				speedrefmax[MOTMAX];
	double				distance[MOTMAX];
	double				kp[MOTMAX];
	double				ki[MOTMAX];
	double 				kd[MOTMAX];
	double				e[MOTMAX];
	double				emax[MOTMAX];
	double 				elast[MOTMAX];
	double 				etotal[MOTMAX];
	double 				etotalmax[MOTMAX];
	double				out[MOTMAX];
	double				outmax[MOTMAX];
	int16_t				pwm[MOTMAX];
	uint16_t			pwmout[MOTMAX];
	int16_t				pwmmax[MOTMAX];
	int16_t				pwmdeath[MOTMAX];
	eDir					dir[MOTMAX];
	uint16_t			adcval[MOTMAX*1000];
	double				current[MOTMAX];
	double				currentmax[MOTMAX];
	double				currenterror[MOTMAX];
	uint32_t			curenttime[MOTMAX];
}sMot;
typedef struct
{
	sStatus		Status;
	sCom			Com;
	sMot			Mot;
}sControl;

#define LED_PORT		GPIOE
#define LED1_PIN		GPIO_ODR_ODR_2
#define LED2_PIN		GPIO_ODR_ODR_3
#define LED3_PIN		GPIO_ODR_ODR_4
#define LED4_PIN		GPIO_ODR_ODR_5
#define LED5_PIN		GPIO_ODR_ODR_6

#define LED1_ON			LED_PORT->ODR |= LED1_PIN;
#define LED1_OFF		LED_PORT->ODR &= ~LED1_PIN;
#define LED1_TOG		LED_PORT->ODR ^= LED1_PIN;
#define LED2_ON			LED_PORT->ODR |= LED2_PIN;
#define LED2_OFF		LED_PORT->ODR &= ~LED2_PIN;
#define LED2_TOG		LED_PORT->ODR ^= LED2_PIN;
#define LED3_ON			LED_PORT->ODR |= LED3_PIN;
#define LED3_OFF		LED_PORT->ODR &= ~LED3_PIN;
#define LED3_TOG		LED_PORT->ODR ^= LED3_PIN;
#define LED4_ON			LED_PORT->ODR |= LED4_PIN;
#define LED4_OFF		LED_PORT->ODR &= ~LED4_PIN;
#define LED4_TOG		LED_PORT->ODR ^= LED4_PIN;
#define LED5_ON			LED_PORT->ODR |= LED5_PIN;
#define LED5_OFF		LED_PORT->ODR &= ~LED5_PIN;
#define LED5_TOG		LED_PORT->ODR ^= LED5_PIN;
#define LEDALL_ON		LED_PORT->ODR |= LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN | LED5_PIN;
#define LEDALL_OFF	LED_PORT->ODR &= ~LED1_PIN & ~LED2_PIN & ~LED3_PIN & ~LED4_PIN & ~LED5_PIN;
#define LEDALL_TOG	LED_PORT->ODR ^= LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN  | LED5_PIN;


void Control_Conf(void);
void delay_ms(uint32_t ms);

#endif
