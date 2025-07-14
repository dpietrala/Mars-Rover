#ifndef	_CONTROL
#define _CONTROL

//-----wybor czy sterujemy manipulatorem czy laboratorium----------------
#define MANIPULATOR
//#define LABORATORIUM

typedef enum {Off =	0, On =	1} eOnOff;
typedef enum {ManipJoin =	0, ManipTool =	1, ManipBase = 2, ManipAutpPtp = 3} eManipCS;
typedef enum {Satel =	0, Wifi =	1, AutoStm = 2, AutoNV = 3} eHostName;
typedef enum {ASManual = 245, ASWait = 246, ASDrivetopoint = 247, ASLookball = 248, ASDrivetoball = 249, ASFoundball = 250} eAutoState;
typedef enum {LSNeutral = 0, LSOn = 1, LSMeas = 2} eLS;
typedef enum {LabWorkoff = 0, LabWorkmanual, LabWorkremote, LabWorkauto} eLabMode;
#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "Comunication.h"
#include "Case.h"
#include "LCD_Dwin.h"

#define COMTICKMAX		300
#define BUF_LEN				250
#define ADCMAX				600
#define AI_CHMAX			8
#define AI_ALLCHMAX		24
#define FRAMEMAX			7
#define BUTDEBMAX			4
typedef enum
{
	AI_MJ_X = 0, // Manip Joy X
	AI_MJ_Y = 1, // Manip Joy Y
	AI_MJ_Z = 2, // Manip Joy Z
	AI_MJ_RX = 3, // Manip Joy RX
	AI_MJ_RY = 4, // Manip Joy RY
	AI_MJ_RZ = 5, // Manip Joy RZ
	AI_M_GS = 6, // Manip Gripper
	AI_D_SP = 7, // Station pos
	AI_MP_X = 8, // Manip Pot X
	AI_MP_Y = 9, // Manip Pot Y
	AI_MP_Z = 10, // Manip Pot Z
	AI_MP_RX = 11, // Manip Pot RX
	AI_MP_RY = 12, // Manip Pot RY
	AI_MP_RZ = 13, // Manip Pot RZ
	AI_GUP_0 = 22, //Gimball górny 0
	AI_GUP_1 = 23, //Gimball górny 1
	AI_D_SL = 16, // jazda mnoznik predkosci lewy
	AI_D_SR = 17, //jazda mnoznik predkosci prawy
	AI_D_DSR = 18, // jazda na boki prawy joy
	AI_D_FSR = 19, // jazda do przodu prawy joy
	AI_D_DSL = 20, // jazda na boki lewy joy
	AI_D_FSL = 21, // jazda do przodu lewy joy
	AI_GDW_0 = 14, //Gimball dolny 0
	AI_GDW_1 = 15, //Gimball dolny 1
}eAI_NUM;
typedef enum 
{
	Frametype_null = 0,
	Frametype_DriveGeneral = 1,
	Frametype_ManipGeneral = 2,
	Frametype_LabGeneral = 3,
	Frametype_ManipCmd = 4,
	Frametype_ManipSave = 5,
	Frametype_NvidiaGenaral = 6,
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
	cmdmanip_null = 0,
	cmdmanip_enable = 1,
	cmdmanip_disable = 2,
	cmdmanip_calibrate = 3,
	cmdmanip_join = 4,
	cmdmanip_base = 5,
	cmdmanip_tool = 6,
	cmdmanip_laseron = 7,
	cmdmanip_laseroff = 8,
	cmdmanip_magneson = 9,
	cmdmanip_magnesoff = 10,
	cmdmanip_automove = 11,
	cmdmanip_clearseq = 12,
	cmdmanip_telemerrorsen = 13,
	cmdmanip_telemerrorsdis = 14,
	cmdmanip_telemposen = 15,
	cmdmanip_telemposdis = 16,
	cmdmanip_telemallen = 17,
	cmdmanip_telemalldis = 18,
}eCmdManip;
typedef enum 
{
	cmdmanipval_home = 0,
	cmdmanipval_park = 1,
	cmdmanipval_work = 2,
	cmdmanipval_take = 3,
}eCmdManipVal;
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
		cmdLab_MoveZPosSwab2 = 26,
		cmdLab_MoveZPosLumi = 19,
		cmdLab_MoveXYPosSwab = 20,
		cmdLab_MoveXYPosLumi = 21,
		cmdLab_MoveXYPosMeas = 22,
		cmdLab_AutoRunSeq = 23,
		cmdLab_AutoNextPoint = 24,
		cmdLab_AutoPrevPoint = 25,
}eCmdLab;
typedef enum 
{
	cmddrive_null = 0,
	cmddrive_satel = 1,
	cmddrive_wifi = 2,
	cmddrive_autostm = 3,
	cmddrive_autonvidia,
	cmddrive_select_cam1_next,
	cmddrive_select_cam1_back,
	cmddrive_select_cam2_next,
	cmddrive_select_cam2_back,
	cmddrive_autopause,
	cmddrive_autocontinue,
	cmddrive_autonext,
	cmddrive_autostage1,//brak
	cmddrive_autostage2,//brak
	cmddrive_autostage3,//brak
	cmddrive_autostage4,//brak
	cmddrive_resetdev1,
	cmddrive_resetdev2,
	cmddrive_gimdigiton,
	cmddrive_gimdigitoff,
	
	cmddrive_TelemGpsEn = 20,
	cmddrive_TelemGpsDis = 21,
	cmddrive_TelemBallEn = 23,
	cmddrive_TelemBallDis = 24,
	cmddrive_TelemErrorsEn = 25,
	cmddrive_TelemErrorsDis = 26,
	cmddrive_TelemAllEn = 27,
	cmddrive_TelemAllDis = 28,
	cmddrive_EmerSoftEn = 29,
	cmddrive_EmerSoftDis = 30,
	
	cmddrive_clearyawoffset = 243,
	cmddrive_setyawoffset = 244,
	
	cmddrive_out_manual = 245,
	cmddrive_out_wait = 246,
	cmddrive_out_drive = 247,
	cmddrive_out_lookball = 248,
	cmddrive_out_drivetoball = 249,
	cmddrive_out_foundball = 250,
	cmddrive_out_endlook = 251,
}eCmdDrive;
typedef enum
{
	Fnum_Drive = 0,
	Fnum_Manip = 1,
	Fnum_Us1 = 2,
	Fnum_Us2 = 3,
	Fnum_Us6 = 4,
	Fnum_SP = 5,
	Fnum_Lab = 6,
}eFNum;

typedef struct
{
	uint8_t	rbuf[BUF_LEN];
	eOnOff	rfull;
	uint8_t	rlen;
	uint8_t	wbuf[BUF_LEN];
	eOnOff	wfull;
	uint8_t	wlen;
	uint8_t cmd;
	uint8_t cmdval;
}sFrame;
typedef struct
{
	sFrame	frames[FRAMEMAX];
	uint8_t buftoanalysis[BUF_LEN];
	uint8_t bufsatelread[BUF_LEN];
	uint8_t bufsatelwrite[BUF_LEN];
	uint8_t bufethread[BUF_LEN];
	uint8_t bufethwrite[BUF_LEN];
	uint8_t	tick;
	uint8_t framenum;
}sCom;
typedef struct
{
	uint32_t			times;
	eOnOff				hostcom;
	uint32_t			hostcomtick;
}sStatus;
typedef struct
{
	//Sygnaly analogowe
	uint16_t		adc1val[AI_CHMAX*ADCMAX];
	uint16_t		adc2val[AI_CHMAX*ADCMAX];
	uint16_t		adc3val[AI_CHMAX*ADCMAX];
	double			hyst[AI_ALLCHMAX];
	double			val_adc[AI_ALLCHMAX];
	double			val_adc_min[AI_ALLCHMAX];
	double			val_adc_mean[AI_ALLCHMAX];
	double			val_adc_max[AI_ALLCHMAX];
	double 			val_adc1[AI_CHMAX];
	double 			val_adc2[AI_CHMAX];
	double 			val_adc3[AI_CHMAX];
	
	double			gimbal_up[2];
	double			gimbal_down[2];

	double			station_pos;
	double			station_pos_prev;
	double			gripper_speed;
	
	double			frontspeed_L;
	double			dirspeed_L;
	double			frontspeed_R;
	double			dirspeed_R;
	double			speed_L;
	double			speed_R;
	
	int8_t			manip_joy_x;
	int8_t			manip_joy_y;
	int8_t			manip_joy_z;
	int8_t			manip_joy_Rx;
	int8_t			manip_joy_Ry;
	int8_t			manip_joy_Rz;
	
	int8_t			manip_x_speed;
	int8_t			manip_y_speed;
	int8_t			manip_z_speed;
	int8_t			manip_Rx_speed;
	int8_t			manip_Ry_speed;
	int8_t			manip_Rz_speed;

	//sygnaly cyfrowe manipulator
	eOnOff				joint;
	eOnOff				tool;
	eOnOff				base;
	eOnOff				home;
	eOnOff				park;
	eOnOff				work;
	eOnOff				take;
	eOnOff				calibration;
	
	eOnOff				joint_prev;
	eOnOff				tool_prev;
	eOnOff				base_prev;
	eOnOff				home_prev;
	eOnOff				park_prev;
	eOnOff				work_prev;
	eOnOff				take_prev;
	eOnOff				calibration_prev;
	
	eOnOff				jointtemp;
	eOnOff				tooltemp;
	eOnOff				basetemp;
	eOnOff				hometemp;
	eOnOff				parktemp;
	eOnOff				worktemp;
	eOnOff				taketemp;
	eOnOff				calibrationtemp;
	
	uint16_t				joint_num;
	uint16_t				tool_num;
	uint16_t				base_num;
	uint16_t				home_num;
	uint16_t				park_num;
	uint16_t				work_num;
	uint16_t				take_num;
	uint16_t				calibration_num;
	
	
	//sygnaly cyfrowe laboratorium
	eOnOff				labgrip;
	eOnOff				lablumiopen;
	eLS						lablumiturn;
	eOnOff				labmoterror[4];
	
	//obsluga kamer
	eOnOff				select_cam1_next;
	eOnOff				select_cam1_back;
	eOnOff				select_cam2_next;
	eOnOff				select_cam2_back;
	
	eOnOff				select_cam1_next_prev;
	eOnOff				select_cam1_back_prev;
	eOnOff				select_cam2_next_prev;
	eOnOff				select_cam2_back_prev;
	
	eOnOff				select_cam1_nexttemp;
	eOnOff				select_cam1_backtemp;
	eOnOff				select_cam2_nexttemp;
	eOnOff				select_cam2_backtemp;
	
	uint16_t				select_cam1_next_num;
	uint16_t				select_cam1_back_num;
	uint16_t				select_cam2_next_num;
	uint16_t				select_cam2_back_num;
	
	//glowne pudelko
	eOnOff				gripper_open;
	eOnOff				gripper_close;
	eOnOff				laseron;
	eOnOff				laseroff;
	eOnOff				magneson;
	eOnOff				magnesoff;
	eOnOff				autostm;
	eOnOff				autonvidia;
	eOnOff				autopause;
	eOnOff				autocontinue;
	eOnOff				autonext;
	eOnOff				resetdev1;
	eOnOff				resetdev2;
	eOnOff				gimdigiton;
	eOnOff				gimdigitoff;
	eOnOff				automove;
	eOnOff				clearseq;
	eOnOff				driveTelemAllEn;
	eOnOff				driveTelemAllDis;
	eOnOff				manipTelemAllEn;
	eOnOff				manipTelemAllDis;
	
	eOnOff				gripper_open_prev;
	eOnOff				gripper_close_prev;
	eOnOff				laseron_prev;
	eOnOff				laseroff_prev;
	eOnOff				magneson_prev;
	eOnOff				magnesoff_prev;
	eOnOff				autostm_prev;
	eOnOff				autonvidia_prev;
	eOnOff				autopause_prev;
	eOnOff				autocontinue_prev;
	eOnOff				autonext_prev;
	eOnOff				resetdev1_prev;
	eOnOff				resetdev2_prev;
	eOnOff				gimdigiton_prev;
	eOnOff				gimdigitoff_prev;
	eOnOff				automove_prev;
	eOnOff				clearseq_prev;
	eOnOff				driveTelemAllEn_prev;
	eOnOff				driveTelemAllDis_prev;
	eOnOff				manipTelemAllEn_prev;
	eOnOff				manipTelemAllDis_prev;
	
	eOnOff				gripper_opentemp;
	eOnOff				gripper_closetemp;
	eOnOff				laserontemp;
	eOnOff				laserofftemp;
	eOnOff				magnesontemp;
	eOnOff				magnesofftemp;
	eOnOff				autostmtemp;
	eOnOff				autonvidiatemp;
	eOnOff				autopausetemp;
	eOnOff				autocontinuetemp;
	eOnOff				autonexttemp;
	eOnOff				resetdev1temp;
	eOnOff				resetdev2temp;
	eOnOff				gimdigitontemp;
	eOnOff				gimdigitofftemp;
	eOnOff				automovetemp;
	eOnOff				clearseqtemp;
	eOnOff				driveTelemAllEntemp;
	eOnOff				driveTelemAllDistemp;
	eOnOff				manipTelemAllEntemp;
	eOnOff				manipTelemAllDistemp;
	
	uint16_t				gripper_open_num;
	uint16_t				gripper_close_num;
	uint16_t				laseron_num;
	uint16_t				laseroff_num;
	uint16_t				magneson_num;
	uint16_t				magnesoff_num;
	uint16_t				autostm_num;
	uint16_t				autonvidia_num;
	uint16_t				autopause_num;
	uint16_t				autocontinue_num;
	uint16_t				autonext_num;
	uint16_t				resetdev1_num;
	uint16_t				resetdev2_num;
	uint16_t				gimdigiton_num;
	uint16_t				gimdigitoff_num;
	uint16_t				automove_num;
	uint16_t				clearseq_num;
	uint16_t				driveTelemAllEn_num;
	uint16_t				driveTelemAllDis_num;
	uint16_t				manipTelemAllEn_num;
	uint16_t				manipTelemAllDis_num;

	//komunikacja
	eOnOff			select_satel;
	eOnOff			select_wifi;
	
	eOnOff			select_satel_prev;
	eOnOff			select_wifi_prev;

	eOnOff			select_sateltemp;
	eOnOff			select_wifitemp;
	
	uint16_t			select_satel_num;
	uint16_t			select_wifi_num;
	
	//manipulator sterownaie silnikami
	eOnOff			manip_x_L;
	eOnOff			manip_x_R;
	eOnOff			manip_y_L;
	eOnOff			manip_y_R;
	eOnOff			manip_z_L;
	eOnOff			manip_z_R;
	eOnOff			manip_Rx_L;
	eOnOff			manip_Rx_R;
	eOnOff			manip_Ry_L;
	eOnOff			manip_Ry_R;
	eOnOff			manip_Rz_L;
	eOnOff			manip_Rz_R;
	
	eOnOff			manip_x_Ltemp;
	eOnOff			manip_x_Rtemp;
	eOnOff			manip_y_Ltemp;
	eOnOff			manip_y_Rtemp;
	eOnOff			manip_z_Ltemp;
	eOnOff			manip_z_Rtemp;
	eOnOff			manip_Rx_Ltemp;
	eOnOff			manip_Rx_Rtemp;
	eOnOff			manip_Ry_Ltemp;
	eOnOff			manip_Ry_Rtemp;
	eOnOff			manip_Rz_Ltemp;
	eOnOff			manip_Rz_Rtemp;
	
	uint16_t			manip_x_L_num;
	uint16_t			manip_x_R_num;
	uint16_t			manip_y_L_num;
	uint16_t			manip_y_R_num;
	uint16_t			manip_z_L_num;
	uint16_t			manip_z_R_num;
	uint16_t			manip_Rx_L_num;
	uint16_t			manip_Rx_R_num;
	uint16_t			manip_Ry_L_num;
	uint16_t			manip_Ry_R_num;
	uint16_t			manip_Rz_L_num;
	uint16_t			manip_Rz_R_num;
	//grzyb
	eOnOff			EmergencySwitchEn;
	eOnOff			EmergencySwitchDis;
	eOnOff			EmergencySwitchEntemp;
	eOnOff			EmergencySwitchDistemp;
	uint16_t			EmergencySwitchEn_num;
	uint16_t			EmergencySwitchDis_num;
}sCase;
typedef struct
{
	eOnOff			emer_soft; //emergency button in control box
	eOnOff			emer_hard; //emergency button on the rover
	eHostName		hostname;
	eManipCS		manipcs;
	eOnOff			manipcalibrated;
	eOnOff			drivemoter[4];
	eOnOff			manipmoter[8];
	double			imuyaw;
	double			imupitch;
	double			imupitchmax;
	eOnOff			imupitcher;
	double			imuroll;
	double			imurollmax;
	eOnOff			imuroller;
	double			gpslon;
	double			gpslat;
	uint8_t			autostate;
	uint8_t 		autonrp;
	
	double 			roverspeed;
	
	eOnOff			labmoter[4];
	eOnOff			labgripstate;
	eOnOff			lablumiopenstate;
	eLS					lablumiturnstate;
	eOnOff			labwibstate;
	eLabMode		labwork;
	eOnOff			labmotlimit[4];
	
}sTeleData;
typedef struct
{
	sStatus				Status;
	sCom					Com;
	sCase					Case;
	sTeleData			Tele;
}sControl;

#define WifiIdr									GPIOG->IDR & GPIO_IDR_IDR_8
#define SatelIdr								GPIOG->IDR & GPIO_IDR_IDR_7
#define AutostmIdr							GPIOG->IDR & GPIO_IDR_IDR_5
#define AutonvidiaIdr 		  		GPIOG->IDR & GPIO_IDR_IDR_6
#define AutopauseIdr						GPIOI->IDR & GPIO_IDR_IDR_5
#define AutocontinueIdr					GPIOI->IDR & GPIO_IDR_IDR_4
#define AutonextIdr							GPIOE->IDR & GPIO_IDR_IDR_0
#define DriveTelemAllEnIdr 		  GPIOB->IDR & GPIO_IDR_IDR_7
#define DriveTelemAllDisIdr 		GPIOB->IDR & GPIO_IDR_IDR_5
#define ManipTelemAllEnIdr 		  GPIOB->IDR & GPIO_IDR_IDR_6
#define ManipTelemAllDisIdr 		GPIOB->IDR & GPIO_IDR_IDR_4
#define Resetdev1Idr						GPIOB->IDR & GPIO_IDR_IDR_9
#define Resetdev2Idr						GPIOB->IDR & GPIO_IDR_IDR_8
#define GripperOpenIdr					GPIOF->IDR & GPIO_IDR_IDR_14
#define GripperCloseIdr					GPIOF->IDR & GPIO_IDR_IDR_13
#define RedButtonOnIdr					GPIOG->IDR & GPIO_IDR_IDR_15
#define RedButtonOffIdr					GPIOB->IDR & GPIO_IDR_IDR_3

#define Cam1NextIdr							GPIOG->IDR & GPIO_IDR_IDR_2
#define Cam1BackIdr							GPIOD->IDR & GPIO_IDR_IDR_15
#define Cam2NextIdr							GPIOG->IDR & GPIO_IDR_IDR_4
#define Cam2BackIdr							GPIOG->IDR & GPIO_IDR_IDR_3

#define ManipX_LIdr							GPIOH->IDR & GPIO_IDR_IDR_11
#define ManipX_RIdr							GPIOH->IDR & GPIO_IDR_IDR_10
#define ManipY_LIdr							GPIOH->IDR & GPIO_IDR_IDR_9
#define ManipY_RIdr							GPIOH->IDR & GPIO_IDR_IDR_8
#define ManipZ_LIdr							GPIOH->IDR & GPIO_IDR_IDR_7
#define ManipZ_RIdr							GPIOH->IDR & GPIO_IDR_IDR_6
#define ManipRX_LIdr						GPIOE->IDR & GPIO_IDR_IDR_15
#define ManipRX_RIdr						GPIOE->IDR & GPIO_IDR_IDR_14
#define ManipRY_LIdr						GPIOE->IDR & GPIO_IDR_IDR_13
#define ManipRY_RIdr						GPIOE->IDR & GPIO_IDR_IDR_12
#define ManipRZ_LIdr						GPIOE->IDR & GPIO_IDR_IDR_11
#define ManipRZ_RIdr						GPIOE->IDR & GPIO_IDR_IDR_10

#define ManipHomeIdr						GPIOI->IDR & GPIO_IDR_IDR_11
#define ManipParkIdr						GPIOI->IDR & GPIO_IDR_IDR_9
#define ManipTakeIdr						GPIOI->IDR & GPIO_IDR_IDR_8
#define ManipWorkIdr						GPIOC->IDR & GPIO_IDR_IDR_13
#define ManipJointIdr						GPIOI->IDR & GPIO_IDR_IDR_10
#define ManipToolIdr						GPIOC->IDR & GPIO_IDR_IDR_15
#define ManipBaseIdr						GPIOC->IDR & GPIO_IDR_IDR_14
#define ManipCalibIdr						GPIOE->IDR & GPIO_IDR_IDR_6
#define ManipLaserOffIdr				GPIOE->IDR & GPIO_IDR_IDR_8
#define ManipLaserOnIdr					GPIOG->IDR & GPIO_IDR_IDR_1
#define ManipMagnesOffIdr				GPIOE->IDR & GPIO_IDR_IDR_9
#define ManipMagnesOnIdr				GPIOE->IDR & GPIO_IDR_IDR_7


#define LED_PORTG						GPIOG
#define LEDJoint_PIN				GPIO_ODR_ODR_14
#define LEDJoint_OFF				LED_PORTG->ODR |= LEDJoint_PIN
#define LEDJoint_ON					LED_PORTG->ODR &= ~LEDJoint_PIN
#define LEDJoint_TOG				LED_PORTG->ODR ^= LEDJoint_PIN
#define LEDTool_PIN					GPIO_ODR_ODR_13
#define LEDTool_OFF					LED_PORTG->ODR |= LEDTool_PIN
#define LEDTool_ON					LED_PORTG->ODR &= ~LEDTool_PIN
#define LEDTool_TOG					LED_PORTG->ODR ^= LEDTool_PIN
#define LEDBase_PIN					GPIO_ODR_ODR_12
#define LEDBase_OFF					LED_PORTG->ODR |= LEDBase_PIN
#define LEDBase_ON					LED_PORTG->ODR &= ~LEDBase_PIN
#define LEDBase_TOG					LED_PORTG->ODR ^= LEDBase_PIN
#define LEDRedButton				GPIO_ODR_ODR_11
#define LEDRedButton_OFF		LED_PORTG->ODR |= LEDRedButton
#define LEDRedButton_ON			LED_PORTG->ODR &= ~LEDRedButton
#define LEDRedButton_TOG		LED_PORTG->ODR ^= LEDRedButton

#define LEDAuto1_OFF				GPIOD->ODR |= GPIO_ODR_ODR_7
#define LEDAuto1_ON					GPIOD->ODR &= ~GPIO_ODR_ODR_7
#define LEDAuto1_TOG				GPIOD->ODR ^= GPIO_ODR_ODR_7
#define LEDAuto2_OFF				GPIOG->ODR |= GPIO_ODR_ODR_10
#define LEDAuto2_ON					GPIOG->ODR &= ~GPIO_ODR_ODR_10
#define LEDAuto2_TOG				GPIOG->ODR ^= GPIO_ODR_ODR_10

#define LEDImu_OFF					GPIOG->ODR |= GPIO_ODR_ODR_9
#define LEDImu_ON						GPIOG->ODR &= ~GPIO_ODR_ODR_9
#define LEDImu_TOG					GPIOG->ODR ^= GPIO_ODR_ODR_9

#define LEDWheel1_OFF				GPIOE->ODR |= GPIO_ODR_ODR_4
#define LEDWheel1_ON				GPIOE->ODR &= ~GPIO_ODR_ODR_4
#define LEDWheel1_TOG				GPIOE->ODR ^= GPIO_ODR_ODR_4
#define LEDWheel2_OFF				GPIOE->ODR |= GPIO_ODR_ODR_2
#define LEDWheel2_ON				GPIOE->ODR &= ~GPIO_ODR_ODR_2
#define LEDWheel2_TOG				GPIOE->ODR ^= GPIO_ODR_ODR_2
#define LEDWheel3_OFF				GPIOE->ODR |= GPIO_ODR_ODR_5
#define LEDWheel3_ON				GPIOE->ODR &= ~GPIO_ODR_ODR_5
#define LEDWheel3_TOG				GPIOE->ODR ^= GPIO_ODR_ODR_5
#define LEDWheel4_OFF				GPIOE->ODR |= GPIO_ODR_ODR_3
#define LEDWheel4_ON				GPIOE->ODR &= ~GPIO_ODR_ODR_3
#define LEDWheel4_TOG				GPIOE->ODR ^= GPIO_ODR_ODR_3

#define LED_PORTI				GPIOI
#define LEDMotor5				GPIO_ODR_ODR_0
#define LEDMotor5_OFF		LED_PORTI->ODR |= LEDMotor5
#define LEDMotor5_ON		LED_PORTI->ODR &= ~LEDMotor5
#define LEDMotor5_TOG		LED_PORTI->ODR ^= LEDMotor5
#define LEDMotor1				GPIO_ODR_ODR_1
#define LEDMotor1_OFF		LED_PORTI->ODR |= LEDMotor1
#define LEDMotor1_ON		LED_PORTI->ODR &= ~LEDMotor1
#define LEDMotor1_TOG		LED_PORTI->ODR ^= LEDMotor1
#define LEDMotor4				GPIO_ODR_ODR_2
#define LEDMotor4_OFF		LED_PORTI->ODR |= LEDMotor4
#define LEDMotor4_ON		LED_PORTI->ODR &= ~LEDMotor4
#define LEDMotor4_TOG		LED_PORTI->ODR ^= LEDMotor4
#define LEDMotor7				GPIO_ODR_ODR_3
#define LEDMotor7_OFF		LED_PORTI->ODR |= LEDMotor7
#define LEDMotor7_ON		LED_PORTI->ODR &= ~LEDMotor7
#define LEDMotor7_TOG		LED_PORTI->ODR ^= LEDMotor7

#define LED_PORTH				GPIOH
#define LEDMotor3				GPIO_ODR_ODR_13
#define LEDMotor3_OFF		LED_PORTH->ODR |= LEDMotor3
#define LEDMotor3_ON		LED_PORTH->ODR &= ~LEDMotor3
#define LEDMotor3_TOG		LED_PORTH->ODR ^= LEDMotor3
#define LEDMotor6				GPIO_ODR_ODR_14
#define LEDMotor6_OFF		LED_PORTH->ODR |= LEDMotor6
#define LEDMotor6_ON		LED_PORTH->ODR &= ~LEDMotor6
#define LEDMotor6_TOG		LED_PORTH->ODR ^= LEDMotor6
#define LEDMotor2				GPIO_ODR_ODR_15
#define LEDMotor2_OFF		LED_PORTH->ODR |= LEDMotor2
#define LEDMotor2_ON		LED_PORTH->ODR &= ~LEDMotor2
#define LEDMotor2_TOG		LED_PORTH->ODR ^= LEDMotor2

#define LED_PORTD			GPIOD
#define Buzzer_OFF		LED_PORTD->ODR |= GPIO_ODR_ODR_12
#define Buzzer_ON			LED_PORTD->ODR &= ~GPIO_ODR_ODR_12
	
void Control_Conf(void);
void delay_ms(uint32_t ms);

#endif
