#ifndef _AXES
#define _AXES
#include "Control.h"
void Axes_Act(void);
void Axes_Defpos(void);
void Axes_CoordinateAct(eHostCmd);
void Axes_ManipEnable(void);
void Axes_ManipDisable(void);
void Axes_AutoSystemOn(uint8_t, uint8_t);
void Axes_AutoClearSeq(uint8_t);
void Axes_AutoSavePoint(uint8_t, uint8_t, uint8_t, eAutoType, int8_t, uint8_t);
void Axes_AutoSavePointToBackup(void);
void Axes_AutoReadPointFromBackup(void);


#endif
