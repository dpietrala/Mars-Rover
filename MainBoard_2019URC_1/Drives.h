#ifndef _DRIVES
#define _DRIVES

#include "Control.h"

void Drive_Conf(void);
void Drive_Act(void);
void Drive_AutoNextPoint(void);
void Drive_FindGeoFromDist(double lo, double la, double az, double d, double* lonB, double* latB);
#endif
