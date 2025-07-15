#ifndef _Kin
#define _Kin

#include "Control.h"

#define MAX		4
#define JOINTS	6
#define d1 73.0
#define a1 0.0
#define d2 -98.0
#define a2 400.0
#define d3 0.0
#define a3 0.0
#define d4 (500.9 + 3.6)	//W sterej wersji bylo 500.9mm
#define a4 0.0
#define d5 0.0
#define a5 0.0
#define d6 400	// Dawniej(350 + 40.9 + 37.0)
#define a6 0.0

void Matxmat(double M1[4][4], double M2[4][4], double wynik[4][4]);
void Kin_Forward(double Out[4][4], double In[6]);
void Kin_Reference(double Out[4][4], double In[]);
void Kin_Funrpy(double Out[4][4], double In[]);
void Kin_FunInv(double Out[4][4], double In[]);

uint8_t Kin_FindRpy(double M[4][4], double Posact[6], double Out[6]);
void Kin_Calc(double Mref[4][4], double Posact[6], double Out[6]);
#endif
