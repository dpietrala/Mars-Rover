#include "Kin.h"
extern sControl* pC;

double FunFor_0_0(double T[6]){return sin(T[0])*(cos(T[4])*cos(T[5])*sin(T[3])+cos(T[3])*sin(T[5]))+cos(T[0])*(cos(T[3])*cos(T[4])*cos(T[5])*sin(T[1]+T[2])+cos(T[1]+T[2])*cos(T[5])*sin(T[4])-sin(T[1]+T[2])*sin(T[3])*sin(T[5]));}
double FunFor_0_1(double T[6]){return cos(T[5])*(cos(T[3])*sin(T[0])-cos(T[0])*sin(T[1]+T[2])*sin(T[3]))-(cos(T[4])*sin(T[0])*sin(T[3])+cos(T[0])*(cos(T[3])*cos(T[4])*sin(T[1]+T[2])+cos(T[1]+T[2])*sin(T[4])))*sin(T[5]);}
double FunFor_0_2(double T[6]){return cos(T[0])*cos(T[1]+T[2])*cos(T[4])-(cos(T[0])*cos(T[3])*sin(T[1]+T[2])+sin(T[0])*sin(T[3]))*sin(T[4]);}
double FunFor_0_3(double T[6]){return cos(T[0])*(a1+a3*cos(T[2])*sin(T[1])+cos(T[1])*(a2+a3*sin(T[2]))+a4*cos(T[3])*sin(T[1]+T[2])+a5*cos(T[3])*cos(T[4])*sin(T[1]+T[2])+a6*cos(T[3])*cos(T[4])*cos(T[5])*sin(T[1]+T[2])+d5*sin(T[1]+T[2])*sin(T[3])-d6*cos(T[3])*sin(T[1]+T[2])*sin(T[4])+cos(T[1]+T[2])*(d4+d6*cos(T[4])+(a5+a6*cos(T[5]))*sin(T[4]))-a6*sin(T[1]+T[2])*sin(T[3])*sin(T[5]))-sin(T[0])*(d2+d3-sin(T[3])*(a4+cos(T[4])*(a5+a6*cos(T[5]))-d6*sin(T[4]))+cos(T[3])*(d5-a6*sin(T[5])));}
double FunFor_1_0(double T[6]){return cos(T[5])*(cos(T[4])*(cos(T[2])*cos(T[3])*sin(T[0])*sin(T[1])+cos(T[1])*cos(T[3])*sin(T[0])*sin(T[2])-cos(T[0])*sin(T[3]))+cos(T[1]+T[2])*sin(T[0])*sin(T[4]))-(cos(T[0])*cos(T[3])+sin(T[0])*sin(T[1]+T[2])*sin(T[3]))*sin(T[5]);}
double FunFor_1_1(double T[6]){return cos(T[0])*(-(cos(T[3])*cos(T[5]))+cos(T[4])*sin(T[3])*sin(T[5]))-sin(T[0])*(cos(T[5])*sin(T[1]+T[2])*sin(T[3])+(cos(T[2])*cos(T[3])*cos(T[4])*sin(T[1])+cos(T[1])*cos(T[3])*cos(T[4])*sin(T[2])+cos(T[1]+T[2])*sin(T[4]))*sin(T[5]));}
double FunFor_1_2(double T[6]){return cos(T[1]+T[2])*cos(T[4])*sin(T[0])-(cos(T[2])*cos(T[3])*sin(T[0])*sin(T[1])+cos(T[1])*cos(T[3])*sin(T[0])*sin(T[2])-cos(T[0])*sin(T[3]))*sin(T[4]);}
double FunFor_1_3(double T[6]){return sin(T[0])*(a1+a3*cos(T[2])*sin(T[1])+a5*cos(T[2])*cos(T[3])*cos(T[4])*sin(T[1])+a6*cos(T[2])*cos(T[3])*cos(T[4])*cos(T[5])*sin(T[1])+a4*cos(T[3])*sin(T[1]+T[2])+d5*sin(T[1]+T[2])*sin(T[3])-d6*cos(T[2])*cos(T[3])*sin(T[1])*sin(T[4])+cos(T[1]+T[2])*(d4+d6*cos(T[4])+(a5+a6*cos(T[5]))*sin(T[4]))+cos(T[1])*(a2+sin(T[2])*(a3+cos(T[3])*(cos(T[4])*(a5+a6*cos(T[5]))-d6*sin(T[4]))))-a6*sin(T[1]+T[2])*sin(T[3])*sin(T[5]))+cos(T[0])*(d2+d3-sin(T[3])*(a4+cos(T[4])*(a5+a6*cos(T[5]))-d6*sin(T[4]))+cos(T[3])*(d5-a6*sin(T[5])));}
double FunFor_2_0(double T[6]){return -(cos(T[5])*sin(T[1]+T[2])*sin(T[4]))+cos(T[1]+T[2])*(cos(T[3])*cos(T[4])*cos(T[5])-sin(T[3])*sin(T[5]));}
double FunFor_2_1(double T[6]){return sin(T[1])*(cos(T[5])*sin(T[2])*sin(T[3])+(cos(T[3])*cos(T[4])*sin(T[2])+cos(T[2])*sin(T[4]))*sin(T[5]))+cos(T[1])*(sin(T[2])*sin(T[4])*sin(T[5])-cos(T[2])*(cos(T[5])*sin(T[3])+cos(T[3])*cos(T[4])*sin(T[5])));}
double FunFor_2_2(double T[6]){return -(cos(T[2])*(cos(T[4])*sin(T[1])+cos(T[1])*cos(T[3])*sin(T[4])))+sin(T[2])*(-(cos(T[1])*cos(T[4]))+cos(T[3])*sin(T[1])*sin(T[4]));}
double FunFor_2_3(double T[6]){return d1-a2*sin(T[1])-d6*cos(T[2])*cos(T[4])*sin(T[1])-a3*sin(T[1])*sin(T[2])-d4*sin(T[1]+T[2])+d6*cos(T[3])*sin(T[1])*sin(T[2])*sin(T[4])-a5*sin(T[1]+T[2])*sin(T[4])-a6*cos(T[5])*sin(T[1]+T[2])*sin(T[4])+cos(T[1])*(-(d6*cos(T[4])*sin(T[2]))+cos(T[2])*(a3-d6*cos(T[3])*sin(T[4])))+cos(T[1]+T[2])*(cos(T[3])*(a4+cos(T[4])*(a5+a6*cos(T[5])))+sin(T[3])*(d5-a6*sin(T[5])));}
double FunFor_3_0(double T[6]){return 0;}
double FunFor_3_1(double T[6]){return 0;}
double FunFor_3_2(double T[6]){return 0;}
double FunFor_3_3(double T[6]){return 1;}

double FunRef_0_0(double T[6]){return -(cos(T[3])*cos(T[5])*sin(T[4]))+sin(T[3])*sin(T[5]);}
double FunRef_0_1(double T[6]){return cos(T[5])*sin(T[3])+cos(T[3])*sin(T[4])*sin(T[5]);}
double FunRef_0_2(double T[6]){return cos(T[3])*cos(T[4]);}
double FunRef_0_3(double T[6]){return T[0];}
double FunRef_1_0(double T[6]){return -(cos(T[5])*sin(T[3])*sin(T[4]))-cos(T[3])*sin(T[5]);}
double FunRef_1_1(double T[6]){return -(cos(T[3])*cos(T[5]))+sin(T[3])*sin(T[4])*sin(T[5]);}
double FunRef_1_2(double T[6]){return cos(T[4])*sin(T[3]);}
double FunRef_1_3(double T[6]){return T[1];}
double FunRef_2_0(double T[6]){return cos(T[4])*cos(T[5]);}
double FunRef_2_1(double T[6]){return -(cos(T[4])*sin(T[5]));}
double FunRef_2_2(double T[6]){return sin(T[4]);}
double FunRef_2_3(double T[6]){return T[2];}
double FunRef_3_0(double T[6]){return 0;}
double FunRef_3_1(double T[6]){return 0;}
double FunRef_3_2(double T[6]){return 0;}
double FunRef_3_3(double T[6]){return 1;}

double Fun_0_0(double T[6]){return cos(T[4])*cos(T[5]);}
double Fun_0_1(double T[6]){return -(cos(T[4])*sin(T[5]));}
double Fun_0_2(double T[6]){return sin(T[4]);}
double Fun_0_3(double T[6]){return T[0];}
double Fun_1_0(double T[6]){return cos(T[5])*sin(T[3])*sin(T[4]) + cos(T[3])*sin(T[5]);}
double Fun_1_1(double T[6]){return cos(T[3])*cos(T[5]) - sin(T[3])*sin(T[4])*sin(T[5]);}
double Fun_1_2(double T[6]){return -(cos(T[4])*sin(T[3]));}
double Fun_1_3(double T[6]){return T[1];}
double Fun_2_0(double T[6]){return -(cos(T[3])*cos(T[5])*sin(T[4])) + sin(T[3])*sin(T[5]);}
double Fun_2_1(double T[6]){return cos(T[5])*sin(T[3]) + cos(T[3])*sin(T[4])*sin(T[5]);}
double Fun_2_2(double T[6]){return cos(T[3])*cos(T[4]);}
double Fun_2_3(double T[6]){return T[2];}
double Fun_3_0(double T[6]){return 0;}
double Fun_3_1(double T[6]){return 0;}
double Fun_3_2(double T[6]){return 0;}
double Fun_3_3(double T[6]){return 1;}

double FunInv2_0_0(double q[3]){return cos(q[0])*sin(q[1] + q[2]);}
double FunInv2_0_1(double q[3]){return sin(q[0])*sin(q[1] + q[2]);}
double FunInv2_0_2(double q[3]){return cos(q[1] + q[2]);}
double FunInv2_0_3(double q[3]){return -a3 - d1*cos(q[1] + q[2]) - a2*sin(q[2]) - a1*sin(q[1] + q[2]);}
double FunInv2_1_0(double q[3]){return sin(q[0]);}
double FunInv2_1_1(double q[3]){return -cos(q[0]);}
double FunInv2_1_2(double q[3]){return 0;}
double FunInv2_1_3(double q[3]){return d2 + d3;}
double FunInv2_2_0(double q[3]){return cos(q[0])*cos(q[1] + q[2]);}
double FunInv2_2_1(double q[3]){return cos(q[1] + q[2])*sin(q[0]);}
double FunInv2_2_2(double q[3]){return -sin(q[1] + q[2]);}
double FunInv2_2_3(double q[3]){return -(a2*cos(q[2])) - a1*cos(q[1] + q[2]) + d1*sin(q[1] + q[2]);}
double FunInv2_3_0(double q[3]){return 0;}
double FunInv2_3_1(double q[3]){return 0;}
double FunInv2_3_2(double q[3]){return 0;}
double FunInv2_3_3(double q[3]){return 1;}
//Funkcje do obliczania pierwszego napedu
double rr1(double px, double py){return -(sqrt((px*px)*(-(d2*d2)+(px*px)+(py*py)))/px);}
double rr2(double px, double py){return sqrt((px*px)*(-(d2*d2)+(px*px)+(py*py)))/px;}
double qq1(double px, double py){return atan2(-((d2*(px*px)+py*sqrt((px*px)*(-(d2*d2)+(px*px)+pow(py,2))))/(pow(px,3)+px*(py*py))),(d2*py-sqrt(pow(px,2)*(-pow(d2,2)+pow(px,2)+pow(py,2))))/(pow(px,2)+pow(py,2)));;}
double qq2(double px, double py){return atan2(-((d2*(px*px)-py*sqrt((px*px)*(-(d2*d2)+(px*px)+pow(py,2))))/(pow(px,3)+px*(py*py))),(d2*py+sqrt(pow(px,2)*(-pow(d2,2)+pow(px,2)+pow(py,2))))/(pow(px,2)+pow(py,2)));}
double Axspr(double r, double q){return r*cos(q)-d2*sin(q);}
double Ayspr(double r, double q){return d2*cos(q)+r*sin(q);}
//Funkcje do obliczania drugiego i trzeciego napedu
double B_q11(double h1, double h2){return atan2(-(((a2*a2*a2)*(h2*h2) + a2*(h2*h2)*(-(d4*d4) + (h1*h1) + (h2*h2)) +h1*sqrt(-((a2*a2)*(h2*h2)*((a2*a2*a2*a2) + pow(-(d4*d4) + (h1*h1) + (h2*h2),2) - 2*(a2*a2)*((d4*d4) + (h1*h1) + (h2*h2))))))/((a2*a2)*h2*((h1*h1) + (h2*h2)))), ((a2*a2*a2)*h1 + a2*h1*(-(d4*d4) + (h1*h1) + (h2*h2)) - sqrt(-((a2*a2)*(h2*h2)*((a2*a2*a2*a2) + pow(-(d4*d4) + (h1*h1) + (h2*h2),2) - 2*(a2*a2)*((d4*d4) + (h1*h1) + (h2*h2))))))/((a2*a2)*((h1*h1) + (h2*h2))));}
double B_q12(double h1, double h2){return atan2(sqrt(-((a2*a2)*(h2*h2)*((a2*a2*a2*a2) + pow(-(d4*d4) + (h1*h1) + (h2*h2),2) - 2*(a2*a2)*((d4*d4) + (h1*h1) + (h2*h2)))))/((a2*a2)*d4*h2), (-(a2*a2) - (d4*d4) + (h1*h1) + (h2*h2))/(a2*d4));}
double B_q21(double h1, double h2){return atan2((-((a2*a2*a2)*(h2*h2)) - a2*(h2*h2)*(-(d4*d4) + (h1*h1) + (h2*h2)) +h1*sqrt(-((a2*a2)*(h2*h2)*((a2*a2*a2*a2) + pow(-(d4*d4) + (h1*h1) + (h2*h2),2) - 2*(a2*a2)*((d4*d4) + (h1*h1) + (h2*h2))))))/((a2*a2)*h2*((h1*h1) + (h2*h2))), ((a2*a2*a2)*h1 + a2*h1*(-(d4*d4) + (h1*h1) + (h2*h2)) + sqrt(-((a2*a2)*(h2*h2)*((a2*a2*a2*a2) + pow(-(d4*d4) + (h1*h1) + (h2*h2),2) - 2*(a2*a2)*((d4*d4) + (h1*h1) + (h2*h2))))))/((a2*a2)*((h1*h1) + (h2*h2))));}
double B_q22(double h1, double h2){return atan2(-(sqrt(-((a2*a2)*(h2*h2)*((a2*a2*a2*a2) + pow(-(d4*d4) + (h1*h1) + (h2*h2),2) - 2*(a2*a2)*((d4*d4) + (h1*h1) + (h2*h2)))))/((a2*a2)*d4*h2)), (-(a2*a2) - (d4*d4) + (h1*h1) + (h2*h2))/(a2*d4));}

void Matxmat(double M1[4][4], double M2[4][4], double wynik[4][4])
{
    double suma=0;
    for(uint8_t i=0;i<4;i++)
        for(uint8_t j=0;j<4;j++)
        {
            suma=0;
            for(uint8_t k=0;k<4;k++)
                suma += M1[i][k] * M2[k][j];
            wynik[i][j]=suma;
        }
}
void Kin_Forward(double Out[4][4], double In[6])
{
    Out[0][0] = FunFor_0_0(In);
    Out[0][1] = FunFor_0_1(In);
    Out[0][2] = FunFor_0_2(In);
    Out[0][3] = FunFor_0_3(In);
    Out[1][0] = FunFor_1_0(In);
    Out[1][1] = FunFor_1_1(In);
    Out[1][2] = FunFor_1_2(In);
    Out[1][3] = FunFor_1_3(In);
    Out[2][0] = FunFor_2_0(In);
    Out[2][1] = FunFor_2_1(In);
    Out[2][2] = FunFor_2_2(In);
    Out[2][3] = FunFor_2_3(In);
    Out[3][0] = FunFor_3_0(In);
    Out[3][1] = FunFor_3_1(In);
    Out[3][2] = FunFor_3_2(In);
    Out[3][3] = FunFor_3_3(In);
}
void Kin_Reference(double Out[4][4], double In[])
{
    Out[0][0] = FunRef_0_0(In);
    Out[0][1] = FunRef_0_1(In);
    Out[0][2] = FunRef_0_2(In);
    Out[0][3] = FunRef_0_3(In);
    Out[1][0] = FunRef_1_0(In);
    Out[1][1] = FunRef_1_1(In);
    Out[1][2] = FunRef_1_2(In);
    Out[1][3] = FunRef_1_3(In);
    Out[2][0] = FunRef_2_0(In);
    Out[2][1] = FunRef_2_1(In);
    Out[2][2] = FunRef_2_2(In);
    Out[2][3] = FunRef_2_3(In);
    Out[3][0] = FunRef_3_0(In);
    Out[3][1] = FunRef_3_1(In);
    Out[3][2] = FunRef_3_2(In);
    Out[3][3] = FunRef_3_3(In);
}
void Kin_Funrpy(double Out[4][4], double In[])
{
    Out[0][0] = Fun_0_0(In);
    Out[0][1] = Fun_0_1(In);
    Out[0][2] = Fun_0_2(In);
    Out[0][3] = Fun_0_3(In);
    Out[1][0] = Fun_1_0(In);
    Out[1][1] = Fun_1_1(In);
    Out[1][2] = Fun_1_2(In);
    Out[1][3] = Fun_1_3(In);
    Out[2][0] = Fun_2_0(In);
    Out[2][1] = Fun_2_1(In);
    Out[2][2] = Fun_2_2(In);
    Out[2][3] = Fun_2_3(In);
    Out[3][0] = Fun_3_0(In);
    Out[3][1] = Fun_3_1(In);
    Out[3][2] = Fun_3_2(In);
    Out[3][3] = Fun_3_3(In);
}
void Kin_FunInv(double Out[4][4], double In[])
{
    Out[0][0] = FunInv2_0_0(In);
    Out[0][1] = FunInv2_0_1(In);
    Out[0][2] = FunInv2_0_2(In);
    Out[0][3] = FunInv2_0_3(In);
    Out[1][0] = FunInv2_1_0(In);
    Out[1][1] = FunInv2_1_1(In);
    Out[1][2] = FunInv2_1_2(In);
    Out[1][3] = FunInv2_1_3(In);
    Out[2][0] = FunInv2_2_0(In);
    Out[2][1] = FunInv2_2_1(In);
    Out[2][2] = FunInv2_2_2(In);
    Out[2][3] = FunInv2_2_3(In);
    Out[3][0] = FunInv2_3_0(In);
    Out[3][1] = FunInv2_3_1(In);
    Out[3][2] = FunInv2_3_2(In);
    Out[3][3] = FunInv2_3_3(In);
}
static void Kin_Q1(double A[3], double C[3], double Out[6])
{
    double q[2]={qq1(C[0], C[1]), qq2(C[0], C[1])};
    double r[2]={rr1(C[0], C[1]), rr2(C[0], C[1])};
    if((fabs(Axspr(fabs(r[0]), q[0]) - C[0]) < 0.5) && (fabs(Ayspr(fabs(r[0]), q[0]) - C[1]) < 0.5))
    {
        Out[0] = q[0];
        A[0] = fabs(d2)*sin(Out[0]);
        A[1] = -fabs(d2)*cos(Out[0]);
        A[2] = d1;
    }
    if((fabs(Axspr(fabs(r[1]), q[1]) - C[0]) < 0.5) && (fabs(Ayspr(fabs(r[1]), q[1]) - C[1]) < 0.5))
    {
        Out[0] = q[1];
        A[0] = fabs(d2)*sin(Out[0]);
        A[1] = -fabs(d2)*cos(Out[0]);
        A[2] = d1;
    }
}
static void Kin_Q2_3(double A[3], double C[3], double Posact[6], double Out[6])
{
    double rrr[3] = {C[0]-A[0], C[1]-A[1], C[2]-A[2]};
    double hz=rrr[2];
    double hxy=(rrr[0]) / cos(atan2(rrr[1], rrr[0]));

    if(hz < 0.01 && hz >= 0.0)					hz = 0.01;
    else if(hz > -0.01 && hz <= 0.0)		hz = -0.01;
    if(hxy < 0.01 && hxy >= 0.0)				hxy = 0.01;
    else if(hxy > -0.01 && hxy <= 0.0)	hxy = -0.01;
    if(Posact[2] >= 0.0)
    {
        if(B_q12(hxy, hz) >= 0.0)
        {
            Out[1] = B_q11(hxy, hz);
            Out[2] = B_q12(hxy, hz);
        }
        else
        {
            Out[1] = B_q21(hxy, hz);
            Out[2] = B_q22(hxy, hz);
        }
    }
    else
    {
        if(B_q12(hxy, hz) <= 0.0)
        {
            Out[1] = B_q11(hxy, hz);
            Out[2] = B_q12(hxy, hz);
        }
        else
        {
            Out[1] = B_q21(hxy, hz);
            Out[2] = B_q22(hxy, hz);
        }
    }
}
static void Kin_Q4_6(double Mref[4][4], double Posact[6], double Out[6])
{
    double M[4][4], M2[4][4];
    Kin_FunInv(M2, Out);
    Matxmat(M2, Mref, M);
    Out[4] = atan2(sqrt(1-pow(M[2][2] ,2)), M[2][2]);
    if(sin(Out[4]) >= 0)
    {
        Out[3] = atan2(-M[1][2], -M[0][2]);
        Out[5] = atan2(-M[2][1], M[2][0]);
    }
    else
    {
        Out[3] = atan2(M[1][2], M[0][2]);
        Out[5] = atan2(M[2][1], -M[2][0]);
    }

    if(fabs(Out[3] - Posact[3]) > M_PI_2 && fabs(Out[5] - Posact[5]) > M_PI_2)
    {
        Out[4] *= -1;
        if(sin(Out[4]) >= 0)
        {
            Out[3] = atan2(-M[1][2], -M[0][2]);
            Out[5] = atan2(-M[2][1], M[2][0]);
        }
        else
        {
            Out[3] = atan2(M[1][2], M[0][2]);
            Out[5] = atan2(M[2][1], -M[2][0]);
        }
    }
}
void Kin_Calc(double Mref[4][4], double Posact[6], double Out[6])
{
	double A[3], C[3] = {Mref[0][3]-d6*Mref[0][2], Mref[1][3]-d6*Mref[1][2], Mref[2][3]-d6*Mref[2][2]};
  Kin_Q1(A, C, Out);
  Kin_Q2_3(A, C, Posact, Out);
  Kin_Q4_6(Mref, Posact, Out);
}
uint8_t Kin_FindRpy(double M[4][4], double Posact[6], double Out[6])
{
	double temp[10][6];
  for(uint8_t i=0;i<10;i++)
  {
	temp[i][0] = M[0][3];
    temp[i][1] = M[1][3];
    temp[i][2] = M[2][3];
  }
  temp[0][3] = atan2(M[1][2], M[0][2]);
  temp[0][4] = atan2(M[2][2], sqrt(pow(M[2][1], 2) + pow(M[2][0], 2)));
  temp[0][5] = atan2(-M[2][1], M[2][0]);

  temp[1][3] = temp[0][3] + M_PI;
  temp[1][4] = -temp[0][4] + M_PI;
  temp[1][5] = temp[0][5] + M_PI;

  temp[2][3] = temp[0][3] - M_PI;
  temp[2][4] = -temp[0][4] + M_PI;
  temp[2][5] = temp[0][5] + M_PI;

  temp[3][3] = temp[0][3] + M_PI;
  temp[3][4] = -temp[0][4] + M_PI;
  temp[3][5] = temp[0][5] - M_PI;

  temp[4][3] = temp[0][3] - M_PI;
  temp[4][4] = -temp[0][4] + M_PI;
  temp[4][5] = temp[0][5] - M_PI;

  temp[5][3] = temp[0][3] + M_PI;
  temp[5][4] = -temp[0][4] - M_PI;
  temp[5][5] = temp[0][5] + M_PI;

  temp[6][3] = temp[0][3] - M_PI;
  temp[6][4] = -temp[0][4] - M_PI;
  temp[6][5] = temp[0][5] + M_PI;

  temp[7][3] = temp[0][3] + M_PI;
  temp[7][4] = -temp[0][4] - M_PI;
  temp[7][5] = temp[0][5] - M_PI;

  temp[8][3] = temp[0][3] - M_PI;
  temp[8][4] = -temp[0][4] - M_PI;
  temp[8][5] = temp[0][5] - M_PI;

  for(uint8_t i=1;i<9;i++)
		if(fabs(temp[i][3]) < M_PI && fabs(temp[i][4]) < M_PI && fabs(temp[i][5]) < M_PI)
    {
			temp[9][3] = temp[i][3];
			temp[9][4] = temp[i][4];
			temp[9][5] = temp[i][5];
    }

  double q2[6], q3[6], M1[4][4], M2[4][4];
  Kin_Reference(M1, temp[0]);
  Kin_Calc(M1, Posact, q2);
  eOnOff flag = Off;
  for(uint8_t i=0;i<6;i++)
		if(fabs(Posact[i] - q2[i]) > 0.01)
			flag = On;
				
  Kin_Reference(M2, temp[9]);
  Kin_Calc(M2, Posact, q3);
  eOnOff flag2 = Off;
  for(uint8_t i=0;i<6;i++)
		if(fabs(Posact[i] - q3[i]) > 0.01)
			flag2 = On;

  if(flag == Off)
  {
		for(uint8_t i=0;i<6;i++)
			Out[i] = temp[0][i];
		return 0;
  }
  else if(flag2 == Off)
  {
		for(uint8_t i=0;i<6;i++)
			Out[i] = temp[9][i];
		return 0;
  }
  return 1;
}
