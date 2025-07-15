#include "Control.h"
sControl Control;
sControl* pC = &Control;
int main()
{
	Control_Conf();
	Mot_Conf();
	Com_Conf();
	AddDev_Conf();
	while(1)
	{
		LED5_TOG;
		Com_SendToHost();
		delay_ms(150);
	}
}
