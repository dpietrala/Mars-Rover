#include "Control.h"
sControl Control;
sControl* pC = &Control;
int main()
{
	Control_Conf();
	Mot_Conf();
	COM_Conf();
	while(1)
	{
		delay_ms(100);
		LED1_TOG;
	}
}
