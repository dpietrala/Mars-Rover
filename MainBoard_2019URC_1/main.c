#include "Control.h"
sControl Control;
sControl* pC = &Control;
int main()
{
	SystemStart();
	Control_Conf();
	Com_Conf();
	Drive_Conf();
	Dev_Conf();
	while(1)
	{
		LED1_TOG;
		delay_ms(100);
	}
}
