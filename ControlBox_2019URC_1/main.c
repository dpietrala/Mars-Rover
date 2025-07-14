#include "Control.h"
sControl Control;
sControl* pC = &Control;
int main()
{
	Control_Conf();
	COM_Conf();
	AI_Conf();
	DI_Conf();
	while(1)
	{
		delay_ms(10);
	}
}
