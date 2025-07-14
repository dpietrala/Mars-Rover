#include "Control.h"
sControl Control;
sControl* pC = &Control;
int main()
{
	Control_Conf();
	Drive_Conf();
	COM_Conf();
	while(1)
	{
		delay_ms(100);
	}
}
