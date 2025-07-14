#include "Control.h"
sControl Control;
sControl* pC = &Control;
int main()
{
	Control_Conf();
	Case_Conf();
	COM_Conf();
	LCD_Dwin_Conf();
	while(1)
	{
//		USART3->DR = 123;
		delay_ms(100);
	}
}

