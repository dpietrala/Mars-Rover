#ifndef _DEV
#define	_DEV
#include "Control.h"

#define LED_PORT		GPIOC
#define LED1_PIN		GPIO_ODR_ODR_0
#define LED2_PIN		GPIO_ODR_ODR_1
#define LED3_PIN		GPIO_ODR_ODR_2
#define LED4_PIN		GPIO_ODR_ODR_3
#define LED1_ON			LED_PORT->ODR |= LED1_PIN;
#define LED1_OFF		LED_PORT->ODR &= ~LED1_PIN;
#define LED1_TOG		LED_PORT->ODR ^= LED1_PIN;
#define LED2_ON			LED_PORT->ODR |= LED2_PIN;
#define LED2_OFF		LED_PORT->ODR &= ~LED2_PIN;
#define LED2_TOG		LED_PORT->ODR ^= LED2_PIN;
#define LED3_ON			LED_PORT->ODR |= LED3_PIN;
#define LED3_OFF		LED_PORT->ODR &= ~LED3_PIN;
#define LED3_TOG		LED_PORT->ODR ^= LED3_PIN;
#define LED4_ON			LED_PORT->ODR |= LED4_PIN;
#define LED4_OFF		LED_PORT->ODR &= ~LED4_PIN;
#define LED4_TOG		LED_PORT->ODR ^= LED4_PIN;
#define LEDALL_ON		LED_PORT->ODR |= LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN;
#define LEDALL_OFF	LED_PORT->ODR &= ~LED1_PIN & ~LED2_PIN & ~LED3_PIN & ~LED4_PIN;
#define LEDALL_TOG	LED_PORT->ODR ^= LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN;

//------- Definiowanie pinów sterujacych przelaczaniem kamer -----------------------------------------------
#define MUX_PORT				GPIOE
#define MUX1A_PIN				GPIO_ODR_ODR_7
#define MUX1B_PIN				GPIO_ODR_ODR_8
#define MUX1C_PIN				GPIO_ODR_ODR_9
#define MUX1D_PIN				GPIO_ODR_ODR_10
#define MUX2A_PIN				GPIO_ODR_ODR_11
#define MUX2B_PIN				GPIO_ODR_ODR_12
#define MUX2C_PIN				GPIO_ODR_ODR_13
#define MUX2D_PIN				GPIO_ODR_ODR_14

//------- Definiowanie makr do sterowania pinami mux kamer ---------------------------------------------
#define MUX1A_LOW			MUX_PORT->ODR &= ~MUX1A_PIN
#define MUX1A_HIGH		MUX_PORT->ODR |= MUX1A_PIN
#define MUX1B_LOW			MUX_PORT->ODR &= ~MUX1B_PIN
#define MUX1B_HIGH		MUX_PORT->ODR |= MUX1B_PIN
#define MUX1C_LOW			MUX_PORT->ODR &= ~MUX1C_PIN
#define MUX1C_HIGH		MUX_PORT->ODR |= MUX1C_PIN
#define MUX1D_LOW			MUX_PORT->ODR &= ~MUX1D_PIN
#define MUX1D_HIGH		MUX_PORT->ODR |= MUX1D_PIN
#define MUX2A_LOW			MUX_PORT->ODR &= ~MUX2A_PIN
#define MUX2A_HIGH		MUX_PORT->ODR |= MUX2A_PIN
#define MUX2B_LOW			MUX_PORT->ODR &= ~MUX2B_PIN
#define MUX2B_HIGH		MUX_PORT->ODR |= MUX2B_PIN
#define MUX2C_LOW			MUX_PORT->ODR &= ~MUX2C_PIN
#define MUX2C_HIGH		MUX_PORT->ODR |= MUX2C_PIN
#define MUX2D_LOW			MUX_PORT->ODR &= ~MUX2D_PIN
#define MUX2D_HIGH		MUX_PORT->ODR |= MUX2D_PIN

//------- Definiowanie pinów do sterowania serwami napedzajacymi kamery --------------------------------
#define CAM_SERW_PWM0_REG		TIM4->CCR1
#define CAM_SERW_PWM1_REG		TIM4->CCR2
#define CAM_SERW_PWM2_REG		TIM4->CCR3
#define CAM_SERW_PWM3_REG		TIM4->CCR4

//------- Definiowanie pinów sterujacych muxem od komunikacji -------------------------------------------
#define COMMUX_PORT			GPIOB
#define COMMUXA_PIN			GPIO_ODR_ODR_3
#define COMMUXB_PIN			GPIO_ODR_ODR_4
#define COMMUXC_PIN			GPIO_ODR_ODR_5

#define COMMUXA_LOW			COMMUX_PORT->ODR &= ~COMMUXA_PIN
#define COMMUXA_HIGH		COMMUX_PORT->ODR |= COMMUXA_PIN
#define COMMUXB_LOW			COMMUX_PORT->ODR &= ~COMMUXB_PIN
#define COMMUXB_HIGH		COMMUX_PORT->ODR |= COMMUXB_PIN
#define COMMUXC_LOW			COMMUX_PORT->ODR &= ~COMMUXC_PIN
#define COMMUXC_HIGH		COMMUX_PORT->ODR |= COMMUXC_PIN

//------- Definiowanie pinów sterujacych sygnalizatorami do autonomii ------------------------------------
#define SIG0_ON					GPIOA->ODR |= GPIO_ODR_OD12
#define SIG0_OFF				GPIOA->ODR &= ~GPIO_ODR_OD12
#define SIG1_ON					GPIOA->ODR |= GPIO_ODR_OD11
#define SIG1_OFF				GPIOA->ODR &= ~GPIO_ODR_OD11
#define RESETDEV1_ON		GPIOA->ODR &= ~GPIO_ODR_OD9
#define RESETDEV1_OFF		GPIOA->ODR |= GPIO_ODR_OD9
#define RESETDEV2_ON		GPIOA->ODR &= ~GPIO_ODR_OD9
#define RESETDEV2_OFF		GPIOA->ODR |= GPIO_ODR_OD9

void Dev_Conf(void);
void Dev_Act(void);
void Dev_ResetDevice1(void);
void Dev_ResetDevice2(void);

#endif
