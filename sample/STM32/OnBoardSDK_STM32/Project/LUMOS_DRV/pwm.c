#include "pwm.h"

void TriggerInit()
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	
//	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

//	GPIO_SetBits(GPIOC,GPIO_Pin_13);
}

void TIM3_PWM_Init(int psc,int prd)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef				 TIM3_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


	//PWM1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;               
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM3_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM3_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM3_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM3_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
	TIM3_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM3_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM3_OCInitStructure.TIM_Pulse=0;
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM3, &TIM3_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM3_OCInitStructure);        
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &TIM3_OCInitStructure);  
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM3_OCInitStructure);  	
	TIM_Cmd(TIM3, ENABLE);
	PWMC1=930;
	PWMC2=850;
}
void TIM4_PWM_Init(int psc,int prd)		//cannot work
{
		GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef				 TIM4_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//PWM1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;               
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM4_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM4_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM4_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM4_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
	TIM4_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM4_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM4_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM4, &TIM4_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM4_OCInitStructure);        
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM4_OCInitStructure);  
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM4_OCInitStructure);  	
	TIM_Cmd(TIM4, ENABLE);

}

void TIM1_PWM_Init(int psc,int prd)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef				 TIM1_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	//PWM1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler         = psc;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period            = prd;               
	TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM1_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM1_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM1_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;         
	TIM1_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
	TIM1_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM1_OCInitStructure.TIM_Pulse=0;
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM1, &TIM1_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM1_OCInitStructure);        
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM1_OCInitStructure);  
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &TIM1_OCInitStructure);  	
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);			//important difference from normal tim!!!
}


void pwmtest()
{
	PWMA1=500;
	PWMA2=1500;
	PWMA3=2000;
}

