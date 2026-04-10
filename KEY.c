#include "key.h"

/**************************************
函数名称：KEY_Config
函数功能：按键的底层驱动初始化
函数入口：无
函数出口：无
函数作者：
版本迭代：
其他说明：
	WKUP    PA0        下拉输入模式
	KEY0    PE4        上拉输入模式
	KEY1    PE3        上拉输入模式
**************************************/



	void KEY_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  // 使能GPIOE时钟
    
    // KEY1（PE3）配置：上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入，未按下时为高电平
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    // KEY0（PE4）配置：上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}




/**************************************
函数名称：KEY_Config
函数功能：按键的底层驱动初始化
函数入口：无
函数出口：
	0
	1		WKUP点击
	2		KEY0点击
	3		KEY1点击
函数作者：
版本迭代：
其他说明：
**************************************/
uint8_t KEY_GetVal(void)
{
	if(WKUP_State)//按键可能被人按下了
	{
		DELAY_Nus(10);//等一会儿--让他去闹一会儿
		if(WKUP_State)//这是真有人按下了
		{
			while(WKUP_State);//松手检测
			return 1;
		}
	}
	if(!KEY0_State)//按键可能被人按下了
	{
		DELAY_Nus(10);//等一会儿--让他去闹一会儿
		if(!KEY0_State)//这是真有人按下了
		{
			while(!KEY0_State);//松手检测
			return 2;
		}
	}
	if(!KEY1_State)//按键可能被人按下了
	{
		DELAY_Nus(10);//等一会儿--让他去闹一会儿
		if(!KEY1_State)//这是真有人按下了
		{
			while(!KEY1_State);//松手检测
			return 3;
		}
	}
	
	return 0;
}

