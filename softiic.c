#include "softiic.h"

/*************************
函数名称：IIC_Start
函数功能：模拟IIC起始信号
函数入口：无
函数出口：无
函数作者：张一凡
其他补充：
*************************/
void IIC_Start(void)
{
	SOFT_SCL(1);
	SOFT_SDA(1);
	SOFT_SDA(0);
	SOFT_SCL(0);
}

/*************************
函数名称：IIC_Stop
函数功能：模拟IIC停止信号
函数入口：无
函数出口：无
函数作者：张一凡
其他补充：
*************************/
void IIC_Stop(void)
{
	SOFT_SCL(1);
	SOFT_SDA(0);
	SOFT_SDA(1);
	
}

/*************************
函数名称：IIC_WaitAck
函数功能：模拟IIC等待应答信号
函数入口：无
函数出口：无
函数作者：张一凡
其他补充：
*************************/
void IIC_WaitAck(void)
{
	SOFT_SCL(1);
	SOFT_SCL(0);
}

/*************************
函数名称：IIC_WriteByte
函数功能：模拟IIC发送一个字节
函数入口：无
函数出口：无
函数作者：张一凡
其他补充：
*************************/
void IIC_WriteByte(uint8_t byte)
{
    uint8_t i=0, temp=0, data=0;
	
    data = byte;
    SOFT_SCL(0);
    for(i=0;i<8;i++)
    {
        temp = data;
        temp = temp & 0x80;
        if(temp == 0x80) SOFT_SDA(1);
        else SOFT_SDA(0);
        data = data << 1;
        SOFT_SCL(1);
        SOFT_SCL(0);
    }
}

/*************************
函数名称：SoftIIC_Config
函数功能：模拟IIC初始化
函数入口：无
函数出口：无
函数作者：张一凡
其他补充：
*************************/
void SoftIIC_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure = {0};
	
 	IICSCL_ClockCmd(IICSCL_Periph, ENABLE);
	IICSDA_ClockCmd(IICSDA_Periph, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = IICSCL_PIN;	 
 	GPIO_Init(IICSCL_PORT, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = IICSDA_PIN;	 
 	GPIO_Init(IICSDA_PORT, &GPIO_InitStructure);
	
	SOFT_SCL(1);
	SOFT_SDA(1);
}

/*************************
函数名称：IIC_WriteCommand
函数功能：模拟IIC写指令函数
函数入口：无
函数出口：无
函数作者：张一凡
其他补充：
*************************/
void IIC_WriteCommand(uint8_t command)
{
	IIC_Start();
	IIC_WriteByte(0x78);     //Slave address,SA0=0
	IIC_WaitAck();	
	IIC_WriteByte(0x00);		//write command
	IIC_WaitAck();	
	IIC_WriteByte(command); 
	IIC_WaitAck();	
	IIC_Stop();
}

/*************************
函数名称：IIC_WriteData
函数功能：模拟IIC写数据函数
函数入口：无
函数出口：无
函数作者：张一凡
其他补充：
*************************/
void IIC_WriteData(uint8_t data)
{
	IIC_Start();
	IIC_WriteByte(0x78);			//D/C#=0; R/W#=0
	IIC_WaitAck();	
	IIC_WriteByte(0x40);			//write data
	IIC_WaitAck();	
	IIC_WriteByte(data);
	IIC_WaitAck();	
	IIC_Stop();
}
