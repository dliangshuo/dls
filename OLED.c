#include "oled.h"

/****************************
函数名称：OLED_ReadWriteByte
函数作用：OLED数据读写
函数参数：
		Data	发送数据或命令
		Cmd		数据/命令选择（OLED_CMD、OLED_DATA）
函数出口：
		Temp	接收数据
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
****************************/
uint8_t OLED_ReadWriteByte(uint8_t Data,OLED_MODE Cmd)
{
	uint8_t Temp = 0X00;
	
	if(Cmd)
		IIC_WriteData(Data);
	else 
		IIC_WriteCommand(Data);
	
	return Temp;
}

/****************************
函数名称：OLED_Config
函数作用：OLED初始化
函数参数：无
函数出口：无
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
****************************/
void OLED_Config(void)
{
	//管脚初始化
	SoftIIC_Config();
	
	//OLED驱动编程
	OLED_ReadWriteByte(0XAE,OLED_CMD);	//开显示屏
	//设置起始地址
	OLED_ReadWriteByte(0x00,OLED_CMD); 	//低
	OLED_ReadWriteByte(0x10,OLED_CMD); 	//高
	OLED_ReadWriteByte(0x40,OLED_CMD); 	//设置起始行：0
	OLED_ReadWriteByte(0x81,OLED_CMD); 	//亮度，后接亮度
	OLED_ReadWriteByte(0xFF,OLED_CMD);
	OLED_ReadWriteByte(0x22,OLED_CMD); 	//页寻址模式
	OLED_ReadWriteByte(0xB0,OLED_CMD); 	//设置起始页：0
	OLED_ReadWriteByte(0x7F,OLED_CMD); 	//0~128
	OLED_ReadWriteByte(0xA1,OLED_CMD); 	//左右
	OLED_ReadWriteByte(0xA6,OLED_CMD); 	//正反向
	OLED_ReadWriteByte(0xA8,OLED_CMD); /*multiplex ratio*/
	OLED_ReadWriteByte(0x3F,OLED_CMD); /*duty = 1/64*/
	OLED_ReadWriteByte(0xC8,OLED_CMD); /*Com scan direction上下*/
	OLED_ReadWriteByte(0xD3,OLED_CMD); /*set display offset*/
	OLED_ReadWriteByte(0x00,OLED_CMD);
	OLED_ReadWriteByte(0xD5,OLED_CMD); /*set osc division*/
	OLED_ReadWriteByte(0x80,OLED_CMD);
	OLED_ReadWriteByte(0xD9,OLED_CMD); /*set pre-charge period*/
	OLED_ReadWriteByte(0x1f,OLED_CMD);
	OLED_ReadWriteByte(0xDA,OLED_CMD); /*set COM pins*/
	OLED_ReadWriteByte(0x12,OLED_CMD);
	OLED_ReadWriteByte(0xdb,OLED_CMD); /*set vcomh*/
	OLED_ReadWriteByte(0x30,OLED_CMD);
	OLED_ReadWriteByte(0x8d,OLED_CMD); /*set charge pump enable*/
	OLED_ReadWriteByte(0x14,OLED_CMD);
	OLED_ReadWriteByte(0xAF,OLED_CMD); /*display ON*/
	
	OLED_Clear(OLED_CLOSE);
}

/****************************
函数名称：OLED_SetPos
函数作用：OLED设置坐标点
函数参数：
		x		列
		y		页
函数出口：无
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
****************************/
void OLED_SetPos(uint16_t x, uint16_t y)
{
	//设置页地址和列地址
	OLED_ReadWriteByte(0xB0+x,OLED_CMD);								//设置页地址
	OLED_ReadWriteByte(0x00 + (y & 0x0f)		 ,OLED_CMD);//列地址低4位
	OLED_ReadWriteByte(0x10 + ((y & 0xf0)>>4),OLED_CMD);//列地址高4位
}

/****************************
函数名称：OLED_Clear
函数作用：OLED清屏
函数参数：无
函数出口：无
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
****************************/
void OLED_Clear(uint8_t data)
{
  uint8_t i = 0;
  uint8_t j = 0;
	
  for(i=0;i<OLED_PAGE;i++)
  {
		OLED_SetPos(i,0);
    for(j=0;j<OLED_WIGH;j++)
    {
      //写数据
      OLED_ReadWriteByte(data,OLED_DATA);
    }
  }
}

/****************************
函数名称：OLED_ShowASCII
函数作用：OLED显示字符
函数参数：
	page	页数（0~7）
	line	列数（0~127）
	buf	字模数组
函数出口：无
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
补充说明：
	字符取模（PCtoLCD2002）:阴码 列行 逆向 C51
		阳码：背景为1 阴码：字体为1
****************************/
void OLED_ShowASCII(uint8_t page,uint8_t line,uint8_t *buf)
{
	uint8_t i=0,j=0;
	
	for(i=0;i<2;i++)
	{
		OLED_SetPos(page+i,line);
		for(j=0;j<8;j++)
		{
			OLED_ReadWriteByte(*buf,OLED_DATA);
			buf++;
		}
	}
}

/****************************
函数名称：OLED_ShowChinese
函数作用：OLED显示汉字
函数参数：
	page	页数（0~7）
	line	列数（0~127）
	buf		字模数组
函数出口：无
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
补充说明：
	字符取模（PCtoLCD2002）:阴码 列行 逆向 C51
		阳码：背景为1 阴码：字体为1
****************************/
void OLED_ShowChinese(uint8_t page,uint8_t line,uint8_t *buf)
{
	uint8_t i=0,j=0;
	
	for(i=0;i<2;i++)
	{
		OLED_SetPos(page+i,line);
		for(j=0;j<16;j++)
		{
			OLED_ReadWriteByte(*buf,OLED_DATA);
			buf++;
		}
	}
}

/****************************
函数名称：OLED_ShowPhoto
函数作用：OLED显示图片
函数参数：
	page	页数（0~7）
	line	列数（0~127）
	High	图片高度
	Wide	图片宽度
	buf		字模数组
函数出口：无
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
补充说明：
	图片取模（Img2LCD）:
		数据水平，字节垂直
		单色
		字节内像素反序
****************************/
void OLED_ShowPhoto(uint8_t page,uint8_t line,uint8_t High,uint8_t Wide,uint8_t *buf)
{
	uint8_t i=0,j=0,temp_high = 0;
	
	if((High%8) == 0)
		temp_high = High/8;
	else
		temp_high = High/8 + 1;
	
	for(i=0;i<temp_high;i++)
	{
		OLED_SetPos(page+i,line);
		for(j=0;j<Wide;j++)
		{
			OLED_ReadWriteByte(*buf,OLED_DATA);
			buf++;
		}
	}
}

/****************************
函数名称：OLED_ShowString
函数作用：OLED显示字符串
函数参数：
	page	页数（0~7）
	line	列数（0~127）
	str		字模数组
函数出口：无
函数作者：张一凡
创建时间：2021.04.23
修改时间：2021.04.23
****************************/

void OLED_ShowString(uint8_t page,uint8_t line,uint8_t *str)
{
	uint8_t Temp_page = 0,Temp_line = 0;
	int Lib_offset = 0;
	Temp_page = page;
	Temp_line = line;
//	uint8_t Temp_LibBuf[33] = "\0";
	
	while(*str != '\0')//判断字符串的结束
	{
		//判断字符是中文还是英文
		if(*str < 128)//ASCII码表内为英文
		{
			//判断显示空间够不够
			if(Temp_line > 120)
			{
				Temp_page += 2;
				Temp_line = line;
			}
				
			Lib_offset = ASCII_GetOffset(*str);
			if(Lib_offset != (-1))
				OLED_ShowASCII(Temp_page,Temp_line,&ASCIIFont_Lib[Lib_offset*16]);
			else
			{
				if(*str == 10)
					Temp_page += 2;
					Temp_line = line;
			}
			Temp_line+=8;
			str++;
		}
		else//其他为中文
		{
			//判断显示空间够不够
			if(Temp_line > 112)
			{
				Temp_page += 2;
				Temp_line = line;
			}
			
//			if(FlashFontLib_Flag == 1)//Flash有字库
//			{
//				Lib_offset = ((*str -0XA1)*94 + (*(str+1) - 0XA1))*32;
//				sFLASH_ReadBuffer(Temp_LibBuf,FONT_ADDR+Lib_offset,32);
//				OLED_ShowChinese(Temp_page,Temp_line,Temp_LibBuf);
//			}
//			else
//			{
				Lib_offset = Chinese_GetOffset(*str,*(str+1));
				if(Lib_offset != (-1))
					OLED_ShowChinese(Temp_page,Temp_line,&ChineseFont_Lib[Lib_offset*32]);
//			}
			Temp_line+=16;
			str+=2;
		}
	}
}

