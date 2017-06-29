#include "lcd.h"
#include "stdlib.h"
#include "delay.h"

//ThatRozen edit
//=======================================液晶屏数据线接线==========================================//
//DB0       接PD14 
//DB1       接PD15 
//DB2       接PD0 
//DB3       接PD1 
//DB4~DB12  依次接PE7~PE15
//DB13      接PD8 
//DB14      接PD9
//DB15      接PD10  
//=======================================液晶屏控制线接线==========================================//
//LCD_CS	接PG12 NE4	//片选信号
//LCD_RS	接PG0	FSMC_A10//寄存器/数据选择信号
//LCD_WR	接PD5	FSMC_NOE//写信号
//LCD_RD	接PD4	FSMC_NOE//读信号
//LCD_RST	接PC5	//复位信号
//LCD_LED	接PB15	//背光控制信号(高电平点亮)
//=========================================触摸屏触接线=========================================//
//不使用触摸或者模块本身不带触摸，则可不连接
//MO(MISO)	接PF8	//SPI总线输出
//MI(MOSI)	接PF9	//SPI总线输入
//PEN		接PF10	//触摸屏中断信号
//TCS		接PB2	//触摸IC片选
//CLK		接PB1	//SPI总线时钟
//********************************************
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

//******************************************************************
//函数名：  LCD_WR_REG
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    向液晶屏总线写入写16位指令
//输入参数：Reg:待写入的指令值
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_WR_REG(u16 data)
{ 
	LCD->LCD_REG=data;//写入要写的寄存器序号	 	
}
//******************************************************************
//函数名：  LCD_WR_DATA
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    向液晶屏总线写入写16位数据
//输入参数：Data:待写入的数据
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_WR_DATA(u16 data)
{
	LCD->LCD_RAM=data;

}
//******************************************************************
//函数名：  LCD_DrawPoint_16Bit
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    写入一个16位数据
//输入参数：(x,y):光标坐标
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_DrawPoint_16Bit(u16 color)
{
	LCD_WR_DATA(color); 
}
//******************************************************************
//函数名：  LCD_WriteReg
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    写寄存器数据
//输入参数：LCD_Reg:寄存器地址
//			LCD_RegValue:要写入的数据
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
{	
	LCD->LCD_REG = LCD_Reg;		//写入要写的寄存器序号	 
	LCD->LCD_RAM = LCD_RegValue;//写入数据	     		 
}		   
	 
	 
//******************************************************************
//函数名：  LCD_WriteRAM_Prepare
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    开始写GRAM
//			在给液晶屏传送RGB数据前，应该发送写GRAM指令
//输入参数：无
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

//******************************************************************
//函数名：  LCD_DrawPoint
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    在指定位置写入一个像素点数据
//输入参数：(x,y):光标坐标
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);
	LCD_WR_DATA(POINT_COLOR); 
}

//******************************************************************
//函数名：  LCD_Clear
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    LCD全屏填充清屏函数
//输入参数：Color:要清屏的填充色
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_Clear(u16 Color)
{	
	u32 index=0; 
	     
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);	

	for(index=0;index<153600;index++)
	{
		LCD->LCD_RAM=Color;		  
	}
} 
//berif : LCD_GPIOInit
//cc		:	Rozen
void LCD_GPIOInit(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
	FSMC_NORSRAMTimingInitTypeDef  writeTiming;
	
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);	//enable fsmc
	//control sign
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG ,ENABLE);
	
	//sign line D0~D1,D8~D15,
	//D4 is LCD_RD,D5 is LCD_WR
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//point D0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);
	//D1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC);
	//D2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FSMC);
	//D3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FSMC);
	//D4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FSMC);
	//D5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FSMC);
	//D6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FSMC);
	//D7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);
	//D8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);
	//D9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);
	//D10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);
	//D11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);
	//D12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC);
	//D13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FSMC);
	//D14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FSMC);
	//D15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);
	
	//LCD_CS	接PG12 NE4	//片选信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource12,GPIO_AF_FSMC);
	
	//LCD_RS	接PG0	FSMC_A10//寄存器/数据选择信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource0,GPIO_AF_FSMC);
	
	//LCD-RD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);
	
	//LCD-WR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC);
	
	//LCD_RST	接PC5	//复位信号
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//LCD_LED	接PB15	//背光控制信号(高电平点亮)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	
	readWriteTiming.FSMC_AddressSetupTime = 0x01;	 //地址建立时间（ADDSET）为2个HCLK 1/36M=27ns
    readWriteTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（ADDHLD）模式A未用到	
    readWriteTiming.FSMC_DataSetupTime = 0x0f;		 // 数据保存时间为16个HCLK,因为液晶驱动IC的读数据的时候，速度不能太快，尤其对1289这个IC。
    readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
    readWriteTiming.FSMC_CLKDivision = 0x00;
    readWriteTiming.FSMC_DataLatency = 0x00;
    readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 
    

	writeTiming.FSMC_AddressSetupTime = 0x02;	 //0x01 地址建立时间（ADDSET）为1个HCLK  
    writeTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（A		
    writeTiming.FSMC_DataSetupTime = 0x05;		 ////0x03 数据保存时间为4个HCLK	
    writeTiming.FSMC_BusTurnAroundDuration = 0x00;
    writeTiming.FSMC_CLKDivision = 0x00;
    writeTiming.FSMC_DataLatency = 0x00;
    writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 

 
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;//  这里我们使用NE4 ，也就对应BTCR[6],[7]。
		FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // 不复用数据地址
		FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM   
		FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//存储器数据宽度为16bit   
		FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
		FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
		FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
		FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
		FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
		FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	//  存储器写使能
		FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
		FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; // 读写使用不同的时序
		FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
		FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming; //读写时序
		FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &writeTiming;  //写时序

		FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //初始化FSMC配置

		FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);  // 使能BANK1 
}
//******************************************************************
//函数名：  LCD_Reset
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    LCD复位函数，液晶初始化前要调用此函数
//输入参数：无
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_RESET(void)
{
	LCD_RST_CLR;
	delay_ms(100);	
	LCD_RST_SET;
	delay_ms(50);
}
//******************************************************************
//函数名：  LCD_Init
//作者：    xiao冯@全动电子
//日期：    2013-02-22
//功能：    LCD初始化
//输入参数：无
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_Init(void)
{  
     										 
	LCD_GPIOInit();
 	LCD_RESET();

	//************* Start Initial Sequence **********//		
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xC1); 
	LCD_WR_DATA(0X30); 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x7A); 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA(0x1B);   //VRH[5:0] 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA(0x01);   //SAP[2:0];BT[3:0] 
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA(0x30); 	 //3F
	LCD_WR_DATA(0x30); 	 //3C
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA(0XB7); 
	LCD_WR_REG(0x36);    // Memory Access Control 
	LCD_WR_DATA(0x48); 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA(0x55); 
	LCD_WR_REG(0xB1);   
	LCD_WR_DATA(0x00);   
	LCD_WR_DATA(0x1A); 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA(0x01); 
	LCD_WR_REG(0xE0);    //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x2A); 
	LCD_WR_DATA(0x28); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x0E); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x54); 
	LCD_WR_DATA(0XA9); 
	LCD_WR_DATA(0x43); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 		 
	LCD_WR_REG(0XE1);    //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x15); 
	LCD_WR_DATA(0x17); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x06); 
	LCD_WR_DATA(0x2B); 
	LCD_WR_DATA(0x56); 
	LCD_WR_DATA(0x3C); 
	LCD_WR_DATA(0x05); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_REG(0x2B); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);	 
	LCD_WR_REG(0x11); //Exit Sleep
	delay_ms(120);
	LCD_WR_REG(0x29); //display on	


	LCD_SetParam();//设置LCD参数	 
	ILI9341G_BackLed_Control(ENABLE);//点亮背光	 
	LCD_Clear(WHITE);
}
/*************************************************
函数名：LCD_SetWindows
功能：设置lcd显示窗口，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);	

	LCD_WriteRAM_Prepare();	//开始写入GRAM				
}   
/*************************************************
函数名：LCD_SetCursor
功能：设置光标位置
入口参数：xy坐标
返回值：无
*************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(Xpos>>8);
	LCD_WR_DATA(0x00FF&Xpos);		

	
	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(Ypos>>8);
	LCD_WR_DATA(0x00FF&Ypos);		

	LCD_WriteRAM_Prepare();	//开始写入GRAM	
} 

//设置LCD参数
//方便进行横竖屏模式切换
void LCD_SetParam(void)
{ 	
	lcddev.wramcmd=0x2C;
#if USE_HORIZONTAL==1	//使用横屏	  
	lcddev.dir=1;//横屏
	lcddev.width=320;
	lcddev.height=240;
	lcddev.setxcmd=0x2A;
	lcddev.setycmd=0x2B;			
	LCD_WriteReg(0x36,0x6C);

#else//竖屏
	lcddev.dir=0;//竖屏				 	 		
	lcddev.width=240;
	lcddev.height=320;
	lcddev.setxcmd=0x2A;
	lcddev.setycmd=0x2B;	
	LCD_WriteReg(0x36,0xC9);
#endif
}	 
void ILI9341G_BackLed_Control ( FunctionalState enumState )
{
	if ( enumState )
		 GPIO_SetBits( GPIOB, GPIO_Pin_15 );	
	else
		 GPIO_ResetBits( GPIOB, GPIO_Pin_15 );
		
}
