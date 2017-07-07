#include "ov7670.h"
#include "delay.h"
#include "lcd.h"
#include "usart.h"

volatile uint16_t frame_buffer[IMG_ROWS];
/*give OV7670 clock*/
void PCLCK_Init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);//ʹ��DCMI��GPIOʱ��
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);//MCO1:PA8
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	     
    RCC_MCO2Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_3);
}
void Cam_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
  	DCMI_InitTypeDef DCMI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);//DCMI_HSYNC 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);//DCMI_PIXCLK
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);//DCMI_D5 			  
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);//DCMI_VSYNC 
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI);//DCMI_D6 
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI);//DCMI_D7 			  
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);//DCMI_D0 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);//DCMI_D1 			  
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI);//DCMI_D2 
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI);//DCMI_D3 
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI);//DCMI_D4 		
	
  	
		


  	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);//DCMI 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | 
                           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��DCMI��GPIOʱ��
	



   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ; 
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6; 
    GPIO_Init(GPIOE, &GPIO_InitStructure);     
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);		 

  	
		DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
		DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
		DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
		DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
		DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
		DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
		DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
		DCMI_Init(&DCMI_InitStructure);
		
		/* DCMI Interrupts config ***************************************************/
		DCMI_ITConfig(DCMI_IT_FRAME,ENABLE);//����֡�ж� 
		DCMI_ITConfig(DCMI_IT_LINE,ENABLE); //�������ж�
		DCMI_ITConfig(DCMI_IT_VSYNC,ENABLE); //�������ж�	
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		

		//DMA_Cmd(DMA2_Stream1, ENABLE);
		DCMI_Cmd(ENABLE);
		DCMI_CaptureCmd(ENABLE);
}
void DMA_init(u32 DMA_Memory0BaseAddr,u16 DMA_BufferSize,u32 DMA_MemoryDataSize,u32 DMA_MemoryInc){
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//DMA2
	
		 /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  //ͨ��1 DCMIͨ�� 
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&DCMI->DR;//�����ַΪ:DCMI->DR
  DMA_InitStructure.DMA_Memory0BaseAddr = DMA_Memory0BaseAddr;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��ģʽ
  DMA_InitStructure.DMA_BufferSize = DMA_BufferSize;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//�������ݳ���:32λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize;//�洢�����ݳ��� 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// ʹ��ѭ��ģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//�����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable; //FIFOģʽ        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//ʹ��ȫFIFO 
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//����ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//�洢��ͻ�����δ���
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);//��ʼ��DMA Stream
		
		
	// DMA Stream enable
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	//DMA_ITConfig(DMA2_Stream1, DMA_IT_TE, ENABLE);	
	
		/* DMA2 IRQ channel Configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		

		
}

u8 OV7670_Init(void)
{
  u8 i;
	PCLCK_Init();
	
  Cam_Init();
	SCCB_Init();
	DMA_init((u32)&LCD->LCD_RAM,10,DMA_MemoryDataSize_HalfWord,DMA_MemoryInc_Disable);
	OV_Reset();
	delay_ms(10);
	printf("\rreturn camer Lw x%x\n",OV_ReadID_L());
	//OV_ReadID_L();
	//OV_Reset();
	delay_ms(10);
	printf("\rreturn camer Hi x%x\n",OV_ReadID_H());
	//OV_ReadID_H();
	//LCD_Num(180,50,OV_ReadID(),3,WHITE);//ov7670 IDΪ0x73
  	for(i=0;i<OV7670_REG_NUM;i++)
  	{
    	if(OV_WriteReg(OV7670_reg[i][0],OV7670_reg[i][1])){
				printf("\r WriteReg failed!\n");
				return 1;
			}
  	}
		printf("\r Register config success\n");
	return 0; 
}

void Cam_Start(void)
{
		LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);	
  	//LCD_WriteReg(0x03,0x1018);
  	//LCD_SetCursor(0,319);  
	//int index;
	
	//LCD_Scan_Dir(U2D_L2R);		   //���ϵ���,������	
	//LCD_SetWindows(0,0,320,240); //LCD������ʾ���ڣ�����ı��˷ֱ��ʣ�������Ҫ����
	LCD_SetCursor(0,0);  
	LCD_WriteRAM_Prepare();			//��ʼд��GRAM
	DMA_Cmd(DMA2_Stream1, ENABLE);//����DMA2,Stream1 
	DCMI_CaptureCmd(ENABLE);//DCMI����ʹ��  
	
}
								  
void OV7670_HW(u16 hstart,u16 vstart,u16 hstop,u16 vstop)
{
	u8 v;		
	OV_WriteReg(0x17,(hstart>>3)&0xff);//HSTART
	OV_WriteReg(0x18,(hstop>>3)&0xff);//HSTOP
	OV_ReadReg(0x32,&v);
	v=(v&0xc0)|((hstop&0x7)<<3)|(hstart&0x7);
	OV_WriteReg(0x32,v);//HREF
	
	OV_WriteReg(0x19,(vstart>>2)&0xff);//VSTART ��ʼ��8λ
	OV_WriteReg(0x1a,(vstop>>2)&0xff);//VSTOP	������8λ
	OV_ReadReg(0x03,&v);
	v=(v&0xf0)|((vstop&0x3)<<2)|(vstart&0x3);	
	OV_WriteReg(0x03,v);//VREF																 
	OV_WriteReg(0x11,0x00);
}								
/////////////////////////////////////
void SCCB_Init(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure;
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  //SCCB_SIC:PB10
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			  //SCCB_SID:PB11
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SCCB_SID_OUT(void)//����SCCB_SIDΪ���
{
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;               
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void SCCB_SID_IN(void)//����SCCB_SIDΪ����
{
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;               
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SCCB_Start(void)
{
    SCCB_SID_H();     //�����߸ߵ�ƽ
    delay_us(50);
    SCCB_SIC_H();	   //��ʱ���߸ߵ�ʱ���������ɸ�����
    delay_us(50);
    SCCB_SID_L();
    delay_us(50);
    SCCB_SIC_L();	 //�����߻ָ��͵�ƽ��������������Ҫ
    delay_us(50);
}

void SCCB_Stop(void)
{
    SCCB_SID_L();
    delay_us(50);
    SCCB_SIC_H();	
    delay_us(50);  
    SCCB_SID_H();	
    delay_us(50);  
}

void noAck(void)
{	
	SCCB_SID_H();	
	delay_us(50);	
	SCCB_SIC_H();	
	delay_us(50);	
	SCCB_SIC_L();	
	delay_us(50);	
	SCCB_SID_L();	
	delay_us(50);
}

u8 SCCB_Write(u8 m_data)
{
	u8 j,tem;

	for(j=0;j<8;j++) //ѭ��8�η�������
	{
		if((m_data<<j)&0x80)SCCB_SID_H();
		else SCCB_SID_L();
		delay_us(500);
		SCCB_SIC_H();	
		delay_us(500);
		SCCB_SIC_L();	
		delay_us(500);
	}
	delay_us(100);
	SCCB_DATA_IN;
	delay_us(500);
	SCCB_SIC_H();	
	delay_us(100);
	if(SCCB_SID_STATE)tem=0;//SDA=1����ʧ��
	else tem=1;//SDA=0���ͳɹ�������1
	SCCB_SIC_L();	
	delay_us(500);	
    SCCB_DATA_OUT;
	return tem;  
}

u8 SCCB_Read(void)
{
	u8 read,j;
	read=0x00;
	
	SCCB_DATA_IN;
	delay_us(500);
	for(j=8;j>0;j--) //ѭ��8�ν�������
	{		     
		delay_us(500);
		SCCB_SIC_H();
		delay_us(500);
		read=read<<1;
		if(SCCB_SID_STATE)read=read+1; 
		SCCB_SIC_L();
		delay_us(500);
	}	
    SCCB_DATA_OUT;
	return read;
}

//дOV7670�Ĵ���
u8 OV_WriteReg(u8 regID, u8 regDat)
{
	SCCB_Start();//����SCCB ���߿�ʼ��������
	if(SCCB_Write(0x42)==0)//д��ַ
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 1;//���󷵻�
	}
	delay_us(10);
  	if(SCCB_Write(regID)==0)//������ID
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 2;//���󷵻�
	}
	delay_us(10);
  	if(SCCB_Write(regDat)==0)//д���ݵ�������
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 3;//���󷵻�
	}
  	SCCB_Stop();//����SCCB ����ֹͣ��������	
  	return 0;//�ɹ�����
}

//��OV7660�Ĵ���
u8 OV_ReadReg(u8 regID, u8 *regDat)
{
	//ͨ��д�������üĴ�����ַ
	SCCB_Start();
	if(SCCB_Write(0x42)==0)//д��ַ
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 1;//���󷵻�
	}
	delay_us(1000);
  	if(SCCB_Write(regID)==0)//������ID
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 2;//���󷵻�
	}
	SCCB_Stop();//����SCCB ����ֹͣ��������	
	delay_us(1000);	
	//���üĴ�����ַ�󣬲��Ƕ�
	SCCB_Start();
	if(SCCB_Write(0x43)==0)//����ַ
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 3;//���󷵻�
	}
	delay_us(10);
  	*regDat=SCCB_Read();//���ض�����ֵ
  	noAck();//����NACK����
  	SCCB_Stop();//����SCCB ����ֹͣ��������
  	return 0;//�ɹ�����
}

void OV_Reset(void)
{
	OV_WriteReg(0x12,0x80);
}

u8 OV_ReadID_L(void)
{
	u8 temp;
	OV_ReadReg(0x0b,&temp);
  	return temp;
}
u8 OV_ReadID_H(void)
{
	u8 temp;
	OV_ReadReg(0x0a,&temp);
  	return temp;
}
