#include "led.h"

/******************************************************************************************
*��������LED_GPIO_Conf()
* ������void
* ����ֵ��void
* ���ܣ�LED�Ĺܽ�����
*********************************************************************************************/
void LED_GPIO_Conf(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;		                                              //����GPIO��ʼ���ṹ��
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);			  					  //ʹ��GPIOB��AHP1ʱ��
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;										  //���ó�ʼ��ģʽΪ���ģʽ
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;										  //�����������Ϊ�������
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;									  //��������ٶ�Ϊ100Mhz
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;										  
   GPIO_InitStructure.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);								  //�����ʼ���Ĺܽ�ΪPin14	Pin15
   GPIO_Init(GPIOF, &GPIO_InitStructure);												  //��ʼ��GPIOB��Pin_14�ܽ�
   GPIO_SetBits(GPIOF,GPIO_Pin_9);														  //ʹGPIOB��Pin_14����ߵ�ƽ
   GPIO_SetBits(GPIOF,GPIO_Pin_10);														  //ʹGPIOB��Pin_15����ߵ�ƽ
}
