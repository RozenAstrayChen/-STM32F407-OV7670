
#include "stm32f4xx.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "lcd.h"
#include "ov7670.h"
#include "test.h"
#include "timer.h"
#include <stdbool.h>
u8 ov_frame;
static volatile bool frame_flag = false;
static volatile bool send_sync_frame = false;
volatile uint8_t temp_buffer[IMG_ROWS * IMG_COLUMNS];

/*************************************************************************************
  * brief：main()
  * param    ：void
  * retval  ：void
  * description    ：main
  *************************************************************************************/
int main(void)
{ 
	//SystemInit();			                     //CLOCK is 168Mhz
	LED_GPIO_Conf();										   
	SysTick_Init();
	LCD_Init();
	main_test();
  USART1_Conf();
	TIM3_Int_Init(10000-1,8400-1);//10Khz计数,1秒钟中断一次
  
  delay_ms(100);

 	printf("\r start init OV7670\r\n");

 	//LCD_String(20,20,"DCMI Camera demo",RED);
	if(OV7670_Init())
	{
		//LCD_String(20,50,"OV7670 init failed!",RED);
		while(1)
		{
			LED0(On);
			delay_ms(300);
			LED0(Off);
			delay_ms(300);
			
		}
	}
	delay_ms(1000);
	LCD_Clear(BLUE);
	Cam_Start();
  //while(1){
		//main_test();
		//LED1(On);
		//delay_ms(300);
		//LED1(Off);
		//delay_ms(300);
		
	
	// Infinite program loop
	while (1) {
		
	}
}
/**
  * @brief   DMA receive
  * @param  None
  * @retval None
  */
void DMA2_Stream1_IRQHandler(void)
{        
	if(DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1)==SET)//DMA2_Steam1,传输完成标志
	{  
		 DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);//清除传输完成中断
			//datanum++;
	}    											 
}  
/**
  * @brief   DCMIReceive data
  * @param  None
  * @retval None
  */
void DCMI_IRQHandler(void) {
		int i=0;
		if (DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET) { // Frame received
			DCMI_ClearFlag(DCMI_FLAG_FRAMERI);
			// After receiving a full frame we disable capture and the DMA transfer. This is probably a very inefficient way of capturing and sending frames
			// but it's the only way I've gotten to reliably work.
			//DMA_Cmd(DMA2_Stream1, DISABLE);
			//DCMI_Cmd(DISABLE);
			//DCMI_CaptureCmd(DISABLE);
			
	}if (DCMI_GetITStatus(DCMI_IT_VSYNC) != RESET)
    {
        
        DCMI_ClearITPendingBit(DCMI_IT_VSYNC);
				
   }if (DCMI_GetITStatus(DCMI_IT_LINE) != RESET)
    {
        DCMI_ClearITPendingBit(DCMI_IT_LINE);
				ov_frame++;//fps++
    }
    if (DCMI_GetITStatus(DCMI_IT_ERR) != RESET)
    {
        DCMI_ClearITPendingBit(DCMI_IT_ERR);
    }
}

/*void USART2_IRQHandler(void) {
	// Wait until the host sends us a S before sending the first sync frame
	if (Serial_read() == 0xbb) {
		send_sync_frame = true;
	}
	USART_ClearFlag(USART2, USART_FLAG_RXNE);

}*/

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

