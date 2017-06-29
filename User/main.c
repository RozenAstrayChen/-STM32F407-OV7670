
#include "stm32f4xx.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "lcd.h"
#include "ov7670.h"
#include "test.h"
#include <stdbool.h>

static volatile bool frame_flag = false;
static volatile bool send_sync_frame = false;
volatile uint8_t temp_buffer[IMG_ROWS * IMG_COLUMNS];
/*************************************************************************************
  * 函数名称：dumpFrame
  * 参数    ：void
  * 返回值  ：void
  * 描述    ：receive image
  *************************************************************************************/
void dumpFrame(void) {

	uint8_t *buffer = (uint8_t *) frame_buffer;
	int length = IMG_ROWS * IMG_COLUMNS * 2;
	// Copy every other byte from the main frame buffer to our temporary buffer (this converts the image to grey scale)
	int i;
	for (i = 1; i < length; i += 2) {
		temp_buffer[i / 2] = buffer[i];
	}
	// We only send the sync frame if it has been requested
	if (send_sync_frame) {
		for (i = 0x7f; i > 0; i--) {
			uint8_t val = i;
			//Serial_sendb(&val);
			printf("%s",&val);
		}
		send_sync_frame = false;
	}

	for (i = 0; i < (length / 2); i++) {
		if (i > 100) {
			//Serial_sendb(&temp_buffer[i]);
			printf("%s",&temp_buffer[i]);
		} else {
			uint8_t val = 0xff;
			//Serial_sendb(&val); // Change first 100 pixels to white to provide a reference for where the frame starts
			printf("%s",&val);
		}
	}
	// Enable capture and DMA after we have sent the photo. This is a workaround for the timing issues I've been having where
	// the DMA transfer is not in sync with the frames being sent
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
}
/*************************************************************************************
  * brief：main()
  * param    ：void
  * retval  ：void
  * description    ：main
  *************************************************************************************/
int main(void)
{ 
	SystemInit();			                     //CLOCK is 168Mhz
	LED_GPIO_Conf();										   
	SysTick_Init();
	LCD_Init();
  USART1_Conf();
  
  delay_ms(100);

 	printf("\r 123！\r\n");

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
  /*while(1){
		//main_test();
		LED1(On);
		delay_ms(300);
		LED1(Off);
		delay_ms(300);
		
	}*/
	// Infinite program loop
	while (1) {
		if (frame_flag == true) {
			frame_flag = false;
			dumpFrame();
		}
	}
}

void DCMI_IRQHandler(void) {
	if (DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET) { // Frame received
		DCMI_ClearFlag(DCMI_FLAG_FRAMERI);
		// After receiving a full frame we disable capture and the DMA transfer. This is probably a very inefficient way of capturing and sending frames
		// but it's the only way I've gotten to reliably work.
		DMA_Cmd(DMA2_Stream1, DISABLE);
		DCMI_Cmd(DISABLE);
		DCMI_CaptureCmd(DISABLE);
	}
	if (DCMI_GetFlagStatus(DCMI_FLAG_OVFRI) == SET) { // Overflow
		// Not used, just for debug
		DCMI_ClearFlag(DCMI_FLAG_OVFRI);
	}
	if (DCMI_GetFlagStatus(DCMI_FLAG_ERRRI) == SET) { // Error
		// Not used, just for debug
		DCMI_ClearFlag(DCMI_FLAG_ERRRI);
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

