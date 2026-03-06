/**
 * @file judge_task.cpp
 * @brief Referee system RX task and USART3 idle interrupt DMA rearm.
 */

#include "judge_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdbool.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
extern DMA_HandleTypeDef hdma_usart3_rx;

/*裁判系统发过来的数据暂存在这里*/
uint8_t Judge_Buffer[JUDGE_BUFFER_LEN] = {0};
uint8_t usart1_rx_flag;

void Judge_Task(void const * argumt)
{
	  // Keep parsing referee frames at a fixed period.
	  usart3_init();
		vTaskDelay(20);
	  uint32_t currentTime;
		while(1)
		{
		currentTime = xTaskGetTickCount();//当前系统时间			
		Judge_Read_Data(Judge_Buffer);		//读取裁判系统数据	
		vTaskDelayUntil(&currentTime, 2);
		}
}

extern "C"
{
/*串口3中断函数*/
void USART3_IRQHandler(void)
{	
	 // IDLE line indicates one frame burst is received, then restart DMA window.
	 if(USART3->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
		}	
	  __HAL_DMA_DISABLE(huart3.hdmarx);
	//????????????????????????????????????
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);
//	__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5 );
	
//    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
//    {
//       HAL_UART_Receive_DMA(&huart3,Judge_Buffer,JUDGE_BUFFER_LEN);
//    }
//	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    __HAL_DMA_SET_COUNTER(&hdma_usart3_rx, JUDGE_BUFFER_LEN);

		__HAL_DMA_ENABLE(huart3.hdmarx);
		HAL_UART_Receive_DMA(&huart3,Judge_Buffer,JUDGE_BUFFER_LEN);		
//				Judge_Read_Data(Judge_Buffer);
}

}
