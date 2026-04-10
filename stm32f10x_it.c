/**
  ******************************************************************************
  * @file    SPI/SPI_FLASH/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x.h"  // 添加这个头文件

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SPI_FLASH
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// ==================== 添加全局变量 ====================
// 系统计时器变量（每1ms递增一次）
// 注意：这个变量需要在其他文件中使用 extern 声明
volatile u32 system_timer = 0;

// 如果需要在中断中使用 WiFi 接收缓冲区，可以在这里添加外部声明
// extern u8 wifi_recv_buf[];
// extern volatile u16 wifi_recv_cnt;
// extern volatile u8 wifi_recv_over;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    // 每1ms递增一次系统计时器
    system_timer++;
    
    // 注意：如果 system_timer 溢出（约49天后），会自动回绕到0
    // 如果需要更长时间运行，可以添加溢出处理逻辑
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/
/* Add here the Interrupt Handler for the used peripheral(s) (PPP), for the   */
/* available peripheral interrupt handler's name please refer to the startup  */
/* file (startup_stm32f10x_xx.s).                                            */

/**
  * @brief  This function handles USART2 global interrupt.
  * @note   If you are using WiFi module, you need to implement this handler
  * @param  None
  * @retval None
  */
// void USART2_IRQHandler(void)
// {
//     if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
//         u8 data = USART_ReceiveData(USART2);
//         
//         // 这里添加 WiFi 数据接收处理
//         // 例如：extern u8 wifi_recv_buf[];
//         //       extern volatile u16 wifi_recv_cnt;
//         //       extern volatile u8 wifi_recv_over;
//         
//         USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//     }
// }

/**
  * @brief  This function handles USART1 global interrupt.
  * @param  None
  * @retval None
  */
// void USART1_IRQHandler(void)
// {
//     if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//         u8 data = USART_ReceiveData(USART1);
//         // 处理接收数据
//         USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//     }
// }

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/