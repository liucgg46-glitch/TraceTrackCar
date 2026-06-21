/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "bsp_timer.h"

extern volatile uint32_t sys_tick;

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

#include "stm32f4xx.h"
#include "bsp_uart.h"

/* USART1 空闲中断：用于判断一帧接收结束 */
void USART1_IRQHandler(void)
{
    uint32_t tmp;
    uint16_t len;

    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {

        /* 停止 DMA */
        DMA_Cmd(DMA2_Stream2, DISABLE);
        while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE) {
            ;
        }

        /* 清 IDLE 标志 */
        tmp = USART1->SR;
        tmp = USART1->DR;
        (void)tmp;

        /* 计算接收长度 */
        len = UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2);

        if (len > 0) {
            UART1_RX_IdleCallback(len);
        }

        /* 关键修复：清 DMA 标志  */
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 |
                                       DMA_FLAG_HTIF2 |
                                       DMA_FLAG_TEIF2 |
                                       DMA_FLAG_DMEIF2 |
                                       DMA_FLAG_FEIF2);

        /* 重新装载 DMA */
        DMA2_Stream2->NDTR = UART_RX_BUF_SIZE;
        DMA2_Stream2->M0AR = (uint32_t)uart1_dma_rx_buf;

        /* 重新启动 DMA */
        DMA_Cmd(DMA2_Stream2, ENABLE);
    }
}

/* USART1 发送 DMA 完成中断 */
void DMA2_Stream7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET) {
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
        UART1_TX_DMACallback();
    }
}

void USART2_IRQHandler(void)
{
    uint32_t tmp;
    uint16_t pos;
    uint16_t len;

    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {

        /* 清 IDLE 标志：必须先读 SR 再读 DR */
        tmp = USART2->SR;
        tmp = USART2->DR;
        (void)tmp;

        /* 当前 DMA 写到的位置 */
        pos = UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);

        /* 计算本次新增接收长度 */
        if (pos != uart2_rx_pos) {
            if (pos > uart2_rx_pos) {
                len = pos - uart2_rx_pos;
            } else {
                len = UART_RX_BUF_SIZE - uart2_rx_pos + pos;
            }

            UART2_RX_IdleCallback(len);
        }
    }
}

void DMA1_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET) {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        UART2_TX_DMACallback();
    }
}


/*
 * 把下面 4 个 IRQHandler 加到 stm32f4xx_it.c 里。
 * 注意：你的 UART2_TX 已经占用 DMA1_Stream6，不要把 I2C1_TX 配成 Stream6。
 */

#include "bsp_i2c.h"

void I2C1_EV_IRQHandler(void)
{
    I2C1_EV_ISR();
}

void I2C1_ER_IRQHandler(void)
{
    I2C1_ER_ISR();
}

void DMA1_Stream0_IRQHandler(void)
{
    I2C1_DMA_RX_ISR();
}

void DMA1_Stream7_IRQHandler(void)
{
    I2C1_DMA_TX_ISR();
}



#include "bsp_spi.h"

void DMA2_Stream0_IRQHandler(void)
{
    SPI1_DMA_IRQHandler_RX();
}

void DMA2_Stream3_IRQHandler(void)
{
    SPI1_DMA_IRQHandler_TX();
}

//=======================遥控器测试================
#include "bsp_rc_pwm.h"
void EXTI2_IRQHandler(void)
{
    RC_PWM_EXTI2_IRQHandler();
}

void EXTI3_IRQHandler(void)
{
    RC_PWM_EXTI3_IRQHandler();
}

void EXTI15_10_IRQHandler(void)
{
    RC_PWM_EXTI15_10_IRQHandler();
}

/*
 * Add this include and these IRQHandlers to stm32f4xx_it.c.
 * Do not duplicate an IRQHandler if it already exists in your project.
 */

#include "bsp_uart3.h"

void USART3_IRQHandler(void)
{
    UART3_IRQHandler();
}

void DMA1_Stream3_IRQHandler(void)
{
    UART3_DMA_TX_IRQHandler();
}

















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
  {
  }
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
  {
  }
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
  {
  }
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
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  sys_tick++;// 每1ms加1
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


