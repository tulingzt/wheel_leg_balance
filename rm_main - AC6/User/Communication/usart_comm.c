#include "usart_comm.h"
#include "prot_dr16.h"
#include "prot_judge.h"
#include "string.h"
#include "data_log.h"

#define DEBUG_DATA_LEN 10
#define JUDGE_DATA_LEN 150

uint8_t dr16_dma_rx_buf[DR16_DATA_LEN];
uint8_t judge_data_rx_buf[JUDGE_DATA_LEN];
uint8_t debug_dma_rx_buf[DEBUG_DATA_LEN];

/*
 * @brief  串口初始化，开启空闲中断并开始DMA接收数据
 * @retval void
 */
void usart_comm_init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dr16_dma_rx_buf, DR16_DATA_LEN);
    
    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, judge_data_rx_buf, JUDGE_DATA_LEN);
    judge_init(&JUDGE_HUART);

    __HAL_UART_CLEAR_IDLEFLAG(&DEBUG_HUART);
    __HAL_UART_ENABLE_IT(&DEBUG_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DEBUG_HUART, debug_dma_rx_buf, DEBUG_DATA_LEN);
    log_init(&DEBUG_HUART);
}

/*
 * @brief  串口中断，添加各串口的数据接收函数
 * @retval void
 */
void usart_user_handler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_AbortReceive(huart);
        if (huart == &DBUS_HUART) {
            dr16_get_data(&rc, dr16_dma_rx_buf);
            HAL_UART_Receive_DMA(huart, dr16_dma_rx_buf, DR16_DATA_LEN);
        } else if (huart == &JUDGE_HUART) {
            judge_get_data(judge_data_rx_buf);
            HAL_UART_Receive_DMA(huart, judge_data_rx_buf, JUDGE_DATA_LEN);
        } else if (huart == &DEBUG_HUART) {
            ;
        }
    }
}

//__weak void usart_user_handler(UART_HandleTypeDef *huart)
//{
//	;
//}

//#define DEBUG_DATA_LEN 10

//uint8_t dr16_dma_rx_buf[2][DR16_DATA_LEN];
//uint8_t debug_dma_rx_buf[2][DEBUG_DATA_LEN];

//static HAL_StatusTypeDef HAL_UART_Receive_DMA_Double(UART_HandleTypeDef *huart, uint8_t *pData1, uint8_t *pData2, uint16_t Size);

///*
// * @brief  串口初始化，硬件双缓冲接收
// * @retval void
// */
//void usart_comm_init(void)
//{
//    HAL_UART_Receive_DMA_Double(&DBUS_HUART, dr16_dma_rx_buf[0], dr16_dma_rx_buf[1], DR16_DATA_LEN);
//    HAL_UART_Receive_DMA_Double(&DEBUG_HUART, debug_dma_rx_buf[0], debug_dma_rx_buf[1], DEBUG_DATA_LEN);
//}

///*
// * @brief  前缓冲区接收完成回调函数
// * @retval void
// */
//static void UART_DMAReceiveCplt_M0(DMA_HandleTypeDef *hdma)
//{
//    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)(hdma->Parent);
//    if (huart == &DBUS_HUART) {
//		dr16_get_data(&rc, dr16_dma_rx_buf[0]);
//	} else if (huart == &DEBUG_HUART) {
//        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	}
//}

///*
// * @brief  后缓冲区接收完成回调函数
// * @retval void
// */
//static void UART_DMAReceiveCplt_M1(DMA_HandleTypeDef *hdma)
//{
//    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)(hdma->Parent);
//    if (huart == &DBUS_HUART) {
//		dr16_get_data(&rc, dr16_dma_rx_buf[1]);
//	} else if (huart == &DEBUG_HUART) {
//        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	}
//}

///************************************实现hal库dam双缓冲接收函数*******************************************/
//static HAL_StatusTypeDef UART_Start_Receive_DMA_Double(UART_HandleTypeDef *huart, uint8_t *pData1, uint8_t *pData2, uint16_t Size)
//{
//  huart->pRxBuffPtr = pData1;
//  huart->RxXferSize = Size;

//  huart->ErrorCode = HAL_UART_ERROR_NONE;
//  huart->RxState = HAL_UART_STATE_BUSY_RX;

//  if (huart->hdmarx != NULL)
//  {
//    /* Set the UART DMA transfer complete callback */
//    huart->hdmarx->XferCpltCallback = UART_DMAReceiveCplt_Double;
//    huart->hdmarx->XferM1CpltCallback = UART_DMAReceiveCplt_Double;
//    /* Set the UART DMA Half transfer complete callback */
//    huart->hdmarx->XferHalfCpltCallback = NULL;

//    /* Set the DMA error callback */
//    huart->hdmarx->XferErrorCallback = NULL;

//    /* Set the DMA abort callback */
//    huart->hdmarx->XferAbortCallback = NULL;

//    /* Enable the DMA channel */
//      
//    if (HAL_DMAEx_MultiBufferStart_IT(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)pData1, (uint32_t)pData2, Size) != HAL_OK)
//    {
//      /* Set error code to DMA */
//      huart->ErrorCode = HAL_UART_ERROR_DMA;

//      __HAL_UNLOCK(huart);

//      /* Restore huart->gState to ready */
//      huart->gState = HAL_UART_STATE_READY;

//      return HAL_ERROR;
//    }
//  }
//  __HAL_UNLOCK(huart);

//  /* Enable the UART Parity Error Interrupt */
//  SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

//  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//  SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

//  /* Enable the DMA transfer for the receiver request by setting the DMAR bit
//  in the UART CR3 register */
//  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

//  return HAL_OK;
//}

//static HAL_StatusTypeDef HAL_UART_Receive_DMA_Double(UART_HandleTypeDef *huart, uint8_t *pData1, uint8_t *pData2, uint16_t Size)
//{
//  /* Check that a Rx process is not already ongoing */
//  if (huart->RxState == HAL_UART_STATE_READY)
//  {
//    if ((pData1 == NULL) || (pData2 == NULL) || (Size == 0U))
//    {
//      return HAL_ERROR;
//    }

//    __HAL_LOCK(huart);

//    /* Set Reception type to Standard reception */
//    huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

//    if (!(IS_LPUART_INSTANCE(huart->Instance)))
//    {
//      /* Check that USART RTOEN bit is set */
//      if (READ_BIT(huart->Instance->CR2, USART_CR2_RTOEN) != 0U)
//      {
//        /* Enable the UART Receiver Timeout Interrupt */
//        SET_BIT(huart->Instance->CR1, USART_CR1_RTOIE);
//      }
//    }

//    return (UART_Start_Receive_DMA_Double(huart, pData1, pData2, Size));
//  }
//  else
//  {
//    return HAL_BUSY;
//  }
//}
