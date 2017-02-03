/***********************************************************************************************************************
* @file     DrvUART.c
* @brief
* @author   Boyan Bonev
* @version  1.0
* @date     06 Sep 2016
* @brief
*
* @copyright
***********************************************************************************************************************/

///=====================================================================================================================
/// INCLUDE SECTION
///=====================================================================================================================
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "DrvUART.h"

///=====================================================================================================================
/// CONSTANT DATA
///=====================================================================================================================
#define USART_BAUDRATE                      115200
#define USART_NUMBER                        1
///---------------------------------------------------------------------------------------------------------------------
#if USART_NUMBER == 1
#define USART_ID                            USART1
#define USART_PORT                          GPIOA
#define USART_TX_PIN                        GPIO_Pin_9
#define USART_RX_PIN                        GPIO_Pin_10
#define USART_IRQ                           USART1_IRQn
#elif USART_NUMBER == 2
#define USART_ID                            USART2
#define USART_PORT                          GPIOA
#define USART_TX_PIN                        GPIO_Pin_2
#define USART_RX_PIN                        GPIO_Pin_3
#define USART_IRQ                           USART2_IRQn
#endif
///---------------------------------------------------------------------------------------------------------------------
#define configRX_BUFFER_LENGTH              ((uint8_t)32)   /// [bytes]
#define configTX_BUFFER_LENGTH              ((uint8_t)64)   /// [bytes]
///---------------------------------------------------------------------------------------------------------------------
static const uint8_t GREETING[] = "EVAL BOARD: STM32F103C8T6 72MHz\r\n";

///=====================================================================================================================
/// LOCAL DATA
///=====================================================================================================================
static QueueHandle_t RxQueue;
static uint8_t rxQueueStorage[configRX_BUFFER_LENGTH];
static StaticQueue_t RxQueueBuffer;

static QueueHandle_t TxQueue;
static uint8_t txQueueStorage[configTX_BUFFER_LENGTH];
static StaticQueue_t TxQueueBuffer;

///=====================================================================================================================
/// EXPORTED DATA
///=====================================================================================================================
StackType_t xUartStack[configUART_TASK_STACK_SIZE];
StaticTask_t xUartTaskTCBBuffer;

///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================
static void Init_RCC(void);
static void Init_NVIC(void);
static void Init_GPIO(void);
static void Init_USART(void);
static void Start_USART(void);

static uint32_t get_char(char *ch);
static uint32_t put_char(char ch);

///=====================================================================================================================
/// EXPORTED FUNCTIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvUART_Init( void )
{
    /// Create a queue capable of containing inout and output data.
    RxQueue = xQueueCreateStatic(   configRX_BUFFER_LENGTH,
                                    sizeof( portCHAR ),
                                    rxQueueStorage,
                                    &RxQueueBuffer );

    TxQueue = xQueueCreateStatic(   configTX_BUFFER_LENGTH,
                                    sizeof( portCHAR ),
                                    txQueueStorage,
                                    &TxQueueBuffer );
    /// Init hardware
    Init_RCC();
    Init_NVIC();
    Init_GPIO();
    Init_USART();

    Start_USART();

    /// @brief Send greetings after task scheduler is called
    DrvUART_Send(GREETING, sizeof(GREETING) - 1);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvUART_Task( void *pvParameters )
{
    static const TickType_t xFrequency = 50;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    char ch;

    DrvUART_Init();

    while(1)
    {
        //Echo back
        if ( get_char(&ch) )
        {
            put_char(ch);
        }
        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvUART_Send( uint8_t const* pstr, uint8_t length )
{
    while(length--)
    {
        put_char(*pstr);
        pstr++;
    }
}

///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void Init_RCC(void)
{
#if USART_NUMBER == 1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#elif USART_NUMBER == 2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif
    /// enable UART clock, TX and RX pins, Alternative Functions clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void Init_NVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /// Enable the USARTx Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void Init_USART(void)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART_ID, &USART_InitStructure);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void Init_GPIO(void)
{
    GPIO_InitTypeDef GPIO_Struct;

    GPIO_Struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Struct.GPIO_Pin = USART_TX_PIN;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART_PORT, &GPIO_Struct);

    GPIO_Struct.GPIO_Pin = USART_RX_PIN;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART_PORT, &GPIO_Struct);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void Start_USART(void)
{
    USART_ClearITPendingBit(USART_ID, USART_IT_RXNE);
    USART_ITConfig(USART_ID, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART_ID, USART_IT_TXE, ENABLE);
    USART_Cmd(USART_ID, ENABLE);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static uint32_t get_char(char *ch)
{
    if( xQueueReceive( RxQueue, ch, 0 ) == pdPASS )
    {
        return pdTRUE;
    }
    return pdFALSE;
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static uint32_t put_char(char ch)
{
    if( xQueueSend( TxQueue, &ch, 10 ) == pdPASS )
    {
        USART_ITConfig(USART_ID, USART_IT_TXE, ENABLE);
        return pdTRUE;
    }
    else
    {
        return pdFAIL;
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void USART1_IRQHandler(void)
{
    long xHigherPriorityTaskWoken = pdFALSE;
    uint8_t ch;
    //if Receive interrupt
    if (USART_GetITStatus(USART_ID, USART_IT_RXNE) != RESET)
    {
        ch = (uint8_t)USART_ReceiveData(USART_ID);
        xQueueSendToBackFromISR( RxQueue, &ch, &xHigherPriorityTaskWoken );
    }

    if (USART_GetITStatus(USART_ID, USART_IT_TXE) != RESET)
    {
        if( xQueueReceiveFromISR( TxQueue, &ch, &xHigherPriorityTaskWoken ) )
        {
            USART_SendData(USART_ID, (uint16_t)ch);
        }
        else
        {
            //disable Transmit Data Register empty interrupt
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
