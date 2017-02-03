
#ifndef INC_USART_H
#define INC_USART_H

///---------------------------------------------------------------------------------------------------------------------
/// @brief USART hardware init
///---------------------------------------------------------------------------------------------------------------------
extern void DrvUART_Init( void );

///---------------------------------------------------------------------------------------------------------------------
/// @brief USART send string
///---------------------------------------------------------------------------------------------------------------------
extern void DrvUART_Send( uint8_t const* pstr, uint8_t length);

///---------------------------------------------------------------------------------------------------------------------
/// @brief USART OS interface
///---------------------------------------------------------------------------------------------------------------------
#define configUART_TASK_STACK_SIZE          ( configMINIMAL_STACK_SIZE * 2 )
#define configUART_TASK_PRIORITY            ( tskIDLE_PRIORITY )

extern StackType_t xUartStack[configUART_TASK_STACK_SIZE];
extern StaticTask_t xUartTaskTCBBuffer;

extern void DrvUART_Task( void *pvParameters );
///---------------------------------------------------------------------------------------------------------------------

#endif /* INC_USART_H */
