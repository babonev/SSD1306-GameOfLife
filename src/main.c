/***********************************************************************************************************************
* @file     main.c
* @author   Boyan Bonev
* @version  1.0
* @date     11 Jan 2017
* @brief
*
* @copyright
***********************************************************************************************************************/
#include "stm32f10x_conf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "DrvUART.h"
#include "DrvRTC.h"
#include "DrvI2C.h"
#include "DrvSSD1306.h"
#include "GameOfLife.h"

///=====================================================================================================================
/// LOCAL DATA
///=====================================================================================================================


///=====================================================================================================================
/// @brief
///=====================================================================================================================
 /* Declare a variable to hold the handle of the created event group. */
EventGroupHandle_t xEventGroupHandle;
/* Declare a variable to hold the data associated with the created    event group. */
StaticEventGroup_t xDrawEventGroup;


///=====================================================================================================================
/// @brief
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
int main(void)
{
    TaskHandle_t hTask;

    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    /* Attempt to create the event group. */
    xEventGroupHandle = xEventGroupCreateStatic( &xDrawEventGroup );

    /* pxEventGroupBuffer was not null so expect the event group to have been created? */
    assert_param( xEventGroupHandle );

    /// @brief Creates the task to handle all USART functions
    hTask = xTaskCreateStatic( &DrvUART_Task,
                        ( char * ) "DrvUART_Task",
                        configUART_TASK_STACK_SIZE,
                        NULL,
                        configUART_TASK_PRIORITY,
                        xUartStack,
                        &xUartTaskTCBBuffer);
    assert_param( hTask != 0 );

    /// @brief Creates the task to handle all RTC functions
    hTask = xTaskCreateStatic( &DrvRTC_Task,
                        ( char * ) "DrvRTC_Task",
                        configRTC_TASK_STACK_SIZE,
                        NULL,
                        configRTC_TASK_PRIORITY,
                        xRtcStack,
                        &xRtcTaskTCBBuffer);
    assert_param( hTask != 0 );

    /// @brief Creates the task to handle all LCD functions
    hTask = xTaskCreateStatic( &DrvDisplay_Task,
                        ( char * ) "DrvDisplay_Task",
                        configDISPLAY_TASK_STACK_SIZE,
                        NULL,
                        configDISPLAY_TASK_PRIORITY,
                        xDisplayStack,
                        &xDisplayTaskTCBBuffer);
    assert_param( hTask != 0 );

    /// @brief Creates the task to handle all Game of Life
    hTask = xTaskCreateStatic( &GOL_Task,
                        ( char * ) "GOL_Task",
                        configGOL_TASK_STACK_SIZE,
                        NULL,
                        configGOL_TASK_PRIORITY,
                        xGOLStack,
                        &xGOLTaskTCBBuffer);
    assert_param( hTask != 0 );

    /// @brief Start of the RTOS scheduler, this function should never return
    vTaskStartScheduler();

    while(1);

    return -1;
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed portCHAR *pcTaskName )
{
    assert_param( pdFALSE );
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    static StackType_t xIdleStack[configMINIMAL_STACK_SIZE / 64];
    static StaticTask_t xIdleTaskTCBBuffer;

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = xIdleStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void vApplicationIdleHook()
{
    while(1);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void vApplicationTickHook( void )
{

}


///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void assert_failed(char* file, unsigned long line)
{
    // User can add his own implementation to report the file name and line number,
    // ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
    asm("bkpt");
}
