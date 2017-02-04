/***********************************************************************************************************************
* @file     DrvRTC.h
* @author   Boyan Bonev
* @version  1.0
* @date     22 Sep 2016
* @brief
*
* @copyright
***********************************************************************************************************************/


///=====================================================================================================================
/// INCLUDE SECTION
///=====================================================================================================================
#include "stm32f10x_rtc.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "DrvRTC.h"
#include "DrvSSD1306.h"


///=====================================================================================================================
/// CONST DATA
///=====================================================================================================================
/// @brief Constants used in calculations
#define SECONDS_IN_MINUTE           ((uint8_t)60)
#define MINUTES_IN_HOUR             ((uint8_t)60)
#define HOURS_IN_DAY                ((uint8_t)24)
#define DAYS_IN_YEAR                ((uint16_t)365)
///---------------------------------------------------------------------------------------------------------------------
/// @brief Number of days in each month
static const uint8_t NOD[] = {31,28,31,30,31,30,31,31,30,31,30,31};
///---------------------------------------------------------------------------------------------------------------------
/// @brief Months names as string
static const uint8_t MONTHS_NAME[][4] = { "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec" };


///=====================================================================================================================
/// EXPORTED DATA
///=====================================================================================================================
StackType_t xRtcStack[configRTC_TASK_STACK_SIZE];
StaticTask_t xRtcTaskTCBBuffer;
TaskHandle_t xRtcToNotify = 0;


///=====================================================================================================================
/// LOCAL DATA
///=====================================================================================================================
static DateTimeType CurrentDateTime;

///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================
static void Init_RCC(void);
static void Init_NVIC(void);
static void Init_RTC(void);
static void GetDateTime( DateTimeType* const dt );
static void SetDateTime( DateTimeType* const dt );


///=====================================================================================================================
/// EXPORTED FUNCTIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvRTC_Init( void )
{
    Init_RCC();
    Init_NVIC();
    Init_RTC();

    DrvRTC_SetUTC(0);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void DrvRTC_Task( void *pvParameters )
{
    uint32_t ulNotificationValue = 1;
    uint8_t timeStr[9];
    Canvas_t canvas;

    /* Store the handle of the calling task. */
    xRtcToNotify = xTaskGetCurrentTaskHandle();

    DrvRTC_Init();

    while(1)
    {
        if ( DrvDisplay_GetDrawSurface(&canvas, 0) )
        {
            DrvRTC_GetTimeStr(timeStr);
            DrvDisplay_DrawString(timeStr, sizeof(timeStr), 0);
            DrvDisplay_ReleaseDrawSurface( DRAW_RTC_BIT );

            taskYIELD();
        }

        ulNotificationValue = ulTaskNotifyTake( pdFALSE, 0 );

        if ( ulNotificationValue != 0 )
        {
            GetDateTime(&CurrentDateTime);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval tRTC counter
///---------------------------------------------------------------------------------------------------------------------
uint32_t DrvRTC_GetUTC( void )
{
    return RTC_GetCounter();
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval tRTC counter
///---------------------------------------------------------------------------------------------------------------------
void DrvRTC_SetUTC( const uint32_t utc )
{
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* Change the current time */
    RTC_SetCounter(utc);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvRTC_SetDate( const uint8_t day, const uint8_t month, const uint16_t year )
{
    DateTimeType dt;

	dt = CurrentDateTime;

	dt.day = day;
	dt.month = month;
	dt.year = year;

    SetDateTime(&dt);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvRTC_SetTime( const uint8_t hour, const uint8_t minutes, const uint8_t seconds )
{
    DateTimeType dt;

	dt = CurrentDateTime;

	dt.hour = hour;
	dt.minutes = minutes;
	dt.seconds = seconds;

    SetDateTime(&dt);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvRTC_GetTimeStr( uint8_t* const currTimeStr )
{
    currTimeStr[0] = (uint8_t)((CurrentDateTime.hour	/ 10) + '0');   /// 0
    currTimeStr[1] = (uint8_t)((CurrentDateTime.hour	% 10) + '0');	/// 0
    currTimeStr[2] = (uint8_t)(':');							        /// :
    currTimeStr[3] = (uint8_t)((CurrentDateTime.minutes / 10) + '0');	/// 0
    currTimeStr[4] = (uint8_t)((CurrentDateTime.minutes % 10) + '0');	/// 0
    currTimeStr[5] = (uint8_t)(':');							        /// :
    currTimeStr[6] = (uint8_t)((CurrentDateTime.seconds / 10) + '0');	/// 0
    currTimeStr[7] = (uint8_t)((CurrentDateTime.seconds % 10) + '0');	/// 0
    currTimeStr[8] = (uint8_t)(0);						                /// termination
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvRTC_GetDateStr( uint8_t* const currDateStr )
{
    vTaskSuspend(xRtcToNotify);

	currDateStr[0]  = (uint8_t)((CurrentDateTime.day / 10) + '0');          /// 0
	currDateStr[1]  = (uint8_t)((CurrentDateTime.day % 10) + '0');		    /// 1
	currDateStr[2]  = (uint8_t)('.');							            /// .
	currDateStr[3]  = MONTHS_NAME[CurrentDateTime.month][0];		        /// J
	currDateStr[4]  = MONTHS_NAME[CurrentDateTime.month][1];		        /// u
	currDateStr[5]  = MONTHS_NAME[CurrentDateTime.month][2];		        /// l
	currDateStr[6]  = (uint8_t)('.');							            /// .
	currDateStr[7]  = (uint8_t)(((CurrentDateTime.year / 100) / 10) + '0'); /// 1
	currDateStr[8]  = (uint8_t)(((CurrentDateTime.year / 100) % 10) + '0');	/// 9
	currDateStr[9]  = (uint8_t)(((CurrentDateTime.year % 100) / 10) + '0');	/// 8
	currDateStr[10] = (uint8_t)(((CurrentDateTime.year % 100) % 10) + '0');	/// 0
	currDateStr[11] = (uint8_t)(0);							                /// termination

	vTaskResume(xRtcToNotify);
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
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  Configures the nested vectored interrupt controller.
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void Init_NVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /// Enable the RTC Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  Initializes and start internal RTC.
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void Init_RTC(void)
{
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void GetDateTime( DateTimeType* const dt )
{
    uint32_t ts = 0;
    uint32_t n, d, i;

    ts = RTC_GetCounter();

    dt->seconds = (uint8_t)(ts % SECONDS_IN_MINUTE);
    ts = ts / SECONDS_IN_MINUTE;
    dt->minutes = (uint8_t)(ts % MINUTES_IN_HOUR);
    ts = ts / MINUTES_IN_HOUR;
    dt->hour = (uint8_t)(ts % HOURS_IN_DAY);
    ts = ts / HOURS_IN_DAY;

    dt->year = (uint16_t)(1970 + ts / 1461 * 4);
    ts = ts % 1461;
	n = (ts >= 1096) ? (ts - 1) : (ts);
	n = n / DAYS_IN_YEAR;
	dt->year += n;
	ts -= n * 365 + (n > 2 ? 1 : 0);

	for (i = 0; i < 12; i++)
    {
		d = NOD[i];
		if (i == 1 && n == 2) d++;
		if (ts < d) break;
		ts -= d;
	}
	dt->month = (uint8_t)(1 + i);
	dt->day = (uint8_t)(1 + ts);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void SetDateTime( DateTimeType* const dt )
{
	uint32_t utc, i, y;

	y = dt->year - 1970;
	if (y > 2106 || !dt->month || !dt->day)
    {
        assert_param(pdFALSE);
    }
    else
    {
        utc = y / 4 * 1461; y %= 4;
        utc += y * 365 + (y > 2 ? 1 : 0);
        for (i = 0; i < 12 && i + 1 < dt->month; i++)
        {
            utc += NOD[i];
            if (i == 1 && y == 2) utc++;
        }
        utc += dt->day - 1;
        utc *= 86400;
        utc += dt->hour * 3600 + dt->minutes * 60 + dt->seconds;

        DrvRTC_SetUTC(utc);
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  This function handles RTC global interrupt request.
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void RTC_IRQHandler(void)
{
    long xHigherPriorityTaskWoken = pdFALSE;

    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        /* Clear the RTC Second interrupt */
        RTC_ClearITPendingBit(RTC_IT_SEC);

        if ( 0 != xRtcToNotify )
        {
            /* Notify the task that the transmission is complete. */
            vTaskNotifyGiveFromISR( xRtcToNotify, &xHigherPriorityTaskWoken );
        }

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }

    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
