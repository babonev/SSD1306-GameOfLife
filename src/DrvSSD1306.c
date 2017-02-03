/*
 * DrvDisplay.c
 *
 *  Created on: Sep 4, 2016
 *      Author: scraber
 */


///=====================================================================================================================
/// INCLUDE SECTION
///=====================================================================================================================
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "DrvSSD1306.h"
#include "DrvI2C.h"
#include "DrvRTC.h"
#include "font.h"
#include <string.h>

///=====================================================================================================================
/// CONSTANT DATA
///=====================================================================================================================
#define DISPLAY_BUFFER_SIZE                 (DISPLAY_COLS_COUNT*DISPLAY_PAGES_COUNT)

#define SLAVE_SSD1306_ADDRESS               0x78
#define MASTER_DRIVER_ADDRESS               0x1B
#define I2C_SPEED                           360000
#define I2C_TIMEOUT                         1000
#define I2C_ID                              I2C1
#define SSD1306_COLUMNADDR                  0x21
#define SSD1306_PAGEADDR                    0x22

///=====================================================================================================================
/// LOCAL TYPES
///=====================================================================================================================


///=====================================================================================================================
/// EXPORTED DATA
///=====================================================================================================================
StackType_t xDisplayStack[configDISPLAY_TASK_STACK_SIZE];
StaticTask_t xDisplayTaskTCBBuffer;
TaskHandle_t xDrawToNotify = 0;

///=====================================================================================================================
/// LOCAL DATA
///=====================================================================================================================
static SemaphoreHandle_t xDrawMutex = 0;
static StaticSemaphore_t xDrawMutexBuffer;
static uint8_t displayBuff[2][DISPLAY_BUFFER_SIZE] = { 0 };
static uint8_t showBuffIdx;

///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================
static void SSD1306_Init( void );
static void SSD1306_Refresh( void );
static void SSD1306_SendCommand( const uint8_t slave_data );
static void SSD1306_DrawBuffer( uint8_t *buffer_pointer );
static void SSD1306_Clear( void );
static void WaitFor(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);


///=====================================================================================================================
/// EXPORTED FUNCTIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void DrvDisplay_Init( void )
{
    DrvI2C_Init(I2C_ID, I2C_SPEED, MASTER_DRIVER_ADDRESS);

    showBuffIdx = 0;

    // Init sequence for 128x64 OLED module
    SSD1306_Init();
    SSD1306_Clear();
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void DrvDisplay_Task( void *pvParameters )
{
    uint32_t ulNotificationValue;

    xDrawToNotify = xTaskGetCurrentTaskHandle();
    xDrawMutex = xSemaphoreCreateMutexStatic( &xDrawMutexBuffer );

    xSemaphoreTake( xDrawMutex, (TickType_t) 0 );
    DrvDisplay_Init();
    xSemaphoreGive( xDrawMutex );

    while(1)
    {
        ulNotificationValue = ulTaskNotifyTake( pdTRUE, (TickType_t) 500 );

        if ( ulNotificationValue != 0 )
        {
            SSD1306_DrawBuffer(displayBuff[showBuffIdx]);
        }

        taskYIELD();
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
BaseType_t DrvDisplay_GetDrawSurface( Canvas_t* const pCanvas, const uint8_t startPage )
{
    BaseType_t res = pdFALSE;

    if ( 0 != xDrawMutex )
    {
        res = xSemaphoreTake( xDrawMutex, (TickType_t) 100 );

        if ( pdFALSE != res )
        {
            pCanvas->pCanvas = &displayBuff[1 - showBuffIdx][startPage * DISPLAY_COLS_COUNT];
            pCanvas->width = DISPLAY_ROWS_COUNT - (startPage * 8);
            pCanvas->height = DISPLAY_COLS_COUNT;
        }
    }

    return res;
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  DrvDisplay_ReleaseDrawSurface
/// @param  shouldRedraw
///---------------------------------------------------------------------------------------------------------------------
void DrvDisplay_ReleaseDrawSurface( const BaseType_t shouldRedraw )
{
    if ( (0 != xDrawMutex) && (0 != xDrawToNotify) )
    {
        SSD1306_Refresh();
        xSemaphoreGive( xDrawMutex );

        /// Redraw only if the last task requires it
        if ( pdFALSE != shouldRedraw )
        {
            xTaskNotifyGive( xDrawToNotify );
        }
    }
}

///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
static void SSD1306_Init( void )
{
    // Sends the init commands to the display
    SSD1306_SendCommand(0xAE);

    SSD1306_SendCommand(0x00 | 0x0);      // low col = 0

    SSD1306_SendCommand(0x10 | 0x0);      // hi col = 0
    SSD1306_SendCommand(0x40 | 0x0);      // line #0

    SSD1306_SendCommand(0x81);            // Set Contrast 0x81
    SSD1306_SendCommand(0xCF);
    SSD1306_SendCommand(0xA1);            // Segremap - 0xA1
    SSD1306_SendCommand(0xC8);            // COMSCAN DEC 0xC8 C0
    SSD1306_SendCommand(0xA6);            // Normal Display 0xA6 (Invert A7)

    SSD1306_SendCommand(0xA4);            // DISPLAY ALL ON RESUME - 0xA4
    SSD1306_SendCommand(0xA8);            // Set Multiplex 0xA8
    SSD1306_SendCommand(0x3F);            // 1/64 Duty Cycle

    SSD1306_SendCommand(0xD3);            // Set Display Offset 0xD3
    SSD1306_SendCommand(0x0);             // no offset

    SSD1306_SendCommand(0xD5);            // Set Display Clk Div 0xD5
    SSD1306_SendCommand(0x80);            // Recommneded resistor ratio 0x80

    SSD1306_SendCommand(0xD9);            // Set Precharge 0xd9
    SSD1306_SendCommand(0xF1);

    SSD1306_SendCommand(0xDA);            // Set COM Pins0xDA
    SSD1306_SendCommand(0x12);

    SSD1306_SendCommand(0xDB);            // Set VCOM Detect - 0xDB
    SSD1306_SendCommand(0x40);

    SSD1306_SendCommand(0x20);            // Set Memory Addressing Mode
    SSD1306_SendCommand(0x00);            // 0x00 - Horizontal

    SSD1306_SendCommand(0x40 | 0x0);      // Set start line at line 0 - 0x40

    SSD1306_SendCommand(0x8D);            // Charge Pump -0x8D
    SSD1306_SendCommand(0x14);

    SSD1306_SendCommand(0xA4);            //--turn on all pixels - A5. Regular mode A4
    SSD1306_SendCommand(0xAF);            //--turn on oled panel - AF
}

///---------------------------------------------------------------------------------------------------------------------
/// Sends I2C data over I2C_ID:
///  1) Sends Start Condition. Checks for I2C EV5
///  2) Sends 7 bit address & checks for EV6
///  3) Sends 8 bit command byte (0x00) & checks for EV8
///  4) Sends 8 bits (1 byte) of data & checks for EV8
///  5) Sends Stop Condition
///---------------------------------------------------------------------------------------------------------------------
static void SSD1306_SendCommand( const uint8_t slave_data )
{
    static const uint8_t COMMAND_BYTE = 0x00;

    /// 1) Send I2C1 START condition. Test on I2C1 EV5 and clear it
    I2C_GenerateSTART(I2C_ID, ENABLE);
    WaitFor(I2C_ID, I2C_EVENT_MASTER_MODE_SELECT);

    /// 2) Send SSD1306 7 bit slave Address for write. Check to make sure ACK received. Test on I2C1 EV6 and clear it
    I2C_Send7bitAddress(I2C_ID, SLAVE_SSD1306_ADDRESS, I2C_Direction_Transmitter);
    WaitFor(I2C_ID, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    /// 3) Sends 8 bit command byte (0x00). Checks for EV8
    I2C_SendData(I2C_ID, COMMAND_BYTE);
    WaitFor(I2C_ID, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    /// 4) Sends 8 bits (1 byte) of data. Checks for EV8
    I2C_SendData(I2C_ID, slave_data);
    WaitFor(I2C_ID, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    /// 5) Sends Stop Condition
    I2C_GenerateSTOP(I2C_ID, ENABLE);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
static void SSD1306_DrawBuffer( uint8_t *buffer_pointer )
{
    static const uint8_t DATA_BYTE          = 0x40;

    SSD1306_SendCommand(SSD1306_COLUMNADDR);
    SSD1306_SendCommand(0x00);            // Column Start address
    SSD1306_SendCommand(127);             // Column end address

    SSD1306_SendCommand(SSD1306_PAGEADDR);
    SSD1306_SendCommand(0x00);            // Page Start address
    SSD1306_SendCommand(0x07);            // Page end address

    /// 1) Send I2C1 START condition. Test on I2C1 EV5 and clear it.
    I2C_GenerateSTART(I2C_ID, ENABLE);
    WaitFor(I2C_ID, I2C_EVENT_MASTER_MODE_SELECT);

    /// 2) Send SSD1306 7 bit slave Address for write. Check to make sure ACK received. Test on I2C1 EV6 and clear it.
    I2C_Send7bitAddress(I2C_ID, SLAVE_SSD1306_ADDRESS, I2C_Direction_Transmitter);
    WaitFor(I2C_ID, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    /// 3) Send data. Wait for EV8.
    I2C_SendData(I2C_ID, DATA_BYTE);
    WaitFor(I2C_ID, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    DrvI2C_WriteBuffer(I2C_ID, buffer_pointer, DISPLAY_BUFFER_SIZE);
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
static void SSD1306_Clear( void )
{
    taskENTER_CRITICAL();
    memset(displayBuff, 0, DISPLAY_BUFFER_SIZE  * 2);
    taskEXIT_CRITICAL();
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
static void SSD1306_Refresh( void )
{
    taskENTER_CRITICAL();
    memcpy(displayBuff[showBuffIdx], displayBuff[1 - showBuffIdx], DISPLAY_BUFFER_SIZE);
    /// Swap "show" and "draw" buffers
    showBuffIdx = 1 - showBuffIdx;
    taskEXIT_CRITICAL();
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
void DrvDisplay_DrawString( const uint8_t* pstr, uint8_t length, const Alignment_t align)
{
    const uint8_t drawSize = ((length-1) * 6) + ((length-1) * font_param[FONT6x8].char_space);
    const uint8_t startColumn = (DISPLAY_COLS_COUNT - drawSize)/2;
    const uint8_t endColumn = startColumn + drawSize;

    uint8_t* page0 = &displayBuff[1 - showBuffIdx][0];
    uint8_t col;
    uint8_t charCol = 0;
    uint8_t chIdx = 0;
    uint8_t strIdx = (pstr[0] - ' ') * 6;

    for( col = 0; col < DISPLAY_COLS_COUNT; col++ )
    {
        if ( (startColumn >= col) || (col > endColumn) )
        {
            *page0 = 0;
        }
        else
        {
            *page0 = pCharset6x8[strIdx + charCol];

            charCol++;
            if ( charCol >= 6 )
            {
                chIdx++;
                if ( chIdx < length )
                {
                    strIdx = (pstr[chIdx] - ' ') * 6;
                }
                charCol = 0;
            }
        }
        page0++;
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
static void WaitFor(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
    uint16_t timeCount = I2C_TIMEOUT;

    do
    {
        vTaskDelay(1);

        timeCount--;
        if (timeCount == 0)
        {
            I2C_GenerateSTOP(I2C_ID, ENABLE);
            break;
        }
    }
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT));
}


