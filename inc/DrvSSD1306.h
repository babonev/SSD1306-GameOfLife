/*
 * DrvDisplay.h
 *
 *  Created on: Sep 4, 2016
 *      Author: scraber
 */

#ifndef DRV_DISPLAY_SSD1306_H
#define DRV_DISPLAY_SSD1306_H
///---------------------------------------------------------------------------------------------------------------------
/// @brief EXPORTED CONTANTS
///---------------------------------------------------------------------------------------------------------------------
#define DISPLAY_PAGES_COUNT                 (8)
#define DISPLAY_ROWS_COUNT                  (DISPLAY_PAGES_COUNT * 8)
#define DISPLAY_COLS_COUNT                  (128)

///---------------------------------------------------------------------------------------------------------------------
/// @brief EXPORTED TYPES
///---------------------------------------------------------------------------------------------------------------------
typedef enum { eAlignLeft, eAlignCenter, eAlignRignt, eAlignInvalid } Alignment_t;
///---------------------------------------------------------------------------------------------------------------------
typedef struct
{
    uint8_t (*pCanvas)[][DISPLAY_COLS_COUNT];
    uint8_t width;
    uint8_t height;
} Canvas_t;
///---------------------------------------------------------------------------------------------------------------------
void DrvDisplay_Init( void );
void DrvDisplay_Write( const uint8_t* const PData, const uint8_t Count);
void DrvDisplay_DrawString( const uint8_t* pstr, uint8_t length, const Alignment_t align);
BaseType_t DrvDisplay_GetDrawSurface( Canvas_t* const pCanvas, const uint8_t startPage );
void DrvDisplay_ReleaseDrawSurface( const BaseType_t shouldRedraw );

///=====================================================================================================================
/// @brief Display OS interface
///=====================================================================================================================
#define configDISPLAY_TASK_STACK_SIZE       ( configMINIMAL_STACK_SIZE )
#define configDISPLAY_TASK_PRIORITY         ( tskIDLE_PRIORITY + 2 )

extern StackType_t xDisplayStack[configDISPLAY_TASK_STACK_SIZE];
extern StaticTask_t xDisplayTaskTCBBuffer;

extern void DrvDisplay_Task( void *pvParameters );
///---------------------------------------------------------------------------------------------------------------------

#endif // DRV_DISPLAY_SSD1306_H
