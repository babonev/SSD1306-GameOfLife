/***********************************************************************************************************************
* @file     GameOfLife.c
* ----------------------------------------------------------------------------------------------------------------------
* @details  Conway's Game of Life
*
*           Conway was interested in a problem presented in the 1940s by mathematician John von Neumann, who attempted
*           to find a hypothetical machine that could build copies of itself and succeeded when he found a mathematical
*           model for such a machine with very complicated rules on a rectangular grid. The Game of Life emerged as
*           Conway's successful attempt to drastically simplify von Neumann's ideas.
* ----------------------------------------------------------------------------------------------------------------------
* @author   Boyan Bonev
* ----------------------------------------------------------------------------------------------------------------------
* @version  1.0
* @date     16 Jan 2017
*
***********************************************************************************************************************/

///=====================================================================================================================
/// INCLUDE SECTION
///=====================================================================================================================
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "GameOfLife.h"
#include "DrvSSD1306.h"
#include "random.h"

///=====================================================================================================================
/// LOCAL MACRO
///=====================================================================================================================
#define NUMBER_OF_SEEDS                 63
#define NUMBER_OF_COLUMNS               DISPLAY_COLS_COUNT
#define NUMBER_OF_PAGES                 (DISPLAY_PAGES_COUNT - 2)

///=====================================================================================================================
/// LOCAL TYPES
///=====================================================================================================================
typedef uint8_t Status_t;
#define DEAD                            ((Status_t)0)
#define ALIVE                           ((Status_t)1)

typedef struct
{
    Status_t status;
    uint8_t neighbours;
    uint16_t x;
    uint8_t y;
} Life_t;

///=====================================================================================================================
/// LOCAL DATA
///=====================================================================================================================
static uint8_t page = 0;

///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================
static void seed( uint8_t (*gameBoard)[NUMBER_OF_PAGES][NUMBER_OF_COLUMNS] );
static void evolve( uint8_t (*gameBoard)[NUMBER_OF_PAGES][NUMBER_OF_COLUMNS] );
static BaseType_t applyRules( Life_t* const life );
static uint8_t countNeighbours( uint32_t neighbours );
static void draw( const Life_t* const life,  uint8_t (*gameBoard)[NUMBER_OF_PAGES][NUMBER_OF_COLUMNS] );


///=====================================================================================================================
/// EXPORTED DATA
///=====================================================================================================================
StackType_t xGOLStack[configGOL_TASK_STACK_SIZE];
StaticTask_t xGOLTaskTCBBuffer;

///=====================================================================================================================
/// EXPORTED FUNCTIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void GOL_Task( void *pvParameters )
{
    Canvas_t canvas;
    BaseType_t isSeeded = pdFALSE;

    while(1)
    {
        if ( pdFALSE != DrvDisplay_GetDrawSurface( &canvas, 2 ) )
        {
            if ( isSeeded )
            {
                evolve( canvas.pCanvas );
            }
            else
            {
                seed( canvas.pCanvas );
                isSeeded = pdTRUE;
            }

            if ( page >= NUMBER_OF_PAGES )
            {
                DrvDisplay_ReleaseDrawSurface(DRAW_GOL_BIT);
            }
            else
            {
                DrvDisplay_ReleaseDrawSurface(0);
            }

            taskYIELD();
        }
    }
}

///=====================================================================================================================
/// LOCAL FUNCTIONS DEFINITIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief  seed
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void seed( uint8_t (*gameBoard)[NUMBER_OF_PAGES][NUMBER_OF_COLUMNS] )
{
    Life_t life;
    uint32_t random;
    uint8_t i, j;

    life.status = ALIVE;
    life.neighbours = 0;

    for( i = 0; i < NUMBER_OF_SEEDS; i++ )
    {
        random = LIB_GetRandomNumber();

        for( j = 0; j < 8; j += 2 )
        {
            random = random + 0x333300;

            life.x = ((((random & 0x00FFFF00) >> 8) % NUMBER_OF_COLUMNS) * j);

            // assure that the result is divisible by 2
            life.x = ((life.x >> 1) << 1);
            life.y = j;

            draw(&life, gameBoard);
        }
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  evolve
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void evolve( uint8_t (*gameBoard)[NUMBER_OF_PAGES][NUMBER_OF_COLUMNS] )
{
    Life_t life;
    uint8_t col, row, pageUp, pageDown, colLeft, colRight;
    uint32_t left, mid, right, center;

    /// Start scanning
    if ( page >= NUMBER_OF_PAGES)
    {
        page = 0;
    }

    /// Scan one page per task
    {
        pageUp = (page == 0) ? (NUMBER_OF_PAGES - 1) : (page - 1);
        pageDown = (page == (NUMBER_OF_PAGES - 1)) ? 0 : (page + 1);

        for( col = 0; col < NUMBER_OF_COLUMNS; col += 2 )
        {
            colLeft =  (col == 0) ? (NUMBER_OF_COLUMNS - 2) : (col - 2);
            colRight = (col == (NUMBER_OF_COLUMNS - 2)) ? 0 : (col + 2);

            for( row = 0; row < 8; row += 2 )
            {
                left   = (*gameBoard)[pageUp][colLeft];
                left  += (*gameBoard)[page][colLeft] << 8;

                mid    = (*gameBoard)[pageUp][col];
                mid   += (*gameBoard)[page][col] << 8;

                right  = (*gameBoard)[pageUp][colRight];
                right += (*gameBoard)[page][colRight] << 8;

                if ( pageDown == 0 )
                {
                    left  += ((*gameBoard)[pageDown][colLeft] >> 6) << 16;
                    mid   += ((*gameBoard)[pageDown][col] >> 6) << 16;
                    right += ((*gameBoard)[pageDown][colRight] >> 6) << 16;
                }
                else
                {
                    left  += (*gameBoard)[pageDown][colLeft] << 16;
                    mid   += (*gameBoard)[pageDown][col] << 16;
                    right += (*gameBoard)[pageDown][colRight] << 16;
                }

                center = mid & (0x00000300 << row);
                left   = left & (0x00000FC0 << row);
                mid    = mid & (0x00000CC0 << row);
                right  = right & (0x00000FC0 << row);

                life.neighbours  = countNeighbours(mid);
                life.neighbours += countNeighbours(left);
                life.neighbours += countNeighbours(right);

                life.status = (0 != center) ? ALIVE : DEAD;
                life.x = (page * NUMBER_OF_COLUMNS) + col;
                life.y = row;

                if ( applyRules(&life) )
                {
                    draw(&life, gameBoard);
                }
            }
        }

        page++;
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  countNeighbours
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static uint8_t countNeighbours( uint32_t neighbours )
{
    uint8_t res = 0;
    uint8_t i;

    neighbours = neighbours >> 6;

    for( i = 0; (i < 12) && (neighbours != 0); i++)
    {
        if ( neighbours & 1 )
        {
            res++;
        }
        neighbours = neighbours >> 2;
    }
    return res;
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  applyRules
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static BaseType_t applyRules( Life_t* const life )
{
    BaseType_t res = pdFALSE;

    /// 4) Any dead cell with exactly three live neighbors becomes a live cell, as if by reproduction.
    if (3 == life->neighbours)
    {
        if ( DEAD == life->status )
        {
            life->status = ALIVE;
            res = pdTRUE;
        }
    }
    else
    {
        /// 2) Any live cell with two or three live neighbors lives on to the next generation.
        if (2 == life->neighbours)
        {

        }
        /// 1) Any live cell with fewer than two live neighbors dies, as if caused by underpopulation.
        /// 3) Any live cell with more than three live neighbors dies, as if by overpopulation.
        else if ( DEAD != life->status )
        {
            life->status = DEAD;
            res = pdTRUE;
        }
    }

    return res;
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief  Draw
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
static void draw( const Life_t* const life, uint8_t (*gameBoard)[NUMBER_OF_PAGES][NUMBER_OF_COLUMNS] )
{
    uint8_t lifeMask = (uint8_t)(0x03 << life->y);
    uint8_t* pbyteColumn = &((*gameBoard)[life->x / NUMBER_OF_COLUMNS][life->x % NUMBER_OF_COLUMNS]);

    if ( DEAD == life->status )
    {
        pbyteColumn[0] = (uint8_t)(pbyteColumn[0] & (~lifeMask));
        pbyteColumn[1] = (uint8_t)(pbyteColumn[1] & (~lifeMask));
    }
    else
    {
        pbyteColumn[0] = (uint8_t)(pbyteColumn[0] | lifeMask);
        pbyteColumn[1] = (uint8_t)(pbyteColumn[1] | lifeMask);
    }
}

