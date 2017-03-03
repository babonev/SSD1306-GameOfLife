///*********************************************************************************************************************
/// \file     MdlCmd.c
/// \author   Boyan Bonev
/// \version  1.0
/// \date     04.Feb.2017
/// \brief
/// \copyright
///*********************************************************************************************************************

///=====================================================================================================================
/// INCLUDE SECTION
///=====================================================================================================================
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MDL_Cmd.h"
#include "MDL_Cmd_Cfg.h"
#include "DrvUART.h"
#include <string.h>

///=====================================================================================================================
/// CONST DATA
///=====================================================================================================================

///=====================================================================================================================
/// LOCAL TYPES
///=====================================================================================================================
typedef enum
{
    eParseCommand,
    eParseParameter,
    eParseExecute,
    eParseCount,
} ParsingState_t;

///=====================================================================================================================
/// LOCAL DATA
///=====================================================================================================================
static uint8_t cmdline[MAX_LINE_LENGTH];
static uint8_t lineIdx = 0;
static ParsingState_t parseState = eParseCommand;

///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================
static void help_out( const uint8_t* const helpStr );
///---------------------------------------------------------------------------------------------------------------------
static CmdIdx_t parse_command( const uint8_t* const inStr );
///---------------------------------------------------------------------------------------------------------------------
static ErrorStatus handler_time( const uint8_t* const strTime );
static ErrorStatus handler_date( const uint8_t* const strDate );
static ErrorStatus handler_error( const uint8_t* const strDate );
///---------------------------------------------------------------------------------------------------------------------
static const Commands_t dictionary[] = INIT_COMMANDS_DICTIONARY;
#define DICTIONARY_LENGTH               (sizeof(dictionary)/sizeof(dictionary[0]))
///---------------------------------------------------------------------------------------------------------------------

///=====================================================================================================================
/// EXPORTED FUNCTIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief  CmdParser
/// @param  ch
/// @return
///---------------------------------------------------------------------------------------------------------------------
void MDL_Cmd_AddInput( const uint8_t ch )
{
    static CmdIdx_t cmdIdx;
    static uint8_t paramCount;

    if ( lineIdx < MAX_LINE_LENGTH )
    {
        cmdline[lineIdx] = ch;

        switch(parseState)
        {
        case eParseCommand:
            if ( (ch == ' ') || (ch == '\n') || (ch == '\r') )
            {
                cmdIdx = parse_command(cmdline);
                paramCount = dictionary[cmdIdx].paramCount;
                parseState = eParseParameter;
            }
            break;
        case eParseParameter:
            if ( paramCount > 0 )
            {
                cmdIdx = parse_command(cmdline);
                paramCount--;
                parseState = eParseParameter;
            }
            else
            {
                dictionary[cmdIdx].handler(cmdline);
            }
            break;
        default:
            help_out(cmdline);
            break;
        }

        lineIdx++;
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// \brief  handler_time
/// \param  strTime
/// \return
///---------------------------------------------------------------------------------------------------------------------
static CmdIdx_t parse_command( const uint8_t* const inStr )
{
    CmdIdx_t idx;

    for( idx = 0; idx < (DICTIONARY_LENGTH - 1); idx++ )
    {
        if ( 0 == strncmp((char*)inStr, dictionary[idx].cmdStr, lineIdx) )
        {
            break;
        }
    }

    return idx;
}

///---------------------------------------------------------------------------------------------------------------------
/// \brief  help_out
/// \param  helpStr
///---------------------------------------------------------------------------------------------------------------------
static void help_out( const uint8_t* const helpStr )
{
    uint8_t length;

    if ( 0 != helpStr )
    {
        length = strlen((char*)helpStr);
        DrvUART_Send((uint8_t*)helpStr, length);
    }
}

///=====================================================================================================================
/// \brief HANDLERS DEFINITIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// \brief  handler_time
/// \param  strTime
/// \return
///---------------------------------------------------------------------------------------------------------------------
static ErrorStatus handler_time( const uint8_t* const strTime )
{
    return SUCCESS;
}

///---------------------------------------------------------------------------------------------------------------------
/// \brief  handler_date
/// \param  strDate
/// \return
///---------------------------------------------------------------------------------------------------------------------
static ErrorStatus handler_date( const uint8_t* const strDate )
{
    return SUCCESS;
}


///---------------------------------------------------------------------------------------------------------------------
/// \brief  handler_error
/// \param  strDate
/// \return
///---------------------------------------------------------------------------------------------------------------------
static ErrorStatus handler_error( const uint8_t* const strDate )
{
    return SUCCESS;
}
