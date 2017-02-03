/***********************************************************************************************************************
* @file     random.c
* ----------------------------------------------------------------------------------------------------------------------
* @details  http://stackoverflow.com/questions/14796398/how-generate-real-random-number-using-stm32-mcu
* ----------------------------------------------------------------------------------------------------------------------
* @author   Boyan Bonev
* ----------------------------------------------------------------------------------------------------------------------
* @version  1.0
* @date     20 Jan 2017
*
***********************************************************************************************************************/

///=====================================================================================================================
/// INCLUDE SECTION
///=====================================================================================================================
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "random.h"

///=====================================================================================================================
/// CONSTANT DATA
///=====================================================================================================================


///=====================================================================================================================
/// LOCAL FUNCTIONS
///=====================================================================================================================
static void Init_ADC( void );


///=====================================================================================================================
///
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
uint32_t LIB_GetRandomNumber(void)
{
    //enable ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Initialize ADC 14MHz RC
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);

    while (!RCC_GetFlagStatus(RCC_FLAG_HSIRDY));

    Init_ADC();

    //enable internal channel
    ADC_TempSensorVrefintCmd(ENABLE);

    // Enable ADCperipheral
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);

    // Convert the ADC1 temperature sensor, user shortest sample time to generate most noise
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_1Cycles5 );

    // Enable CRC clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

    for (uint8_t i = 0; i < 8; i++)
    {
        //Start ADC1 Software Conversion
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        //wait for conversion complete
        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

        CRC_CalcCRC(ADC_GetConversionValue(ADC1));
        //clear EOC flag
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    }

    //disable ADC1 to save power
    ADC_Cmd(ADC1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);

    return CRC_CalcCRC(0xBADA55E5);
}


///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
///---------------------------------------------------------------------------------------------------------------------
static void Init_ADC( void )
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
}
