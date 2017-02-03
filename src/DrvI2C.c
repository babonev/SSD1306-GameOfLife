/*
 *  See AN2824 STM32F10xxx I2C optimized examples
 *
 *  This code implements polling based solution
 *
 */

#include <stm32f10x.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_dma.h>

#include "FreeRTOS.h"
#include "task.h"
#include "DrvI2C.h"

#define USE_DMA                 0
#define USE_LED                 0

#define Timed(x) Timeout = 0xFFFF; while (x) { if (Timeout-- == 0) goto errReturn; }


/**
 *  Names of events used in stdperipheral library
 *
 *      I2C_EVENT_MASTER_MODE_SELECT                          : EV5
 *      I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6
 *      I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
 *      I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
 *
 **/

///=====================================================================================================================
/// CONSTANT DATA
///=====================================================================================================================


///=====================================================================================================================
/// LOCAL DATA
///=====================================================================================================================


///=====================================================================================================================
/// LOCAL FUNCTIONS DEFINITIONS
///=====================================================================================================================


///=====================================================================================================================
/// EXPORTED FUNCTIONS DEFINITIONS
///=====================================================================================================================

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DrvI2C_Init(I2C_TypeDef* I2Cx, int ClockSpeed, int OwnAddress)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    /* Enable GPIOB clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure I2C clock and GPIO
    GPIO_StructInit(&GPIO_InitStructure);

    if (I2Cx == I2C1)
    {
        /* I2C1 clock enable */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

        /* I2C1 SDA and SCL configuration */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* I2C1 Reset */
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
    }
    else
    {
        /* I2C2 clock enable */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

        /* I2C1 SDA and SCL configuration */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* I2C2  Reset */
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
    }

#if USE_DMA == 1
    {
        //NVIC_InitTypeDef NVIC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        DMA_Channel_TypeDef* DMA_Channel_I2C;
        //uint32_t DMA_IRQ;
        uint32_t I2C_DR_ADDRESS;

        /* Enable DMA1 clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        if (I2Cx == I2C1)
        {
            DMA_Channel_I2C = DMA1_Channel6;
            //DMA_IRQ = DMA1_Channel6_IRQn;
            I2C_DR_ADDRESS = I2C1->DR;
        }
        else
        {
            DMA_Channel_I2C = DMA1_Channel4;
            //DMA_IRQ = DMA1_Channel4_IRQn;
            I2C_DR_ADDRESS = I2C2->DR;
        }

        /* DMA1 channel6 configuration */
        DMA_DeInit(DMA_Channel_I2C);
        DMA_InitStructure.DMA_PeripheralBaseAddr = I2C_DR_ADDRESS;
        /* The address will be updated on transfer start */
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
        DMA_InitStructure.DMA_BufferSize = 1;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_Init(DMA_Channel_I2C, &DMA_InitStructure);

        /* NVIC configure DMA interrupt */
        /*
        NVIC_InitStructure.NVIC_IRQChannel = DMA_IRQ;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        */
    }
#endif

    I2C_Cmd(I2Cx, ENABLE);

    /* Configure I2Cx                */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = OwnAddress;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
    I2C_Init(I2Cx, &I2C_InitStructure);
}

///---------------------------------------------------------------------------------------------------------------------
///
/// Read process is documented in RM008
///
///   There are three cases  -- read  1 byte  AN2824 Figure 2
///                             read  2 bytes AN2824 Figure 2
///                             read >2 bytes AN2824 Figure 1
///---------------------------------------------------------------------------------------------------------------------
Status_t DrvI2C_Read(I2C_TypeDef* I2Cx, uint8_t *buf,uint32_t nbyte, uint8_t SlaveAddress)
{
    __IO uint32_t Timeout;

    //    I2Cx->CR2 |= I2C_IT_ERR;  interrupts for errors
    if (!nbyte)
        return eSuccess;

    // Wait for idle I2C interface
    Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    // Enable Acknowledgment, clear POS flag
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);

    // Initiate Start Sequence (wait for EV5
    I2C_GenerateSTART(I2Cx, ENABLE);
    Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    // Send Address
    I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Receiver);

    // EV6
    Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR));

    if (nbyte == 1)
    {
        // Clear Ack bit
        I2C_AcknowledgeConfig(I2Cx, DISABLE);

        // EV6_1 -- must be atomic -- Clear ADDR, generate STOP
        __disable_irq();
        (void) I2Cx->SR2;
        I2C_GenerateSTOP(I2Cx,ENABLE);
        __enable_irq();

        // Receive data   EV7
        Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE));
        *buf++ = I2C_ReceiveData(I2Cx);

    }
    else if (nbyte == 2)
    {
        // Set POS flag
        I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);

        // EV6_1 -- must be atomic and in this order
        __disable_irq();
        (void) I2Cx->SR2;                           // Clear ADDR flag
        I2C_AcknowledgeConfig(I2Cx, DISABLE);       // Clear Ack bit
        __enable_irq();

        // EV7_3  -- Wait for BTF, program stop, read data twice
        Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));

        __disable_irq();
        I2C_GenerateSTOP(I2Cx,ENABLE);
        *buf++ = I2Cx->DR;
        __enable_irq();

        *buf++ = I2Cx->DR;
    }
    else
    {
        (void) I2Cx->SR2;                           // Clear ADDR flag
        while (nbyte-- != 3)
        {
            // EV7 -- cannot guarantee 1 transfer completion time, wait for BTF
            //        instead of RXNE
            Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
            *buf++ = I2C_ReceiveData(I2Cx);
        }

        Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));

        // EV7_2 -- Figure 1 has an error, doesn't read N-2 !
        I2C_AcknowledgeConfig(I2Cx, DISABLE);           // clear ack bit

        __disable_irq();
        *buf++ = I2C_ReceiveData(I2Cx);             // receive byte N-2
        I2C_GenerateSTOP(I2Cx,ENABLE);                  // program stop
        __enable_irq();

        *buf++ = I2C_ReceiveData(I2Cx);             // receive byte N-1

        // wait for byte N
        Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
        *buf++ = I2C_ReceiveData(I2Cx);

        nbyte = 0;
    }

    // Wait for stop
    Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
    return eSuccess;

errReturn:

    // Any cleanup here
    return eError;

}

///---------------------------------------------------------------------------------------------------------------------
/// Write buffer of bytes -- AN2824 Figure 3
///
///
///---------------------------------------------------------------------------------------------------------------------
Status_t DrvI2C_Write(I2C_TypeDef* I2Cx, const uint8_t* buf,  uint32_t nbyte, uint8_t SlaveAddress)
{
    __IO uint32_t Timeout;

    /* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
    if (nbyte)
    {
        Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

        // Intiate Start Sequence
        I2C_GenerateSTART(I2Cx, ENABLE);
        Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

        // Send Address  EV5
        I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Transmitter);
        Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

        // EV6
        // Write first byte EV8_1
        I2C_SendData(I2Cx, *buf++);
        return eSuccess;

        while (--nbyte)
        {
            // wait on BTF
            Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
            I2C_SendData(I2Cx, *buf++);
        }

        Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
        I2C_GenerateSTOP(I2Cx, ENABLE);
        Timed(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
    }

    return eSuccess;
errReturn:
    return eError;
}

///---------------------------------------------------------------------------------------------------------------------
/// Write buffer of bytes -- AN2824 Figure 3
///
///
///---------------------------------------------------------------------------------------------------------------------
Status_t DrvI2C_WriteBuffer(I2C_TypeDef* I2Cx, const uint8_t* buf,  uint32_t nbyte)
{
    __IO uint32_t Timeout;

    /* Data transmission */
    if (nbyte)
    {
#if USE_DMA == 1
        if (I2Cx == I2C1)
        {
            /* Assign base address */
            DMA1_Channel6->CMAR = (uint32_t)buf;
            DMA1_Channel6->CNDTR = nbyte;

            /* DMA1 Channel1 transfer complete interrupt enable */
            //DMA_ClearITPendingBit(DMA1_IT_GL6 | DMA1_IT_TC6 | DMA1_IT_HT6);
            //DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);

            /* Enable I2C1 DMA */
            I2C_DMACmd(I2C1, ENABLE);
            /* Enable DMA1 Channel6 */
            DMA_Cmd(DMA1_Channel6, ENABLE);
            /* DMA1 Channel6 transfer complete test */
            Timed(!DMA_GetFlagStatus(DMA1_FLAG_TC6));
            /* Disable DMA1 Channel6 */
            DMA_Cmd(DMA1_Channel6, DISABLE);
        }
        else
        {
            /* Assign base address */
            DMA1_Channel4->CMAR = (uint32_t)buf;
            DMA1_Channel4->CNDTR = nbyte;

            /* DMA1 Channel1 transfer complete interrupt enable */
            //DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4 | DMA1_IT_HT4);
            //DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

            /* Enable I2C1 DMA */
            I2C_DMACmd(I2C2, ENABLE);
            /* Enable DMA1 Channel4 */
            DMA_Cmd(DMA1_Channel4, ENABLE);
            /* DMA1 Channel4 transfer complete test */
            Timed(!DMA_GetFlagStatus(DMA1_FLAG_TC4));
            /* Disable DMA1 Channel4 */
            DMA_Cmd(DMA1_Channel4, ENABLE);
        }
#else
        {
            uint16_t i;

            for( i = 0; i < nbyte; i++)
            {
                I2C_SendData(I2Cx, buf[i]);
                /// Wait for EV8
                Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
            }
        }
#endif
        /// 5) Sends Stop Condition
        I2C_GenerateSTOP(I2Cx, ENABLE);
    }
    return eSuccess;
errReturn:
    return eError;
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DMA1_Channel6_IRQHandler(void)
{
    //Test on DMA1 Channel1 Transfer Complete interrupt
    if(DMA_GetITStatus(DMA1_IT_TC6))
    {
        /* Disable DMA1 Channel6 */
        DMA_ClearITPendingBit(DMA1_IT_GL6 | DMA1_IT_TC6 | DMA1_IT_HT6);
        DMA_Cmd(DMA1_Channel6, DISABLE);
        /* Send I2C1 STOP Condition */
        I2C_GenerateSTOP(I2C1, ENABLE);
    }
}

///---------------------------------------------------------------------------------------------------------------------
/// @brief
/// @param  None
/// @retval None
///---------------------------------------------------------------------------------------------------------------------
void DMA1_Channel4_IRQHandler(void)
{
    //Test on DMA1 Channel1 Transfer Complete interrupt
    if(DMA_GetITStatus(DMA1_IT_TC4))
    {
        /* Disable DMA1 Channel4 */
        DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4 | DMA1_IT_HT4);
        /* Send I2C1 STOP Condition */
        I2C_GenerateSTOP(I2C2, ENABLE);
    }
}
