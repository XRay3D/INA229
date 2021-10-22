#include "main.h"
#include "../ina229.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include <span>
#include <stdio.h>

#define MEM_TO_USART_7 DMA1, LL_DMA_CHANNEL_7
#define SPI1_NSS SPI1_NSS_GPIO_Port, SPI1_NSS_Pin

void SystemClock_Config(void);

int main(void) {

    /* MCU Configuration--------------------------------------------------------*/

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/

    SystemClock_Config();

    NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    NVIC_DisableIRQ(DMA1_Channel7_IRQn);

    MX_GPIO_Init();

    MX_DMA_Init();

    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_TIM6_Init();

    // LL_TIM_EnableDMAReq_UPDATE(TIM6);

    LL_TIM_EnableCounter(TIM6);

    LL_DMA_SetPeriphAddress(MEM_TO_USART_7, LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT)); // USART << MEM
    LL_USART_EnableDMAReq_TX(USART2);
    LL_USART_Enable(USART2);

    //    auto delay_us = [](uint16_t val) {
    //        TIM6->ARR = 0xFFFF;
    //        TIM6->CNT = 0;
    //        while (TIM6->CNT < val) { }
    //    };

    //    auto write_spi1 = [](uint8_t val) {
    //        LL_SPI_TransmitData8(SPI1, val);
    //        while (LL_SPI_IsActiveFlag_BSY(SPI1)) { }
    //    };

    //    LL_GPIO_SetOutputPin(RESET_GPIO_Port, RESET_Pin);
    //    delay_us(1000);
    //    LL_GPIO_ResetOutputPin(RESET_GPIO_Port, RESET_Pin);
    //    delay_us(1000);

    //    uint8_t PixelDump[][1000] { { 0xAA, 0x55 }, { 0xAA, 0x55 } };
    //    bool fl {};

    INA229 ina229 { SPI1 };

    while (1) {
        if (!ina229) {
            ina229.init();
            continue;
        }

        //        { // Frame Capture
        //            LL_GPIO_ResetOutputPin(SPI1_NSS);
        //            delay_us(1);
        //            write_spi1(0x13 | W);
        //            write_spi1(0x83);
        //            delay_us(1);
        //            LL_GPIO_SetOutputPin(SPI1_NSS);
        //        }
        //        delay_us(100);
        //        fl = !fl;
        //        { // Pixel Dump
        //            LL_GPIO_ResetOutputPin(SPI1_NSS);
        //            delay_us(1);
        //            write_spi1(0x40);
        //            delay_us(50);

        //            LL_DMA_SetDataLength(MEM_TO_SPI_3, 900);
        //            LL_DMA_SetDataLength(SPI_TO_MEM_2, 900);
        //            LL_DMA_SetMemoryAddress(MEM_TO_SPI_3, uint32_t(PixelDump[fl]) + 2);
        //            LL_DMA_SetMemoryAddress(SPI_TO_MEM_2, uint32_t(PixelDump[fl]) + 2);
        //            TIM6->ARR = 7; // 15 us
        //            TIM6->CNT = 0;
        //            LL_DMA_EnableChannel(SPI_TO_MEM_2);
        //            LL_DMA_EnableChannel(MEM_TO_SPI_3);

        //            while (!LL_DMA_IsActiveFlag_TC3(DMA1)) { }
        //            LL_DMA_ClearFlag_TC3(DMA1);
        //            LL_DMA_DisableChannel(MEM_TO_SPI_3);
        //            LL_DMA_DisableChannel(SPI_TO_MEM_2);
        //            delay_us(1);
        //            LL_GPIO_SetOutputPin(SPI1_NSS);

        //            for (auto& pix : std::span { PixelDump[fl] + 2, 900 })
        //                pix <<= 2;
        //        }

        char buf[100] {};
        auto size = sprintf(buf, "dieTemp:% 6.3f, vBus:% 6.3f, vShunt:% 6.3f, current:% 6.3f\n", ina229.dieTemp(), ina229.vBus(), ina229.vShunt(), ina229.current());

        //        char buf[100] {};
        //        auto size = sprintf(buf, "vBus: %f\n", ina229.vBus());

        if (1) { // USART
            while (LL_DMA_IsEnabledChannel(MEM_TO_USART_7) && !LL_DMA_IsActiveFlag_TC7(DMA1)) { }
            LL_DMA_ClearFlag_TC7(DMA1);
            LL_DMA_DisableChannel(MEM_TO_USART_7);
            LL_DMA_SetDataLength(MEM_TO_USART_7, size);
            LL_DMA_SetMemoryAddress(MEM_TO_USART_7, uint32_t(buf));
            LL_DMA_EnableChannel(MEM_TO_USART_7);
        }

        LL_mDelay(100);
        LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_RCC_HSI_Enable();

    while (LL_RCC_HSI_IsReady() != 1) {
    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 10, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();

    while (LL_RCC_PLL_IsReady() != 1) {
    }
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    LL_Init1msTick(80000000);

    LL_SetSystemCoreClock(80000000);
}

/**
 * @brief This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {

    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {

    /* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
