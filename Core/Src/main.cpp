#include "main.h"
#include "../../ina229/ina229.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
//#include "usart.h"
#include <span>
#include <stdio.h>

#define MEM_TO_USART_7 DMA1, LL_DMA_CHANNEL_7
#define SPI1_NSS SPI1_NSS_GPIO_Port, SPI1_NSS_Pin

extern "C" void SystemClock_Config(void);
void PeriphCommonClock_Config(void);

int main(void)
{

    /* MCU Configuration--------------------------------------------------------*/

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/

    SystemClock_Config();
    PeriphCommonClock_Config();

    NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    NVIC_DisableIRQ(DMA1_Channel7_IRQn);

    MX_GPIO_Init();

    MX_DMA_Init();

    { // USB
        MX_USB_DEVICE_Init();
        LL_GPIO_ResetOutputPin(USB_ENUM_GPIO_Port, USB_ENUM_Pin);
        LL_GPIO_SetOutputPin(USB_ENUM_GPIO_Port, USB_ENUM_Pin);
        LL_mDelay(10);
    }

    //MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_TIM6_Init();

    // LL_TIM_EnableDMAReq_UPDATE(TIM6);

    LL_TIM_EnableCounter(TIM6);

    //    LL_DMA_SetPeriphAddress(MEM_TO_USART_7, LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT)); // USART << MEM
    //    LL_USART_EnableDMAReq_TX(USART2);
    //    LL_USART_Enable(USART2);

    INA229::Device ina229 { SPI1 };
    ina229.setShunt(0.002);
    ina229.setShuntCal(9.346718 /*INA*/ / 9.386 /*REF*/); //~1.0

    auto usartTx = [](char* buf, uint8_t size) {
        CDC_Transmit_FS(reinterpret_cast<uint8_t*>(buf), size);
        //        while (LL_DMA_IsEnabledChannel(MEM_TO_USART_7) && !LL_DMA_IsActiveFlag_TC7(DMA1)) { }
        //        LL_DMA_ClearFlag_TC7(DMA1);
        //        LL_DMA_DisableChannel(MEM_TO_USART_7);
        //        LL_DMA_SetDataLength(MEM_TO_USART_7, size);
        //        LL_DMA_SetMemoryAddress(MEM_TO_USART_7, uint32_t(buf));
        //        LL_DMA_EnableChannel(MEM_TO_USART_7);
        //        while (!LL_DMA_IsActiveFlag_TC7(DMA1)) { }
        LL_mDelay(10);
    };
    char buf[100] {};
    while (1) {
        if (!ina229) {
            usartTx(buf, sprintf(buf, "INA229 is absent!\n"));
        } else {
            if (ina229.conversionIsComplete()) {
                usartTx(buf, sprintf(buf, "\ndieTemp:%f °C\n", ina229.dieTemp()));
                usartTx(buf, sprintf(buf, "vBus:   %f V\n", ina229.vBus()));
                usartTx(buf, sprintf(buf, "vShunt: %f mV\n", ina229.vShunt() * 1000));
                usartTx(buf, sprintf(buf, "current:%f A\n", ina229.current()));
                usartTx(buf, sprintf(buf, "power:  %f W\n", ina229.power()));
                //                usartTx(buf, sprintf(buf, "charge: %f C\n", ina229.charge()));
                //                usartTx(buf, sprintf(buf, "energy: %f J\n", ina229.energy()));
                LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            }
        }
        LL_mDelay(10);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
extern "C" void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_RCC_MSI_Enable();

    /* Wait till MSI is ready */
    while (LL_RCC_MSI_IsReady() != 1) {
    }
    LL_RCC_MSI_EnableRangeSelection();
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
    LL_RCC_MSI_SetCalibTrimming(0);
    LL_PWR_EnableBkUpAccess();
    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
    LL_RCC_LSE_Enable();

    /* Wait till LSE is ready */
    while (LL_RCC_LSE_IsReady() != 1) {
    }
    LL_RCC_MSI_EnablePLLMode();
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(80000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
}

void PeriphCommonClock_Config(void)
{
    LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 24, LL_RCC_PLLSAI1Q_DIV_2);
    LL_RCC_PLLSAI1_EnableDomain_48M();
    LL_RCC_PLLSAI1_Enable();

    /* Wait till PLLSAI1 is ready */
    while (LL_RCC_PLLSAI1_IsReady() != 1) {
    }
}

/**
 * @brief This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{

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
void assert_failed(uint8_t* file, uint32_t line)
{

    /* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
