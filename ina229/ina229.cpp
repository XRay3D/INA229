// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

/****************************************************************************
*                                                                           *
* Author    :  Damir Bakiev                                                 *
* Version   :  na                                                           *
* Date      :  22 Okt 2021                                                  *
* Website   :  na                                                           *
* Copyright :  Damir Bakiev 2016-2021                                       *
*                                                                           *
* This software component is licensed by ST under BSD 3-Clause license,     *
* the "License"; You may not use this file except in compliance with the    *
* License. You may obtain a copy of the License at:                         *
*                        opensource.org/licenses/BSD-3-Clause               *
*****************************************************************************/

#include "ina229.h"
#include "main.h"

#define MEM_TO_SPI_3 DMA1, LL_DMA_CHANNEL_3
#define SPI_TO_MEM_2 DMA1, LL_DMA_CHANNEL_2
namespace INA229 {

constexpr uint8_t operator|(Register reg, Register rw) { return uint8_t(reg) + uint8_t(rw); }

constexpr auto size = [](Register Reg) -> uint8_t {
    switch (Reg) {
    case Register::CONFIG:
        return 16 / 8;
    case Register::ADC_CONFIG:
        return 16 / 8;
    case Register::SHUNT_CAL:
        return 16 / 8;
    case Register::SHUNT_TEMPCO:
        return 16 / 8;
    case Register::VSHUNT:
        return 24 / 8;
    case Register::VBUS:
        return 24 / 8;
    case Register::DIETEMP:
        return 16 / 8;
    case Register::CURRENT:
        return 24 / 8;
    case Register::POWER:
        return 24 / 8;
    case Register::ENERGY:
        return 40 / 8;
    case Register::CHARGE:
        return 40 / 8;
    case Register::DIAG_ALRT:
        return 16 / 8;
    case Register::SOVL:
        return 16 / 8;
    case Register::SUVL:
        return 16 / 8;
    case Register::BOVL:
        return 16 / 8;
    case Register::BUVL:
        return 16 / 8;
    case Register::TEMP_LIMIT:
        return 16 / 8;
    case Register::PWR_LIMIT:
        return 16 / 8;
    case Register::MANUFACTURER_ID:
        return 16 / 8;
    case Register::DEVICE_ID:
        return 16 / 8;
    default:
        return {};
    }
};

void Device::dmaRead(Register reg) const {
    data.reg = reg | Register::Read;
    auto size_ = size(reg) + 1;
    LL_DMA_SetDataLength(MEM_TO_SPI_3, size_);
    LL_DMA_SetDataLength(SPI_TO_MEM_2, size_);
    GPIOA->BRR = LL_GPIO_PIN_4;
    LL_DMA_EnableChannel(SPI_TO_MEM_2);
    LL_DMA_EnableChannel(MEM_TO_SPI_3);
    while (!LL_DMA_IsActiveFlag_TC2(DMA1)) { }
    GPIOA->BSRR = LL_GPIO_PIN_4;
    std::reverse(data.data, data.data + size_ - 1);
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableChannel(MEM_TO_SPI_3);
    LL_DMA_DisableChannel(SPI_TO_MEM_2);
}

void Device::dmaWrite(Register reg) const {
    data.reg = reg | Register::Write;
    auto size_ = size(reg) + 1;
    std::reverse(data.data, data.data + size_ - 1);
    LL_DMA_SetDataLength(MEM_TO_SPI_3, size_);
    LL_DMA_SetDataLength(SPI_TO_MEM_2, size_);
    GPIOA->BRR = LL_GPIO_PIN_4;
    LL_DMA_EnableChannel(SPI_TO_MEM_2);
    LL_DMA_EnableChannel(MEM_TO_SPI_3);
    while (!LL_DMA_IsActiveFlag_TC2(DMA1)) { }
    GPIOA->BSRR = LL_GPIO_PIN_4;
    std::reverse(data.data, data.data + size_ - 1);
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableChannel(MEM_TO_SPI_3);
    LL_DMA_DisableChannel(SPI_TO_MEM_2);
}

Device::Device(SPI_TypeDef* SPIx)
    : SPIx { SPIx } {
    init();
}

constexpr uint8_t operator""_ms(unsigned long long val) { return val >> 1; }

void Device::init() {
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    GPIOA->BSRR = LL_GPIO_PIN_4;

    LL_DMA_DisableChannel(MEM_TO_SPI_3);
    LL_DMA_DisableChannel(SPI_TO_MEM_2);
    LL_DMA_SetMemoryAddress(MEM_TO_SPI_3, uint32_t(&data));
    LL_DMA_SetMemoryAddress(SPI_TO_MEM_2, uint32_t(&data));
    LL_DMA_SetPeriphAddress(MEM_TO_SPI_3, LL_SPI_DMA_GetRegAddr(SPIx)); // SPI << MEM
    LL_DMA_SetPeriphAddress(SPI_TO_MEM_2, LL_SPI_DMA_GetRegAddr(SPIx)); // SPI >> MEM
    LL_SPI_EnableDMAReq_RX(SPIx);
    LL_SPI_EnableDMAReq_TX(SPIx);
    LL_SPI_SetNSSMode(SPIx, LL_SPI_NSS_SOFT);
    LL_SPI_Enable(SPIx);

    data.clear();
    dmaRead(Register::MANUFACTURER_ID);
    present = data.manufacturerId.Id1 == 0x54 && data.manufacturerId.Id0 == 0x49; // 'T''I'
    if (!present)
        return;

    data.clear();
    data.config.Rst = RST::SystemReset;
    dmaWrite(Register::CONFIG);

    data.clear();
    data.config.AdcRange = adcrange_ = ADCRANGE::_163_84mV;
    data.config.Convdly = 64_ms;
    //    data.config.RstAcc = RSTACC::ClearsRegistersENERGY_CHARGE;
    dmaWrite(Register::CONFIG);

    data.clear();
    data.adcConfig.avg = AVG::_4;
    data.adcConfig.vbusct = ConvTime::_4120us;
    data.adcConfig.vshct = ConvTime::_4120us;
    data.adcConfig.vtct = ConvTime::_4120us;
    data.adcConfig.mode = MODE::ContinuousTUI;
    dmaWrite(Register::ADC_CONFIG);
}

float Device::vBus() {
    dmaRead(Register::VBUS);
    return data.vbus.value * 0.000'195'312'5f; // Conversion factor: 195.3125 μV/LSB
}

float Device::vShunt() {
    dmaRead(Register::VSHUNT);
    return data.vshunt.value * (adcrange_ == ADCRANGE::_163_84mV ? 0.000'000'312'5f : 0.000'078'125f);
}

float Device::dieTemp() {
    dmaRead(Register::DIETEMP);
    return data.dietemp * 0.007'812'5f; // 7.8125 m°C/LSB
}

float Device::current() {
    dmaRead(Register::CURRENT);
    return data.current.value * CURRENT_LSB;
}

float Device::power() {
    dmaRead(Register::POWER);
    return data.power.value * 3.2 * CURRENT_LSB;
}

float Device::charge() {
    dmaRead(Register::CHARGE);
    return data.charge.value * CURRENT_LSB;
}

float Device::energy() {
    dmaRead(Register::ENERGY);
    return data.energy.value * 16 * 3.2 * CURRENT_LSB;
}

void Device::setShuntRes(float res) {
    if (res_ == res)
        return;
    res_ = res;
    data.clear();
    data.shuntCal.currLsb = 13'107'200'000ULL * CURRENT_LSB * res_;
    dmaWrite(Register::SHUNT_CAL);
}

}
