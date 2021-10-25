// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

/****************************************************************************
*                                                                       *
* Author    :  Damir Bakiev                                                 *
* Version   :  na                                                           *
* Date      :  22 Okt 2021                                                  *
* Website   :  na                                                           *
* Copyright :  Damir Bakiev 2016-2021                                       *
*                                                                       *
* This software component is licensed by ST under BSD 3-Clause license,     *
* the "License"; You may not use this file except in compliance with the    *
* License. You may obtain a copy of the License at:                         *
*                    opensource.org/licenses/BSD-3-Clause               *
*****************************************************************************/

#include "ina229.h"
#include "main.h"

/*
╔════════════════╤═══════════════════╤════════════════════╤════════════════════╤════════════════════╗
║ ADC CONVERSION │   OUTPUT SAMPLE   │   OUTPUT SAMPLE    │  NOISE-FREE ENOB   │  NOISE-FREE ENOB   ║
║  TIME PERIOD   │AVERAGING [SAMPLES]│     PERIOD[ms]     │    (±163.84-mV)    │     (±40.96-mV)    ║
║     [µs]       │                   │                    │    (ADCRANGE=0)    │    (ADCRANGE = 1)  ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │0.05                │12.4                │10.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │0.084               │12.6                │10.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │0.15                │13.3                │11.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │0.28                │13.8                │11.8                ║
╟────────────────┤         1         ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │0.54                │14.2                │12.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │1.052               │14.5                │12.6                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │2.074               │15.3                │13.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │4.12                │16.0                │13.8                ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │0.2                 │13.1                │11.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │0.336               │13.9                │11.8                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │0.6                 │14.3                │12.2                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │1.12                │14.9                │12.8                ║
╟────────────────┤         4         ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │2.16                │15.1                │13.0                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │4.208               │15.8                │13.8                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │8.296               │16.1                │14.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │16.48               │16.5                │14.4                ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │0.8                 │13.9                │12.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │1.344               │14.7                │12.9                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │2.4                 │15.1                │13.0                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │4.48                │15.8                │13.7                ║
╟────────────────┤        16         ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │8.64                │16.3                │14.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │16.832              │16.5                │14.6                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │33.184              │17.1                │15.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │65.92               │17.7                │15.9                ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │3.2                 │15.0                │13.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │5.376               │15.9                │13.8                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │9.6                 │16.4                │14.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │17.92               │16.9                │14.5                ║
╟────────────────┤        64         ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │34.56               │17.7                │15.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │67.328              │17.7                │15.9                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │132.736             │18.1                │16.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │263.68              │18.7                │16.5                ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │6.4                 │15.5                │13.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │10.752              │16.3                │14.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │19.2                │16.9                │14.7                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │35.84               │17.1                │15.2                ║
╟────────────────┤        128        ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │69.12               │18.1                │15.9                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │134.656             │18.1                │16.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │265.472             │18.7                │16.9                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │527.36              │19.7                │17.1                ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │12.8                │15.5                │14.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │21.504              │16.7                │14.7                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │38.4                │17.4                │15.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │71.68               │17.7                │15.7                ║
╟────────────────┤        256        ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │138.24              │18.7                │16.1                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │269.312             │18.7                │16.7                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │530.944             │19.7                │17.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │1054.72             │19.7                │17.7                ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │25.6                │16.7                │14.3                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │43                  │17.4                │15.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │76.8                │17.7                │15.5                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │143.36              │18.7                │16.3                ║
╟────────────────┤        512        ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │276.48              │18.7                │16.5                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │538.624             │19.7                │17.4                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │1061.888            │19.7                │17.7                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │2109.44             │19.7                │18.7                ║
╟────────────────┼───────────────────┼────────────────────┼────────────────────┼────────────────────╢
║50              │                   │51.2                │17.1                │15.0                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║84              │                   │86.016              │17.7                │15.9                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║150             │                   │153.6               │18.1                │16.0                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║280             │                   │286.72              │18.7                │16.9                ║
╟────────────────┤       1024        ├────────────────────┼────────────────────┼────────────────────╢
║540             │                   │552.96              │19.7                │17.1                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║1052            │                   │1077.248            │19.7                │17.7                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║2074            │                   │2123.776            │19.7                │18.1                ║
╟────────────────┤                   ├────────────────────┼────────────────────┼────────────────────╢
║4120            │                   │4218.88             │20                  │18.7                ║
╚════════════════╧═══════════════════╧════════════════════╧════════════════════╧════════════════════╝

╔════════════════════╤═════════════════════════╤════════════════════╗
║   PARAMETER        │    FULL SCALE VALUE     │     RESOLUTION     ║
╟────────────────────┼─────────────────────────┼────────────────────╢
║                    │±163.84 mV (ADCRANGE = 0)│312.5 nV/LSB        ║
║    Shunt voltage   ├─────────────────────────┼────────────────────╢
║                    │ ±40.96 mV (ADCRANGE = 1)│78.125 nV/LSB       ║
╟────────────────────┼─────────────────────────┼────────────────────╢
║    Bus voltage     │0 V to 85 V              │195.3125 μV/LSB     ║
╟────────────────────┼─────────────────────────┼────────────────────╢
║    Temperature     │–40°C to +125°C          │7.8125 m°C/LSB      ║
╚════════════════════╧═════════════════════════╧════════════════════╝
*/

#define MEM_TO_SPI_3 DMA1, LL_DMA_CHANNEL_3
#define SPI_TO_MEM_2 DMA1, LL_DMA_CHANNEL_2
namespace INA229 {

constexpr float ShuntVoltage163_84mV_LSB = 0.000000'312'5f;
constexpr float ShuntVoltage40_96mV_LSB = 0.000000'078'125f;
constexpr float BusVoltageLSB = 0.000195'312'5f;
constexpr float TemperatureLSB = 0.007812'5f;

constexpr float shuntVoltageLSB(ADCRANGE adcRange) { return adcRange == ADCRANGE::_163_84mV ? ShuntVoltage163_84mV_LSB : ShuntVoltage40_96mV_LSB; }

constexpr uint8_t operator|(Register reg, Register rw) { return uint8_t(reg) | uint8_t(rw); }

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

void Device::dmaRead(Register reg) const
{
    data.reg = reg | Register::Read;
    const auto size_ = size(reg);
    LL_DMA_SetDataLength(MEM_TO_SPI_3, size_ + 1);
    LL_DMA_SetDataLength(SPI_TO_MEM_2, size_ + 1);
    GPIOA->BRR = LL_GPIO_PIN_4;
    LL_DMA_EnableChannel(SPI_TO_MEM_2);
    LL_DMA_EnableChannel(MEM_TO_SPI_3);
    while (!LL_DMA_IsActiveFlag_TC2(DMA1)) { }
    GPIOA->BSRR = LL_GPIO_PIN_4;
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableChannel(MEM_TO_SPI_3);
    LL_DMA_DisableChannel(SPI_TO_MEM_2);
    std::reverse(data.data, data.data + size_);
}

void Device::dmaWrite(Register reg) const
{
    data.reg = reg | Register::Write;
    const auto size_ = size(reg);
    std::reverse(data.data, data.data + size_);
    LL_DMA_SetDataLength(MEM_TO_SPI_3, size_ + 1);
    LL_DMA_SetDataLength(SPI_TO_MEM_2, size_ + 1);
    GPIOA->BRR = LL_GPIO_PIN_4;
    LL_DMA_EnableChannel(SPI_TO_MEM_2);
    LL_DMA_EnableChannel(MEM_TO_SPI_3);
    while (!LL_DMA_IsActiveFlag_TC2(DMA1)) { }
    GPIOA->BSRR = LL_GPIO_PIN_4;
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableChannel(MEM_TO_SPI_3);
    LL_DMA_DisableChannel(SPI_TO_MEM_2);
    std::reverse(data.data, data.data + size_);
}

Device::Device(SPI_TypeDef* SPIx)
    : SPIx { SPIx }
{
    init();
}

bool Device::setPPM(uint16_t ppm)
{
    if (ppm > 0x3FFF)
        return {};
    data.clear();
    data.shuntTempco.value = ppm;
    dmaWrite(Register::SHUNT_TEMPCO);
    dmaRead(Register::CONFIG);
    data.config.tempComp = TEMPCOMP::ShuntTemperatureCompensationEnabled;
    dmaWrite(Register::SHUNT_TEMPCO);
    return true;
}

constexpr uint8_t operator""_ms(unsigned long long val) { return val >> 1; }

void Device::init()
{
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
    //LL_SPI_SetNSSMode(SPIx, LL_SPI_NSS_SOFT);
    LL_SPI_Enable(SPIx);

    data.clear();
    dmaRead(Register::MANUFACTURER_ID);
    present = data.manufacturerId == 0x5449; // 'T''I'
    if (!present)
        return;

    data.clear();
    data.config.reset = RST::SystemReset;
    dmaWrite(Register::CONFIG);
    do {
        dmaRead(Register::CONFIG);
    } while (data.config.reset == RST::SystemReset);

    data.clear();
    data.config.adcRange = adcRange_ = ADCRANGE::_40_96mV;
    //data.config.adcRange = adcRange_ = ADCRANGE::_163_84mV;
    data.config.convDelay = 64_ms;
    dmaWrite(Register::CONFIG);

    data.clear();
    data.adcConfig.avg = AVG::_1024;
    data.adcConfig.vBusConvTime = ConvTime::_1052us;
    data.adcConfig.vShtConvTime = ConvTime::_1052us;
    data.adcConfig.tempConvTime = ConvTime::_1052us;
    data.adcConfig.mode = MODE::ContinuousTUI;
    dmaWrite(Register::ADC_CONFIG);
}

float Device::vBus()
{
    dmaRead(Register::VBUS);
    return data.vbus.value * BusVoltageLSB;
}

float Device::vShunt()
{
    dmaRead(Register::VSHUNT);
    return data.vshunt.value * shuntVoltageLSB(adcRange_);
}

float Device::dieTemp()
{
    dmaRead(Register::DIETEMP);
    return data.dietemp * TemperatureLSB;
}

float Device::current()
{
    dmaRead(Register::CURRENT);
    return data.current.value * currentLsb;
}

float Device::power()
{
    dmaRead(Register::POWER);
    return data.power.value * 3.2 * currentLsb;
}

float Device::charge()
{
    dmaRead(Register::CHARGE);
    return data.charge.value * currentLsb;
}

float Device::energy()
{
    dmaRead(Register::ENERGY);
    return data.energy.value * 16 * 3.2 * currentLsb;
}

void Device::setShunt(float shuntRes)
{
    if (shuntRes_ == shuntRes)
        return;
    shuntRes_ = shuntRes;
    currentLsb = shuntVoltageLSB(adcRange_) / shuntRes;
}

void Device::setShuntCal(float shuntCal)
{
    if (shuntCal == 1.0)
        return;
    data.clear();
    data.shuntCal.value = 0x1000 * shuntCal; // 13'107'200'000ULL * shuntVoltageLSB(adcRange_);
    dmaWrite(Register::SHUNT_CAL);
}

}
