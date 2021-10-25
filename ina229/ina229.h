#pragma once
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
#include "stm32l4xx.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace INA229 {

enum class Register : uint8_t {
    // clang-format off
        CONFIG =            0x00 << 2, // Configuration (bits 16)
        ADC_CONFIG =        0x01 << 2, // ADC Configuration (bits 16)
        SHUNT_CAL =         0x02 << 2, // Shunt Calibration (bits 16)
        SHUNT_TEMPCO =      0x03 << 2, // Shunt Temperature Coefficient (bits 16)
        VSHUNT =            0x04 << 2, // Shunt Voltage Measurement (bits 16)
        VBUS =              0x05 << 2, // Bus Voltage Measurement (bits 16)
        DIETEMP =           0x06 << 2, // Temperature Measurement (bits 16)
        CURRENT =           0x07 << 2, // Current Result (bits 16)
        POWER =             0x08 << 2, // Power Result (bits 16)
        ENERGY =            0x09 << 2, // Energy Result (bits 16)
        CHARGE =            0x0A << 2, // Charge Result (bits 16)
        DIAG_ALRT =         0x0B << 2, // Diagnostic Flags and Alert (bits 16)
        SOVL =              0x0C << 2, // Shunt Overvoltage Threshold (bits 16)
        SUVL =              0x0D << 2, // Shunt Undervoltage Threshold (bits 16)
        BOVL =              0x0E << 2, // Bus Overvoltage Threshold (bits 16)
        BUVL =              0x0F << 2, // Bus Undervoltage Threshold (bits 16)
        TEMP_LIMIT =        0x10 << 2, // Temperature Over-Limit Threshold (bits 16)
        PWR_LIMIT =         0x11 << 2, // Power Over-Limit Threshold (bits 16)
        MANUFACTURER_ID =   0x3E << 2, // Manufacturer ID (bits 16)
        DEVICE_ID =         0x3F << 2, // Device ID (unsigned bits:1)

        Read =              0x01,
        Write =             0x00,
        };

#pragma pack(push, 1)

enum class RST : uint8_t {
    //Reset Bit. Setting this bit to '1' generates a system reset that is the same as power-on reset.
    NormalOperation,
    SystemReset=1
};

enum class RSTACC : uint8_t {
    //Resets the contents of accumulation registers ENERGY and CHARGE to 0
    NormalOperation,
    ClearsRegistersENERGY_CHARGE=1
};

enum class TEMPCOMP : uint8_t {
    //Enables temperature compensation of an external shunt
    ShuntTemperatureCompensationDisabled,
    ShuntTemperatureCompensationEnabled=1
};

enum class ADCRANGE : uint8_t {
    //Shunt full scale range selection across IN+ and IN–.
    _163_84mV,
    _40_96mV=1
};

struct CONFIG {
    unsigned _        : 4; // R
    ADCRANGE adcRange : 1; // R/W Shunt full scale range selection across IN+ and IN–.
    TEMPCOMP tempComp : 1; // R/W Enables temperature compensation of an external shunt
    unsigned convDelay: 8; // R/W Sets the Delay for initial ADC conversion in steps of unsigned ms:1;//.
                           //     0h = 0 s
                           //     1h = 2 ms
                           //     FFh = 510 ms
    RSTACC rstAcc     : 1; // R/W Resets the contents of accumulation registers ENERGY and CHARGE to 0
    RST reset         : 1; // R/W Reset Bit. Setting this bit to '1' generates a system reset that is the same as power-on reset. Resets all registers to default values.
};

enum class MODE : uint8_t { // The user can set the MODE bits for continuous or triggered mode on bus voltage, shunt voltage or temperature measurement.
    Shutdown =      0x0, //0h = Shutdown
    TriggeredU =    0x1, //1h = Triggered bus voltage, single shot
    TriggeredI =    0x2, //2h = Triggered shunt voltage triggered, single shot
    TriggeredUI =   0x3, //3h = Triggered shunt voltage and bus voltage, single shot
    TriggeredT =    0x4, //4h = Triggered temperature, single shot
    TriggeredTU =   0x5, //5h = Triggered temperature and bus voltage, single shot
    TriggeredTI =   0x6, //6h = Triggered temperature and shunt voltage, single shot
    TriggeredTUI =  0x7, //7h = Triggered bus voltage, shunt voltage and temperature, single shot
    //Shutdown =    0x8, //8h = Shutdown
    ContinuousU =   0x9, //9h = Continuous bus voltage only
    ContinuousI =   0xA, //Ah = Continuous shunt voltage only
    ContinuousUI =  0xB, //Bh = Continuous shunt and bus voltage
    ContinuousT =   0xC, //Ch = Continuous temperature only
    ContinuousTU =  0xD, //Dh = Continuous bus voltage and temperature
    ContinuousTI =  0xE, //Eh = Continuous temperature and shunt voltage
    ContinuousTUI = 0xF, //Fh = Continuous bus, shunt voltage and temperature
};

    enum class ConvTime : uint8_t {
        // Sets the conversion time of the bus voltage measurement:
        _50us = 0x0,
        _84us = 0x1,
        _150us = 0x2,
        _280us = 0x3,
        _540us = 0x4,
        _1052us = 0x5,
        _2074us = 0x6,
        _4120us = 0x7,
        };

    enum class AVG : uint8_t {
        /* Selects ADC sample averaging count.
         * The averaging setting applies to all active inputs.
         * When >0h, the output registers are updated after the averaging has completed.
         */
        _1 = 0x0,
        _4 = 0x1,
        _16 = 0x2,
        _64 = 0x3,
        _128 = 0x4,
        _256 = 0x5,
        _512 = 0x6,
        _1024 = 0x7,
        };

struct ADC_CONFIG {
    AVG avg         : 3; // R/W (1) Selects ADC sample averaging count. The averaging setting applies to all active inputs.
                         //          When >0h, the output registers are updated after the averaging has completed.
    ConvTime tempConvTime   : 3; // R/W (1052us) Sets the conversion time of the temperature measurement:
    ConvTime vShtConvTime  : 3; // R/W (1052us) Sets the conversion time of the shunt voltage measurement:
    ConvTime vBusConvTime : 3; // R/W (1052us) Sets the conversion time of the bus voltage measurement:
    MODE mode       : 4; // R/W (ContinuousTUI) The user can set the MODE bits for continuous or triggered mode on bus voltage, shunt voltage or temperature measurement.
};

struct SHUNT_CAL {
    uint16_t value : 15; //R/W 1000h The register provides the device with a conversion constant value
                           //          that represents shunt resistance used to calculate current value in Amperes.
                           //          This also sets the resolution for the CURRENT register.
                           //          Value calculation under Section 8.1.2.
    unsigned _      : 1;
};

struct SHUNT_TEMPCO {
    uint16_t value : 14;   //R/W 0h Temperature coefficient of the shunt for temperature compensation correction.
                           //       Calculated with respect to +25°C.
                           //       The full scale value of the register is 16383 ppm/°C.
                           //       The 16 bit register provides a resolution of 1ppm/°C/LSB
                           //       0h = 0 ppm/°C
                           //       3FFFh = 16383 ppm/°C
    unsigned _     : 2;
};

struct VSHUNT {
    unsigned _    : 4;
    int32_t value : 20; //R 0h Differential voltage measured across the shunt output. Two's complement value.
                         //     Conversion factor:
                         //     312.5 nV/LSB when ADCRANGE = 0
                         //     78.125 nV/LSB when ADCRANGE = 1
};

struct VBUS {
    unsigned _     : 4;
    uint32_t value : 20; //R 0h Bus voltage output. Two's complement value, however always positive.
                         //     Conversion factor: 195.3125 μV/LSB
};

using DIETEMP = uint16_t; //R 0h Internal die temperature measurement. Two's complement value. Conversion factor: 7.8125 m°C/LSB

struct CURRENT {
    unsigned : 4;
    int32_t value : 20; //R 0h Calculated current output in Amperes. Two's complement value. Value description under Section 8.1.2.
};

struct POWER {
    unsigned value : 24; //R 0h Calculated power output. Output value in watts. Unsigned representation.
                         //     Positive value. Value description under Section 8.1.2.
};

struct ENERGY {
    uint64_t value : 40; //R 0h Calculated energy output. Output value is in Joules.Unsigned representation. Positive value.
                         //     Value description under Section 8.1.2.
};

struct CHARGE {
    int64_t value : 40; //R 0h Calculated charge output. Output value is in Coulombs.Two's complement value.
                         //     Value description under Section 8.1.2.

};

enum class ALATCH : uint8_t {
    //R/W 0h When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit reset to the idle state when the fault has been cleared.
    //       When the Alert Latch Enable bit is set to Latch mode, the Alert pin and Alert Flag bit remain active following a fault until the DIAG_ALRT
    //       Register has been read.
    Transparent,
    Latched
};

enum class CNVR : uint8_t {
    //R/W 0h Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a
    //       conversion cycle has completed.
    DisableConversionReadyFlag, //0h = Disable conversion ready flag on ALERT pin
    EnablesConversionReadyFlag, //1h = Enables conversion ready flag on ALERT pin
};

enum class SLOWALERT : uint8_t {
    //R/W 0h ALERT function is asserted on the completed averaged value.
    //       This gives the flexibility to delay the ALERT after the averaged value.
    AlertComparisonOnNonAveragedValue, // 0h = ALERT comparison on non-averaged (ADC) value
    AlertComparisonOnAveragedValue,    // 1h = ALERT comparison on Averaged value
};

enum class APOL : uint8_t {
    //R/W 0h Alert Polarity bit sets the Alert pin polarity.
    Normal,   // 0h = Normal (Active-low, open-drain)
    Inverted, // 1h = Inverted (active-high, open-drain )
};

enum class ENERGYOF : uint8_t {
    //R 0h This bit indicates the health of the ENERGY register.
    //     If the 40 bit ENERGY register has overflowed this bit is set to 1.
    //     Clears when the ENERGY register is read.
    Normal,   // 0h = Normal
    Overflow, // 1h = Overflow
};

enum class CHARGEOF : uint8_t {
    //R 0h This bit indicates the health of the CHARGE register.
    //      If the 40 bit CHARGE register has overflowed this bit is set to 1.
    //      Clears when the CHARGE register is read.
    Normal,   // 0h = Normal
    Overflow, // 1h = Overflow
};

enum class MATHOF : uint8_t {
    //R 0h This bit is set to 1 if an arithmetic operation resulted in an overflow error.
    //      It indicates that current and power data may be invalid.
    //      Must be manually cleared by triggering another conversion or by clearing the accumulators with the RSTACC bit.
    Normal,  // 0h = Normal
    Overflow // 1h = Overflow
};

enum class TMPOL : uint8_t {
    //R/W 0h This bit is set to 1 if the temperature measurement exceeds the threshold limit in the temperature over-limit register.
    //       When ALATCH = 1 this bit is cleared by reading the DIAG_ALERT register.
    Normal,       // 0h = Normal
    OverTempEvent,// 1h = Over Temp Event
};

enum class SHNTOL : uint8_t {
    //R/W 0h This bit is set to 1 if the shunt voltage measurement exceeds the threshold limit in the shunt over-limit register.
    //       When ALATCH = 1 this bit is cleared by reading the register.
    Normal,                // 0h = Normal
    OverShuntVoltageEvent, // 1h = Over Shunt Voltage Event
};

enum class SHNTUL : uint8_t {
    //R/W 0h This bit is set to 1 if the shunt voltage measurement falls below the threshold limit in the shunt under-limit register.
    //       When ALATCH = 1 this bit is cleared by reading the register.
    Normal,                // 0h = Normal
    UnderShuntVoltageEvent // 1h = Under Shunt Voltage Event
};

enum class BUSOL : uint8_t {
    //R/W 0h This bit is set to 1 if the bus voltage measurement exceeds the threshold limit in the bus over-limit register.
    //       When ALATCH = 1 this bit is cleared by reading the register.
    Normal,           // 0h = Normal
    BusOverLimitEvent // 1h = Bus Over-Limit Event
};

enum class BUSUL : uint8_t {
    //R/W 0h This bit is set to 1 if the bus voltage measurement falls below the threshold limit in the bus under-limit register.
    //       When ALATCH = 1 this bit is cleared by reading the register.
    Normal,            // 0h = Normal
    BusUnderLimitEvent // 1h = Bus Under-Limit Event
};

enum class POL : uint8_t {
    //R/W 0h This bit is set to 1 if the power measurement exceeds the threshold limit in the power limit register.
    //       When ALATCH = 1 this bit is cleared by reading the register.
    Normal,              // 0h = Normal
    PowerOverLimitEvent, // 1h = Power Over-Limit Event
};

enum class CNVRF : uint8_t {
    //R/W 0h This bit is set to 1 if the conversion is completed.
    //       When ALATCH =1 this bit is cleared by reading the register or starting a new triggered conversion.
    Normal,              // 0h = Normal
    ConversionIsComplete // 1h = Conversion is complete
};

enum class MEMSTAT : uint8_t {
    //R/W 1h This bit is set to 0 if a checksum error is detected in the device trim memory space.
    MemoryChecksumError, // 0h = Memory Checksum Error
    NormalOperation      // 1h = Normal Operation
};

struct DIAG_ALRT {
    MEMSTAT memStat     : 1; // R/W 1h
    CNVRF cnvrf         : 1; // R/W 0h
    POL pol             : 1; // R/W 0h
    BUSUL busUl         : 1; // R/W 0h
    BUSOL busOl         : 1; // R/W 0h
    SHNTUL shntUl       : 1; // R/W 0h
    SHNTOL shntOl       : 1; // R/W 0h
    TMPOL tmpOl         : 1; // R/W 0h
    unsigned _          : 1; // R   0h
    MATHOF mathOf       : 1; // R   0h
    CHARGEOF chargeOf   : 1; // R   0h
    ENERGYOF energyOf   : 1; // R   0h
    APOL apOl           : 1; // R/W 0h
    SLOWALERT slowAlert : 1; // R/W 0h
    CNVR cnvr           : 1; // R/W 0h
    ALATCH aLatch       : 1; // R/W 0h
};

using SOVL = uint16_t;  //R/W 7FFFh Sets the threshold for comparison of the value to detect Shunt Overvoltage (overcurrent protection). Two's complement value.
                        //          Conversion Factor:
                        //          5 μV/LSB when ADCRANGE = 0
                        //          1.25 μV/LSB when ADCRANGE = 1.


using SUVL = uint16_t;  //R/W 8000h Sets the threshold for comparison of the value to detect Shunt Undervoltage (undercurrent protection). Two's complement value.
                        //          Conversion Factor:
                        //          5 μV/LSB when ADCRANGE = 0
                        //          1.25 μV/LSB when ADCRANGE = 1.
struct BOVL {
    unsigned busOvl : 15; //R/W 7FFFh Sets the threshold for comparison of the value to detect Bus Overvoltage (overvoltage protection). Unsigned representation,
                        //          positive value only. Conversion factor: 3.125 mV/LSB.
    unsigned _      : 1;
};

struct BUVL {
    unsigned busUvl : 15; //R/W 0h Sets the threshold for comparison of the value to detect Bus Undervoltage (undervoltage protection). Unsigned representation,
                        //            positive value only. Conversion factor: 3.125 mV/LSB.
    unsigned _      : 1;
};

using TEMP_LIMIT /*TOL*/= uint16_t;  //R/W 7FFFh Sets the threshold for comparison of the value to detect over temperature measurements. Two's complement value.
                                     //          The value entered in this field compares directly against the value from the DIETEMP register to determine
                                     //          if an over temperature condition exists. Conversion factor: 7.8125 m°C/LSB.

using PWR_LIMIT /*POL*/ = uint16_t;  //R/W FFFFh Sets the threshold for comparison of the value to detect power overlimit measurements.
                                     //          Unsigned representation, positive value only.
                                     //          The value entered in this field compares directly against the value from the POWER register to determine
                                     //          if an over power condition exists. Conversion factor: 256 × Power LSB.

using MANUFACTURER_ID = uint16_t;    //R 5449h Reads back TI in ASCII.


struct DEVICE_ID {
    unsigned revId :  4; // R 1h    Device revision identification.
    unsigned dieId : 12; // R 229h  Stores the device identification bits.
};
#pragma pack(pop)
// clang-format on

///////////////////////////////////////////////////////////////////////
/// \brief The Device class
///
class Device {

public:
    Device(SPI_TypeDef* SPIx);

    //////////////////////////////////
    /// \brief Настройка INA229
    void init();

    //////////////////////////////////
    /// @brief Установка сопротивления шунта в омах
    /// @param Значение в омах.
    void setShunt(float shuntRes);

    //////////////////////////////////
    /// @brief Установка калибровки шунта (INA/REF -> ~1.0)
    /// @param Значение ~1.0.
    void setShuntCal(float shuntCal);

    //////////////////////////////////
    /// \brief Установка ТКС шунта
    /// \param ppm 0 - 16'383
    /// \return 0 если переполнение
    bool setPPM(uint16_t ppm);

    //////////////////////////////////
    /// \brief 1 если присутствует
    operator bool() const { return present; }

    //////////////////////////////////
    /// \brief Напряжение В
    /// \return
    float vBus();

    //////////////////////////////////
    /// \brief Напряжение шунта В
    /// \return
    float vShunt();

    //////////////////////////////////
    /// \brief Температура °C
    /// \return
    float dieTemp();

    //////////////////////////////////
    /// \brief Ток А
    /// \return
    float current();

    //////////////////////////////////
    /// \brief Мощьность Вт
    /// \return
    float power();

    //////////////////////////////////
    /// \brief Заряд Сулон?
    /// \return
    float charge();

    //////////////////////////////////
    /// \brief Энергия Дж
    /// \return
    float energy();

    //////////////////////////////////
    /// \brief Регистри DIAG_ALRT
    auto getDiagAlrt() const
    {
        dmaRead(Register::DIAG_ALRT);
        return data.diagAlrt;
    };

    //////////////////////////////////
    /// \brief Прверка готовности измерения
    /// \return
    bool conversionIsComplete() const { return getDiagAlrt().cnvrf == CNVRF::ConversionIsComplete; }

private:
    SPI_TypeDef* const SPIx;

    float shuntRes_;

#pragma pack(push, 1)
    mutable struct Data {
        uint8_t reg;
        union {
            ADC_CONFIG adcConfig;
            BOVL bovl;
            BUVL buvl;
            CHARGE charge;
            CONFIG config;
            CURRENT current;
            DEVICE_ID deviceId;
            DIAG_ALRT diagAlrt;
            DIETEMP dietemp;
            ENERGY energy;
            MANUFACTURER_ID manufacturerId;
            POWER power;
            PWR_LIMIT pwrLimit;
            SHUNT_CAL shuntCal;
            SHUNT_TEMPCO shuntTempco;
            SOVL sovl;
            SUVL suvl;
            TEMP_LIMIT tempLimit;
            VBUS vbus;
            VSHUNT vshunt;
            uint8_t data[1];
            uint64_t test64;
        };
        void clear() { std::memset(this, 0, sizeof *this); }
    } data;
#pragma pack(pop)
    ADCRANGE adcRange_ {};
    bool present;
    float currentLsb {};

    //////////////////////////////////
    /// \brief Чтение
    /// \param Register Id
    void dmaRead(Register reg) const;

    //////////////////////////////////
    /// \brief Запись
    /// \param Register Id
    void dmaWrite(Register reg) const;
};

}
