#pragma once

#include "stm32l4xx.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

class INA229 {

public:
    INA229(SPI_TypeDef* SPIx);

    void init();

    operator bool() const { return present; }

    float vBus();
    float vShunt();
    float dieTemp();
    float current();

    enum class Register : uint8_t {
        // clang-format off
        //Address Acronym               Register Name Register Size (bits) Section
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
        // clang-format on
    };

private:
    SPI_TypeDef* const SPIx;
    bool present;

#pragma pack(push, 1)

    enum class RST : uint8_t { //Reset Bit. Setting this bit to '1' generates a system reset that is the same as power-on reset.
        NormalOperation,
        SystemReset
    };

    enum class RSTACC : uint8_t { //Resets the contents of accumulation registers ENERGY and CHARGE to 0
        NormalOperation,
        ClearsRegistersENERGY_CHARGE
    };

    enum class TEMPCOMP : uint8_t { //Enables temperature compensation of an external shunt
        ShuntTemperatureCompensationDisabled,
        ShuntTemperatureCompensationEnabled
    };

    enum class ADCRANGE : uint8_t { //Shunt full scale range selection across IN+ and IN–.
        _163_84mV,
        _40_96mV
    };

    struct CONFIG {
        unsigned /*RESERVED*/ : 3; //R
        ADCRANGE Adcrange : 1; //R/W
        TEMPCOMP Tempcomp : 1; //R/W
        unsigned Convdly : 8; //R/W Sets the Delay for initial ADC conversion in steps of unsigned ms:1;//.
        //                      0h = 0 s
        //                      1h = 2 ms
        //                      FFh = 510 ms
        RSTACC Rstacc : 1; //R/W
        RST Rst : 1; //R/W
    };

    enum class MODE : uint8_t { // The user can set the MODE bits for continuous or triggered mode on bus voltage, shunt voltage or temperature measurement.
        Shutdown = 0x0, //0h = Shutdown
        TriggeredU = 0x1, //1h = Triggered bus voltage, single shot
        TriggeredI = 0x2, //2h = Triggered shunt voltage triggered, single shot
        TriggeredUI = 0x3, //3h = Triggered shunt voltage and bus voltage, single shot
        TriggeredT = 0x4, //4h = Triggered temperature, single shot
        TriggeredTU = 0x5, //5h = Triggered temperature and bus voltage, single shot
        TriggeredTI = 0x6, //6h = Triggered temperature and shunt voltage, single shot
        TriggeredTUI = 0x7, //7h = Triggered bus voltage, shunt voltage and temperature, single shot
        Shutdown_ = 0x8, //8h = Shutdown
        ContinuousU = 0x9, //9h = Continuous bus voltage only
        ContinuousI = 0xA, //Ah = Continuous shunt voltage only
        ContinuousUI = 0xB, //Bh = Continuous shunt and bus voltage
        ContinuousT = 0xC, //Ch = Continuous temperature only
        ContinuousTU = 0xD, //Dh = Continuous bus voltage and temperature
        ContinuousTI = 0xE, //Eh = Continuous temperature and shunt voltage
        ContinuousTUI = 0xF, //Fh = Continuous bus, shunt voltage and temperature
    };

    enum class ConvTime : uint8_t { // Sets the conversion time of the bus voltage measurement:
        _50us = 0x0,
        _84us = 0x1,
        _150us = 0x2,
        _280us = 0x3,
        _540us = 0x4,
        _1052us = 0x5,
        _2074us = 0x6,
        _4120us = 0x7,
    };

    enum class AVG : uint8_t { /* Selects ADC sample averaging count. The averaging setting applies to all active inputs. When >0h, the output registers are updated after the averaging has completed.*/
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
        AVG avg : 3; //R/W
        ConvTime vtct : 3; //R/W
        ConvTime vshct : 3; //R/W
        ConvTime vbusct : 3; //R/W
        MODE mode : 4; //R/W
    };

    struct SHUNT_CAL {
        uint16_t currLsb : 15; //R/W 1000h
        unsigned : 1;
    };

    struct SHUNT_TEMPCO {
        uint16_t currLsb : 14; //R/W 0h
        unsigned : 2;
    };

    struct VSHUNT {
        unsigned : 4;
        uint32_t value : 20; //R
    };

    struct VBUS {
        unsigned : 4;
        uint32_t value : 20; //R
    };

    using DIETEMP = uint16_t; //R Internal die temperature measurement. Two's complement value.Conversion factor: 7.unsigned m°C/LS:1;//B

    struct CURRENT {
        unsigned : 4;
        uint32_t value : 20; //R
    };

    struct POWER {
        unsigned power : 24; //R
    };

    struct ENERGY {
        uint64_t Energy : 40; //R
    };

    struct CHARGE {
        uint64_t charge : 40; //R
    };

    enum class ALATCH { //	R/W	0h	When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit reset to the idle state when the fault has been cleared.
        //When the Alert Latch Enable bit is set to Latch mode,
        //the Alert pin and Alert Flag bit remain active following a fault until the DIAG_ALRT Register has been read.
        Transparent,
        Latched
    };

    enum class CNVR { //	R/W	0h	Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a conversion cycle has completed.
        Disable, // conversion ready flag on ALERT pin
        Enables, // conversion ready flag on ALERT pin
    };

    enum class SLOWALERT { //	R/W	0h	ALERT function is asserted on the completed averaged value.
        //This gives the flexibility to delay the ALERT after the averaged value
        _0h, //ALERT comparison on non - averaged(ADC) value
        _1h // ALERT comparison on Averaged value
    };

    enum class APOL { //	R/W	0h	Alert Polarity bit sets the Alert pin polarity.
        Normal, //(Active - low, open - drain)
        Inverted //(active - high, open - drain)
    };

    enum class ENERGYOF { //	R	0h	This bit indicates the health of the ENERGY register.
        //If the 40 bit ENERGY register has overflowed this bit is set to 1.
        Normal,
        Overflow
        //Clears when the ENERGY register is read.
    };

    enum class CHARGEOF { //	R	0h	This bit indicates the health of the CHARGE register.
        //If the 40 bit CHARGE register has overflowed this bit is set to 1.
        Normal,
        Overflow
        //Clears when the CHARGE register is read.
    };

    enum class MATHOF { //	R	0h	This bit is set to 1 if an arithmetic operation resulted in an overflow error.
        //It indicates that current and power data may be invalid
        Normal,
        Overflow,
        //Must be manually cleared by triggering another conversion
        //    or by clearing the accumulators with the RSTACC bit.
    };

    enum class TMPOL { //	R/W	0h	This bit is set to 1 if the temperature measurement exceeds the threshold limit in the temperature over-limit register.
        Normal,
        OverTempEvent
        //When ALATCH    = 1 this bit is cleared by reading the DIAG_ALERT register.
    };

    enum class SHNTOL { //	R/W	0h	This bit is set to 1 if the shunt voltage measurement exceeds the threshold limit in the shunt over-limit register.
        Normal,
        OverShuntVoltageEvent
        //When ALATCH    = 1 this bit is cleared by reading the register.
    };

    enum class SHNTUL { //	R/W	0h	This bit is set to 1 if the shunt voltage measurement falls below the threshold limit in the shunt under-limit register.
        Normal,
        UnderShuntVoltageEvent
        //When ALATCH    = 1 this bit is cleared by reading the register.
    };

    enum class BUSOL { //	R/W	0h	This bit is set to 1 if the bus voltage measurement exceeds the threshold limit in the bus over-limit register.
        Normal,
        BusOver_LimitEvent
        //When ALATCH = 1 this bit is cleared by reading the register.
    };

    enum class BUSUL { //	R/W	0h	This bit is set to 1 if the bus voltage measurement falls below the threshold limit in the bus under-limit register.
        Normal,
        BusUnder_LimitEvent
        //When ALATCH = 1 this bit is cleared by reading the register.
    };

    enum class POL { //	R/W	0h	This bit is set to 1 if the power measurement exceeds the threshold limit in the power limit register.
        Normal,
        PowerOver_LimitEvent
        //When ALATCH = 1 this bit is cleared by reading the register.
    };

    enum class CNVRF { //	R/W	0h	This bit is set to 1 if the conversion is completed.
        Normal,
        ConversionIsComplete
        //When ALATCH    = 1 this bit is cleared by reading the register or starting a new triggered conversion.
    };

    enum class MEMSTAT { //R/W	1h	This bit is set to 0 if a checksum error is detected in the device trim memory space.
        MemoryChecksumError,
        NormalOperation
    };

    struct DIAG_ALRT {
        MEMSTAT memstat : 1; // R/W 1h
        CNVRF cnvrf : 1; // R/W 0h
        POL pol : 1; // R/W 0h
        BUSUL busul : 1; // R/W 0h
        BUSOL busol : 1; // R/W 0h
        SHNTUL shntul : 1; // R/W 0h
        SHNTOL shntol : 1; // R/W 0h
        TMPOL tmpol : 1; // R/W 0h
        unsigned : 1; // R 0h
        MATHOF mathof : 1; // R 0h
        CHARGEOF chargeof : 1; // R 0h
        ENERGYOF energyof : 1; // R 0h
        APOL apol : 1; // R/W 0h
        SLOWALERT slowalert : 1; // R/W 0h
        CNVR cnvr : 1; // R/W 0h
        ALATCH alatch : 1; // R/W 0h
    };

    using SOVL = uint16_t; //R/W 7FFFh Sets the threshold for comparison of the value to detect ShuntOvervoltage (overcurrent protection). Two's complement value. Conversion Factor: unsigned μV/LSB:1;// when ADCRANGE = 01.25 μV/LSB when ADCRANGE = 1
    using SUVL = uint16_t; //R/W 8000h Sets the threshold for comparison of the value to detect ShuntUndervoltage (undercurrent protection). Two's complement value. Conversion Factor: unsigned μV/LSB:1;// when ADCRANGE = 01.25 μV/LSB when ADCRANGE = 1

    struct BOVL {
        unsigned bovl : 15; //R/W 7FFFh
        unsigned : 1;
    };

    struct BUVL {
        unsigned buvl : 15; //R/W 0h
        unsigned : 1;
    };

    using TEMP_LIMIT /*TOL*/ = uint16_t; //R/W 7FFFh Sets the threshold for comparison of the value to detect overtemperature measurements. Two's complement value.The value entered in this field compares directly against the valuefrom the DIETEMP register to determine if an over temperaturecondition exists. Conversion factor: 7.unsigned m°C/LSB:1;//.
    using PWR_LIMIT /*POL*/ = uint16_t; //R/W FFFFh Sets the threshold for comparison of the value to detect power over-limit measurements. Unsigned representation, positive value only.The value entered in this field compares directly against the valuefrom the POWER register to determine if an over power conditionexists. Conversion factor: unsigned ×:1;// Power LSB

    struct MANUFACTURER_ID {
        uint8_t Id0; //R 5449h Reads back TI in ASCII.
        uint8_t Id1;
    };

    struct DEVICE_ID {
        unsigned revid : 4; //R 1h fDevice revision identification.
        unsigned dieid : 12; //R 229h Stores the device identification bits
    };

private:
    struct Data {
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
            uint8_t data[8];
        };
        void clear() { std::memset(this, 0, sizeof *this); }
    } data;
#pragma pack(pop)

    void dmaRead(Register reg);
    void dmaWrite(Register reg);
};
