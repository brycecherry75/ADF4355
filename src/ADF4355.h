/*!
   @file ADF4355.h

   This is part of the Arduino Library for the ADF4355 PLL wideband frequency synthesier

*/

#ifndef ADF4355_H
#define ADF4355_H
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <BigNumber.h>
#include <BitFieldManipulation.h>
#include <BeyondByte.h>

#define ADF4355_PFD_MAX   75000000UL      ///< Maximum Frequency for Phase Detector; PFD > 75 MHz up to 125 MHz requires calculation of two sets of INT/FRAC1/MOD2/FRAC2 so not bothering with this arrangement due to complexity and reduced speed for sweep oscillator application
#define ADF4355_PFD_MIN   125000UL        ///< Minimum Frequency for Phase Detector
#define ADF4355_REFIN_MIN   10000000UL   ///< Minimum Reference Frequency
#define ADF4355_REFIN_MAX_SINGLE_ENDED   250000000UL   ///< Maximum Reference Frequency, single ended
#define ADF4355_REFIN_MAX_DIFFERENTIAL   600000000UL   ///< Maximum Reference Frequency, differential
#define ADF4355_REF_FREQ_DEFAULT 10000000UL  /// < Default Reference Frequency
#define ADF4355_ADC16cyclesDelay_uS 176 // ADC clock of 100 kHz (or 10 uS) * 16 * 1.1

#define ADF4355_REF_UNDIVIDED 0
#define ADF4355_REF_HALF 1
#define ADF4355_REF_DOUBLE 2
#define ADF4355_REF_SINGLE_ENDED 0
#define ADF4355_REF_DIFFERENTIAL 1

#define ADF4355_RegsToWrite 4UL // for high speed sweep

// common to all of the following subroutines
#define ADF4355_ERROR_NONE 0

// SetStepFreq
#define ADF4355_ERROR_STEP_FREQUENCY_EXCEEDS_PFD 1

// setf
#define ADF4355_ERROR_RF_FREQUENCY 2
#define ADF4355_ERROR_POWER_LEVEL 3
#define ADF4355_ERROR_AUX_POWER_LEVEL 4
#define ADF4355_ERROR_ZERO_PFD_FREQUENCY 5
#define ADF4355_ERROR_R_OVERFLOW_ON_PFD_OVER_75MHz 6
#define ADF4355_ERROR_MOD2_RANGE 7
#define ADF4355_ERROR_FRAC2_EXCEEDS_MOD2 8
#define ADF4355_ERROR_FRAC1_RANGE 9
#define ADF4355_ERROR_FRAC2_RANGE 10
#define ADF4355_ERROR_N_RANGE 11
#define ADF4355_ERROR_FREQUENCY_CALCULATION_TIMEOUT 12
#define ADF4355_WARNING_FREQUENCY_ERROR 13

// setrf
#define ADF4355_ERROR_DOUBLER_EXCEEDED 14
#define ADF4355_ERROR_R_RANGE 15
#define ADF4355_ERROR_REF_FREQUENCY 16
#define ADF4355_ERROR_REF_MULTIPLIER_TYPE 17
#define ADF4355_ERROR_REF_INPUT_TYPE 18
#define ADF4355_ERROR_SINGLE_ENDED_RANGE 19
#define ADF4355_ERROR_PFD_LIMITS 20

// ReadCurrentFrequency
#define ADF4355_DIGITS 10
#define ADF4355_DECIMAL_PLACES 6
#define ADF4355_ReadCurrentFrequency_ArraySize (ADF4355_DIGITS + ADF4355_DECIMAL_PLACES + 2) // including decimal point and null terminator

class ADF4355
{
  public:
    ADF4355();
    uint8_t ADF4355_PIN_SS = 10;   ///< Ard Pin for SPI Slave Select

    bool ADCdelayRequired = true; // can be false for speed in sweep operation
    float ChargePumpCurrent = 0.9; // mA - rounded to the nearest 0.3125 mA step - recommended by datasheet for minimum spurs

    uint16_t ReadR();
    uint16_t ReadInt();
    uint32_t ReadFraction1();
    uint16_t ReadFraction2();
    uint16_t ReadMod2();
    uint8_t ReadOutDivider();
    uint8_t ReadOutDivider_PowerOf2();
    uint8_t ReadRDIV2();
    uint8_t ReadRefDoubler();
    double ReadPFDfreq();
    uint8_t ReadADCclockDivider();
    uint16_t ReadTimeout();
    uint8_t ReadVCObandDivision();
    uint8_t ReadALCtimeout();
    uint8_t ReadSynthTimeout();

    void init(uint8_t SSpin, uint8_t LockPinNumber, bool Lock_Pin_Used, uint8_t CEpinNumber, bool CE_Pin_Used);
    int SetStepFreq(uint32_t value);
    int setf(char *freq, uint8_t PowerLevel, uint8_t AuxPowerLevel, uint32_t CalculationTimeout); // set freq and power levels and output mode
    int setrf(uint32_t f, uint16_t r, uint8_t ReferenceDivisionType, uint8_t ReferenceInputType); // set reference freq and reference divider (default is 10 MHz with divide by 1)
    int setPowerLevel(uint8_t PowerLevel);
    int setAuxPowerLevel(uint8_t PowerLevel);

    void ReadSweepValues(uint32_t *regs);
    void WriteSweepValues(const uint32_t *regs);
    void ReadCurrentFrequency(char *freq);

    SPISettings ADF4355_SPI;
    uint32_t ADF4355_reffreq = ADF4355_REF_FREQ_DEFAULT;
    uint32_t ADF4355_R[13] {0x00200000, 0x00000001, 0x00000002, 0x40000003, 0x3000DD84, 0x00800025, 0x15004006, 0x10000067, 0x102D0428, 0x500C4819, 0x00C0067A, 0x0061300B, 0x0001041C};
    uint32_t ADF4355_ChanStep = 100000UL;

    private:
    void SelectSPI();
    void DeselectSPI();
    void InitRegs();
    void UpdateFrequencyRegs();
};

#endif