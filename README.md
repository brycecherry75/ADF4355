# ADF4355
Arduino Library for the ADF4355 Microwave Wideband Frequency Synthesizer chip

## Introduction

This library supports the [ADF4355 Chip](https://www.analog.com/en/products/adf4355.html) from Analog Devices on Arduinos. The chip is a wideband (53.125 MHz to 6.8 GHz) Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO), covering a very wide range frequency range under digital control. Just add an external PLL loop filter, Reference frequency source and a power supply for a very useful frequency generator for applications as a Local Oscillator or Sweep Generator.  

The chip generates the frequency using a programmable Fractional-N and Integer-N Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO) with an external loop filter and frequency reference. The chip is controlled by a SPI interface, which is controlled by a microcontroller such as the Arduino.

The library provides an SPI control interface for the ADF4355, and also provides functions to calculate and set the
frequency, which greatly simplifies the integration of this chip into a design. The calculations are done using the excellent 
[Big Number Arduino Library](https://github.com/nickgammon/BigNumber) by Nick Gammon, as the integter calculations require
great than 32bit integers that are not available on the Arduino. The library also exposes all of the PLL variables, such as FRAC, Mod and INT, so they examined as needed.  

A low phase noise stable oscillator is required for this module. Typically, an Ovenized Crystal Oscillator (OCXO) in the 10 MHz to 100 MHz range is used.

PFD based on reference double/divide by 2 and R value limits to avoid warnings on frequency error and slow calculation thereof:

0.125 to 0.16383: No more than 5 decimal places

> 0.16383 to 1.6383 MHz PFD: No more than 4 decimal places

> 1.6383 to 16.383 MHz PFD: No more than 3 decimal places

> 16.383 to 125 MHz PFD: No more than 2 decimal places

Requires the BitFieldManipulation library: http://github.com/brycecherry75/BitFieldManipulation

Requires the BeyondByte library: http://github.com/brycecherry75/BeyondByte

## Features

+ Frequency Range: 53.125 MHz to 6.8 GHz
+ Output Level: -4 dBm to 5 dBm (in 3 dB steps) 
+ In-Band Phase Noise: -221 dBc/Hz
+ Signal On/Off control
+ All ADF4355_R[] registers can be accessed and manipulated including ChargePumpCurrent (0.3125-5 in mA)

## Library Use

An example program using the library is provided in the source directory [example4355.ino](src/example4355.ino).

init(SSpin, LockPinNumber, Lock_Pin_Used, CEpin, CE_Pin_Used): initialize the ADF4355 with SPI SS pin, lock pin and true/false for lock pin/CE pin use - CE pin is typically LOW (disabled) on reset if used; depending on your board, this pin along with the RF Power Down pin may have a pullup or pulldown resistor fitted and certain boards have the RF Power Down pin (low active) on the header

SetStepFreq(frequency): sets the step frequency in Hz - default is 100 kHz - returns an error code

ReadFraction1(): Returns a uint32_t value for the currently programmed register

ReadR()/ReadInt()/ReadFraction2()/ReadMod2(): returns a uint16_t value for the currently programmed register

ReadOutDivider()/ReadOutDivider_PowerOf2()/ReadRDIV2()/ReadRefDoubler(): returns a uint8_t value for the currently programmed register - ReadOutDivider() is automatically converted from a binary exponent to an actual division ratio and 
ReadOutDivider_PowerOf2() is a binary exponent

ReadPFDfreq(): returns a double for the PFD value

setf(*frequency, PowerLevel, AuxPowerLevel, CalculationTimeout): set the frequency (in Hz with char string) power level/auxiliary 
power level (1-4 in 3dBm steps from -5dBm) and calculation timeout in mS - returns an error code

setrf(frequency, R_divider, ReferenceDivisionType, ReferenceInputType): set the reference frequency, reference divider R, reference frequency division type (ADF4355_REF_(UNDIVIDED/HALF/DOUBLE) and reference frequency input type (ADF4355_REF_SINGLE_ENDED/ADF4355_REF_DIFFERENTIAL) - default is 10 MHz/1/undivided/single ended - returns an error code - this is required for initialization and will also set the charge pump current defined by the ChargePumpCurrent float

WriteSweepRegs(*regs): high speed write for registers when used for frequency sweep (*regs is uint32_t and size is as per ADF4355_RegsToWrite

ReadSweepRegs(*regs): high speed read for registers when used for frequency sweep (*regs is uint32_t and size is as per ADF4355_RegsToWrite

Please note that you should install the provided BigNumber library in your Arduino library directory.

Default settings which may need to be changed as required BEFORE execution of ADF4355 library functions:

MUX logic level (Register 4/Bit 8): 3.3V level (1 - default), 1.8V level (0)

Phase Detector Polarity (Register 4/Bit 6): Negative (passive or noninverting active loop filter) (1 - default), Postive (inverting active loop filter) (0)

Certain RF output circuitry and/or board layouts may result in issues with harmonics when tuned to an RF frequency below 200-250 MHz - a frequency counter can display a harmonic frequency under these conditions.


Error codes:

Common to SetStepFreq/setf/setrf:

ADF4355_ERROR_NONE


SetStepFreq only:

ADF4355_ERROR_STEP_FREQUENCY_EXCEEDS_PFD


setf only:

ADF4355_ERROR_RF_FREQUENCY

ADF4355_ERROR_POWER_LEVEL

ADF4355_ERROR_AUX_POWER_LEVEL

ADF4355_ERROR_ZERO_PFD_FREQUENCY

ADF4355_ERROR_R_OVERFLOW_ON_PFD_OVER_75MHz

ADF4355_ERROR_MOD2_RANGE

ADF4355_ERROR_FRAC2_EXCEEDS_MOD2

ADF4355_ERROR_FRAC1_RANGE

ADF4355_ERROR_FRAC2_RANGE

ADF4355_ERROR_N_RANGE

ADF4355_ERROR_FREQUENCY_CALCULATION_TIMEOUT

ADF4355_WARNING_FREQUENCY_ERROR


setrf only:

ADF4355_ERROR_DOUBLER_EXCEEDED

ADF4355_ERROR_R_RANGE

ADF4355_ERROR_REF_FREQUENCY

ADF4355_ERROR_REF_MULTIPLIER_TYPE

ADF4355_ERROR_REF_INPUT_TYPE

ADF4355_ERROR_SINGLE_ENDED_RANGE

ADF4355_ERROR_PFD_LIMITS

## Installation
Copy the `src/` directory to your Arduino sketchbook directory  (named the directory `example4355`), and install the libraries in your Arduino library directory.  You can also install the adf4355 files separatly as a library.

## References

+ [Big Number Arduino Library](https://github.com/nickgammon/BigNumber) by Nick Gammon
+ Analog Devices ADF4355 datasheet