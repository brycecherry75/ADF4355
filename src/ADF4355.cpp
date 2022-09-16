/*!

   @file adf4355.cpp

   @mainpage ADF4355 Arduino library driver for Wideband Frequency Synthesizer

   @section intro_sec Introduction

   The ADF4355 chip is a wideband freqency synthesizer integrated circuit that can generate frequencies
   from 53.125 MHz to 6.8 GHz. It incorporates a PLL and VCO, along with prescalers, dividers and multipiers.
   The users add a PLL loop filter and reference frequency to create a frequency generator with a very wide
   range, that is tuneable in settable frequency steps.

   The ADF4355 chip provides an SPI interface for setting the device registers that control the
   frequency and output levels, along with several IO pins for gathering chip status and
   enabling/disabling output and power modes.

   The ADF4355 library provides an Arduino API for accessing the features of the ADF chip.

   The basic PLL equations for the ADF4355 are:

   (((INT + ((FRAC1 + (FRAC2 / MOD2)) / MOD1)) * REFIN * ((1 + REFIN doubler) / (REFIN divider * (1 + REFIN divide by 2))))) / RF divider

   where:

   Channel Step = 25 kHz
   REFIN Divider = 1
   Fpfd = (1 + REFIN Doubler) / (REFIN Divider * (1 + REFIN Divide by 2))
   INT = (Frequency * RF divider) / Fpfd
   MOD1 = (2^24)
   FRAC1 = (INT with remainder - INT rounded down) * MOD1
   MOD2 = Fpfd / GCD(Fpfd, Channel Step)
   FRAC2 = (FRAC1 with remainder - FRAC1 rounded down) * MOD2

   @section dependencies Dependencies

   This library uses the BigNumber library from Nick Gammon

   Requires the BitFieldManipulation library: http://github.com/brycecherry75/BitFieldManipulation
   Requires the BeyondByte library: http://github.com/brycecherry75/BeyondByte

   @section author Author

   Bryce Cherry

*/

#include "ADF4355.h"

ADF4355::ADF4355()
{
#if !defined(ARDUINO_SAM_DUE) // Uno or Zero
  SPISettings ADF4355_SPI(16000000UL, MSBFIRST, SPI_MODE0);
#endif
#if defined(ARDUINO_SAM_DUE)
  SPISettings ADF4355_SPI(50000000UL, MSBFIRST, SPI_MODE0);
#endif
}

void ADF4355::SelectSPI() {
  SPI.beginTransaction(ADF4355_SPI);
  digitalWrite(ADF4355_PIN_SS, LOW);
  delayMicroseconds(1);
}

void ADF4355::DeselectSPI() {
  digitalWrite(ADF4355_PIN_SS, HIGH);
  delayMicroseconds(1);
  SPI.endTransaction();
}

void ADF4355::InitRegs() { // ADF4355: Register Initialization Sequence
  uint32_t ADF4355_ADC_CLK_DIV = ReadPFDfreq();
  uint32_t ADF4355_SYNTH_TIMEOUT = ADF4355_ADC_CLK_DIV;
  uint32_t ADF4355_ALC_TIMEOUT;
  uint32_t ADF4355_TIMEOUT = ADF4355_ADC_CLK_DIV;
  uint32_t ADF4355_VCO_BAND_DIV = ADF4355_ADC_CLK_DIV;

  // set ADC clock to ~100 kHz
  ADF4355_ADC_CLK_DIV /= 100000UL;
  ADF4355_ADC_CLK_DIV -= 2;
  ADF4355_ADC_CLK_DIV /= 4;
  ADF4355_ADC_CLK_DIV++; // result is 154 for 61.44 MHz PFD
  if (ADF4355_ADC_CLK_DIV > 255) {
    ADF4355_ADC_CLK_DIV = 255;
  }
  ADF4355_R[10] = BitFieldManipulation.WriteBF_dword(6, 8, ADF4355_R[10], ADF4355_ADC_CLK_DIV);

  // ALC Wait
  ADF4355_ALC_TIMEOUT = 30; // maximize ALC wait time as per ADF4355 datasheet
  ADF4355_R[9] = BitFieldManipulation.WriteBF_dword(9, 5, ADF4355_R[9], ADF4355_ALC_TIMEOUT);

  ADF4355_TIMEOUT /= 20000UL;
  ADF4355_TIMEOUT /= ADF4355_ALC_TIMEOUT; // ALC Wait value
  ADF4355_TIMEOUT++; // result is 103 for 61.44 MHz PFD and ADF4355_ALC_TIMEOUT = 30
  if (ADF4355_TIMEOUT > 1023) {
    ADF4355_TIMEOUT = 1023;
  }
  ADF4355_R[9] = BitFieldManipulation.WriteBF_dword(14, 10, ADF4355_R[9], ADF4355_TIMEOUT);

  ADF4355_SYNTH_TIMEOUT /= ADF4355_TIMEOUT;
  ADF4355_SYNTH_TIMEOUT /= 50000UL; // (10^6 / 50000 = 20000 nS = 20 uS)
  ADF4355_SYNTH_TIMEOUT++; // result is 12 for 61.44 MHz PFD with ADF4355_ALC_TIMEOUT = 30
  if (ADF4355_SYNTH_TIMEOUT > 31) {
    ADF4355_SYNTH_TIMEOUT = 31;
  }
  ADF4355_R[9] = BitFieldManipulation.WriteBF_dword(4, 5, ADF4355_R[9], ADF4355_SYNTH_TIMEOUT);

  ADF4355_VCO_BAND_DIV /= 2400000UL;
  ADF4355_VCO_BAND_DIV++; // result is 26 for 61.44 MHz PFD
  if (ADF4355_VCO_BAND_DIV > 255) {
    ADF4355_VCO_BAND_DIV = 255;
  }
  ADF4355_R[9] = BitFieldManipulation.WriteBF_dword(24, 8, ADF4355_R[9], ADF4355_VCO_BAND_DIV);

  // configure charge pump current
  if (ChargePumpCurrent >= 0.3125 && ChargePumpCurrent <= 5) {
    float ADF4355_ChargePumpCurrent_float = ChargePumpCurrent;
    ADF4355_ChargePumpCurrent_float += 0.15625; // round it
    ADF4355_ChargePumpCurrent_float *= 10000; // convert to an integer
    uint32_t ADF4355_ChargePumpCurrent = ADF4355_ChargePumpCurrent_float;
    ADF4355_ChargePumpCurrent /= 3125; // each step is 0.3125 mA
    ADF4355_ChargePumpCurrent--; // Step 0 is 0.3125 mA
    ADF4355_R[4] = BitFieldManipulation.WriteBF_dword(10, 4, ADF4355_R[4], ADF4355_ChargePumpCurrent);
  }

  for (int i = 12; i >= 0; i--) {
    if (i == 0 && ADCdelayRequired == true) {
      delayMicroseconds(ADF4355_ADC16cyclesDelay_uS);
    }
    SelectSPI();
    BeyondByte.writeDword(0, ADF4355_R[i], 4, BeyondByte_SPI, MSBFIRST);
    DeselectSPI();
  }
}

void ADF4355::UpdateFrequencyRegs() {
  uint8_t RegisterToWrite;
  for (int i = 0; i < 8; i++) {
    switch (i) {
      case 0:
        RegisterToWrite = 10;
        break;
      case 1:
        RegisterToWrite = 4;
        ADF4355_R[RegisterToWrite] = BitFieldManipulation.WriteBF_dword(4, 1, ADF4355_R[RegisterToWrite], 1); // enable counter reset
        break;
      case 2:
        RegisterToWrite = 2;
        break;
      case 3:
        RegisterToWrite = 1;
        break;
      case 4:
        RegisterToWrite = 6;
        break;
      case 5:
        RegisterToWrite = 0;
        ADF4355_R[RegisterToWrite] = BitFieldManipulation.WriteBF_dword(21, 1, ADF4355_R[RegisterToWrite], 0); // disable autocalibration
        break;
      case 6:
        RegisterToWrite = 4;
        ADF4355_R[RegisterToWrite] = BitFieldManipulation.WriteBF_dword(4, 1, ADF4355_R[RegisterToWrite], 0); // disable counter reset
        break;
      case 7:
        if (ADCdelayRequired == true) {
          delayMicroseconds(ADF4355_ADC16cyclesDelay_uS);
        }
        RegisterToWrite = 0;
        ADF4355_R[RegisterToWrite] = BitFieldManipulation.WriteBF_dword(21, 1, ADF4355_R[RegisterToWrite], 1); // enable autocalibration
        break;
    }
    SelectSPI();
    BeyondByte.writeDword(0, ADF4355_R[RegisterToWrite], 4, BeyondByte_SPI, MSBFIRST);
    DeselectSPI();
  }
}

void ADF4355::ReadSweepValues(uint32_t *regs) {
  regs[0] = ADF4355_R[0]; // INT
  regs[1] = ADF4355_R[1]; // FRAC1
  regs[2] = ADF4355_R[2]; // FRAC2/MOD2
  regs[3] = ADF4355_R[6]; // RF divider/level/enables/bleed current
}

void ADF4355::WriteSweepValues(const uint32_t *regs) {
  ADF4355_R[0] = regs[0]; // INT
  ADF4355_R[1] = regs[1]; // FRAC1
  ADF4355_R[2] = regs[2]; // FRAC2/MOD2
  ADF4355_R[6] = regs[3]; // RF divider/level/enables/bleed current
  UpdateFrequencyRegs();
}

uint16_t ADF4355::ReadR() {
  return BitFieldManipulation.ReadBF_dword(15, 10, ADF4355_R[4]);
}

uint16_t ADF4355::ReadInt() {
  return BitFieldManipulation.ReadBF_dword(4, 16, ADF4355_R[0]);
}

uint32_t ADF4355::ReadFraction1() {
  return BitFieldManipulation.ReadBF_dword(4, 24, ADF4355_R[1]);
}

uint16_t ADF4355::ReadFraction2() {
  return BitFieldManipulation.ReadBF_dword(18, 14, ADF4355_R[2]);
}

uint16_t ADF4355::ReadMod2() {
  return BitFieldManipulation.ReadBF_dword(4, 14, ADF4355_R[2]);
}

uint8_t ADF4355::ReadOutDivider() {
  return (1 << BitFieldManipulation.ReadBF_dword(21, 3, ADF4355_R[6]));
}

uint8_t ADF4355::ReadOutDivider_PowerOf2() {
  return BitFieldManipulation.ReadBF_dword(21, 3, ADF4355_R[6]);
}

uint8_t ADF4355::ReadRDIV2() {
  return BitFieldManipulation.ReadBF_dword(25, 1, ADF4355_R[4]);
}

uint8_t ADF4355::ReadRefDoubler() {
  return BitFieldManipulation.ReadBF_dword(26, 1, ADF4355_R[4]);
}

uint8_t ADF4355::ReadADCclockDivider() {
  return BitFieldManipulation.ReadBF_dword(6, 8, ADF4355_R[10]);
}

uint16_t ADF4355::ReadTimeout() {
  return BitFieldManipulation.ReadBF_dword(14, 10, ADF4355_R[9]);
}

uint8_t ADF4355::ReadVCObandDivision() {
  return BitFieldManipulation.ReadBF_dword(24, 8, ADF4355_R[9]);
}

uint8_t ADF4355::ReadALCtimeout() {
  return BitFieldManipulation.ReadBF_dword(9, 5, ADF4355_R[9]);
}

uint8_t ADF4355::ReadSynthTimeout() {
  return BitFieldManipulation.ReadBF_dword(4, 5, ADF4355_R[9]);
}

double ADF4355::ReadPFDfreq() {
  double value = ADF4355_reffreq;
  uint16_t temp = ReadR();
  if (temp == 0) { // avoid division by zero
    return 0;
  }
  value /= temp;
  if (ReadRDIV2() != 0) {
    value /= 2;
  }
  if (ReadRefDoubler() != 0) {
    value *= 2;
  }
  return value;
}

void ADF4355::ReadCurrentFrequency(char *freq)
{
  BigNumber::begin(20);
  char tmpstr[12];
  ultoa(ADF4355_reffreq, tmpstr, 10);
  BigNumber BN_ref = BigNumber(tmpstr);
  if (ReadRDIV2() != 0 && ReadRefDoubler() == 0) {
    BN_ref /= BigNumber(2);
  }
  else if (ReadRDIV2() == 0 && ReadRefDoubler() != 0) {
    BN_ref *= BigNumber(2);
  }
  BN_ref /= BigNumber(ReadR());
  BigNumber BN_freq = BN_ref;
  ultoa(ReadInt(), tmpstr, 10);
  BN_freq *= BigNumber(tmpstr);
  ultoa(ReadFraction1(), tmpstr, 10);
  BigNumber BN_remainder = BigNumber(tmpstr);
  BN_remainder += (BigNumber(ReadFraction2()) / BigNumber(ReadMod2()));
  BN_remainder /= BigNumber("16777216");
  BN_remainder *= BN_ref;
  BN_freq += BN_remainder;
  BN_freq /= BigNumber(ReadOutDivider());
  BigNumber BN_rounding = BigNumber("0.5");
  for (int i = 0; i < ADF4355_DECIMAL_PLACES; i++) {
    BN_rounding /= BigNumber(10);
  }
  BN_freq += BN_rounding;
  char* temp = BN_freq.toString();
  BigNumber::finish();
  uint8_t DecimalPlaceToStart;
  for (int i = 0; i < (ADF4355_DIGITS + 1); i++){
    freq[i] = temp[i];
    if (temp[i] == '.') {
      DecimalPlaceToStart = i;
      DecimalPlaceToStart++;
      break;
    }
  }
  for (int i = DecimalPlaceToStart; i < (DecimalPlaceToStart + ADF4355_DECIMAL_PLACES); i++) {
    freq[i] = temp[i];
  }
  freq[(DecimalPlaceToStart + ADF4355_DECIMAL_PLACES)] = 0x00;
  free(temp);
}

void ADF4355::init(uint8_t SSpin, uint8_t LockPinNumber, bool Lock_Pin_Used, uint8_t CEpinNumber, bool CE_Pin_Used)
{
  ADF4355_PIN_SS = SSpin;
  pinMode(ADF4355_PIN_SS, OUTPUT) ;
  digitalWrite(ADF4355_PIN_SS, HIGH) ;
  if (CE_Pin_Used == true) {
    pinMode(CEpinNumber, OUTPUT) ;
  }
  if (Lock_Pin_Used == true) {
    pinMode(LockPinNumber, INPUT_PULLUP) ;
  }
  SPI.begin();
}

int ADF4355::SetStepFreq(uint32_t value) {
  if (value > ReadPFDfreq()) {
    return ADF4355_ERROR_STEP_FREQUENCY_EXCEEDS_PFD;
  }
  ADF4355_ChanStep = value;
  return ADF4355_ERROR_NONE;
}

int  ADF4355::setf(char *freq, uint8_t PowerLevel, uint8_t AuxPowerLevel, uint32_t CalculationTimeout)
{
  //  calculate settings from freq
  if (PowerLevel < 0 || PowerLevel > 4) return ADF4355_ERROR_POWER_LEVEL;
  if (AuxPowerLevel < 0 || AuxPowerLevel > 4) return ADF4355_ERROR_AUX_POWER_LEVEL;
  if (ReadPFDfreq() == 0) return ADF4355_ERROR_ZERO_PFD_FREQUENCY;
  if (ReadPFDfreq() > 75000000UL && ReadR() > 511) return ADF4355_ERROR_R_OVERFLOW_ON_PFD_OVER_75MHz;

  BigNumber::begin(20); // BigNumber(signed_int_limited_from_-32768_to_32767) / BigNumber("number_string_is_not_limited_to_signed_int")
  if (BigNumber(freq) > BigNumber("6800000000") || BigNumber(freq) < BigNumber("53125000")) {
    BigNumber::finish();
    return ADF4355_ERROR_RF_FREQUENCY;
  }

  uint8_t FrequencyPointer = 0;
  while (true) { // null out any decimal places below 1 Hz increments to avoid GCD calculation input overflow
    if (freq[FrequencyPointer] == '.') { // change the decimal point to a null terminator
      freq[FrequencyPointer] = 0x00;
      break;
    }
    if (freq[FrequencyPointer] == 0x00) { // null terminator reached
      break;
    }
    FrequencyPointer++;
  }

  BigNumber BN_localosc_ratio = BigNumber("3400000000") / BigNumber(freq);
  uint8_t localosc_ratio = (uint8_t) ((uint32_t) BN_localosc_ratio);
  uint8_t ADF4355_outdiv = 1;
  int ADF4355_RFDivSel = 0;

  // select the output divider
  if (BigNumber(freq) != BigNumber("53125000")) {
    while (ADF4355_outdiv <= localosc_ratio && ADF4355_outdiv <= 64) {
      ADF4355_outdiv *= 2;
      ADF4355_RFDivSel++;
    }
  }
  else {
    ADF4355_outdiv = 64;
    ADF4355_RFDivSel = 6;
  }

  char tmpstr[12]; // will fit a long including sign and terminator

  ultoa(ADF4355_reffreq, tmpstr, 10);
  word CurrentR = ReadR();
  uint8_t RDIV2 = ReadRDIV2();
  uint8_t RefDoubler = ReadRefDoubler();
  BigNumber BN_ADF4355_PFDFreq = (BigNumber(tmpstr) * (BigNumber(1) * BigNumber((1 + RefDoubler))) * (BigNumber(1) / BigNumber((1 + RDIV2)))) / BigNumber(CurrentR);
  BigNumber BN_ADF4355_VCO = (BigNumber(freq) * BigNumber(ADF4355_outdiv));

  BigNumber BN_N = BN_ADF4355_VCO / BN_ADF4355_PFDFreq;
  uint32_t ADF4355_N = (uint32_t) ((uint32_t) BN_N); // round off the decimal
  ultoa(ADF4355_N, tmpstr, 10);
  BigNumber BN_Frac1 = (BN_N - BigNumber(tmpstr)) * BigNumber("16777216");
  uint32_t ADF4355_Frac1 = (uint32_t) ((uint32_t) BN_Frac1);

  // calculate the required GCD for MOD2 calculation

  /*
    // previous (slower) GCD routine using BigNumber
    BigNumber BN_t;
    BigNumber BN_ADF4355_PFDFreq2 = BN_ADF4355_PFDFreq; // a duplicate is required here
    ultoa(ADF4355_ChanStep, tmpstr, 10);
    BigNumber BN_CHSP = BigNumber(tmpstr);
    bool CalculationTookTooLong = false;
    uint32_t StartTime = millis();
    while (true) {
      uint32_t StopTime = millis();
      StopTime -= StartTime;
      if (StopTime > CalculationTimeout) {
        CalculationTookTooLong = true;
        break;
      }
      if (BN_CHSP < BigNumber(1)) {
        BN_t = BN_ADF4355_PFDFreq2;
        break;
      }
      if (BN_ADF4355_PFDFreq2 < BigNumber(1)) {
        BN_t = BN_CHSP;
        break;
      }
      if (BN_CHSP == BN_ADF4355_PFDFreq2) {
        BN_t = BN_CHSP;
        break;
      }
      if (BN_CHSP > BN_ADF4355_PFDFreq2) {
        BN_CHSP -= BN_ADF4355_PFDFreq2;
      }
      else {
        BN_ADF4355_PFDFreq2 -= BN_CHSP;
      }
    }
    BigNumber BN_ADF4355_Mod2 = BN_ADF4355_PFDFreq / BN_t;
  */

  // faster GCD routine - GCD inputs for PFD frequency and channel spacing will not exceed ((2^32) - 1) so therefore, we can use uint32_t GCD calculation for speed
  uint32_t GCD_t;
  uint32_t GCD_ADF4355_PFDFreq = (uint32_t)((uint32_t)BN_ADF4355_PFDFreq); // a duplicate is required here
  uint32_t GCD_CHSP = ADF4355_ChanStep;
  bool CalculationTookTooLong = false;
  uint32_t StartTime = millis();
  while (true) {
    if (CalculationTimeout > 0) {
      uint32_t StopTime = millis();
      StopTime -= StartTime;
      if (StopTime > CalculationTimeout) {
        CalculationTookTooLong = true;
        break;
      }
    }
    if (GCD_CHSP == 0) {
      GCD_t = GCD_ADF4355_PFDFreq;
      break;
    }
    if (GCD_ADF4355_PFDFreq == 0) {
      GCD_t = GCD_CHSP;
      break;
    }
    if (GCD_CHSP == GCD_ADF4355_PFDFreq) {
      GCD_t = GCD_CHSP;
      break;
    }
    if (GCD_CHSP > GCD_ADF4355_PFDFreq) {
      GCD_CHSP -= GCD_ADF4355_PFDFreq;
    }
    else {
      GCD_ADF4355_PFDFreq -= GCD_CHSP;
    }
  }
  ultoa(GCD_t, tmpstr, 10);
  BigNumber BN_ADF4355_Mod2 = BN_ADF4355_PFDFreq / BigNumber(tmpstr);

  uint32_t ADF4355_Mod2 = (uint32_t) ((uint32_t) BN_ADF4355_Mod2); // should be 1536 for 2112.8 MHz output / 61.44 MHz PFD
  ultoa(ADF4355_Frac1, tmpstr, 10);
  BigNumber BN_Frac2 = ((BN_Frac1 - BigNumber(tmpstr)) * BigNumber(ADF4355_Mod2)) + BigNumber("0.5");
  uint32_t ADF4355_Frac2 = (uint32_t) ((uint32_t) BN_Frac2); // should be 1024 for 2112.8 MHz output / 61.44 MHz PFD

  BigNumber::finish();

  bool FrequencyError = false;

  if (CalculationTookTooLong == true) {
    return ADF4355_ERROR_FREQUENCY_CALCULATION_TIMEOUT;
  }

  if (ADF4355_N < 23  || ADF4355_N > 32767) {
    return ADF4355_ERROR_N_RANGE;
  }

  if ( ADF4355_Frac1 < 0 || ADF4355_Frac1 > 16777215UL) {
    return ADF4355_ERROR_FRAC1_RANGE;
  }

  if ( ADF4355_Mod2 < 2 || ADF4355_Mod2 > 16383) {
    return ADF4355_ERROR_MOD2_RANGE;
  }

  if (ADF4355_Frac2 < 0) {
    return ADF4355_ERROR_FRAC2_RANGE;
  }

  if (ADF4355_Mod2 > 16383) {
    FrequencyError = true;
    while (ADF4355_Mod2 > 16383) {
      ADF4355_Frac2 /= 2;
      ADF4355_Mod2 /= 2;
    }
  }

  if ( ADF4355_Frac2 > ADF4355_Mod2) {
    return ADF4355_ERROR_FRAC2_EXCEEDS_MOD2;
  }

  ADF4355_R[0] = BitFieldManipulation.WriteBF_dword(4, 16, ADF4355_R[0], ADF4355_N);
  // (0x00, 20, 1, 0); // prescaler - with 4/5 prescaler for maximum of 6.8 GHz, no need to change to 8/9
  ADF4355_R[1] = BitFieldManipulation.WriteBF_dword(4, 24, ADF4355_R[1], ADF4355_Frac1); // main fractional value;
  ADF4355_R[2] = BitFieldManipulation.WriteBF_dword(4, 14, ADF4355_R[2], ADF4355_Mod2); // auxiliary modulus
  ADF4355_R[2] = BitFieldManipulation.WriteBF_dword(18, 14, ADF4355_R[2], ADF4355_Frac2); // auxiliary fraction
  if ((ADF4355_Frac1 != 0 || ADF4355_Frac2 != 0) && ReadPFDfreq() <= 100000000UL) { // enable negative bleed under fractional mode and if PFD <= 100 MHz
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(29, 1, ADF4355_R[6], 1);
  }
  else { // disable negative bleed under integer mode or if PFD > 100 MHz
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(29, 1, ADF4355_R[6], 0);
  }
  if (PowerLevel == 0) {
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(6, 1, ADF4355_R[6], 0);
  }
  else {
    PowerLevel--;
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(6, 1, ADF4355_R[6], 1);
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(4, 2, ADF4355_R[6], PowerLevel);
  }
  if (AuxPowerLevel == 0) {
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(9, 1, ADF4355_R[6], 0);
  }
  else {
    AuxPowerLevel--;
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(9, 1, ADF4355_R[6], 1);
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(7, 2, ADF4355_R[6], AuxPowerLevel);
  }
  ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(21, 3, ADF4355_R[6], ADF4355_RFDivSel);

  // calculate charge pump bleed current based on N value and charge pump current
  uint32_t ADF4355_BleedCurrent = BitFieldManipulation.ReadBF_dword(10, 4, ADF4355_R[4]);
  ADF4355_BleedCurrent++; // Step 0 is 0.3125 mA
  ADF4355_BleedCurrent *= 31250000UL; // 100pA multiples for fidelity per step
  ADF4355_BleedCurrent *= 4;
  ADF4355_BleedCurrent /= ADF4355_N;
  ADF4355_BleedCurrent /= 3750;
  ADF4355_BleedCurrent += 50; // ceiling
  ADF4355_BleedCurrent /= 100; // remove the decimal place
  if (ADF4355_BleedCurrent > 255) {
    ADF4355_BleedCurrent = 255;
  }
  ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(13, 8, ADF4355_R[6], ADF4355_BleedCurrent);  

  UpdateFrequencyRegs();
  if (FrequencyError == false) {
    return ADF4355_ERROR_NONE;
  }
  else {
    return ADF4355_WARNING_FREQUENCY_ERROR;
  }
}

int ADF4355::setrf(uint32_t f, uint16_t r, uint8_t ReferenceDivisionType, uint8_t ReferenceInputType)
{
  if (f > 100000000UL && ReferenceDivisionType == ADF4355_REF_DOUBLE) return ADF4355_ERROR_DOUBLER_EXCEEDED;
  if (r > 1023 || r < 1) return ADF4355_ERROR_R_RANGE;
  if (f < ADF4355_REFIN_MIN || f > ADF4355_REFIN_MAX_DIFFERENTIAL) return ADF4355_ERROR_REF_FREQUENCY;
  if (ReferenceDivisionType != ADF4355_REF_UNDIVIDED && ReferenceDivisionType != ADF4355_REF_HALF && ReferenceDivisionType != ADF4355_REF_DOUBLE) return ADF4355_ERROR_REF_MULTIPLIER_TYPE;
  if (ReferenceInputType != ADF4355_REF_SINGLE_ENDED && ReferenceInputType != ADF4355_REF_DIFFERENTIAL) return ADF4355_ERROR_REF_INPUT_TYPE;
  if (f > ADF4355_REFIN_MAX_SINGLE_ENDED && ReferenceInputType == ADF4355_REF_SINGLE_ENDED) return ADF4355_ERROR_SINGLE_ENDED_RANGE;

  double ReferenceFactor = 1;
  if (ReferenceDivisionType == ADF4355_REF_HALF) {
    ReferenceFactor /= 2;
  }
  else if (ReferenceDivisionType == ADF4355_REF_DOUBLE) {
    ReferenceFactor *= 2;
  }
  double newfreq  =  (double) f  * ( (double) ReferenceFactor / (double) r);  // check the loop freq

  if ( newfreq > ADF4355_PFD_MAX || newfreq < ADF4355_PFD_MIN ) return ADF4355_ERROR_PFD_LIMITS;

  ADF4355_reffreq = f ;
  ADF4355_R[4] = BitFieldManipulation.WriteBF_dword(15, 10, ADF4355_R[4], r);
  if (ReferenceDivisionType == ADF4355_REF_DOUBLE) {
    ADF4355_R[4] = BitFieldManipulation.WriteBF_dword(25, 2, ADF4355_R[4], 0b00000010);
  }
  else if (ReferenceDivisionType == ADF4355_REF_HALF) {
    ADF4355_R[4] = BitFieldManipulation.WriteBF_dword(25, 2, ADF4355_R[4], 0b00000001);
  }
  else {
    ADF4355_R[4] = BitFieldManipulation.WriteBF_dword(25, 2, ADF4355_R[4], 0b00000000);
  }
  if (ReferenceInputType == ADF4355_REF_DIFFERENTIAL) {
    ADF4355_R[4] = BitFieldManipulation.WriteBF_dword(9, 1, ADF4355_R[4], 1);
  }
  else {
    ADF4355_R[4] = BitFieldManipulation.WriteBF_dword(9, 1, ADF4355_R[4], 0);
  }
  InitRegs();
  return ADF4355_ERROR_NONE;
}

int ADF4355::setPowerLevel(uint8_t PowerLevel) {
  if (PowerLevel < 0 && PowerLevel > 4) return ADF4355_ERROR_POWER_LEVEL;
  if (PowerLevel == 0) {
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(6, 1, ADF4355_R[6], 0);
  }
  else {
    PowerLevel--;
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(6, 1, ADF4355_R[6], 1);
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(4, 2, ADF4355_R[6], PowerLevel);
  }
  UpdateFrequencyRegs();
  return ADF4355_ERROR_NONE;
}

int ADF4355::setAuxPowerLevel(uint8_t PowerLevel) {
  if (PowerLevel < 0 && PowerLevel > 4) return ADF4355_ERROR_POWER_LEVEL;
  if (PowerLevel == 0) {
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(9, 1, ADF4355_R[6], 0);
  }
  else {
    PowerLevel--;
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(9, 1, ADF4355_R[6], 1);
    ADF4355_R[6] = BitFieldManipulation.WriteBF_dword(7, 2, ADF4355_R[6], PowerLevel);
  }
  UpdateFrequencyRegs();
  return ADF4355_ERROR_NONE;
}