/*

  ADF4355 demo by Bryce Cherry

  Commands:
  REF reference_frequency_in_Hz reference_divider reference_multiplier(HALF/DOUBLE/UNDIVIDED) reference_input(SINGLE_ENDED/DIFFERENTIAL) - Set reference frequency, reference divider and reference input type
  FREQ frequency_in_Hz power_level(1-4) auxpower_level(1-4) timeout_in_mS - set RF frequency and power levels with frequency calculation timeout
  (BURST/BURST_CONT/BURST_SINGLE) on_time_in_uS off time_in_uS count (AUX) - perform a on/off burst on frequency and power level set with FREQ/FREQ_P - count is only used with BURST_CONT - if AUX is used, will burst on the auxiliary output; otherwise, it will burst on the primary output
  SWEEP start_frequency stop_frequency step_in_mS(1-32767) power_level(1-4) aux_power_level(1-4) timeout_in_mS - sweep RF frequency with frequency calculation timeout
  STEP frequency_in_Hz - set channel step
  STATUS - view status of VFO
  CE (ON/OFF) - enable/disable ADF4355
  ADC_DELAY (ON/OFF) - enable/disable ADC delay for frequency setting (OFF for higher speed in sweep)

*/

#include <ADF4355.h>
#include <BigNumber.h> // obtain at https://github.com/nickgammon/BigNumber

ADF4355 vfo;

// use hardware SPI pins for Data and Clock
const byte SSpin = 10; // LE
const byte LockPin = 12; // MISO
const byte CEpin = 9;

const unsigned long ReferenceFrequency = 10000000UL;
const byte ReferenceInput = ADF4355_REF_SINGLE_ENDED;
const byte ReferenceDivision = ADF4355_REF_UNDIVIDED;
const byte Rvalue = 1;

const word SweepSteps = 14; // SweepSteps * 4 * 7 is the temporary memory calculation (remember to leave enough for BigNumber)

const int CommandSize = 50;
char Command[CommandSize];

// ensures that the serial port is flushed fully on request
const unsigned long SerialPortRate = 9600;
const byte SerialPortRateTolerance = 5; // percent - increase to 50 for rates above 115200 up to 4000000
const byte SerialPortBits = 10; // start (1), data (8), stop (1)
const unsigned long TimePerByte = ((((1000000ULL * SerialPortBits) / SerialPortRate) * (100 + SerialPortRateTolerance)) / 100); // calculated on serial port rate + tolerance and rounded down to the nearest uS, long caters for even the slowest serial port of 75 bps

void FlushSerialBuffer() {
  while (true) {
    if (Serial.available() > 0) {
      byte dummy = Serial.read();
      while (Serial.available() > 0) { // flush additional bytes from serial buffer if present
        dummy = Serial.read();
      }
      if (TimePerByte <= 16383) {
        delayMicroseconds(TimePerByte); // delay in case another byte may be received via the serial port
      }
      else { // deal with delayMicroseconds limitation
        unsigned long DelayTime = TimePerByte;
        DelayTime /= 1000;
        if (DelayTime > 0) {
          delay(DelayTime);
        }
        DelayTime = TimePerByte;
        DelayTime %= 1000;
        if (DelayTime > 0) {
          delayMicroseconds(DelayTime);
        }
      }
    }
    else {
      break;
    }
  }
}

void getField (char* buffer, int index) {
  int CommandPos = 0;
  int FieldPos = 0;
  int SpaceCount = 0;
  while (CommandPos < CommandSize) {
    if (Command[CommandPos] == 0x20) {
      SpaceCount++;
      CommandPos++;
    }
    if (Command[CommandPos] == 0x0D || Command[CommandPos] == 0x0A) {
      break;
    }
    if (SpaceCount == index) {
      buffer[FieldPos] = Command[CommandPos];
      FieldPos++;
    }
    CommandPos++;
  }
  for (int ch = 0; ch < strlen(buffer); ch++) { // correct case of command
    buffer[ch] = toupper(buffer[ch]);
  }
  buffer[FieldPos] = '\0';
}

void PrintVFOstatus() {
  Serial.print(F("R: "));
  Serial.println(vfo.ReadR());
  if (vfo.ReadRDIV2() != 0 && vfo.ReadRefDoubler() != 0) {
    Serial.println(F("Reference doubler and reference divide by 2 enabled - invalid state"));
  }
  else if (vfo.ReadRDIV2() != 0) {
    Serial.println(F("Reference divide by 2"));
  }
  else if (vfo.ReadRefDoubler() != 0) {
    Serial.println(F("Reference doubler enabled"));
  }
  else {
    Serial.println(F("Reference doubler and divide by 2 disabled"));
  }
  Serial.print(F("Int: "));
  Serial.println(vfo.ReadInt());
  Serial.print(F("Fraction1: "));
  Serial.println(vfo.ReadFraction1());
  Serial.print(F("Fraction2: "));
  Serial.println(vfo.ReadFraction2());
  Serial.print(F("Mod2: "));
  Serial.println(vfo.ReadMod2());
  Serial.print(F("Output divider: "));
  Serial.println(vfo.ReadOutDivider());
  Serial.print(F("Output divider power of 2: "));
  Serial.println(vfo.ReadOutDivider_PowerOf2());
  Serial.print(F("PFD frequency (Hz): "));
  Serial.println(vfo.ReadPFDfreq());
  Serial.print(F("Frequency step (Hz): "));
  Serial.println(vfo.ADF4355_ChanStep);
  Serial.print(F("ADC clock divider: "));
  Serial.println(vfo.ReadADCclockDivider());
  Serial.print(F("Timeout: "));
  Serial.println(vfo.ReadTimeout());
  Serial.print(F("VCO band division: "));
  Serial.println(vfo.ReadVCObandDivision());
  Serial.print(F("ALC timeout: "));
  Serial.println(vfo.ReadALCtimeout());
  Serial.print(F("Synthesizer timeout: "));
  Serial.println(vfo.ReadSynthTimeout());
  Serial.print(F("Current frequency (Hz): "));
  char CurrentFreq[ADF4355_ReadCurrentFrequency_ArraySize];
  vfo.ReadCurrentFrequency(CurrentFreq);
  Serial.println(CurrentFreq);
}

void PrintErrorCode(byte value) {
  switch (value) {
    case ADF4355_ERROR_NONE:
      break;
    case ADF4355_ERROR_STEP_FREQUENCY_EXCEEDS_PFD:
      Serial.println(F("PFD frequency is out of range"));
      break;
    case ADF4355_ERROR_RF_FREQUENCY:
      Serial.println(F("RF frequency out of range"));
      break;
    case ADF4355_ERROR_POWER_LEVEL:
      Serial.println(F("Power level incorrect"));
      break;
    case ADF4355_ERROR_AUX_POWER_LEVEL:
      Serial.println(F("Auxiliary power level incorrect"));
      break;
    case ADF4355_ERROR_ZERO_PFD_FREQUENCY:
      Serial.println(F("PFD frequency is zero"));
      break;
    case ADF4355_ERROR_R_OVERFLOW_ON_PFD_OVER_75MHz:
      Serial.println(F("R divider temporary doubling overflow with PFD > 75 MHz"));
      break;
    case ADF4355_ERROR_MOD2_RANGE:
      Serial.println(F("Mod2 is out of range"));
      break;
    case ADF4355_ERROR_FRAC2_EXCEEDS_MOD2:
      Serial.println(F("Fraction 2 exceeds Mod2"));
      break;
    case ADF4355_ERROR_FRAC1_RANGE:
      Serial.println(F("Fraction 1 is out of range"));
      break;
    case ADF4355_ERROR_FRAC2_RANGE:
      Serial.println(F("Fraction 2 is out of range"));
      break;
    case ADF4355_ERROR_N_RANGE:
      Serial.println(F("N is out of range"));
      break;
    case ADF4355_ERROR_FREQUENCY_CALCULATION_TIMEOUT:
      Serial.println(F("Frequency calculation timeout"));
      break;
    case ADF4355_WARNING_FREQUENCY_ERROR:
      Serial.println(F("Actual frequency is different than desired"));
      break;
    case ADF4355_ERROR_DOUBLER_EXCEEDED:
      Serial.println(F("Reference frequency with doubler exceeded"));
      break;
    case ADF4355_ERROR_R_RANGE:
      Serial.println(F("R divider is out of range"));
      break;
    case ADF4355_ERROR_REF_FREQUENCY:
      Serial.println(F("Reference frequency is out of range"));
      break;
    case ADF4355_ERROR_REF_MULTIPLIER_TYPE:
      Serial.println(F("Reference multiplier type is incorrect"));
      break;
    case ADF4355_ERROR_REF_INPUT_TYPE:
      Serial.println(F("Reference input type is incorrect"));
      break;
    case ADF4355_ERROR_SINGLE_ENDED_RANGE:
      Serial.println(F("Reference exceeds single ended frequency limit"));
      break;
    case ADF4355_ERROR_PFD_LIMITS:
      Serial.println(F("PFD frequency is out of range"));
      break;
  }
}

void setup() {
  Serial.begin(SerialPortRate);
  vfo.init(SSpin, LockPin, true, CEpin, true);
  digitalWrite(CEpin, HIGH); // enable the ADF4355
  byte ErrorCode = vfo.setrf(ReferenceFrequency, Rvalue, ReferenceDivision, ReferenceInput);
  PrintErrorCode(ErrorCode);
}

void loop() {
  static int ByteCount = 0;
  if (Serial.available() > 0) {
    char value = Serial.read();
    if (value != '\n' && ByteCount < CommandSize) {
      Command[ByteCount] = value;
      ByteCount++;
    }
    else {
      ByteCount = 0;
      bool ValidField = true;
      char field[20];
      getField(field, 0);
      if (strcmp(field, "REF") == 0) {
        getField(field, 1);
        unsigned long ReferenceFreq = atol(field);
        getField(field, 2);
        word ReferenceDivider = atoi(field);
        getField(field, 3);
        byte ReferenceHalfDouble = ADF4355_REF_UNDIVIDED;
        if (strcmp(field, "DOUBLE") == 0) {
          ReferenceHalfDouble = ADF4355_REF_DOUBLE;
        }
        else if (strcmp(field, "HALF") == 0) {
          ReferenceHalfDouble = ADF4355_REF_HALF;
        }
        getField(field, 4);
        byte InputType;
        if (strcmp(field, "SINGLE_ENDED") == 0) {
          InputType = ADF4355_REF_SINGLE_ENDED;
        }
        else if (strcmp(field, "DIFFERENTIAL") == 0) {
          InputType = ADF4355_REF_DIFFERENTIAL;
        }
        else {
          ValidField = false;
        }
        if (ValidField == true) {
          byte ErrorCode = vfo.setrf(ReferenceFreq, ReferenceDivider, ReferenceHalfDouble, InputType);
          if (ErrorCode != ADF4355_ERROR_NONE) {
            ValidField = false;
            PrintErrorCode(ErrorCode);
          }
        }
      }
      else if (strcmp(field, "FREQ") == 0) {
        getField(field, 2);
        byte PowerLevel = atoi(field);
        getField(field, 3);
        byte AuxPowerLevel = atoi(field);
        getField(field, 4);
        unsigned long CalculationTimeout = atol(field);
        getField(field, 1);
        if (ValidField == true) {
          unsigned long FrequencyWriteTimeStart = millis();
          byte ErrorCode = vfo.setf(field, PowerLevel, AuxPowerLevel, CalculationTimeout);
          if (ErrorCode != ADF4355_ERROR_NONE && ErrorCode != ADF4355_WARNING_FREQUENCY_ERROR) {
            ValidField = false;
          }
          else {
            unsigned long FrequencyWriteTime = millis();
            FrequencyWriteTime -= FrequencyWriteTimeStart;
            Serial.print(F("Time measured during setf() with CPU speed of "));
            Serial.print((F_CPU / 1000000UL));
            Serial.print(F("."));
            Serial.print((F_CPU % 1000000UL));
            Serial.print(F(" MHz: "));
            Serial.print((FrequencyWriteTime / 1000));
            Serial.print(F("."));
            Serial.print((FrequencyWriteTime % 1000));
            Serial.println(F(" seconds"));
            PrintVFOstatus();
          }
          PrintErrorCode(ErrorCode);
        }
      }
      else if (strcmp(field, "BURST") == 0 || strcmp(field, "BURST_CONT") == 0 || strcmp(field, "BURST_SINGLE") == 0) {
        bool ContinuousBurst = false;
        bool SingleBurst = false;
        unsigned long BurstCount;
        if (strcmp(field, "BURST_CONT") == 0) {
          ContinuousBurst = true;
        }
        else if (strcmp(field, "BURST_SINGLE") == 0) {
          SingleBurst = true;
        }
        bool AuxOutput = false;
        getField(field, 1);
        unsigned long BurstOnTime = atol(field);
        getField(field, 2);
        unsigned long BurstOffTime = atol(field);
        getField(field, 3);
        if (strcmp(field, "AUX") == 0) {
          AuxOutput = true;
        }
        else if (ContinuousBurst == false && SingleBurst == false) {
          BurstCount = atol(field);
          getField(field, 4);
          if (strcmp(field, "AUX") == 0) {
            AuxOutput = true;
          }
        }
        unsigned long OnBurstData[ADF4355_RegsToWrite];
        vfo.ReadSweepValues(OnBurstData);
        if (AuxOutput == false) {
          vfo.setPowerLevel(0);
        }
        else {
          vfo.setAuxPowerLevel(0);
        }
        unsigned long OffBurstData[ADF4355_RegsToWrite];
        vfo.ReadSweepValues(OffBurstData);
        Serial.print(F("Burst "));
        Serial.print((BurstOnTime / 1000));
        Serial.print(F("."));
        Serial.print((BurstOnTime % 1000));
        Serial.print(F(" mS on, "));
        Serial.print((BurstOffTime / 1000));
        Serial.print(F("."));
        Serial.print((BurstOffTime % 1000));
        Serial.println(F(" mS off"));
        if (SingleBurst == true) {
          vfo.WriteSweepValues(OffBurstData);
          if (BurstOffTime <= 16383) {
            delayMicroseconds(BurstOffTime);
          }
          else {
            delay((BurstOffTime / 1000));
            delayMicroseconds((BurstOffTime % 1000));
          }
        }
        if (ContinuousBurst == false && SingleBurst == false && BurstCount == 0) {
          ValidField = false;
        }
        if (ValidField == true) {
          FlushSerialBuffer();
          while (true) {
            vfo.WriteSweepValues(OnBurstData);
            if (BurstOnTime <= 16383) {
              delayMicroseconds(BurstOnTime);
            }
            else {
              delay((BurstOnTime / 1000));
              delayMicroseconds((BurstOnTime % 1000));
            }
            vfo.WriteSweepValues(OffBurstData);
            if (ContinuousBurst == false && SingleBurst == false) {
              BurstCount--;
            }
            if ((ContinuousBurst == false && BurstCount == 0) || SingleBurst == true || Serial.available() > 0) {
              for (int i = 0; i < ADF4355_RegsToWrite; i++) {
                vfo.ADF4355_R[i] = OnBurstData[i];
              }
              Serial.println(F("End of burst"));
              break;
            }
            if (BurstOffTime <= 16383) {
              delayMicroseconds(BurstOffTime);
            }
            else {
              delay((BurstOffTime / 1000));
              delayMicroseconds((BurstOffTime % 1000));
            }
          }
        }
      }
      else if (strcmp(field, "SWEEP") == 0) {
        BigNumber::begin(12); // will finish on setf()
        getField(field, 1);
        BigNumber BN_StartFrequency(field);
        getField(field, 2);
        BigNumber BN_StopFrequency(field);
        if (BN_StartFrequency < BN_StopFrequency) {
          getField(field, 3);
          word SweepStepTime = atoi(field);
          getField(field, 4);
          byte PowerLevel = atoi(field);
          getField(field, 5);
          byte AuxPowerLevel = atoi(field);
          getField(field, 6);
          unsigned long CalculationTimeout = atol(field);
          char tmpstr[12];
          ultoa(SweepSteps, tmpstr, 10);
          BigNumber BN_StepSize = ((BN_StopFrequency - BN_StartFrequency) / (BigNumber(tmpstr) - BigNumber("1")));
          uint32_t StepSizeRounding = (uint32_t)((uint32_t) BN_StepSize);
          ultoa(StepSizeRounding, tmpstr, 10);
          BN_StepSize = BigNumber(tmpstr);
          char StepSize[14];
          char StartFrequency[14];
          char* tempstring1 = BN_StepSize.toString();
          for (int i = 0; i < 14; i++) {
            byte temp = tempstring1[i];
            if (temp == '.') {
              StepSize[i] = 0x00;
              break;
            }
            StepSize[i] = temp;
          }
          free(tempstring1);
          char* tempstring2 = BN_StartFrequency.toString();
          BigNumber::finish();
          for (int i = 0; i < 14; i++) {
            byte temp = tempstring2[i];
            if (temp == '.') {
              StepSize[i] = 0x00;
              break;
            }
            StartFrequency[i] = temp;
          }
          free(tempstring2);
          uint32_t regs[(ADF4355_RegsToWrite * SweepSteps)];
          uint32_t reg_temp[ADF4355_RegsToWrite];
          for (word SweepCount = 0; SweepCount < SweepSteps; SweepCount++) {
            Serial.print(F("Calculating step "));
            Serial.print(SweepCount);
            char CurrentFrequency[14]; // enough to hold 10 digits and a sign and null terminator - no decimal places will be used
            BigNumber::begin(12);
            BigNumber BN_CurrentFrequency = (BigNumber(StartFrequency) + (BigNumber(StepSize) * BigNumber(SweepCount)));
            char* tempstring3 = BN_CurrentFrequency.toString();
            BigNumber::finish();
            for (int y = 0; y < 14; y++) {
              byte temp = tempstring3[y];
              if (temp == '.') {
                CurrentFrequency[y] = 0x00;
                break;
              }
              CurrentFrequency[y] = temp;
            }
            free(tempstring3);
            Serial.print(F(" - frequency is now "));
            Serial.print(CurrentFrequency);
            Serial.println(F(" Hz"));
            byte ErrorCode = vfo.setf(CurrentFrequency, PowerLevel, AuxPowerLevel, CalculationTimeout);
            if (ErrorCode != ADF4355_ERROR_NONE && ErrorCode != ADF4355_WARNING_FREQUENCY_ERROR) {
              ValidField = false;
              PrintErrorCode(ErrorCode);
              break;
            }
            PrintVFOstatus();
            vfo.ReadSweepValues(reg_temp);
            for (int y = 0; y < ADF4355_RegsToWrite; y++) {
              regs[(y + (ADF4355_RegsToWrite * SweepCount))] = reg_temp[y];
            }
          }
          if (ValidField == true) {
            Serial.println(F("Now sweeping"));
            FlushSerialBuffer();
            while (true) {
              if (Serial.available() > 0) {
                break;
              }
              for (word SweepCount = 0; SweepCount < SweepSteps; SweepCount++) {
                if (Serial.available() > 0) {
                  break;
                }
                for (int y = 0; y < ADF4355_RegsToWrite; y++) {
                  reg_temp[y] = regs[(y + (ADF4355_RegsToWrite * SweepCount))];
                }
                vfo.WriteSweepValues(reg_temp);
                delay(SweepStepTime);
              }
              Serial.print(F("*"));
            }
            Serial.print(F(""));
            Serial.println(F("End of sweep"));
          }
        }
        else {
          BigNumber::finish();
          Serial.println(F("Stop frequency must be greater than start frequency"));
          ValidField = false;
        }
      }
      else if (strcmp(field, "STEP") == 0) {
        getField(field, 1);
        unsigned long StepFrequency = atol(field);
        byte ErrorCode = vfo.SetStepFreq(StepFrequency);
        if (ErrorCode != ADF4355_ERROR_NONE) {
          ValidField = false;
          PrintErrorCode(ErrorCode);
        }
      }
      else if (strcmp(field, "STATUS") == 0) {
        PrintVFOstatus();
        SPI.end();
        if (digitalRead(LockPin) == LOW) {
          Serial.println(F("Lock pin LOW"));
        }
        else {
          Serial.println(F("Lock pin HIGH"));
        }
        SPI.begin();
      }
      else if (strcmp(field, "CE") == 0) {
        getField(field, 1);
        if (strcmp(field, "ON") == 0) {
          digitalWrite(CEpin, HIGH);
        }
        else if (strcmp(field, "OFF") == 0) {
          digitalWrite(CEpin, LOW);
        }
        else {
          ValidField = false;
        }
      }
      else if (strcmp(field, "ADC_DELAY") == 0) {
        getField(field, 1);
        if (strcmp(field, "ON") == 0) {
          vfo.ADCdelayRequired = true;
        }
        else if (strcmp(field, "OFF") == 0) {
          vfo.ADCdelayRequired = false;
        }
        else {
          ValidField = false;
        }
      }
      else {
        ValidField = false;
      }
      FlushSerialBuffer();
      if (ValidField == true) {
        Serial.println(F("OK"));
      }
      else {
        Serial.println(F("ERROR"));
      }
    }
  }
}