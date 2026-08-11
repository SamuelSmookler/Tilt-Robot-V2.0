#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#if !defined(ARDUINO_TEENSY41)
#error "Select Teensy 4.1 under Tools > Board."
#endif

// ---- Optional station hardware (flip to 1 once wired; defaults keep the ----
// ---- current DUT-only build byte-identical).                            ----
#define ENABLE_ENCODER 0   // AS5047P angle encoder on SPI  (CS = pin 10)
#define ENABLE_ADC 0       // ADS1256 24-bit ADC on SPI1    (CS = 0, DRDY = 1)
//
// !! CHIP MISMATCH -- READ BEFORE SETTING ENABLE_ADC TO 1 (noted 2026-08-01) !!
// The ADC code below targets an ADS1256. The v2 hardware is expected to use an
// AD7124-8 instead. Both are 24-bit SPI ADCs so the wiring is similar (SCLK,
// MOSI, MISO, CS, DRDY), but their register maps and command sets are entirely
// different: adcReadChannelVolts() writes ADS1256 opcodes and will NOT work on
// an AD7124-8. If the AD7124-8 is confirmed, that function needs rewriting, not
// just enabling. Confirm the actual part before wiring or flipping this flag.

#define ENABLE_TILT 1      // Fredericks 1-6200-005 SPI tilt conditioner (CS = pin 9)

#if ENABLE_ENCODER || ENABLE_ADC || ENABLE_TILT
#include <SPI.h>
#endif

#if ENABLE_TILT
#include "tilt005.h"
#endif

static constexpr unsigned long USB_BAUD = 115200;
static constexpr unsigned long DUT_BAUD = 38400;

static constexpr int N_UUT = 8;

static constexpr float DUT_SAMPLE_DELAY_S = 0.0f;
static constexpr uint32_t DUT_MAX_AGE_MS = 250;
static constexpr uint32_t DUT_PROBE_TIMEOUT_MS = 600;
static constexpr uint32_t DUT_COMMAND_TIMEOUT_MS = 500;

static constexpr uint16_t DUT_PACKET_MAX = 128;
static constexpr uint16_t DUT_EXTRA_RX_SIZE = 512;

static HardwareSerialIMXRT* const uart[N_UUT] = {
  &Serial1,
  &Serial2,
  &Serial3,
  &Serial4,
  &Serial5,
  &Serial6,
  &Serial7,
  &Serial8
};

static uint8_t uutExtraRx[N_UUT][DUT_EXTRA_RX_SIZE];

static uint8_t uutBuf[N_UUT][DUT_PACKET_MAX];
static uint16_t uutLen[N_UUT];

static float uutAcc[N_UUT][3];
static float uutPR[N_UUT][2];

static bool uutConfigured[N_UUT];
static bool uutFresh[N_UUT];

static char uutModel[N_UUT][5];
static char uutRevision[N_UUT][5];
static uint32_t uutSerialNumber[N_UUT];

static uint32_t uutBytes[N_UUT];
static uint32_t uutPkts[N_UUT];
static uint32_t uutDataSeq[N_UUT];
static uint32_t uutCrcErr[N_UUT];
static uint32_t uutIncomplete[N_UUT];
static uint32_t uutOverflow[N_UUT];

static uint8_t uutLastFid[N_UUT];

static uint32_t uutLastPktMs[N_UUT];
static uint32_t uutLastDataMs[N_UUT];
static uint32_t uutLastDataUs[N_UUT];

static char lineBuf[128];
static uint8_t lineLen = 0;

static bool usbStreaming = false;
static uint32_t usbStreamPeriodMs = 100;
static uint32_t usbLastStreamMs = 0;

static uint32_t frameSequence = 0;

static void sendOkRaw(long tag, const char* payload) {
  Serial.print('#');
  Serial.print(tag);
  Serial.print(" OK");

  if (payload != nullptr && payload[0] != '\0') {
    Serial.print(' ');
    Serial.print(payload);
  }

  Serial.print('\n');
}

static void sendErr(long tag, const char* code, const char* message) {
  Serial.print('#');
  Serial.print(tag);
  Serial.print(" ERR ");
  Serial.print(code);
  Serial.print(' ');
  Serial.print(message);
  Serial.print('\n');
}

static void sendEvt(const char* name, const char* payload) {
  Serial.print("EVT ");
  Serial.print(name);

  if (payload != nullptr && payload[0] != '\0') {
    Serial.print(' ');
    Serial.print(payload);
  }

  Serial.print('\n');
}

static bool appendf(
    char* buffer,
    size_t capacity,
    size_t& used,
    const char* format,
    ...) {
  if (used >= capacity) {
    return false;
  }

  va_list args;
  va_start(args, format);

  const int written = vsnprintf(
      buffer + used,
      capacity - used,
      format,
      args);

  va_end(args);

  if (written < 0) {
    return false;
  }

  if (static_cast<size_t>(written) >= capacity - used) {
    buffer[capacity - 1] = '\0';
    return false;
  }

  used += static_cast<size_t>(written);
  return true;
}

static uint16_t dutCrc16(const uint8_t* data, int length) {
  uint16_t crc = 0x0000;

  for (int i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;

    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

static float readBigEndianFloat(const uint8_t* data) {
  const uint32_t bits =
      (static_cast<uint32_t>(data[0]) << 24) |
      (static_cast<uint32_t>(data[1]) << 16) |
      (static_cast<uint32_t>(data[2]) << 8) |
      static_cast<uint32_t>(data[3]);

  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static void writeBigEndianFloat(float value, uint8_t* output) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));

  output[0] = static_cast<uint8_t>(bits >> 24);
  output[1] = static_cast<uint8_t>(bits >> 16);
  output[2] = static_cast<uint8_t>(bits >> 8);
  output[3] = static_cast<uint8_t>(bits);
}

static uint32_t readBigEndianUInt32(const uint8_t* data) {
  return
      (static_cast<uint32_t>(data[0]) << 24) |
      (static_cast<uint32_t>(data[1]) << 16) |
      (static_cast<uint32_t>(data[2]) << 8) |
      static_cast<uint32_t>(data[3]);
}

static void copySafeAscii(
    const uint8_t* source,
    char* destination,
    size_t length) {
  for (size_t i = 0; i < length; ++i) {
    const uint8_t value = source[i];

    if (value >= 32 &&
        value <= 126 &&
        value != '"' &&
        value != '\\') {
      destination[i] = static_cast<char>(value);
    } else {
      destination[i] = '.';
    }
  }

  destination[length] = '\0';
}

static bool sendDutFrame(
    HardwareSerial* serialPort,
    uint8_t frameId,
    const uint8_t* payload = nullptr,
    uint16_t payloadLength = 0) {
  const uint16_t packetLength =
      2 + 1 + payloadLength + 2;

  if (packetLength > DUT_PACKET_MAX) {
    return false;
  }

  uint8_t packet[DUT_PACKET_MAX];

  packet[0] = static_cast<uint8_t>(packetLength >> 8);
  packet[1] = static_cast<uint8_t>(packetLength);
  packet[2] = frameId;

  if (payloadLength > 0 && payload != nullptr) {
    memcpy(packet + 3, payload, payloadLength);
  }

  const uint16_t crc =
      dutCrc16(packet, packetLength - 2);

  packet[packetLength - 2] =
      static_cast<uint8_t>(crc >> 8);

  packet[packetLength - 1] =
      static_cast<uint8_t>(crc);

  serialPort->write(packet, packetLength);
  serialPort->flush();

  return true;
}

static void drainDutInput(HardwareSerial* serialPort) {
  while (serialPort->available() > 0) {
    serialPort->read();
  }
}

static void removeBytes(
    uint8_t* buffer,
    uint16_t& length,
    uint16_t count) {
  if (count >= length) {
    length = 0;
    return;
  }

  memmove(
      buffer,
      buffer + count,
      length - count);

  length -= count;
}

static bool waitForDutFrame(
    HardwareSerial* serialPort,
    uint8_t expectedFrameId,
    uint8_t* outputPacket = nullptr,
    uint16_t* outputLength = nullptr,
    uint32_t timeoutMs = DUT_COMMAND_TIMEOUT_MS) {
  uint8_t buffer[DUT_PACKET_MAX];
  uint16_t bufferLength = 0;

  const uint32_t start = millis();

  while (static_cast<uint32_t>(millis() - start) < timeoutMs) {
    while (serialPort->available() > 0) {
      const uint8_t value =
          static_cast<uint8_t>(serialPort->read());

      if (bufferLength < sizeof(buffer)) {
        buffer[bufferLength++] = value;
      } else {
        memmove(
            buffer,
            buffer + 1,
            sizeof(buffer) - 1);

        buffer[sizeof(buffer) - 1] = value;
      }

      bool processAgain = true;

      while (processAgain) {
        processAgain = false;

        if (bufferLength < 2) {
          break;
        }

        const uint16_t packetLength =
            (static_cast<uint16_t>(buffer[0]) << 8) |
            static_cast<uint16_t>(buffer[1]);

        if (packetLength < 5 ||
            packetLength > sizeof(buffer)) {
          removeBytes(buffer, bufferLength, 1);
          processAgain = true;
          continue;
        }

        if (bufferLength < packetLength) {
          break;
        }

        const uint16_t receivedCrc =
            (static_cast<uint16_t>(
                 buffer[packetLength - 2]) << 8) |
            static_cast<uint16_t>(
                buffer[packetLength - 1]);

        const uint16_t calculatedCrc =
            dutCrc16(buffer, packetLength - 2);

        if (receivedCrc != calculatedCrc) {
          removeBytes(buffer, bufferLength, 1);
          processAgain = true;
          continue;
        }

        const uint8_t frameId = buffer[2];

        if (frameId == expectedFrameId) {
          if (outputPacket != nullptr) {
            memcpy(outputPacket, buffer, packetLength);
          }

          if (outputLength != nullptr) {
            *outputLength = packetLength;
          }

          return true;
        }

        removeBytes(
            buffer,
            bufferLength,
            packetLength);

        processAgain = true;
      }
    }

    yield();
  }

  return false;
}

static bool setDutBooleanConfig(
    int index,
    uint8_t configId,
    bool value) {
  const uint8_t payload[2] = {
    configId,
    static_cast<uint8_t>(value ? 1 : 0)
  };

  if (!sendDutFrame(
          uart[index],
          0x06,
          payload,
          sizeof(payload))) {
    return false;
  }

  return waitForDutFrame(
      uart[index],
      0x13,
      nullptr,
      nullptr,
      DUT_COMMAND_TIMEOUT_MS);
}

static bool setDutFilterZero(int index) {
  const uint8_t payload[3] = {
    0x03,
    0x01,
    0x00
  };

  if (!sendDutFrame(
          uart[index],
          0x0C,
          payload,
          sizeof(payload))) {
    return false;
  }

  return waitForDutFrame(
      uart[index],
      0x14,
      nullptr,
      nullptr,
      DUT_COMMAND_TIMEOUT_MS);
}

static bool probeDut(int index) {
  HardwareSerial* serialPort = uart[index];

  memset(uutModel[index], 0, sizeof(uutModel[index]));
  memset(uutRevision[index], 0, sizeof(uutRevision[index]));

  uutSerialNumber[index] = 0;

  if (!sendDutFrame(serialPort, 0x01)) {
    return false;
  }

  uint8_t packet[DUT_PACKET_MAX];
  uint16_t packetLength = 0;

  if (!waitForDutFrame(
          serialPort,
          0x02,
          packet,
          &packetLength,
          DUT_PROBE_TIMEOUT_MS)) {
    return false;
  }

  if (packetLength < 13) {
    return false;
  }

  copySafeAscii(
      packet + 3,
      uutModel[index],
      4);

  copySafeAscii(
      packet + 7,
      uutRevision[index],
      4);

  if (!sendDutFrame(serialPort, 0x34)) {
    return true;
  }

  packetLength = 0;

  if (waitForDutFrame(
          serialPort,
          0x35,
          packet,
          &packetLength,
          DUT_COMMAND_TIMEOUT_MS) &&
      packetLength >= 9) {
    uutSerialNumber[index] =
        readBigEndianUInt32(packet + 3);
  }

  return true;
}

static bool configureDut(int index) {
  HardwareSerial* serialPort = uart[index];

  if (!setDutBooleanConfig(index, 0x06, true)) {
    return false;
  }

  if (!setDutBooleanConfig(index, 0x0F, false)) {
    return false;
  }

  if (!setDutFilterZero(index)) {
    return false;
  }

  const uint8_t components[6] = {
    5,
    0x15,
    0x16,
    0x17,
    0x18,
    0x19
  };

  if (!sendDutFrame(
          serialPort,
          0x03,
          components,
          sizeof(components))) {
    return false;
  }

  delay(10);

  uint8_t acquisition[10];

  acquisition[0] = 0x00;
  acquisition[1] = 0x00;

  writeBigEndianFloat(
      0.0f,
      acquisition + 2);

  writeBigEndianFloat(
      DUT_SAMPLE_DELAY_S,
      acquisition + 6);

  if (!sendDutFrame(
          serialPort,
          0x18,
          acquisition,
          sizeof(acquisition))) {
    return false;
  }

  return waitForDutFrame(
      serialPort,
      0x1A,
      nullptr,
      nullptr,
      DUT_COMMAND_TIMEOUT_MS);
}

static void startDut(int index) {
  drainDutInput(uart[index]);
  uutLen[index] = 0;
  sendDutFrame(uart[index], 0x15);
}

static void stopAllDuts() {
  for (int i = 0; i < N_UUT; ++i) {
    sendDutFrame(uart[i], 0x16);
  }

  delay(80);

  for (int i = 0; i < N_UUT; ++i) {
    drainDutInput(uart[i]);
    uutLen[i] = 0;
  }
}

static int reconfigureAllDuts() {
  stopAllDuts();

  int configuredCount = 0;

  for (int i = 0; i < N_UUT; ++i) {
    uutConfigured[i] = false;
    uutFresh[i] = false;

    uutLastDataMs[i] = 0;
    uutLastDataUs[i] = 0;

    if (!probeDut(i)) {
      continue;
    }

    if (!configureDut(i)) {
      continue;
    }

    uutConfigured[i] = true;
    ++configuredCount;
  }

  for (int i = 0; i < N_UUT; ++i) {
    if (uutConfigured[i]) {
      startDut(i);
    }
  }

  return configuredCount;
}

static bool handleDutPacket(
    int index,
    const uint8_t* packet,
    uint16_t packetLength) {
  if (packet[2] != 0x05) {
    return false;
  }

  const uint8_t* payload = packet + 3;
  const int payloadLength = packetLength - 5;

  if (payloadLength < 1) {
    ++uutIncomplete[index];
    return false;
  }

  const int componentCount = payload[0];

  float acceleration[3] = {};
  float pitchRoll[2] = {};

  uint8_t componentMask = 0;
  int offset = 1;

  for (int component = 0;
       component < componentCount;
       ++component) {
    if (offset + 5 > payloadLength) {
      ++uutIncomplete[index];
      return false;
    }

    const uint8_t componentId =
        payload[offset++];

    const float value =
        readBigEndianFloat(payload + offset);

    offset += 4;

    if (!isfinite(value)) {
      ++uutIncomplete[index];
      return false;
    }

    switch (componentId) {
      case 0x15:
        acceleration[0] = value;
        componentMask |= 0x01;
        break;

      case 0x16:
        acceleration[1] = value;
        componentMask |= 0x02;
        break;

      case 0x17:
        acceleration[2] = value;
        componentMask |= 0x04;
        break;

      case 0x18:
        pitchRoll[0] = value;
        componentMask |= 0x08;
        break;

      case 0x19:
        pitchRoll[1] = value;
        componentMask |= 0x10;
        break;

      default:
        break;
    }
  }

  if (componentMask != 0x1F) {
    ++uutIncomplete[index];
    return false;
  }

  memcpy(
      uutAcc[index],
      acceleration,
      sizeof(acceleration));

  memcpy(
      uutPR[index],
      pitchRoll,
      sizeof(pitchRoll));

  uutFresh[index] = true;
  uutLastDataMs[index] = millis();
  uutLastDataUs[index] = micros();

  ++uutDataSeq[index];

  return true;
}

static void dropBufferedBytes(
    int index,
    uint16_t count) {
  if (count >= uutLen[index]) {
    uutLen[index] = 0;
    return;
  }

  memmove(
      uutBuf[index],
      uutBuf[index] + count,
      uutLen[index] - count);

  uutLen[index] -= count;
}

static void pumpUuts() {
  for (int i = 0; i < N_UUT; ++i) {
    while (uart[i]->available() > 0) {
      const uint8_t value =
          static_cast<uint8_t>(uart[i]->read());

      ++uutBytes[i];

      if (uutLen[i] < sizeof(uutBuf[i])) {
        uutBuf[i][uutLen[i]++] = value;
      } else {
        memmove(
            uutBuf[i],
            uutBuf[i] + 1,
            sizeof(uutBuf[i]) - 1);

        uutBuf[i][sizeof(uutBuf[i]) - 1] = value;
        ++uutOverflow[i];
      }
    }

    bool processAgain = true;

    while (processAgain) {
      processAgain = false;

      if (uutLen[i] < 2) {
        break;
      }

      const uint16_t packetLength =
          (static_cast<uint16_t>(uutBuf[i][0]) << 8) |
          static_cast<uint16_t>(uutBuf[i][1]);

      if (packetLength < 5 ||
          packetLength > sizeof(uutBuf[i])) {
        dropBufferedBytes(i, 1);
        processAgain = true;
        continue;
      }

      if (uutLen[i] < packetLength) {
        break;
      }

      const uint16_t receivedCrc =
          (static_cast<uint16_t>(
               uutBuf[i][packetLength - 2]) << 8) |
          static_cast<uint16_t>(
              uutBuf[i][packetLength - 1]);

      const uint16_t calculatedCrc =
          dutCrc16(
              uutBuf[i],
              packetLength - 2);

      if (receivedCrc != calculatedCrc) {
        ++uutCrcErr[i];
        dropBufferedBytes(i, 1);
        processAgain = true;
        continue;
      }

      ++uutPkts[i];

      uutLastFid[i] = uutBuf[i][2];
      uutLastPktMs[i] = millis();

      handleDutPacket(
          i,
          uutBuf[i],
          packetLength);

      dropBufferedBytes(
          i,
          packetLength);

      processAgain = true;
    }
  }
}

// ---- Optional station hardware: AS5047P encoder + ADS1256 ADC --------------
#if ENABLE_ENCODER
static constexpr int ENCODER_CS_PIN = 10;

static uint16_t encoderAddParity(uint16_t value) {
  uint16_t ones = 0;

  for (int bit = 0; bit < 15; ++bit) {
    if ((value & (1u << bit)) != 0) {
      ++ones;
    }
  }

  if ((ones & 1u) != 0) {
    value |= 0x8000;
  }

  return value;
}

// Shaft angle in degrees mapped to [-180, 180). Valid flag via the error bit.
static bool readEncoderDeg(float& degreesOut) {
  const SPISettings settings(1000000, MSBFIRST, SPI_MODE1);
  const uint16_t command =
      encoderAddParity(0x4000 | 0x3FFF);   // READ | ANGLECOM

  SPI.beginTransaction(settings);

  digitalWrite(ENCODER_CS_PIN, LOW);
  SPI.transfer16(command);
  digitalWrite(ENCODER_CS_PIN, HIGH);

  digitalWrite(ENCODER_CS_PIN, LOW);
  const uint16_t raw = SPI.transfer16(0x0000);
  digitalWrite(ENCODER_CS_PIN, HIGH);

  SPI.endTransaction();

  if ((raw & 0x4000) != 0) {               // error flag from the encoder
    return false;
  }

  float degrees =
      static_cast<float>(raw & 0x3FFF) * 360.0f / 16384.0f;

  if (degrees >= 180.0f) {
    degrees -= 360.0f;
  }

  degreesOut = degrees;
  return true;
}
#endif  // ENABLE_ENCODER

#if ENABLE_ADC
static constexpr int ADC_CS_PIN = 0;
static constexpr int ADC_DRDY_PIN = 1;
static constexpr float ADC_VREF = 2.5f;

// Single-ended channel assignments (AINx vs AINCOM). Adjust to the wiring.
static constexpr uint8_t ADC_CH_TEMP_UUT = 0;
static constexpr uint8_t ADC_CH_TEMP_REF = 1;
static constexpr uint8_t ADC_CH_LEVEL = 2;
static constexpr uint8_t ADC_CH_BASE_LEVEL = 3;

static bool adcWaitDataReady() {
  uint32_t guard = 0;

  while (digitalRead(ADC_DRDY_PIN) == HIGH) {
    if (++guard > 200000) {
      return false;
    }
  }

  return true;
}

// Read one single-ended channel; returns volts. False on DRDY timeout.
static bool adcReadChannelVolts(uint8_t channel, float& voltsOut) {
  const SPISettings settings(1920000, MSBFIRST, SPI_MODE1);

  if (!adcWaitDataReady()) {
    return false;
  }

  SPI1.beginTransaction(settings);
  digitalWrite(ADC_CS_PIN, LOW);

  SPI1.transfer(0x50 | 0x01);              // WREG MUX
  SPI1.transfer(0x00);
  SPI1.transfer(static_cast<uint8_t>((channel << 4) | 0x08));

  SPI1.transfer(0xFC);                     // SYNC
  SPI1.transfer(0x00);                     // WAKEUP
  SPI1.transfer(0x01);                     // RDATA
  delayMicroseconds(10);

  uint32_t raw =
      (static_cast<uint32_t>(SPI1.transfer(0)) << 16) |
      (static_cast<uint32_t>(SPI1.transfer(0)) << 8) |
      static_cast<uint32_t>(SPI1.transfer(0));

  digitalWrite(ADC_CS_PIN, HIGH);
  SPI1.endTransaction();

  int32_t value =
      (raw & 0x800000) != 0
          ? static_cast<int32_t>(raw | 0xFF000000)
          : static_cast<int32_t>(raw);

  voltsOut =
      static_cast<float>(value) / 8388607.0f * (2.0f * ADC_VREF);

  return true;
}

// Engineering conversions — placeholders until bench calibration.
static float adcVoltsToCelsius(float volts) { return volts * 100.0f; }
static float adcVoltsToTiltDeg(float volts) { return volts * 10.0f; }
#endif  // ENABLE_ADC

static bool buildFrame(
    char* buffer,
    size_t capacity) {
  size_t used = 0;

  const uint32_t snapshotUs = micros();
  const uint32_t nowMs = millis();
  const uint32_t currentSequence = ++frameSequence;

  float encoderDeg = 0.0f;
  int encoderValid = 0;

#if ENABLE_ENCODER
  if (readEncoderDeg(encoderDeg)) {
    encoderValid = 1;
  } else {
    encoderDeg = 0.0f;
  }
#endif

  if (!appendf(
          buffer,
          capacity,
          used,
          "{\"frame_seq\":%lu,"
          "\"t_us\":%lu,"
          "\"nunits\":%d,"
          "\"enc_deg\":%.4f,"
          "\"enc_valid\":%d,"
          "\"ref\":[0.0,0.0,0.0],"
          "\"ref_valid\":0,"
          "\"units\":[",
          static_cast<unsigned long>(currentSequence),
          static_cast<unsigned long>(snapshotUs),
          N_UUT,
          static_cast<double>(encoderDeg),
          encoderValid)) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    if (!appendf(
            buffer,
            capacity,
            used,
            "%s[%.7f,%.7f,%.7f]",
            i == 0 ? "" : ",",
            uutAcc[i][0],
            uutAcc[i][1],
            uutAcc[i][2])) {
      return false;
    }
  }

  if (!appendf(
          buffer,
          capacity,
          used,
          "],\"units_pr\":[")) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    if (!appendf(
            buffer,
            capacity,
            used,
            "%s[%.4f,%.4f]",
            i == 0 ? "" : ",",
            uutPR[i][0],
            uutPR[i][1])) {
      return false;
    }
  }

  if (!appendf(
          buffer,
          capacity,
          used,
          "],\"units_configured\":[")) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    if (!appendf(
            buffer,
            capacity,
            used,
            "%s%d",
            i == 0 ? "" : ",",
            uutConfigured[i] ? 1 : 0)) {
      return false;
    }
  }

  if (!appendf(
          buffer,
          capacity,
          used,
          "],\"units_valid\":[")) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    const uint32_t age =
        uutFresh[i]
            ? static_cast<uint32_t>(
                  nowMs - uutLastDataMs[i])
            : UINT32_MAX;

    const bool valid =
        uutConfigured[i] &&
        uutFresh[i] &&
        age <= DUT_MAX_AGE_MS;

    if (!appendf(
            buffer,
            capacity,
            used,
            "%s%d",
            i == 0 ? "" : ",",
            valid ? 1 : 0)) {
      return false;
    }
  }

  if (!appendf(
          buffer,
          capacity,
          used,
          "],\"units_age_ms\":[")) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    const long age =
        uutFresh[i]
            ? static_cast<long>(
                  nowMs - uutLastDataMs[i])
            : -1L;

    if (!appendf(
            buffer,
            capacity,
            used,
            "%s%ld",
            i == 0 ? "" : ",",
            age)) {
      return false;
    }
  }

  if (!appendf(
          buffer,
          capacity,
          used,
          "],\"units_seq\":[")) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    if (!appendf(
            buffer,
            capacity,
            used,
            "%s%lu",
            i == 0 ? "" : ",",
            static_cast<unsigned long>(
                uutDataSeq[i]))) {
      return false;
    }
  }

  if (!appendf(
          buffer,
          capacity,
          used,
          "],\"units_sample_us\":[")) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    if (!appendf(
            buffer,
            capacity,
            used,
            "%s%lu",
            i == 0 ? "" : ",",
            static_cast<unsigned long>(
                uutLastDataUs[i]))) {
      return false;
    }
  }

  float tempC[2] = {0.0f, 0.0f};
  float levelDeg[2] = {0.0f, 0.0f};
  float baseLevelDeg[2] = {0.0f, 0.0f};
  int analogValid = 0;

#if ENABLE_ADC
  {
    float volts = 0.0f;
    bool allOk = true;

    if (adcReadChannelVolts(ADC_CH_TEMP_UUT, volts)) {
      tempC[0] = adcVoltsToCelsius(volts);
    } else {
      allOk = false;
    }

    if (adcReadChannelVolts(ADC_CH_TEMP_REF, volts)) {
      tempC[1] = adcVoltsToCelsius(volts);
    } else {
      allOk = false;
    }

    if (adcReadChannelVolts(ADC_CH_LEVEL, volts)) {
      levelDeg[0] = adcVoltsToTiltDeg(volts);
    } else {
      allOk = false;
    }

    if (adcReadChannelVolts(ADC_CH_BASE_LEVEL, volts)) {
      baseLevelDeg[0] = adcVoltsToTiltDeg(volts);
    } else {
      allOk = false;
    }

    analogValid = allOk ? 1 : 0;
  }
#endif

#if ENABLE_TILT
  // Fredericks 1-6200-005 with two single-axis 0719-3703-99 sensors:
  //   J3 -> X channel, J4 -> Y channel. Board must be jumpered for single-axis
  //   mode (R5 open, R6 = 1 kOhm). Range is only +/-0.5 deg -- it is a precision
  //   NULL detector for finding level, not a general inclinometer.
  {
    float tx = 0.0f, ty = 0.0f, tt = 0.0f;
    if (read005(tx, ty, tt)) {
      levelDeg[0] = tx;
      levelDeg[1] = ty;
      tempC[1] = tt;          // 005 board temperature -> reference temp slot
      analogValid = 1;
    }
  }
#endif

  return appendf(
      buffer,
      capacity,
      used,
      "],"
      "\"temp\":[%.3f,%.3f],"
      "\"level\":[%.4f,%.4f],"
      "\"base_level\":[%.4f,%.4f],"
      "\"analog_valid\":%d}",
      static_cast<double>(tempC[0]),
      static_cast<double>(tempC[1]),
      static_cast<double>(levelDeg[0]),
      static_cast<double>(levelDeg[1]),
      static_cast<double>(baseLevelDeg[0]),
      static_cast<double>(baseLevelDeg[1]),
      analogValid);
}

static bool buildStatus(
    char* buffer,
    size_t capacity) {
  int configuredCount = 0;
  int freshCount = 0;

  const uint32_t nowMs = millis();

  for (int i = 0; i < N_UUT; ++i) {
    if (uutConfigured[i]) {
      ++configuredCount;
    }

    if (uutConfigured[i] &&
        uutFresh[i] &&
        static_cast<uint32_t>(
            nowMs - uutLastDataMs[i]) <=
            DUT_MAX_AGE_MS) {
      ++freshCount;
    }
  }

  const int written = snprintf(
      buffer,
      capacity,
      "{\"state\":\"READY\","
      "\"streaming\":%d,"
      "\"period_ms\":%lu,"
      "\"nunits\":%d,"
      "\"configured\":%d,"
      "\"fresh\":%d}",
      usbStreaming ? 1 : 0,
      static_cast<unsigned long>(
          usbStreamPeriodMs),
      N_UUT,
      configuredCount,
      freshCount);

  return written >= 0 &&
         static_cast<size_t>(written) < capacity;
}

static bool buildInfo(
    char* buffer,
    size_t capacity) {
  size_t used = 0;

  if (!appendf(
          buffer,
          capacity,
          used,
          "{\"nunits\":%d,\"dut\":[",
          N_UUT)) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    if (!appendf(
            buffer,
            capacity,
            used,
            "%s{"
            "\"index\":%d,"
            "\"configured\":%d,"
            "\"model\":\"%s\","
            "\"revision\":\"%s\","
            "\"serial\":%lu}",
            i == 0 ? "" : ",",
            i,
            uutConfigured[i] ? 1 : 0,
            uutModel[i],
            uutRevision[i],
            static_cast<unsigned long>(
                uutSerialNumber[i]))) {
      return false;
    }
  }

  return appendf(
      buffer,
      capacity,
      used,
      "]}");
}

static bool buildDiagnostics(
    char* buffer,
    size_t capacity) {
  size_t used = 0;
  const uint32_t nowMs = millis();

  if (!appendf(
          buffer,
          capacity,
          used,
          "{\"dut\":[")) {
    return false;
  }

  for (int i = 0; i < N_UUT; ++i) {
    const long age =
        uutFresh[i]
            ? static_cast<long>(
                  nowMs - uutLastDataMs[i])
            : -1L;

    if (!appendf(
            buffer,
            capacity,
            used,
            "%s{"
            "\"index\":%d,"
            "\"configured\":%d,"
            "\"fresh\":%d,"
            "\"age_ms\":%ld,"
            "\"bytes\":%lu,"
            "\"packets\":%lu,"
            "\"data\":%lu,"
            "\"crc\":%lu,"
            "\"incomplete\":%lu,"
            "\"overflow\":%lu,"
            "\"last_frame\":%u}",
            i == 0 ? "" : ",",
            i,
            uutConfigured[i] ? 1 : 0,
            uutFresh[i] ? 1 : 0,
            age,
            static_cast<unsigned long>(
                uutBytes[i]),
            static_cast<unsigned long>(
                uutPkts[i]),
            static_cast<unsigned long>(
                uutDataSeq[i]),
            static_cast<unsigned long>(
                uutCrcErr[i]),
            static_cast<unsigned long>(
                uutIncomplete[i]),
            static_cast<unsigned long>(
                uutOverflow[i]),
            static_cast<unsigned int>(
                uutLastFid[i]))) {
      return false;
    }
  }

  return appendf(
      buffer,
      capacity,
      used,
      "]}");
}

static void handleCommand(
    long tag,
    char* commandText) {
  char* verb = strtok(commandText, " ");

  if (verb == nullptr) {
    sendErr(tag, "EBADCMD", "empty");
    return;
  }

  if (strcmp(verb, "PING") == 0) {
    sendOkRaw(tag, "PONG");
    return;
  }

#if ENABLE_TILT
  // Bring-up probe for the Fredericks 005. The board's own temperature sensor
  // works with NO tilt sensors attached, so a sane room temperature proves the
  // whole SPI chain (wiring, 3.3V, SPI_MODE2, CS, one-byte reply lag).
  if (strcmp(verb, "TILT") == 0) {
    float x, y, t;
    if (read005(x, y, t)) {
      char buf[140];
      snprintf(buf, sizeof(buf),
               "x=%.5f y=%.5f tempC=%.2f | raw ack=%02X x=%02X%02X y=%02X%02X t=%02X%02X",
               x, y, t, tiltLastAck,
               tiltLastBytes[0], tiltLastBytes[1], tiltLastBytes[2],
               tiltLastBytes[3], tiltLastBytes[4], tiltLastBytes[5]);
      sendOkRaw(tag, buf);
    } else {
      char buf[120];
      snprintf(buf, sizeof(buf),
               "005 no ack: got 0x%02X (want 0x2A) bytes %02X %02X %02X %02X %02X %02X"
               " | 0xFF=MISO floating 0x00=MISO low other=misaligned",
               tiltLastAck, tiltLastBytes[0], tiltLastBytes[1], tiltLastBytes[2],
               tiltLastBytes[3], tiltLastBytes[4], tiltLastBytes[5]);
      sendErr(tag, "EHW", buf);
    }
    return;
  }
#endif

  if (strcmp(verb, "STATUS") == 0) {
    static char response[256];

    if (!buildStatus(
            response,
            sizeof(response))) {
      sendErr(tag, "EOVERFLOW", "status");
      return;
    }

    sendOkRaw(tag, response);
    return;
  }

  if (strcmp(verb, "INFO") == 0) {
    static char response[1024];

    if (!buildInfo(
            response,
            sizeof(response))) {
      sendErr(tag, "EOVERFLOW", "info");
      return;
    }

    sendOkRaw(tag, response);
    return;
  }

  if (strcmp(verb, "READ") == 0) {
    pumpUuts();

    static char response[2048];

    if (!buildFrame(
            response,
            sizeof(response))) {
      sendErr(tag, "EOVERFLOW", "frame");
      return;
    }

    sendOkRaw(tag, response);
    return;
  }

  if (strcmp(verb, "DIAG") == 0) {
    static char response[2048];

    if (!buildDiagnostics(
            response,
            sizeof(response))) {
      sendErr(tag, "EOVERFLOW", "diagnostics");
      return;
    }

    sendOkRaw(tag, response);
    return;
  }

  if (strcmp(verb, "RECONFIG") == 0) {
    usbStreaming = false;

    const int configuredCount =
        reconfigureAllDuts();

    char response[96];

    snprintf(
        response,
        sizeof(response),
        "{\"configured\":%d,\"nunits\":%d}",
        configuredCount,
        N_UUT);

    sendOkRaw(tag, response);
    return;
  }

  if (strcmp(verb, "STREAM") == 0) {
    char* subcommand = strtok(nullptr, " ");

    if (subcommand == nullptr) {
      sendErr(tag, "EBADCMD", "stream");
      return;
    }

    if (strcmp(subcommand, "START") == 0) {
      char* periodText = strtok(nullptr, " ");

      uint32_t requestedPeriod =
          periodText != nullptr
              ? static_cast<uint32_t>(
                    strtoul(periodText, nullptr, 10))
              : 100;

      if (requestedPeriod < 20) {
        requestedPeriod = 20;
      }

      if (requestedPeriod > 60000) {
        requestedPeriod = 60000;
      }

      usbStreamPeriodMs = requestedPeriod;
      usbLastStreamMs = millis();
      usbStreaming = true;

      char response[64];

      snprintf(
          response,
          sizeof(response),
          "{\"streaming\":1,\"period_ms\":%lu}",
          static_cast<unsigned long>(
              usbStreamPeriodMs));

      sendOkRaw(tag, response);
      return;
    }

    if (strcmp(subcommand, "STOP") == 0) {
      usbStreaming = false;
      sendOkRaw(tag, "{\"streaming\":0}");
      return;
    }

    sendErr(tag, "EBADCMD", subcommand);
    return;
  }

  if (strcmp(verb, "LOG") == 0) {
    sendErr(tag, "ENOTSUP", "use laptop");
    return;
  }

  sendErr(tag, "EBADCMD", verb);
}

static void processLine(char* line) {
  if (line[0] != '#') {
    sendErr(0, "EBADCMD", "no tag");
    return;
  }

  char* separator = strchr(line, ' ');

  if (separator == nullptr) {
    sendErr(0, "EBADCMD", "no verb");
    return;
  }

  *separator = '\0';

  const long tag = strtol(
      line + 1,
      nullptr,
      10);

  handleCommand(
      tag,
      separator + 1);
}

static void readUsb() {
  while (Serial.available() > 0) {
    const char value =
        static_cast<char>(Serial.read());

    if (value == '\r') {
      continue;
    }

    if (value == '\n') {
      lineBuf[lineLen] = '\0';

      if (lineLen > 0) {
        processLine(lineBuf);
      }

      lineLen = 0;
      continue;
    }

    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = value;
    }
  }
}

static void serviceUsbStream() {
  if (!usbStreaming) {
    return;
  }

  const uint32_t nowMs = millis();

  if (static_cast<uint32_t>(
          nowMs - usbLastStreamMs) <
      usbStreamPeriodMs) {
    return;
  }

  usbLastStreamMs += usbStreamPeriodMs;

  if (static_cast<uint32_t>(
          nowMs - usbLastStreamMs) >
      usbStreamPeriodMs * 4) {
    usbLastStreamMs = nowMs;
  }

  static char frame[2048];

  if (!buildFrame(frame, sizeof(frame))) {
    sendEvt(
        "ERROR",
        "{\"code\":\"EOVERFLOW\"}");

    return;
  }

  sendEvt("DATA", frame);
}

void setup() {
  Serial.begin(USB_BAUD);

  const uint32_t usbWaitStart = millis();

  while (!Serial &&
         static_cast<uint32_t>(
             millis() - usbWaitStart) < 2000) {
    yield();
  }

  memset(uutBuf, 0, sizeof(uutBuf));
  memset(uutLen, 0, sizeof(uutLen));
  memset(uutAcc, 0, sizeof(uutAcc));
  memset(uutPR, 0, sizeof(uutPR));
  memset(uutConfigured, 0, sizeof(uutConfigured));
  memset(uutFresh, 0, sizeof(uutFresh));
  memset(uutModel, 0, sizeof(uutModel));
  memset(uutRevision, 0, sizeof(uutRevision));
  memset(uutSerialNumber, 0, sizeof(uutSerialNumber));
  memset(uutBytes, 0, sizeof(uutBytes));
  memset(uutPkts, 0, sizeof(uutPkts));
  memset(uutDataSeq, 0, sizeof(uutDataSeq));
  memset(uutCrcErr, 0, sizeof(uutCrcErr));
  memset(uutIncomplete, 0, sizeof(uutIncomplete));
  memset(uutOverflow, 0, sizeof(uutOverflow));
  memset(uutLastFid, 0, sizeof(uutLastFid));
  memset(uutLastPktMs, 0, sizeof(uutLastPktMs));
  memset(uutLastDataMs, 0, sizeof(uutLastDataMs));
  memset(uutLastDataUs, 0, sizeof(uutLastDataUs));

  for (int i = 0; i < N_UUT; ++i) {
    uart[i]->addMemoryForRead(
        uutExtraRx[i],
        sizeof(uutExtraRx[i]));

    uart[i]->begin(
        DUT_BAUD,
        SERIAL_8N1);
  }

#if ENABLE_ENCODER
  pinMode(ENCODER_CS_PIN, OUTPUT);
  digitalWrite(ENCODER_CS_PIN, HIGH);
  SPI.begin();
#endif

#if ENABLE_TILT
  pinMode(TILT_CS_PIN, OUTPUT);
  digitalWrite(TILT_CS_PIN, HIGH);   // slave select idles high
  SPI.begin();                        // harmless if the encoder already began it
  delay(10);
  tiltPrime();                        // start the reply pipeline (see tilt005.h)
#endif

#if ENABLE_ADC
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  pinMode(ADC_DRDY_PIN, INPUT);
  SPI1.begin();
#endif

  delay(250);

  const int configuredCount =
      reconfigureAllDuts();

  char readyMessage[96];

  snprintf(
      readyMessage,
      sizeof(readyMessage),
      "{\"configured\":%d,\"nunits\":%d}",
      configuredCount,
      N_UUT);

  sendEvt("READY", readyMessage);
}

void loop() {
  pumpUuts();
  readUsb();
  serviceUsbStream();
}
