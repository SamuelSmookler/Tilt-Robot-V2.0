// tilt005.h -- Fredericks 1-6200-005 SPI signal conditioner driver
//
// Paste this block into sensor_teensy.ino, or #include "tilt005.h" near the top.
// Requires <SPI.h> (already included when ENABLE_ENCODER or ENABLE_ADC is set --
// extend that #if to include ENABLE_TILT, see SETUP NOTES at the bottom).
//
// WIRING (005 J1 header -> Teensy 4.1)
//   J1-1 Supply +  -> 3.3V     ** NOT 5V: the Teensy 4.1 is not 5V tolerant **
//   J1-2 Supply -  -> GND
//   J1-3 Ground    -> GND
//   J1-4 SDO       -> MISO (pin 12)
//   J1-5 SDI       -> MOSI (pin 11)
//   J1-6 SCK       -> SCK  (pin 13)
//   J1-7 /SS       -> TILT_CS_PIN below
//
// BOARD CONFIG -- two single-axis 0719-3703-99 sensors:
//   J3 -> X channel, J4 -> Y channel
//   R5 = open (not installed), R6 = 1 kOhm.
//   If R5 is populated and R6 is not, the board is in DUAL-axis mode and
//   NEITHER single-axis channel reads correctly -- attaching a sensor to one
//   input visibly shifts the other, which is the tell.
//
// SENSOR RANGE: +/-0.5 deg only. These are precision NULL detectors for finding
// level, not general inclinometers. Measured noise on the bench matched the
// datasheet repeatability spec (0.0003 deg ~ 1 arcsecond).
//
// PROTOCOL NOTE -- the reply to command N arrives with command N+1. Every read
// therefore sends one extra dummy byte to flush the final value. Getting this
// wrong yields plausible-but-shifted data, which is worse than obvious garbage.
#ifndef TILT005_H
#define TILT005_H

#include <SPI.h>

// ---- configuration ---------------------------------------------------------
static constexpr int   TILT_CS_PIN         = 9;      // any free GPIO for /SS
static constexpr float TILT_FULL_SCALE_DEG = 0.5f;   // 0719-3703-99 = +/-0.5 deg
static constexpr float TILT_SUPPLY_V       = 3.3f;   // MUST match the actual supply
static constexpr uint32_t TILT_SPI_HZ      = 1000000; // datasheet: 500 kHz .. 1 MHz
static constexpr uint8_t  TILT_ACK         = 0x2A;    // '*' = data updated

// ---- one byte in, one byte out --------------------------------------------
// CS is NOT toggled here. Per Fredericks AN1006 the slave select stays LOW for
// the whole exchange -- the 005 keeps command/reply state internally and
// de-asserting CS between bytes resets it, which shows up as intermittent
// desync. read005() asserts CS once around the entire sequence.
static uint8_t tiltXfer(uint8_t cmd) {
  uint8_t reply = SPI.transfer(cmd);
  delay(1);                       // AN1006: 1 ms between transfers
  return reply;
}

// ---- read both axes + board temperature ------------------------------------
// Returns false if the board did not acknowledge (unpowered, miswired, wrong
// SPI mode). On false, the out-params are left untouched.
// Last raw bytes seen, for diagnostics when read005() fails.
// ack 0xFF -> MISO floating high (no power / not connected / wrong pin)
// ack 0x00 -> MISO stuck low (grounded, or board held in reset)
// ack other-> board IS talking but the bytes are misaligned (SPI mode / lag)
static uint8_t tiltLastAck = 0, tiltLastBytes[6] = {0};

static void tiltPrime() { /* nothing needed: each read is self-contained */ }

// Sequence per Fredericks AN1006. The slave answers command N on transfer N+1,
// so every byte read is the reply to the PREVIOUS command. Starts and ends with
// 0x39 ("update measurement"), which the app note calls essential.
static bool read005(float &xDeg, float &yDeg, float &tempC) {
  SPI.beginTransaction(SPISettings(TILT_SPI_HZ, MSBFIRST, SPI_MODE2));
  digitalWrite(TILT_CS_PIN, LOW);          // CS LOW for the ENTIRE exchange

  tiltXfer(0x39);                 // update sensor data
  uint8_t ack = tiltXfer(0x31);   // <- status ('*'), request X high
  uint8_t xhi = tiltXfer(0x32);   // <- X high, request X low
  uint8_t xlo = tiltXfer(0x33);   // <- X low,  request Y high
  uint8_t yhi = tiltXfer(0x34);   // <- Y high, request Y low
  uint8_t ylo = tiltXfer(0x35);   // <- Y low,  request temp high
  uint8_t thi = tiltXfer(0x36);   // <- temp high, request temp low
  uint8_t tlo = tiltXfer(0x39);   // <- temp low, and update again

  digitalWrite(TILT_CS_PIN, HIGH);
  SPI.endTransaction();

  tiltLastAck = ack;
  tiltLastBytes[0] = xhi; tiltLastBytes[1] = xlo; tiltLastBytes[2] = yhi;
  tiltLastBytes[3] = ylo; tiltLastBytes[4] = thi; tiltLastBytes[5] = tlo;

  if (ack != TILT_ACK) return false;

  const uint16_t xraw = ((uint16_t)xhi << 8) | xlo;
  const uint16_t yraw = ((uint16_t)yhi << 8) | ylo;
  const uint16_t traw = ((uint16_t)thi << 8) | tlo;   // 10-bit value, 0..1023

  // tilt is offset binary: 32768 == 0 deg. Cast to float BEFORE subtracting,
  // or the unsigned subtraction wraps for readings below level.
  xDeg = (((float)xraw - 32768.0f) / 32768.0f) * TILT_FULL_SCALE_DEG;
  yDeg = (((float)yraw - 32768.0f) / 32768.0f) * TILT_FULL_SCALE_DEG;

  // datasheet: T(C) = (((output / 1023) * Vsupply) - 0.5) / 0.01
  tempC = ((((float)traw / 1023.0f) * TILT_SUPPLY_V) - 0.5f) / 0.01f;
  return true;
}

// ---- convenience: temperature only (works with NO sensors attached) --------
// Use this as the first bring-up test. The temp sensor is on the 005 itself,
// so a sane room temperature (~20-25 C) proves wiring, SPI mode, CS handling
// and the one-byte lag are all correct before any tilt sensor is involved.
static bool read005TempOnly(float &tempC) {
  float x, y;
  return read005(x, y, tempC);
}

#endif // TILT005_H

// ============================ SETUP NOTES ==================================
//
// 1) Add the enable flag next to the others at the top of sensor_teensy.ino:
//
//        #define ENABLE_TILT 0     // Fredericks 1-6200-005 on SPI (CS = pin 9)
//
// 2) Extend the SPI include guard to cover it:
//
//        #if ENABLE_ENCODER || ENABLE_ADC || ENABLE_TILT
//        #include <SPI.h>
//        #endif
//
// 3) In setup(), before any read:
//
//        #if ENABLE_TILT
//          pinMode(TILT_CS_PIN, OUTPUT);
//          digitalWrite(TILT_CS_PIN, HIGH);   // idle high
//          SPI.begin();                        // harmless if already begun
//        #endif
//
// 4) In buildFrame(), replace the stubbed level fields:
//
//        float lx = 0.0f, ly = 0.0f, ltemp = 0.0f;
//        int   lvalid = 0;
//        #if ENABLE_TILT
//          lvalid = read005(lx, ly, ltemp) ? 1 : 0;
//        #endif
//
//    then emit lx / ly in "level", ltemp in the temperature slot, and lvalid
//    as a *_valid flag so the laptop can tell "reads zero" from "not wired".
//
// 5) SENSOR NOTES (0719-3703-99):
//    - Operating range is only +/-0.5 deg. It is a precision NULL detector for
//      finding level, not a general inclinometer. It saturates almost at once.
//    - Time constant <= 500 ms, so allow 1.5-2.5 s to settle after any motion
//      before trusting a reading. This dominates closed-loop homing timing.
//    - Mount horizontally, isolated from vibration.
//    - Sensor operating range is -20..50 C, narrower than the 005's -40..85 C.
