//########################################
// 02.03.26 - Basic AS5600 reader
// Measures the angle and just prints the RPM to Serial
// Commented by ChatGPT, 02.03.26
//########################################
#include <Arduino.h>
#include <Wire.h>

uint8_t address = 0x36; // I2C address of AS5600 magnetic encoder
uint8_t angleAdress = 0x0C; // Register address for angle MSB
uint32_t sampleRate = 500; // Sampling frequency in Hz
uint32_t samplePeriodUS = 1000000UL / sampleRate; // Sampling period in microseconds


static inline int16_t wrapDelta(uint16_t current, uint16_t prev) // Calculates the wrapped delta between two 12-bit angle values
{
  int16_t delta = (int16_t)current - (int16_t)prev;
  if (delta >  2048) delta -= 4096; 
  if (delta < -2048) delta += 4096;
  return delta;
}

bool readRawAngle(uint16_t &raw) // Reads the raw 12-bit angle value from AS5600 via I2C, Returns true if read successful, false otherwise
{
  Wire.beginTransmission(address);
  Wire.write(angleAdress);
  if (Wire.endTransmission(true) != 0) return false;

  delayMicroseconds(5);

  if (Wire.requestFrom(address, 2, true) != 2) return false; // Request 2 bytes (MSB and LSB)

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  raw = ((msb & 0x0F) << 8) | lsb; // Combine MSB (lower 4 bits) and LSB into 12-bit value
  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Wire.begin();
  Wire.setClock(100000);
}

void loop()
{
  uint32_t next_sample_us = micros();
  bool have_last = false;
  uint16_t last_raw = 0;

  uint32_t now_us = micros();
  if ((now_us - next_sample_us) >= 0) { // Check if it's time to take a sample
    next_sample_us += samplePeriodUS;

    uint16_t raw;
    if (readRawAngle(raw)) { // First reading: just store the value
      if (!have_last) {
        have_last = true;
        last_raw = raw;
      } else {
        int16_t delta_counts = wrapDelta(raw, last_raw); // Calculate angular delta and convert to RPM
        last_raw = raw;

        float dt_s = samplePeriodUS * 1e-6;
        float rps = fabsf((delta_counts / 4096.0f) / dt_s); // Convert counts to revolutions per second, then to RPM
        float rpm = rps * 60;

        Serial.println(rpm);
      }
    }
  }
}