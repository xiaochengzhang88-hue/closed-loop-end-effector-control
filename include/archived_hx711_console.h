#pragma once

#include <Arduino.h>

class Hx711Console {
 public:
  void begin(uint8_t doutPin, uint8_t sckPin);
  void update();
  float getCalibrationFactor() const;
  long getZeroOffset() const;
  bool isCalibrated() const;

 private:
  void printHelp();
  void tareScale(uint8_t samples = 20);
  void calibrateWithKnownWeight(float knownGrams, uint8_t samples = 30);
  void printRaw(uint8_t samples = 10);
  void printWeight(uint8_t samples = 10);
  void printStatus();
  void handleCommand(String line);
  void loadFromNvs();
  void saveToNvs();

  uint8_t doutPin_ = 0;
  uint8_t sckPin_ = 0;
  float calibrationFactor_ = 1.0f;
  bool calibrated_ = false;
  long zeroOffset_ = 0;
};
