#pragma once

#include <Arduino.h>

class MotorCalibrationConsole {
 public:
  struct StartupConfig {
    bool autoEnableMotor1 = false;
    bool runStartupMotor1SelfTest = false;
    long startupMotor1SelfTestSteps = 0;
    unsigned long startupMotor1EnableSettleMs = 0;
    uint16_t startupMotor1SelfTestPulseUs = 400;
    bool runStartupMotor1QueueTest = false;
    unsigned long startupMotor1QueueTestDelayMs = 0;
    long startupMotor1QueueTestSteps = 0;
    uint16_t startupMotor1QueueTestPulseUs = 400;
    bool manualOpenLoopOnly = false;
    bool autoStartClosedLoopMotor1 = false;
    float motor1ClosedLoopLowGrams = 0.0f;
    float motor1ClosedLoopHighGrams = 0.0f;
    bool reverseClosedLoopMotion = false;
    uint8_t closedLoopMeasurementSamples = 8;
    uint8_t closedLoopFilterPasses = 3;
    long closedLoopAdjustStepsPerCycle = 20;
    long closedLoopMinAdjustStepsPerCycle = 1;
    unsigned long closedLoopIntervalMs = 150;
    uint16_t closedLoopStepPulseUs = 400;
    uint16_t closedLoopStepSettleUs = 2000;
    float closedLoopKp = 0.0f;
    float closedLoopKi = 0.0f;
    float closedLoopKd = 0.0f;
    float closedLoopMaxIntegral = 100.0f;
    long closedLoopMaxStepChangePerCycle = 10;
  };

  void begin(uint8_t stepPin1, uint8_t dirPin1, uint8_t enPin1,
             uint8_t stepPin2, uint8_t dirPin2, uint8_t enPin2,
             uint8_t hx711DoutPin, uint8_t hx711SckPin,
             bool enableActiveLow = true);
  void begin(uint8_t stepPin1, uint8_t dirPin1, uint8_t enPin1,
             uint8_t stepPin2, uint8_t dirPin2, uint8_t enPin2,
             uint8_t hx711DoutPin, uint8_t hx711SckPin,
             bool enableActiveLow, const StartupConfig& startupConfig);
  void update();

 private:
  struct PressureConfig {
    uint8_t doutPin = 0;
    uint8_t sckPin = 0;
    float calibrationFactor = 1.0f;
    bool calibrated = false;
    long zeroOffset = 0;
  };

  struct ClosedLoopConfig {
    bool enabled = false;
    bool commandInterfaceEnabled = true;
    size_t axis = 0;
    bool pressureSensorAssigned = false;
    bool reverseMotionEnabled = false;
    float lowThresholdGrams = 0.0f;
    float highThresholdGrams = 0.0f;
    uint8_t measurementSamples = 8;
    uint8_t filterPasses = 3;
    long adjustStepsPerCycle = 20;
    long minAdjustStepsPerCycle = 1;
    unsigned long controlIntervalMs = 150;
    uint16_t stepPulseUs = 400;
    uint16_t stepSettleUs = 2000;
    unsigned long lastControlMs = 0;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float maxIntegral = 100.0f;
    long maxStepChangePerCycle = 10;
    float integralError = 0.0f;
    float lastError = 0.0f;
    bool lastErrorValid = false;
    long lastCorrectionSteps = 0;
    float lastMeasuredGrams = 0.0f;
    bool lastMeasurementValid = false;
  };

  struct PressureMonitorConfig {
    bool enabled = false;
    unsigned long intervalMs = 500;
    unsigned long lastPrintMs = 0;
    uint8_t samples = 5;
  };

  struct AxisConfig {
    uint8_t stepPin = 0;
    uint8_t dirPin = 0;
    uint8_t enPin = 0;
    bool enableActiveLow = true;
    bool enabled = false;
    bool dirForwardHigh = true;
    uint16_t stepPulseUs = 400;
    uint16_t stepSettleMs = 0;
    long currentPositionSteps = 0;
    long softLimitMinSteps = -200000;
    long softLimitMaxSteps = 200000;
    long queuedSteps = 0;
    bool queuedForward = true;
    bool stepPinHigh = false;
    unsigned long nextStepMicros = 0;
    uint16_t activePulseUs = 400;
  };

  void printHelp();
  void printStatus(size_t axis) const;
  void printAllStatus() const;
  void printPressureStatus();
  void printClosedLoopStatus();
  void printRoutingStatus();
  void handleCommand(String line);
  void serviceSerialInput();
  bool parseAxisToken(const String& token, size_t& axis) const;
  bool parseMotionDirectionToken(const String& token, bool& forward) const;
  void setEnabled(size_t axis, bool enabled);
  void setDirection(size_t axis, bool forward);
  bool queueMotorMove(size_t axis, long steps, uint16_t pulseUs, bool verbose = true);
  void serviceMotorMotion();
  bool isMotorBusy(size_t axis) const;
  bool isWithinSoftLimit(size_t axis, long targetPosition) const;
  void tareScale(uint8_t samples = 20);
  void calibrateWithKnownWeight(float knownGrams, uint8_t samples = 30);
  void loadPressureCalibration();
  void savePressureCalibration();
  bool readPressureGrams(float& grams, uint8_t samples);
  bool readFilteredPressureGrams(float& grams);
  void servicePressureMonitor();
  void serviceClosedLoop();
  void disableClosedLoop(const __FlashStringHelper* reason);
  void assignPressureSensorToAxis(size_t axis);
  void applyStartupConfig(const StartupConfig& startupConfig);
  bool runBlockingStartupSelfTest(size_t axis, long steps, uint16_t pulseUs);
  bool runBlockingMotorSteps(size_t axis, long steps, uint16_t pulseUs,
                             uint16_t settleUs);

  AxisConfig axes_[2];
  PressureConfig pressure_;
  ClosedLoopConfig closedLoop_;
  PressureMonitorConfig pressureMonitor_;
  String serialLineBuffer_;
};
