#pragma once

namespace RuntimeControlConfig {

// Manual open-loop build: humans control driver enable and motion by serial.
// Closed-loop/PID source code remains available, but this runtime does not use it.
constexpr bool kManualOpenLoopOnly = true;

// Boot-time behavior.
constexpr bool kAutoEnableMotor1 = false;
constexpr unsigned long kStartupMotor1EnableSettleMs = 800;
// Unused startup diagnostics. Keep archived here until the startup self-test
// wiring is removed from main.cpp / MotorCalibrationConsole.
// constexpr bool kRunStartupMotor1SelfTest = false;
// constexpr long kStartupMotor1SelfTestSteps = 500;
// constexpr unsigned int kStartupMotor1SelfTestPulseUs = 1200;
// constexpr bool kRunStartupMotor1QueueTest = false;
// constexpr unsigned long kStartupMotor1QueueTestDelayMs = 2000;
// constexpr long kStartupMotor1QueueTestSteps = 500;
// constexpr unsigned int kStartupMotor1QueueTestPulseUs = 1200;
constexpr bool kAutoStartClosedLoopMotor1 = false;

// Target pressure window in grams for motor1.
constexpr float kMotor1ClosedLoopLowGrams = 995.0f;
constexpr float kMotor1ClosedLoopHighGrams = 1005.0f;

// Closed-loop tuning.
constexpr bool kClosedLoopReverseMotion = false;
constexpr unsigned char kClosedLoopMeasurementSamples = 1;
constexpr unsigned char kClosedLoopFilterPasses = 1;
constexpr long kClosedLoopAdjustStepsPerCycle = 24;
constexpr long kClosedLoopMinAdjustStepsPerCycle = 1;
constexpr long kClosedLoopMaxStepChangePerCycle = 6;
constexpr unsigned long kClosedLoopIntervalMs = 5;
constexpr unsigned int kClosedLoopStepPulseUs = 100;
constexpr unsigned int kClosedLoopStepSettleUs = 100;

// PID output is in motor steps. Error is target pressure minus measured pressure.
// Start mostly with P; add small I/D only after the direction and Kp feel right.
constexpr float kClosedLoopKp = 0.01f;   // steps per gram
constexpr float kClosedLoopKi = 0.0f;   // steps per gram-second
constexpr float kClosedLoopKd = 0.0f;   // steps per gram/second
constexpr float kClosedLoopMaxIntegral = 80.0f;

}  // namespace RuntimeControlConfig
