#include <Arduino.h>

#include "motor_calibration_console.h"
#include "runtime_control_config.h"

static constexpr uint8_t MOTOR_STEP_PIN = 4;
static constexpr uint8_t MOTOR_DIR_PIN = 5;
static constexpr uint8_t MOTOR_EN_PIN = 6;
static constexpr uint8_t MOTOR2_STEP_PIN = 7;
static constexpr uint8_t MOTOR2_DIR_PIN = 15;
static constexpr uint8_t MOTOR2_EN_PIN = 16;
static constexpr uint8_t HX711_DOUT_PIN = 17;
static constexpr uint8_t HX711_SCK_PIN = 18;

MotorCalibrationConsole g_motorConsole;

void setup() {
  MotorCalibrationConsole::StartupConfig startupConfig;
  startupConfig.autoEnableMotor1 = RuntimeControlConfig::kAutoEnableMotor1;
  startupConfig.startupMotor1EnableSettleMs =
      RuntimeControlConfig::kStartupMotor1EnableSettleMs;
  startupConfig.manualOpenLoopOnly =
      RuntimeControlConfig::kManualOpenLoopOnly;
  startupConfig.autoStartClosedLoopMotor1 =
      RuntimeControlConfig::kAutoStartClosedLoopMotor1;
  startupConfig.motor1ClosedLoopLowGrams =
      RuntimeControlConfig::kMotor1ClosedLoopLowGrams;
  startupConfig.motor1ClosedLoopHighGrams =
      RuntimeControlConfig::kMotor1ClosedLoopHighGrams;
  startupConfig.reverseClosedLoopMotion =
      RuntimeControlConfig::kClosedLoopReverseMotion;
  startupConfig.closedLoopMeasurementSamples =
      RuntimeControlConfig::kClosedLoopMeasurementSamples;
  startupConfig.closedLoopFilterPasses =
      RuntimeControlConfig::kClosedLoopFilterPasses;
  startupConfig.closedLoopAdjustStepsPerCycle =
      RuntimeControlConfig::kClosedLoopAdjustStepsPerCycle;
  startupConfig.closedLoopMinAdjustStepsPerCycle =
      RuntimeControlConfig::kClosedLoopMinAdjustStepsPerCycle;
  startupConfig.closedLoopMaxStepChangePerCycle =
      RuntimeControlConfig::kClosedLoopMaxStepChangePerCycle;
  startupConfig.closedLoopIntervalMs =
      RuntimeControlConfig::kClosedLoopIntervalMs;
  startupConfig.closedLoopStepPulseUs =
      RuntimeControlConfig::kClosedLoopStepPulseUs;
  startupConfig.closedLoopStepSettleUs =
      RuntimeControlConfig::kClosedLoopStepSettleUs;
  startupConfig.closedLoopKp = RuntimeControlConfig::kClosedLoopKp;
  startupConfig.closedLoopKi = RuntimeControlConfig::kClosedLoopKi;
  startupConfig.closedLoopKd = RuntimeControlConfig::kClosedLoopKd;
  startupConfig.closedLoopMaxIntegral =
      RuntimeControlConfig::kClosedLoopMaxIntegral;

  g_motorConsole.begin(MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_EN_PIN,
                       MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, MOTOR2_EN_PIN,
                       HX711_DOUT_PIN, HX711_SCK_PIN, true, startupConfig);
}

void loop() { g_motorConsole.update(); }
