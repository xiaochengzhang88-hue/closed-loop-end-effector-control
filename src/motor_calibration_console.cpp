#include "motor_calibration_console.h"

#include <HX711.h>
#include <Preferences.h>

#include <cmath>

namespace {
constexpr unsigned long kSerialBaud = 115200;
constexpr size_t kAxisCount = 2;
constexpr uint16_t kDefaultManualRunPulseUs = 800;

constexpr const char* kNvsNamespace = "hx711cfg";
constexpr const char* kKeyFactor = "factor";
constexpr const char* kKeyOffset = "offset";
constexpr const char* kKeyReady = "ready";

HX711 g_scale;
Preferences g_prefs;
}  // namespace

void MotorCalibrationConsole::begin(uint8_t stepPin1, uint8_t dirPin1,
                                    uint8_t enPin1, uint8_t stepPin2,
                                    uint8_t dirPin2, uint8_t enPin2,
                                    uint8_t hx711DoutPin,
                                    uint8_t hx711SckPin,
                                    bool enableActiveLow) {
  begin(stepPin1, dirPin1, enPin1, stepPin2, dirPin2, enPin2, hx711DoutPin,
        hx711SckPin, enableActiveLow, StartupConfig{});
}

void MotorCalibrationConsole::begin(uint8_t stepPin1, uint8_t dirPin1,
                                    uint8_t enPin1, uint8_t stepPin2,
                                    uint8_t dirPin2, uint8_t enPin2,
                                    uint8_t hx711DoutPin,
                                    uint8_t hx711SckPin,
                                    bool enableActiveLow,
                                    const StartupConfig& startupConfig) {
  Serial.begin(kSerialBaud);
  delay(300);

  axes_[0].stepPin = stepPin1;
  axes_[0].dirPin = dirPin1;
  axes_[0].enPin = enPin1;
  axes_[0].enableActiveLow = enableActiveLow;

  axes_[1].stepPin = stepPin2;
  axes_[1].dirPin = dirPin2;
  axes_[1].enPin = enPin2;
  axes_[1].enableActiveLow = enableActiveLow;

  for (size_t axis = 0; axis < kAxisCount; ++axis) {
    pinMode(axes_[axis].stepPin, OUTPUT);
    pinMode(axes_[axis].dirPin, OUTPUT);
    pinMode(axes_[axis].enPin, OUTPUT);

    digitalWrite(axes_[axis].stepPin, LOW);
    digitalWrite(axes_[axis].dirPin, LOW);
    setEnabled(axis, false);
  }

  pressure_.doutPin = hx711DoutPin;
  pressure_.sckPin = hx711SckPin;
  assignPressureSensorToAxis(0);
  g_scale.begin(pressure_.doutPin, pressure_.sckPin);

  Serial.println();
  Serial.println("Motor + pressure closed-loop console");
  Serial.print("M1 pins: STEP=GPIO");
  Serial.print(axes_[0].stepPin);
  Serial.print(", DIR=GPIO");
  Serial.print(axes_[0].dirPin);
  Serial.print(", EN=GPIO");
  Serial.println(axes_[0].enPin);
  Serial.print("M2 pins: STEP=GPIO");
  Serial.print(axes_[1].stepPin);
  Serial.print(", DIR=GPIO");
  Serial.print(axes_[1].dirPin);
  Serial.print(", EN=GPIO");
  Serial.println(axes_[1].enPin);
  Serial.print("HX711 pins: DT=GPIO");
  Serial.print(pressure_.doutPin);
  Serial.print(", SCK=GPIO");
  Serial.println(pressure_.sckPin);

  if (!g_scale.wait_ready_timeout(5000)) {
    Serial.println("HX711 not found. Check wiring and power.");
  } else {
    loadPressureCalibration();
    if (!pressure_.calibrated) {
      Serial.println("No saved pressure calibration. Running initial tare...");
      tareScale();
    }
  }

  applyStartupConfig(startupConfig);

  printHelp();
  printAllStatus();
  Serial.println();
  printPressureStatus();
  if (closedLoop_.commandInterfaceEnabled) {
    printClosedLoopStatus();
  }
  printRoutingStatus();
}

void MotorCalibrationConsole::update() {
  serviceMotorMotion();
  serviceClosedLoop();
  servicePressureMonitor();
  serviceSerialInput();
}

void MotorCalibrationConsole::printHelp() {
  Serial.println();
  Serial.println("Motor commands:");
  Serial.println("  mh                          : show this help");
  Serial.println("  ms [m1|m2]                  : print motor status");
  Serial.println("  me <m1|m2> <0|1>            : disable/enable one driver");
  Serial.println("  mdir <m1|m2> <0|1>          : set logical direction");
  Serial.println("  mj <m1|m2> <steps> [pulse]  : jog, default pulse=800us");
  Serial.println("  run <f|r> <steps> [pulse]   : run motor1, default pulse=800us");
  Serial.println("  mrun <m1|m2> <f|r> <steps> [pulse] : run, default pulse=800us");
  Serial.println("  mlimit <m1|m2> <min> <max>  : set soft limits");
  Serial.println();
  Serial.println("Pressure commands:");
  Serial.println("  ph                          : print pressure help/status");
  Serial.println("  pt                          : tare pressure sensor and save");
  Serial.println("  pc <known_grams>            : calibrate pressure sensor and save");
  Serial.println("  pr [samples]                : read pressure in grams");
  Serial.println("  prmon <0|1> [interval] [samples] : stop/start pressure monitor");
  Serial.println("  pp                          : print pressure calibration");
  if (closedLoop_.commandInterfaceEnabled) {
    Serial.println();
    Serial.println("Closed-loop commands:");
    Serial.println("  clmotor <m1>                : assign current pressure sensor to motor1");
    Serial.println("  clrange <low> <high>        : set pressure band in grams");
    Serial.println("  cl <0|1>                    : disable/enable closed-loop mode");
    Serial.println("  clreverse <0|1>             : reverse all closed-loop motor motion");
    Serial.println("  cls                         : print closed-loop status");
  } else {
    Serial.println();
    Serial.println("Closed-loop commands are disabled in this manual open-loop build.");
  }
  Serial.println();
}

void MotorCalibrationConsole::printStatus(size_t axis) const {
  Serial.print("Motor ");
  Serial.println(axis + 1);
  Serial.print("Driver enabled: ");
  Serial.println(axes_[axis].enabled ? "yes" : "no");
  Serial.print("Direction forward high: ");
  Serial.println(axes_[axis].dirForwardHigh ? "yes" : "no");
  Serial.print("Physical position (steps): ");
  Serial.println(axes_[axis].currentPositionSteps);
  Serial.print("Soft limits (steps): ");
  Serial.print(axes_[axis].softLimitMinSteps);
  Serial.print(" .. ");
  Serial.println(axes_[axis].softLimitMaxSteps);
  Serial.print("Default pulse width (us): ");
  Serial.println(axes_[axis].stepPulseUs);
  Serial.print("Busy: ");
  Serial.println(isMotorBusy(axis) ? "yes" : "no");
  Serial.print("Queued steps remaining: ");
  Serial.println(axes_[axis].queuedSteps);
}

void MotorCalibrationConsole::printAllStatus() const {
  for (size_t axis = 0; axis < kAxisCount; ++axis) {
    if (axis > 0) {
      Serial.println();
    }
    printStatus(axis);
  }
}

void MotorCalibrationConsole::printPressureStatus() {
  Serial.println("Pressure sensor");
  Serial.print("Calibrated: ");
  Serial.println(pressure_.calibrated ? "yes" : "no");
  Serial.print("Calibration factor: ");
  Serial.println(pressure_.calibrationFactor, 6);
  Serial.print("Zero offset: ");
  Serial.println(pressure_.zeroOffset);

  float grams = 0.0f;
  if (readPressureGrams(grams, closedLoop_.measurementSamples)) {
    Serial.print("Current pressure: ");
    Serial.print(grams, 3);
    Serial.println(" g");
  } else {
    Serial.println("Current pressure: unavailable");
  }
}

void MotorCalibrationConsole::printClosedLoopStatus() {
  Serial.println("Closed-loop");
  Serial.print("Enabled: ");
  Serial.println(closedLoop_.enabled ? "yes" : "no");
  Serial.print("Controlled motor: m");
  Serial.println(closedLoop_.axis + 1);
  Serial.print("Pressure sensor assigned: ");
  Serial.println(closedLoop_.pressureSensorAssigned ? "yes" : "no");
  Serial.print("Closed-loop motion reversed: ");
  Serial.println(closedLoop_.reverseMotionEnabled ? "yes" : "no");
  Serial.print("Target range (g): ");
  Serial.print(closedLoop_.lowThresholdGrams, 3);
  Serial.print(" .. ");
  Serial.println(closedLoop_.highThresholdGrams, 3);
  Serial.print("Filtered samples: ");
  Serial.println(closedLoop_.measurementSamples);
  Serial.print("PID Kp/Ki/Kd: ");
  Serial.print(closedLoop_.kp, 4);
  Serial.print(" / ");
  Serial.print(closedLoop_.ki, 4);
  Serial.print(" / ");
  Serial.println(closedLoop_.kd, 4);
  Serial.print("Max correction steps/cycle: ");
  Serial.println(closedLoop_.adjustStepsPerCycle);
  Serial.print("Min correction steps/cycle: ");
  Serial.println(closedLoop_.minAdjustStepsPerCycle);
  Serial.print("Max step change/cycle: ");
  Serial.println(closedLoop_.maxStepChangePerCycle);
  if (closedLoop_.lastMeasurementValid) {
    Serial.print("Last measured pressure: ");
    Serial.print(closedLoop_.lastMeasuredGrams, 3);
    Serial.println(" g");
  } else {
    Serial.println("Last measured pressure: unavailable");
  }
}

void MotorCalibrationConsole::printRoutingStatus() {
  Serial.println("Routing");
  if (closedLoop_.commandInterfaceEnabled) {
    Serial.println("Pressure sensor 1 -> motor1 closed-loop");
  } else {
    Serial.println("Manual open-loop control only");
  }
  Serial.println("Motor2 -> manual control only (no pressure feedback)");
}

void MotorCalibrationConsole::handleCommand(String line) {
  line.trim();
  if (line.isEmpty()) {
    return;
  }

  const int split = line.indexOf(' ');
  const String cmd = split > 0 ? line.substring(0, split) : line;
  const String args = split > 0 ? line.substring(split + 1) : "";

  if (cmd == "mh") {
    printHelp();
    return;
  }

  if (cmd == "ms") {
    if (!args.length()) {
      printAllStatus();
      return;
    }

    size_t axis = 0;
    if (!parseAxisToken(args, axis)) {
      Serial.println("Usage: ms [m1|m2]");
      return;
    }
    printStatus(axis);
    return;
  }

  if (cmd == "me") {
    const int firstSplit = args.indexOf(' ');
    if (firstSplit <= 0) {
      Serial.println("Usage: me <m1|m2> <0|1>");
      return;
    }
    size_t axis = 0;
    if (!parseAxisToken(args.substring(0, firstSplit), axis)) {
      Serial.println("Axis must be m1 or m2.");
      return;
    }
    setEnabled(axis, args.substring(firstSplit + 1).toInt() != 0);
    printStatus(axis);
    return;
  }

  if (cmd == "mdir") {
    const int firstSplit = args.indexOf(' ');
    if (firstSplit <= 0) {
      Serial.println("Usage: mdir <m1|m2> <0|1>");
      return;
    }
    size_t axis = 0;
    if (!parseAxisToken(args.substring(0, firstSplit), axis)) {
      Serial.println("Axis must be m1 or m2.");
      return;
    }
    axes_[axis].dirForwardHigh = (args.substring(firstSplit + 1).toInt() != 0);
    if (!isMotorBusy(axis)) {
      setDirection(axis, true);
    }
    printStatus(axis);
    return;
  }

  if (cmd == "mj") {
    const int firstSplit = args.indexOf(' ');
    if (firstSplit <= 0) {
      Serial.println("Usage: mj <m1|m2> <steps> [pulse_us]");
      return;
    }

    size_t axis = 0;
    if (!parseAxisToken(args.substring(0, firstSplit), axis)) {
      Serial.println("Axis must be m1 or m2.");
      return;
    }

    const String remaining = args.substring(firstSplit + 1);
    const int pulseSplit = remaining.indexOf(' ');
    long steps = 0;
    uint16_t pulseUs = kDefaultManualRunPulseUs;
    if (pulseSplit > 0) {
      steps = remaining.substring(0, pulseSplit).toInt();
      pulseUs = (uint16_t)constrain(remaining.substring(pulseSplit + 1).toInt(),
                                    50, 5000);
    } else {
      steps = remaining.toInt();
    }

    if (steps == 0) {
      Serial.println("Usage: mj <m1|m2> <steps> [pulse_us]");
      return;
    }

    if (!queueMotorMove(axis, steps, pulseUs)) {
      Serial.println("Jog was not queued.");
    }
    printStatus(axis);
    return;
  }

  if (cmd == "mrun") {
    const int firstSplit = args.indexOf(' ');
    if (firstSplit <= 0) {
      Serial.println("Usage: mrun <m1|m2> <f|r> <steps> [pulse_us]");
      return;
    }

    size_t axis = 0;
    if (!parseAxisToken(args.substring(0, firstSplit), axis)) {
      Serial.println("Axis must be m1 or m2.");
      return;
    }

    const String remaining = args.substring(firstSplit + 1);
    const int secondSplit = remaining.indexOf(' ');
    if (secondSplit <= 0) {
      Serial.println("Usage: mrun <m1|m2> <f|r> <steps> [pulse_us]");
      return;
    }

    bool forward = true;
    if (!parseMotionDirectionToken(remaining.substring(0, secondSplit), forward)) {
      Serial.println("Direction must be f/r or forward/reverse.");
      return;
    }

    const String stepPart = remaining.substring(secondSplit + 1);
    const int thirdSplit = stepPart.indexOf(' ');
    long steps = 0;
    uint16_t pulseUs = kDefaultManualRunPulseUs;
    if (thirdSplit > 0) {
      steps = labs(stepPart.substring(0, thirdSplit).toInt());
      pulseUs =
          (uint16_t)constrain(stepPart.substring(thirdSplit + 1).toInt(), 50, 5000);
    } else {
      steps = labs(stepPart.toInt());
    }

    if (steps == 0) {
      Serial.println("Usage: mrun <m1|m2> <f|r> <steps> [pulse_us]");
      return;
    }

    const long signedSteps = forward ? steps : -steps;
    if (!queueMotorMove(axis, signedSteps, pulseUs)) {
      Serial.println("Run was not queued.");
    }
    printStatus(axis);
    return;
  }

  if (cmd == "run") {
    const int firstSplit = args.indexOf(' ');
    if (firstSplit <= 0) {
      Serial.println("Usage: run <f|r> <steps> [pulse_us]");
      return;
    }

    bool forward = true;
    if (!parseMotionDirectionToken(args.substring(0, firstSplit), forward)) {
      Serial.println("Direction must be f/r or forward/reverse.");
      return;
    }

    const String stepPart = args.substring(firstSplit + 1);
    const int pulseSplit = stepPart.indexOf(' ');
    long steps = 0;
    uint16_t pulseUs = kDefaultManualRunPulseUs;
    if (pulseSplit > 0) {
      steps = labs(stepPart.substring(0, pulseSplit).toInt());
      pulseUs =
          (uint16_t)constrain(stepPart.substring(pulseSplit + 1).toInt(), 50, 5000);
    } else {
      steps = labs(stepPart.toInt());
    }

    if (steps == 0) {
      Serial.println("Usage: run <f|r> <steps> [pulse_us]");
      return;
    }

    const long signedSteps = forward ? steps : -steps;
    if (!queueMotorMove(0, signedSteps, pulseUs)) {
      Serial.println("Run was not queued.");
    }
    printStatus(0);
    return;
  }

  if (cmd == "mlimit") {
    const int firstSplit = args.indexOf(' ');
    if (firstSplit <= 0) {
      Serial.println("Usage: mlimit <m1|m2> <min> <max>");
      return;
    }

    size_t axis = 0;
    if (!parseAxisToken(args.substring(0, firstSplit), axis)) {
      Serial.println("Axis must be m1 or m2.");
      return;
    }

    const String remaining = args.substring(firstSplit + 1);
    const int secondSplit = remaining.indexOf(' ');
    if (secondSplit <= 0) {
      Serial.println("Usage: mlimit <m1|m2> <min> <max>");
      return;
    }

    const long minSteps = remaining.substring(0, secondSplit).toInt();
    const long maxSteps = remaining.substring(secondSplit + 1).toInt();
    if (maxSteps <= minSteps) {
      Serial.println("Soft limit max must be greater than min.");
      return;
    }

    axes_[axis].softLimitMinSteps = minSteps;
    axes_[axis].softLimitMaxSteps = maxSteps;
    Serial.println("Soft limits updated.");
    printStatus(axis);
    return;
  }

  if (cmd == "ph") {
    printPressureStatus();
    if (closedLoop_.commandInterfaceEnabled) {
      printClosedLoopStatus();
    }
    printRoutingStatus();
    return;
  }

  if (cmd == "pt") {
    tareScale();
    return;
  }

  if (cmd == "pc") {
    calibrateWithKnownWeight(args.toFloat());
    return;
  }

  if (cmd == "pr") {
    const uint8_t samples =
        args.length() ? (uint8_t)constrain(args.toInt(), 1, 50) : 10;
    float grams = 0.0f;
    if (!readPressureGrams(grams, samples)) {
      Serial.println("Pressure read failed.");
      return;
    }
    Serial.print("Pressure: ");
    Serial.print(grams, 3);
    Serial.println(" g");
    return;
  }

  if (cmd == "pp") {
    printPressureStatus();
    return;
  }

  if (cmd == "prmon") {
    if (!args.length()) {
      Serial.println("Usage: prmon <0|1> [interval_ms] [samples]");
      return;
    }

    const int firstSplit = args.indexOf(' ');
    const String enablePart = firstSplit > 0 ? args.substring(0, firstSplit) : args;
    if (enablePart.toInt() == 0) {
      pressureMonitor_.enabled = false;
      Serial.println("Pressure monitor stopped.");
      return;
    }

    pressureMonitor_.enabled = true;
    pressureMonitor_.lastPrintMs = 0;
    pressureMonitor_.intervalMs = 500;
    pressureMonitor_.samples = 5;

    if (firstSplit > 0) {
      const String remaining = args.substring(firstSplit + 1);
      const int secondSplit = remaining.indexOf(' ');
      if (secondSplit > 0) {
        pressureMonitor_.intervalMs = (unsigned long)constrain(
            remaining.substring(0, secondSplit).toInt(), 100, 10000);
        pressureMonitor_.samples = (uint8_t)constrain(
            remaining.substring(secondSplit + 1).toInt(), 1, 50);
      } else {
        pressureMonitor_.intervalMs =
            (unsigned long)constrain(remaining.toInt(), 100, 10000);
      }
    }

    Serial.print("Pressure monitor started. Interval(ms)=");
    Serial.print(pressureMonitor_.intervalMs);
    Serial.print(", samples=");
    Serial.println(pressureMonitor_.samples);
    return;
  }

  const bool isClosedLoopCommand =
      cmd == "clmotor" || cmd == "clrange" || cmd == "cl" ||
      cmd == "clreverse" || cmd == "cls";
  if (isClosedLoopCommand && !closedLoop_.commandInterfaceEnabled) {
    Serial.println("Closed-loop commands are disabled in this manual open-loop build.");
    return;
  }

  if (cmd == "clmotor") {
    size_t axis = 0;
    if (!parseAxisToken(args, axis)) {
      Serial.println("Usage: clmotor <m1>");
      return;
    }
    if (axis != 0) {
      Serial.println("Only motor1 supports pressure closed-loop right now.");
      return;
    }
    assignPressureSensorToAxis(axis);
    printClosedLoopStatus();
    printRoutingStatus();
    return;
  }

  if (cmd == "clrange") {
    const int firstSplit = args.indexOf(' ');
    if (firstSplit <= 0) {
      Serial.println("Usage: clrange <low> <high>");
      return;
    }
    const float firstValue = args.substring(0, firstSplit).toFloat();
    const float secondValue = args.substring(firstSplit + 1).toFloat();
    if (firstValue == secondValue) {
      Serial.println("Pressure range endpoints must be different.");
      return;
    }
    closedLoop_.lowThresholdGrams = min(firstValue, secondValue);
    closedLoop_.highThresholdGrams = max(firstValue, secondValue);
    if (firstValue > secondValue) {
      Serial.println("Pressure range order auto-corrected.");
    }
    printClosedLoopStatus();
    return;
  }

  if (cmd == "cl") {
    if (!args.length()) {
      Serial.println("Usage: cl <0|1>");
      return;
    }
    if (args.toInt() == 0) {
      disableClosedLoop(F("Closed-loop disabled by user."));
      return;
    }

    if (!pressure_.calibrated) {
      Serial.println("Pressure sensor is not calibrated. Run: pt / pc <known_grams>");
      return;
    }
    if (!closedLoop_.pressureSensorAssigned) {
      Serial.println("No pressure sensor is assigned to this closed-loop axis.");
      return;
    }
    if (!axes_[closedLoop_.axis].enabled) {
      Serial.println("Selected motor driver is disabled. Run: me <motor> 1");
      return;
    }
    if (!(closedLoop_.highThresholdGrams > closedLoop_.lowThresholdGrams)) {
      Serial.println("Set a valid pressure range first. Run: clrange <low> <high>");
      return;
    }

    closedLoop_.enabled = true;
    closedLoop_.lastControlMs = 0;
    closedLoop_.integralError = 0.0f;
    closedLoop_.lastError = 0.0f;
    closedLoop_.lastErrorValid = false;
    closedLoop_.lastCorrectionSteps = 0;
    Serial.println("Closed-loop enabled.");
    printClosedLoopStatus();
    return;
  }

  if (cmd == "clreverse") {
    if (!args.length()) {
      Serial.println("Usage: clreverse <0|1>");
      return;
    }
    closedLoop_.reverseMotionEnabled = (args.toInt() != 0);
    printClosedLoopStatus();
    return;
  }

  if (cmd == "cls") {
    printClosedLoopStatus();
    return;
  }

  Serial.println("Unknown command. Type 'mh' for help.");
}

void MotorCalibrationConsole::serviceSerialInput() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();

    if (ch == '\r') {
      continue;
    }

    if (ch == '\b' || ch == 127) {
      if (serialLineBuffer_.length() > 0) {
        serialLineBuffer_.remove(serialLineBuffer_.length() - 1);
        Serial.print("\b \b");
      }
      continue;
    }

    if (ch == '\n') {
      Serial.println();
      String line = serialLineBuffer_;
      serialLineBuffer_ = "";
      handleCommand(line);
      continue;
    }

    if (isPrintable(static_cast<unsigned char>(ch))) {
      if (serialLineBuffer_.length() < 120) {
        serialLineBuffer_ += ch;
        Serial.print(ch);
      }
    }
  }
}

bool MotorCalibrationConsole::parseAxisToken(const String& token,
                                             size_t& axis) const {
  String normalized = token;
  normalized.trim();
  normalized.toLowerCase();
  if (normalized == "m1") {
    axis = 0;
    return true;
  }
  if (normalized == "m2") {
    axis = 1;
    return true;
  }
  return false;
}

bool MotorCalibrationConsole::parseMotionDirectionToken(const String& token,
                                                        bool& forward) const {
  String normalized = token;
  normalized.trim();
  normalized.toLowerCase();

  if (normalized == "f" || normalized == "forward" || normalized == "cw" ||
      normalized == "1") {
    forward = true;
    return true;
  }

  if (normalized == "r" || normalized == "reverse" || normalized == "ccw" ||
      normalized == "-1") {
    forward = false;
    return true;
  }

  return false;
}

void MotorCalibrationConsole::setEnabled(size_t axis, bool enabled) {
  axes_[axis].enabled = enabled;
  const bool pinLevel = axes_[axis].enableActiveLow ? !enabled : enabled;
  digitalWrite(axes_[axis].enPin, pinLevel ? HIGH : LOW);
  if (!enabled) {
    axes_[axis].queuedSteps = 0;
    axes_[axis].stepPinHigh = false;
    digitalWrite(axes_[axis].stepPin, LOW);
  }
}

void MotorCalibrationConsole::setDirection(size_t axis, bool forward) {
  digitalWrite(axes_[axis].dirPin,
               (forward == axes_[axis].dirForwardHigh) ? HIGH : LOW);
}

bool MotorCalibrationConsole::queueMotorMove(size_t axis, long steps,
                                             uint16_t pulseUs, bool verbose) {
  if (!axes_[axis].enabled) {
    if (verbose) {
      Serial.println("Driver is disabled. Run: me <m1|m2> 1");
    }
    return false;
  }
  if (steps == 0) {
    return false;
  }
  if (isMotorBusy(axis)) {
    if (verbose) {
      Serial.println("Motor is busy. Wait for the current move to finish.");
    }
    return false;
  }

  const bool forward = steps > 0;
  const long finalTarget =
      axes_[axis].currentPositionSteps + (forward ? labs(steps) : -labs(steps));
  if (!isWithinSoftLimit(axis, finalTarget)) {
    if (verbose) {
      Serial.println("Move aborted by soft limit.");
    }
    return false;
  }

  axes_[axis].queuedSteps = labs(steps);
  axes_[axis].queuedForward = forward;
  axes_[axis].stepPinHigh = false;
  axes_[axis].activePulseUs = pulseUs;
  axes_[axis].nextStepMicros = micros();
  setDirection(axis, forward);
  return true;
}

void MotorCalibrationConsole::serviceMotorMotion() {
  const unsigned long now = micros();
  for (size_t axis = 0; axis < kAxisCount; ++axis) {
    if (!isMotorBusy(axis)) {
      continue;
    }
    if ((long)(now - axes_[axis].nextStepMicros) < 0) {
      continue;
    }

    if (!axes_[axis].stepPinHigh) {
      const long nextPosition = axes_[axis].currentPositionSteps +
                                (axes_[axis].queuedForward ? 1L : -1L);
      if (!isWithinSoftLimit(axis, nextPosition)) {
        axes_[axis].queuedSteps = 0;
        axes_[axis].stepPinHigh = false;
        digitalWrite(axes_[axis].stepPin, LOW);
        Serial.print("Motor ");
        Serial.print(axis + 1);
        Serial.println(" stopped by soft limit.");
        continue;
      }

      digitalWrite(axes_[axis].stepPin, HIGH);
      axes_[axis].stepPinHigh = true;
      axes_[axis].nextStepMicros = now + axes_[axis].activePulseUs;
      continue;
    }

    digitalWrite(axes_[axis].stepPin, LOW);
    axes_[axis].stepPinHigh = false;
    axes_[axis].currentPositionSteps += axes_[axis].queuedForward ? 1L : -1L;
    --axes_[axis].queuedSteps;

    if (axes_[axis].queuedSteps == 0) {
      axes_[axis].nextStepMicros = now;
    } else {
      axes_[axis].nextStepMicros =
          now + axes_[axis].activePulseUs +
          (unsigned long)axes_[axis].stepSettleMs * 1000UL;
    }
  }
}

bool MotorCalibrationConsole::isMotorBusy(size_t axis) const {
  return axes_[axis].queuedSteps > 0 || axes_[axis].stepPinHigh;
}

bool MotorCalibrationConsole::isWithinSoftLimit(size_t axis,
                                                long targetPosition) const {
  return targetPosition >= axes_[axis].softLimitMinSteps &&
         targetPosition <= axes_[axis].softLimitMaxSteps;
}

void MotorCalibrationConsole::tareScale(uint8_t samples) {
  if (!g_scale.wait_ready_timeout(2000)) {
    Serial.println("HX711 is not ready. Tare aborted.");
    return;
  }

  Serial.println("Taring pressure sensor... keep it unloaded.");
  g_scale.set_scale(1.0f);
  g_scale.tare(samples);
  pressure_.zeroOffset = g_scale.get_offset();
  pressure_.calibrationFactor = 1.0f;
  pressure_.calibrated = false;
  savePressureCalibration();
  Serial.print("Zero offset: ");
  Serial.println(pressure_.zeroOffset);
}

void MotorCalibrationConsole::calibrateWithKnownWeight(float knownGrams,
                                                       uint8_t samples) {
  if (knownGrams <= 0.0f) {
    Serial.println("Known weight must be > 0.");
    return;
  }
  if (!g_scale.wait_ready_timeout(2000)) {
    Serial.println("HX711 is not ready. Calibration aborted.");
    return;
  }

  Serial.print("Calibrating pressure sensor with ");
  Serial.print(knownGrams, 3);
  Serial.println(" g");

  g_scale.set_scale(1.0f);
  g_scale.set_offset(pressure_.zeroOffset);
  delay(300);

  const float raw = g_scale.get_value(samples);
  if (fabs(raw) < 1.0f) {
    Serial.println("Raw delta too small. Check sensor/load/known weight.");
    return;
  }

  pressure_.calibrationFactor = raw / knownGrams;
  pressure_.calibrated = true;
  g_scale.set_scale(pressure_.calibrationFactor);
  g_scale.set_offset(pressure_.zeroOffset);

  Serial.print("Raw delta: ");
  Serial.println(raw, 3);
  Serial.print("Calibration factor: ");
  Serial.println(pressure_.calibrationFactor, 6);
  savePressureCalibration();
}

void MotorCalibrationConsole::loadPressureCalibration() {
  if (!g_prefs.begin(kNvsNamespace, true)) {
    Serial.println("NVS open(read) failed. Using defaults.");
    g_scale.set_scale(1.0f);
    g_scale.set_offset(0);
    pressure_.calibrationFactor = 1.0f;
    pressure_.zeroOffset = 0;
    pressure_.calibrated = false;
    return;
  }

  pressure_.calibrated = g_prefs.getBool(kKeyReady, false);
  pressure_.calibrationFactor = g_prefs.getFloat(kKeyFactor, 1.0f);
  pressure_.zeroOffset = g_prefs.getLong(kKeyOffset, 0);
  g_prefs.end();

  g_scale.set_offset(pressure_.zeroOffset);
  if (pressure_.calibrated && fabs(pressure_.calibrationFactor) > 0.000001f) {
    g_scale.set_scale(pressure_.calibrationFactor);
    Serial.println("Loaded saved pressure calibration from NVS.");
  } else {
    g_scale.set_scale(1.0f);
    pressure_.calibrationFactor = 1.0f;
    pressure_.calibrated = false;
  }
}

void MotorCalibrationConsole::savePressureCalibration() {
  if (!g_prefs.begin(kNvsNamespace, false)) {
    Serial.println("NVS open(write) failed. Values not saved.");
    return;
  }

  g_prefs.putBool(kKeyReady, pressure_.calibrated);
  g_prefs.putFloat(kKeyFactor, pressure_.calibrationFactor);
  g_prefs.putLong(kKeyOffset, pressure_.zeroOffset);
  g_prefs.end();
  Serial.println("Pressure calibration saved to NVS.");
}

bool MotorCalibrationConsole::readPressureGrams(float& grams, uint8_t samples) {
  if (!g_scale.wait_ready_timeout(500)) {
    closedLoop_.lastMeasurementValid = false;
    return false;
  }

  if (!pressure_.calibrated) {
    grams = g_scale.get_value(samples);
  } else {
    g_scale.set_scale(pressure_.calibrationFactor);
    g_scale.set_offset(pressure_.zeroOffset);
    grams = g_scale.get_units(samples);
  }

  closedLoop_.lastMeasuredGrams = grams;
  closedLoop_.lastMeasurementValid = true;
  return true;
}

bool MotorCalibrationConsole::readFilteredPressureGrams(float& grams) {
  float sum = 0.0f;
  uint8_t validReads = 0;
  const uint8_t filterPasses = max<uint8_t>(1, closedLoop_.filterPasses);

  for (uint8_t i = 0; i < filterPasses; ++i) {
    float oneRead = 0.0f;
    if (!readPressureGrams(oneRead, closedLoop_.measurementSamples)) {
      continue;
    }
    sum += oneRead;
    ++validReads;
  }

  if (validReads == 0) {
    closedLoop_.lastMeasurementValid = false;
    return false;
  }

  grams = sum / validReads;
  closedLoop_.lastMeasuredGrams = grams;
  closedLoop_.lastMeasurementValid = true;
  return true;
}

void MotorCalibrationConsole::servicePressureMonitor() {
  if (!pressureMonitor_.enabled) {
    return;
  }

  if (millis() - pressureMonitor_.lastPrintMs < pressureMonitor_.intervalMs) {
    return;
  }
  pressureMonitor_.lastPrintMs = millis();

  float grams = 0.0f;
  if (!readPressureGrams(grams, pressureMonitor_.samples)) {
    Serial.println("[prmon] Pressure: unavailable");
    return;
  }

  Serial.print("[prmon] Pressure: ");
  Serial.print(grams, 3);
  Serial.println(" g");
}

void MotorCalibrationConsole::serviceClosedLoop() {
  if (!closedLoop_.commandInterfaceEnabled) {
    return;
  }

  if (!closedLoop_.enabled) {
    return;
  }

  const unsigned long nowMs = millis();
  if (nowMs - closedLoop_.lastControlMs < closedLoop_.controlIntervalMs) {
    return;
  }
  const float dtSeconds =
      closedLoop_.lastControlMs == 0
          ? closedLoop_.controlIntervalMs / 1000.0f
          : max(0.001f, (nowMs - closedLoop_.lastControlMs) / 1000.0f);
  closedLoop_.lastControlMs = nowMs;

  if (!axes_[closedLoop_.axis].enabled) {
    disableClosedLoop(F("Closed-loop stopped: motor driver disabled."));
    return;
  }

  float grams = 0.0f;
  if (!readFilteredPressureGrams(grams)) {
    Serial.println("Closed-loop warning: pressure read failed.");
    return;
  }

  const float targetGrams =
      (closedLoop_.lowThresholdGrams + closedLoop_.highThresholdGrams) * 0.5f;
  const float deadbandGrams =
      (closedLoop_.highThresholdGrams - closedLoop_.lowThresholdGrams) * 0.5f;
  float errorGrams = targetGrams - grams;
  if (fabs(errorGrams) <= deadbandGrams) {
    closedLoop_.integralError = 0.0f;
    closedLoop_.lastError = errorGrams;
    closedLoop_.lastErrorValid = true;
    closedLoop_.lastCorrectionSteps = 0;
    return;
  }

  errorGrams += errorGrams > 0.0f ? -deadbandGrams : deadbandGrams;
  closedLoop_.integralError = constrain(
      closedLoop_.integralError + errorGrams * dtSeconds,
      -closedLoop_.maxIntegral, closedLoop_.maxIntegral);

  const float derivativeGramsPerSecond =
      closedLoop_.lastErrorValid
          ? (errorGrams - closedLoop_.lastError) / dtSeconds
          : 0.0f;
  float correction =
      closedLoop_.kp * errorGrams + closedLoop_.ki * closedLoop_.integralError +
      closedLoop_.kd * derivativeGramsPerSecond;
  correction = constrain(correction, -(float)closedLoop_.adjustStepsPerCycle,
                         (float)closedLoop_.adjustStepsPerCycle);

  const long minAllowedSteps =
      closedLoop_.lastCorrectionSteps - closedLoop_.maxStepChangePerCycle;
  const long maxAllowedSteps =
      closedLoop_.lastCorrectionSteps + closedLoop_.maxStepChangePerCycle;
  long correctionSteps =
      constrain(lroundf(correction), minAllowedSteps, maxAllowedSteps);
  if (labs(correctionSteps) < closedLoop_.minAdjustStepsPerCycle &&
      fabs(correction) > 0.0f && closedLoop_.minAdjustStepsPerCycle > 0) {
    correctionSteps =
        correction > 0.0f ? closedLoop_.minAdjustStepsPerCycle
                          : -closedLoop_.minAdjustStepsPerCycle;
    correctionSteps = constrain(correctionSteps, minAllowedSteps, maxAllowedSteps);
  }
  if (closedLoop_.reverseMotionEnabled) {
    correctionSteps = -correctionSteps;
  }

  closedLoop_.lastError = errorGrams;
  closedLoop_.lastErrorValid = true;
  closedLoop_.lastCorrectionSteps =
      closedLoop_.reverseMotionEnabled ? -correctionSteps : correctionSteps;

  if (correctionSteps == 0) {
    return;
  }

  if (!runBlockingMotorSteps(closedLoop_.axis, correctionSteps,
                             closedLoop_.stepPulseUs,
                             closedLoop_.stepSettleUs)) {
    disableClosedLoop(
        F("Closed-loop stopped: PID correction hit soft limit."));
  }
}

void MotorCalibrationConsole::disableClosedLoop(
    const __FlashStringHelper* reason) {
  closedLoop_.enabled = false;
  closedLoop_.integralError = 0.0f;
  closedLoop_.lastError = 0.0f;
  closedLoop_.lastErrorValid = false;
  closedLoop_.lastCorrectionSteps = 0;
  Serial.println(reason);
  printClosedLoopStatus();
}

void MotorCalibrationConsole::assignPressureSensorToAxis(size_t axis) {
  closedLoop_.axis = axis;
  closedLoop_.pressureSensorAssigned = (axis == 0);
}

void MotorCalibrationConsole::applyStartupConfig(
    const StartupConfig& startupConfig) {
  closedLoop_.measurementSamples =
      constrain(startupConfig.closedLoopMeasurementSamples, 1, 50);
  closedLoop_.filterPasses =
      constrain(startupConfig.closedLoopFilterPasses, 1, 20);
  closedLoop_.adjustStepsPerCycle =
      max(1L, startupConfig.closedLoopAdjustStepsPerCycle);
  closedLoop_.minAdjustStepsPerCycle =
      constrain(startupConfig.closedLoopMinAdjustStepsPerCycle, 0L,
                closedLoop_.adjustStepsPerCycle);
  closedLoop_.maxStepChangePerCycle =
      max(1L, startupConfig.closedLoopMaxStepChangePerCycle);
  closedLoop_.controlIntervalMs =
      max(1UL, startupConfig.closedLoopIntervalMs);
  closedLoop_.reverseMotionEnabled = startupConfig.reverseClosedLoopMotion;
  closedLoop_.stepPulseUs = max<uint16_t>(50, startupConfig.closedLoopStepPulseUs);
  closedLoop_.stepSettleUs =
      max<uint16_t>(50, startupConfig.closedLoopStepSettleUs);
  closedLoop_.kp = max(0.0f, startupConfig.closedLoopKp);
  closedLoop_.ki = max(0.0f, startupConfig.closedLoopKi);
  closedLoop_.kd = max(0.0f, startupConfig.closedLoopKd);
  closedLoop_.maxIntegral = max(0.0f, startupConfig.closedLoopMaxIntegral);
  closedLoop_.commandInterfaceEnabled = !startupConfig.manualOpenLoopOnly;
  if (startupConfig.manualOpenLoopOnly) {
    closedLoop_.enabled = false;
  }

  if (startupConfig.motor1ClosedLoopLowGrams !=
      startupConfig.motor1ClosedLoopHighGrams) {
    closedLoop_.lowThresholdGrams = min(startupConfig.motor1ClosedLoopLowGrams,
                                        startupConfig.motor1ClosedLoopHighGrams);
    closedLoop_.highThresholdGrams =
        max(startupConfig.motor1ClosedLoopLowGrams,
            startupConfig.motor1ClosedLoopHighGrams);
  }

  assignPressureSensorToAxis(0);

  if (startupConfig.autoEnableMotor1) {
    setEnabled(0, true);
    Serial.println("Startup: motor1 driver enabled automatically.");
    if (startupConfig.startupMotor1EnableSettleMs > 0) {
      delay(startupConfig.startupMotor1EnableSettleMs);
    }
  }

  if (startupConfig.runStartupMotor1SelfTest) {
    const long selfTestSteps = startupConfig.startupMotor1SelfTestSteps;
    if (selfTestSteps == 0) {
      Serial.println("Startup: motor1 self-test skipped because step count is 0.");
    } else if (runBlockingStartupSelfTest(
                   0, selfTestSteps, startupConfig.startupMotor1SelfTestPulseUs)) {
      Serial.print("Startup: motor1 blocking self-test completed for ");
      Serial.print(selfTestSteps);
      Serial.println(" steps.");
    } else {
      Serial.println("Startup: motor1 self-test move could not run.");
    }
  }

  if (startupConfig.runStartupMotor1QueueTest) {
    if (startupConfig.startupMotor1QueueTestSteps == 0) {
      Serial.println("Startup: motor1 queue test skipped because step count is 0.");
    } else {
      Serial.println("Startup: motor1 queue test is disabled in production mode.");
    }
  }

  if (startupConfig.manualOpenLoopOnly) {
    Serial.println("Startup: manual open-loop mode. Use me/mrun/mj commands.");
    return;
  }

  if (!startupConfig.autoStartClosedLoopMotor1) {
    return;
  }

  if (!pressure_.calibrated) {
    Serial.println(
        "Startup: closed-loop not started because pressure calibration is not saved.");
    return;
  }

  if (!axes_[0].enabled) {
    Serial.println(
        "Startup: closed-loop not started because motor1 is disabled.");
    return;
  }

  if (!(closedLoop_.highThresholdGrams > closedLoop_.lowThresholdGrams)) {
    Serial.println(
        "Startup: closed-loop not started because the pressure range is invalid.");
    return;
  }

  closedLoop_.enabled = true;
  closedLoop_.lastControlMs = 0;
  closedLoop_.integralError = 0.0f;
  closedLoop_.lastError = 0.0f;
  closedLoop_.lastErrorValid = false;
  closedLoop_.lastCorrectionSteps = 0;
  Serial.print("Startup: motor1 closed-loop enabled at ");
  Serial.print(closedLoop_.lowThresholdGrams, 3);
  Serial.print(" .. ");
  Serial.print(closedLoop_.highThresholdGrams, 3);
  Serial.println(" g");
}


bool MotorCalibrationConsole::runBlockingStartupSelfTest(size_t axis, long steps,
                                                         uint16_t pulseUs) {
  return runBlockingMotorSteps(axis, steps, pulseUs, pulseUs);
}

bool MotorCalibrationConsole::runBlockingMotorSteps(size_t axis, long steps,
                                                    uint16_t pulseUs,
                                                    uint16_t settleUs) {
  if (axis >= kAxisCount || !axes_[axis].enabled) {
    return false;
  }

  const bool forward = steps > 0;
  const long totalSteps = labs(steps);
  if (totalSteps == 0) {
    return false;
  }

  const long finalTarget =
      axes_[axis].currentPositionSteps + (forward ? totalSteps : -totalSteps);
  if (!isWithinSoftLimit(axis, finalTarget)) {
    return false;
  }

  setDirection(axis, forward);
  delay(5);

  for (long step = 0; step < totalSteps; ++step) {
    const long nextPosition =
        axes_[axis].currentPositionSteps + (forward ? 1L : -1L);
    if (!isWithinSoftLimit(axis, nextPosition)) {
      return false;
    }

    digitalWrite(axes_[axis].stepPin, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(axes_[axis].stepPin, LOW);
    delayMicroseconds(settleUs);
    axes_[axis].currentPositionSteps = nextPosition;
  }

  return true;
}
