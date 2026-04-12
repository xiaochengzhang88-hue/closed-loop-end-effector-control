#include "archived_hx711_console.h"

#include <HX711.h>
#include <Preferences.h>

namespace {
constexpr const char* kNvsNamespace = "hx711cfg";
constexpr const char* kKeyFactor = "factor";
constexpr const char* kKeyOffset = "offset";
constexpr const char* kKeyReady = "ready";

HX711 scale;
Preferences prefs;
}  // namespace

void Hx711Console::begin(uint8_t doutPin, uint8_t sckPin) {
  doutPin_ = doutPin;
  sckPin_ = sckPin;

  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("HX711 calibration tool");
  Serial.print("Pins: DT=GPIO");
  Serial.print(doutPin_);
  Serial.print(", SCK=GPIO");
  Serial.println(sckPin_);

  scale.begin(doutPin_, sckPin_);
  if (!scale.wait_ready_timeout(5000)) {
    Serial.println("HX711 not found. Check wiring and power.");
    while (true) {
      delay(1000);
    }
  }

  loadFromNvs();

  if (!calibrated_) {
    Serial.println("No saved calibration. Running initial tare...");
    tareScale();
  }

  printHelp();
}

void Hx711Console::update() {
  if (!Serial.available()) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  handleCommand(line);
}

float Hx711Console::getCalibrationFactor() const { return calibrationFactor_; }

long Hx711Console::getZeroOffset() const { return zeroOffset_; }

bool Hx711Console::isCalibrated() const { return calibrated_; }

void Hx711Console::printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h                 : show this help");
  Serial.println("  t                 : tare (empty scale) and save");
  Serial.println("  c <known_grams>   : calibrate with known weight and save");
  Serial.println("  r [samples]       : print raw value (tare removed)");
  Serial.println("  w [samples]       : print weight in grams");
  Serial.println("  s <factor>        : set calibration factor and save");
  Serial.println("  p                 : print current status");
  Serial.println();
}

void Hx711Console::loadFromNvs() {
  if (!prefs.begin(kNvsNamespace, true)) {
    Serial.println("NVS open(read) failed. Using defaults.");
    scale.set_scale(1.0f);
    scale.set_offset(0);
    calibrationFactor_ = 1.0f;
    zeroOffset_ = 0;
    calibrated_ = false;
    return;
  }

  calibrated_ = prefs.getBool(kKeyReady, false);
  calibrationFactor_ = prefs.getFloat(kKeyFactor, 1.0f);
  zeroOffset_ = prefs.getLong(kKeyOffset, 0);
  prefs.end();

  if (calibrated_ && fabs(calibrationFactor_) > 0.000001f) {
    scale.set_scale(calibrationFactor_);
    scale.set_offset(zeroOffset_);
    Serial.println("Loaded saved calibration from NVS.");
    printStatus();
  } else {
    scale.set_scale(1.0f);
    scale.set_offset(0);
    calibrated_ = false;
    calibrationFactor_ = 1.0f;
    zeroOffset_ = 0;
  }
}

void Hx711Console::saveToNvs() {
  if (!prefs.begin(kNvsNamespace, false)) {
    Serial.println("NVS open(write) failed. Values not saved.");
    return;
  }

  prefs.putBool(kKeyReady, calibrated_);
  prefs.putFloat(kKeyFactor, calibrationFactor_);
  prefs.putLong(kKeyOffset, zeroOffset_);
  prefs.end();
  Serial.println("Saved to NVS.");
}

void Hx711Console::tareScale(uint8_t samples) {
  Serial.println("Taring... keep the scale empty.");
  scale.tare(samples);
  zeroOffset_ = scale.get_offset();
  Serial.print("Zero offset: ");
  Serial.println(zeroOffset_);
  saveToNvs();
  Serial.println("Tare done.");
}

void Hx711Console::calibrateWithKnownWeight(float knownGrams, uint8_t samples) {
  if (knownGrams <= 0.0f) {
    Serial.println("Known weight must be > 0.");
    return;
  }

  Serial.print("Calibrating with ");
  Serial.print(knownGrams, 3);
  Serial.println(" g");

  scale.set_scale(1.0f);
  delay(300);
  float raw = scale.get_value(samples);
  if (fabs(raw) < 1.0f) {
    Serial.println("Raw delta too small. Check wiring/load/known weight.");
    return;
  }

  calibrationFactor_ = raw / knownGrams;
  scale.set_scale(calibrationFactor_);
  zeroOffset_ = scale.get_offset();
  calibrated_ = true;

  Serial.print("Raw delta: ");
  Serial.println(raw, 3);
  Serial.print("Calibration factor: ");
  Serial.println(calibrationFactor_, 6);

  saveToNvs();
  Serial.println("Calibration done.");
}

void Hx711Console::printRaw(uint8_t samples) {
  float raw = scale.get_value(samples);
  Serial.print("Raw (tare removed): ");
  Serial.println(raw, 3);
}

void Hx711Console::printWeight(uint8_t samples) {
  if (!calibrated_) {
    Serial.println("Not calibrated yet. Run: c <known_grams>");
    return;
  }
  float grams = scale.get_units(samples);
  Serial.print("Weight: ");
  Serial.print(grams, 3);
  Serial.println(" g");
}

void Hx711Console::printStatus() {
  Serial.print("Calibrated: ");
  Serial.println(calibrated_ ? "yes" : "no");
  Serial.print("Calibration factor: ");
  Serial.println(calibrationFactor_, 6);
  Serial.print("Zero offset: ");
  Serial.println(zeroOffset_);
}

void Hx711Console::handleCommand(String line) {
  line.trim();
  if (line.isEmpty()) {
    return;
  }

  const char cmd = tolower(line.charAt(0));
  const int split = line.indexOf(' ');
  String arg = "";
  if (split > 0) {
    arg = line.substring(split + 1);
    arg.trim();
  }

  if (cmd == 'h') {
    printHelp();
    return;
  }
  if (cmd == 't') {
    tareScale();
    return;
  }
  if (cmd == 'p') {
    printStatus();
    return;
  }
  if (cmd == 'c') {
    calibrateWithKnownWeight(arg.toFloat());
    return;
  }
  if (cmd == 's') {
    const float factor = arg.toFloat();
    if (fabs(factor) < 0.000001f) {
      Serial.println("Invalid factor.");
      return;
    }
    calibrationFactor_ = factor;
    scale.set_scale(calibrationFactor_);
    zeroOffset_ = scale.get_offset();
    calibrated_ = true;
    saveToNvs();
    printStatus();
    return;
  }
  if (cmd == 'r') {
    const uint8_t samples = arg.length() ? (uint8_t)constrain(arg.toInt(), 1, 50) : 10;
    printRaw(samples);
    return;
  }
  if (cmd == 'w') {
    const uint8_t samples = arg.length() ? (uint8_t)constrain(arg.toInt(), 1, 50) : 10;
    printWeight(samples);
    return;
  }

  Serial.println("Unknown command. Type 'h' for help.");
}
