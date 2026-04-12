# HX711 Calibration and Usage

## What this module does

This project includes an `Hx711Console` module that:

- Reads HX711 data (`DT=GPIO4`, `SCK=GPIO5`)
- Supports serial commands for tare/calibration/read
- Stores calibration data in ESP32 NVS (non-volatile storage)
- Loads saved values automatically on boot

## What is stored permanently

Stored in NVS namespace `hx711cfg`:

- `factor` (`float`): calibration factor (`raw_count / gram`)
- `offset` (`long`): zero offset (tare baseline)
- `ready` (`bool`): whether calibration is valid

These values are updated after:

- `t` (tare)
- `c <known_grams>` (calibration)
- `s <factor>` (manual factor set)

## Serial commands

- `h` show help
- `t` tare and save zero offset
- `c <known_grams>` calibrate and save factor
- `w [samples]` print weight in grams
- `r [samples]` print raw value (tare removed)
- `p` print status (calibrated/factor/offset)
- `s <factor>` set factor manually and save

## How to use in code

`main.cpp` uses the module like this:

```cpp
static constexpr uint8_t HX711_DOUT_PIN = 4;
static constexpr uint8_t HX711_SCK_PIN = 5;
Hx711Console g_hx711Console;

void setup() {
  g_hx711Console.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
}

void loop() {
  g_hx711Console.update();
}
```

## Read the two key numbers directly

You can call getters at runtime:

```cpp
float factor = g_hx711Console.getCalibrationFactor();
long offset = g_hx711Console.getZeroOffset();
bool ok = g_hx711Console.isCalibrated();
```

Typical use:

- Print factor/offset to log
- Send factor/offset to another module
- Decide whether your app can start measurement (`ok == true`)

## First-time calibration flow

1. Keep scale empty, send `t`
2. Put known weight (for example 100g), send `c 100`
3. Send `p` to verify saved `factor` and `offset`
4. Reboot board, send `p` again to confirm values were loaded
