# PauloPaperMobileSed_rj

See the Wiki for documentation: [MobileTurbidity Wiki](https://github.com/Robwerg/MobileTurbidity/wiki)

## Changes from V107

- New reusable sdcard function for writing sd files.
- Turb reading and pump. Clashes with PT100 sensor so can't use that with this code.
- Start RTC earlier because if debug() is called beforeRTC init then it failes because of FsDateTime::setCallback(dateTime)
- Now only call FsDateTime::setCallback(dateTime) once because you only need to call it once.


## TODO

- Calibrate ALS 0 sensor. Need minimum 2 sets of temp/min/max values to work correctly.
- Maybe rename the ino file?
- backport sd function and changes to V107 maybe?
- FsDateTime::setCallback(dateTime) is 2 hours behind. Maybe a timezone issue?

## Board

The code is compiled for the **SparkFun SAMD21 Pro RF** board.

- FQBN: `SparkFun:samd:samd21_proRF`
- 1W variant (if using the 1W LoRa module): `SparkFun:samd:samd21_proRF_1w`

The SparkFun boards are **not** in the default Arduino core, so add the SparkFun
board package via the Boards Manager. In **File > Preferences**, paste this into
**Additional boards manager URLs**:

```
https://raw.githubusercontent.com/sparkfun/Arduino_Boards/main/IDE_Board_Manager/package_sparkfun_index.json
```

Then install **"SparkFun SAMD21 Boards"** from the Boards Manager.

> Note: `Wire.h` and other Arduino-* core libraries are provided by the
> **Arduino SAMD21 core** (board package `arduino:samd`) and do not need a
> separate install.

## Libraries

Use these **exact** versions for maximum compatibility and to avoid compile
errors. Install via the Arduino **Library Manager**, or manually (the header
name in the sketch does not always match the Library Manager package name).

| Header in sketch | Library Manager name | Version |
| --- | --- | --- |
| `RTClib.h` | RTClib | 2.1.4 |
| `Wire.h` | *(Arduino SAMD21 core)* | built-in |
| `SdFat.h` | SdFat | 2.3.0 |
| `RTCZero.h` | RTCZero | 1.6.0 |
| `RH_RF95.h` | RadioHead | 1.143.1 |
| `SDI12.h` | SDI-12 | 2.3.2 |
| `ADS1X15.h` | ADS1X15 | 0.6.2 |
| `OneWire.h` | OneWire | 2.3.8 |
| `DallasTemperature.h` | DallasTemperature | 4.0.6 |
| `SparkFunBME280.h` | SparkFun BME280 | 2.0.11 |
| `SparkFun_TMP117.h` | SparkFun High Precision Temperature Sensor TMP117 Qwiic | 1.2.5 |
| `Adafruit_MAX31865.h` | Adafruit MAX31865 | 1.6.2 |
| `QuickMedianLib.h` | QuickMedianLib | 1.1.1 |

### Transitive dependencies

These are required by the libraries above and should be present (usually
installed automatically with their parent). Pin them to the same versions.

| Library | Version |
| --- | --- |
| Adafruit Bus IO | 1.17.4 |
| Adafruit Unified Sensor | 1.1.15 |

> `FlashAsEEPROM.h` is referenced in the source but commented out
> (`#include <FlashAsEEPROM.h>` is not active) and is **not** required to build.
