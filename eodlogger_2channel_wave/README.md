# eodlogger_2channel_wave

- Records from 2 channels, one from each ADC
- Data are saved in wave files together with relevant metadata:
  sampling rate, number of channels and pin IDs, bit resolution,
  data and time, Teensy board version and its unique MAC address.
- Data acquisition (sampling rate, channels, averaging, etc.) can
  be easily changed.
- A configuration file can be used to setup the logger.


## Dependencies

The logger is based on the following libraries:

- [Arduino Time Library](https://github.com/PaulStoffregen/Time)
- [ADC](https://github.com/pedvide/ADC)
- [SdFat version2](https://github.com/greiman/SdFat)
- [TeeRec](https://github.com/janscience/TeeRec) library.


## Installation

The [Arduino Time Library](https://github.com/PaulStoffregen/Time) and
[ADC](https://github.com/pedvide/ADC) libraries are already included
in [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html).

For installing [SdFat version2](https://github.com/greiman/SdFat) open in
the Arduino IDE Tools - Manage libraries. Search for SdFat and install it.

For [TeeRec](https://github.com/janscience/TeeRec) clone the
repository in 'Arduino/libraries':
```sh
cd Arduino/libraries
git clone https://github.com/janscience/TeeRec.git
```

Alternatively, download the whole repository as a zip archive (open
https://github.com/janscience/TeeRec in your browser and click on the
green "Code" button). Unpack the zip file:
```sh
cd Arduino/libraries
unzip ~/Downloads/TeeRec-main.zip
```

Close the Arduino IDE and open it again. Then the Arduino IDE knows
about the [TeeRec](https://github.com/janscience/TeeRec) library.


## Setup

Open the `eodlogger_2channel_wave` sketch in the Arduino IDE.

### Real-time clock

By default, the on board real-time clock of the Teensy is used. If you
want to use a clock provided by an external DS1307RTC instead
(e.g. the AT24C32 RTC Modul), then uncomment the Wire and DS1307RTC
include at the top of the sketch so that it looks like this:
```c
#include <Configurator.h>
#include <ContinuousADC.h>
#include <SDWriter.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <RTClock.h>
...
```

### Data acquisition

In the top section marked as "Default settings" you may adapt some settings
according to your needs. The first block is about the data
acquisition. You can set the sampling rate, input pins (channels), bit
resolution, and number of averages per sample. If you change these
settings, check the output on the serial monitor and the performance
before using the logger! See
[TeeRec](https://github.com/janscience/TeeRec) for various sketches
and tools that help you to select the best settings for the data
acquisition.

### File size and naming

The second section is about the files that store the data on the SD
card.  The files are stored in a directory whose name is specified by
the `path` variable. The file names in this directory are specified by
the `fileName` variable. The file name can be an arbitrary string, but
should end with the '.wav' extension. The following special strings in
the file name are replaced by the current date, time or a number:

- `DATE`: the current data as ISO string (YYYY-MM-DD)
- `SDATE`: "short date" - the current date as YYYYMMDD
- `TIME`: the current time as ISO string but with dashes instead of colons (HH-MM-SS)
- `STIME`: "short time" - the current time as HHMMSS
- `DATETIME`: the current date and time as YYYY-MM-DDTHH-MM-SS
- `SDATETIME`: "short data and time" - the current date and time as YYYYMMDDTHHMMSS
- `ANUM`: a two character string numbering the files: 'aa', 'ab', 'ac', ..., 'ba', 'bb', ...
- `NUM`: a two character string numbering the files: '01', '02', '03', ..., '10', '11', ...

`fileSaveTime` specifies for how many seconds data should be saved into
each file. The default is 10min.

Once you modified the sketch to your needs, upload it to the Teensy (`Ctrl-U`).


## Configuration

Most of the settings described above can be configured via a
configuration file. Simply place a configuration file named
`logger.cfg` into the root folder of the SD card. If present, this
file is read once on startup. You find an example configuration file
along with the logger sketch in this directory. The content should
look like this:

```txt
# Configuration file for EOD logger.

Settings:
  Path: recordings       # path where to store data
  FileName: logger1-SDATETIME.wav  # may include DATE, SDATE, TIME, STIME, DATETIME, SDATETIME, ANUM, NUM
  FileTime: 10min        # s, min, or h
  PulseFreq: 400Hz       # Hz, kHz, or MHz

ADC:
  SamplingRate: 44.1kHz  # Hz, kHz, or MHz
  Averaging   : 4
  Conversion  : high
  Sampling    : high
  Resolution  : 12bit
  Reference   : 3.3V
``` 

Everything behind '#' is a comment. All lines without a colon are
ignored, that is empty lines are ignored, but also lines with text
without colon.  Matching keys and most values is case
insensitive. Unknown keys are ignored but reported. Times and
frequencies understand various units as indicated in the
comments. Check the serial monitor to confirm the right settings.


## Usage

Connect the Teensy to a battery and let it record the data.


### LED

The on-board LED of the Teensy indicates the following events:

- On startup the LED is switched on during the initialization of the
  data acqusition and SD card. This can last for up to 2 seconds
  (timeout for establishing a serial connection with a computer).

- Normal operation, i.e. data acquisition is running and data are
  written to SD card: the LED briefly flashes every 5 seconds.

- Whenever a file is closed (and a new one opened), the LED lights for
  1 second. Then it continues with flashes every 5 seconds.

- The LED is switched off completely if no data can be written on the
  SD card (no SD card present or SD card full) or data acquisition is
  not working.  Connect the Teensy to a computer and open the serial
  monitor of the Arduino IDE (`Ctrl+Shift+M`). On startup the settings
  for the data acquisition are reported and in case of problems an
  error message is displayed.

