#include <Configurator.h>
#include <ContinuousADC.h>
#include <SDWriter.h>
//#include <Wire.h>
//#include <DS1307RTC.h>
#include <RTClock.h>
#include <Settings.h>
#include <Blink.h>
#include <TestSignals.h>


// Default settings: -----------------------------------------------------------------------
// (may be overwritten by config file logger.cfg)

uint32_t samplingRate = 50000; // samples per second and channel in Hertz
int8_t channel0 = A2;           // input pin for ADC0
int8_t channel1 = A16;          // input pin for ADC1
int bits = 12;                  // resolution: 10bit 12bit, or 16bit
int averaging = 4;              // number of averages per sample: 0, 4, 8, 16, 32

char path[] = "recordings";     // directory where to store files on SD card.
char fileName[] = "logger1-SDATETIME.wav";   // may include DATE, SDATE, TIME, STIME, DATETIME, SDATETIME, ANUM, NUM
float fileSaveTime = 10*60;     // seconds

int pulseFrequency = 200;       // Hertz
int signalPins[] = {2, 3, -1};  // pins where to put out test signals


// ------------------------------------------------------------------------------------------
 
Configurator config;
ContinuousADC aidata;
SDCard sdcard;
SDWriter file(sdcard, aidata);
Settings settings("recordings", fileName, fileSaveTime, pulseFrequency);
RTClock rtclock;
Blink blink;


void setupADC() {
  aidata.setChannel(0, channel0);
  aidata.setChannel(1, channel1);
  aidata.setRate(samplingRate);
  aidata.setResolution(bits);
  aidata.setAveraging(averaging);
  aidata.setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  aidata.setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  aidata.check();
}


void openNextFile() {
  String name = rtclock.makeStr(settings.FileName, true);
  name = file.incrementFileName(name);
  if (name.length() == 0) {
    Serial.println("WARNING: failed to open file on SD card.");
    Serial.println("SD card probably not inserted.");
    Serial.println();
    return;
  }
  char dts[20];
  rtclock.dateTime(dts);
  file.openWave(name.c_str(), aidata, -1, dts);
  file.writeData();
  Serial.println(name);
  if (file.isOpen()) {
    blink.set(5000, 5);
    blink.blink(2000, 1000);
  }
  else {
    Serial.println();
    Serial.println("WARNING: failed to open file on SD card.");
    Serial.println("SD card probably not inserted.");
  }
}


void setupStorage() {
  file.dataDir(settings.Path);
  file.setWriteInterval(aidata);
  file.setMaxFileTime(settings.FileTime);
  file.startWrite();
  openNextFile();
}


void storeData() {
  if (file.needToWrite()) {
    size_t samples = file.writeData();
    if (samples == 0) {
      blink.clear();
      Serial.println();
      Serial.println("ERROR: data acquisition not running.");
      Serial.println("sampling rate probably too high,");
      Serial.println("given the number of channels, averaging, sampling and conversion speed.");
      while(1) {};
    }
    if (file.endWrite()) {
      file.close();
      blink.clear();
      openNextFile();
    }
  }
}


// ------------------------------------------------------------------------------------------

void setup() {
  blink.switchOn();
  Serial.begin(9600);
  while (!Serial && millis() < 2000) {};
  rtclock.check();
  setupADC();
  config.setConfigFile("logger.cfg");
  config.configure(sdcard);
  setupTestSignals(signalPins, settings.PulseFrequency);
  aidata.check();
  blink.switchOff();
  setupStorage();
  aidata.start();
  Serial.println();
  aidata.report();
}


void loop() {
  storeData();
  blink.update();
}
