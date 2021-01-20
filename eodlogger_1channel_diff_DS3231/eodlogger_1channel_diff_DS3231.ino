//*************************************************************************
// One channel EOD logger programm - adapted from Lydia Federman's sketch
// Features:
// - use DS3231 RTC to generate timestamps for log file names
// - write textfile with metadata alongside logs on microSD card
// - supports exFat formatted microSD cards
// - uses interrupt to transfer data from buffer to microSD card
//*************************************************************************

#include <ADC.h>              // library for easier ADC implementation
#include <DMAChannel.h>       // library for easier DMA implementation
#include <array>              // use C++ array structs
#include <SdFat.h>            // work with SD card
#include <Wire.h>             // communicate with RTC via I2C
#include "RTClib.h"           // library for easy implementation of DS3231 RTC

/*----------------------------------------------------------------*/
const uint32_t pdbfreq = 50000;  // sampling speed [Hz] max ~300MHz - actually the trigger frequency of the programmable delay block
uint32_t duration = 10;           // duration of each measure-cycle [s]
String ID = "Log";                // file name prefix
unsigned long debug_start;
bool single_ended = false;        // switch between single-ended and differential recording
/*----------------------------------------------------------------*/

/*METADATA--------------------------------------------------------*/
String location = "Tierhaus";
String experimenter = "Dr. Jekyll";
String experiment_name = "24h Activity measurements";
const int amp_gain = 5;
const int high_pass = 300;
const int low_pass = 20000;
/*----------------------------------------------------------------*/

/*DECLARATIONS RTC------------------------------------------------*/
RTC_DS3231 rtc;
/*----------------------------------------------------------------*/

/*DECLARATIONS adc and pins---------------------------------------*/
ADC *adc = new ADC();                           // Declare adc object. Update: adc as pointer, use adc->adcX
const uint8_t diff_pin1 = A10;                  // digital pin A10 for differential
const uint8_t diff_pin2 = A11;                  // digital pin A11 for differential
/*----------------------------------------------------------------*/

/*Buffer declarations----------------------------------------------*/
std::array<volatile uint16_t, (uint32_t)128 * 512> buffer __attribute__ ((aligned (32 * 1024)));      // size of buffer is limited due to the Teensy's program memory

uint32_t      BUF_DIM       = 32768;                                          //size of buffer that holds data from ADC
uint32_t      FILE_SIZE     = 0;                                              //initial variables for filesize and pointer to...
uint32_t      last          = 0;     
uint32_t      bytes         = 0;
float         preceil       = 0;
float         scale         = 0;
/*----------------------------------------------------------------*/

/*DECLARATIONS dma and tcd----------------------------------------*/
DMAChannel dma;                                 // used to declare the dma.### object for the first channel

typeof(*dma.TCD)  tcd_mem[4] __attribute__ ((aligned (32))) ;   // alignment of four 32-byte blocks; needed for four different TCDs
/*----------------------------------------------------------------*/

/*DECLARATIONS for Interrupt Service Routine (ISR)----------------*/
// The transfer of data from the buffer to the SD card is initiated via an interrupt service routine

uint32_t dma0_isr_counter = 0;                  // counter that will be incremented after a major loop completion (hardware)
uint32_t old_dma0_isr_counter = 0;              // counter that is compared to the dma0_isr_counter to register the hardware increment
uint32_t bufPtr = 0;                            // pointer to the buffer section in which the data is currently transferred
/*----------------------------------------------------------------*/

//  DMA interrupt service routines
/*----------------------------------------------------------------*/
void dma0_isr(void) {                           // method that deletes interrupt and increments a counter; method is later attached to the resepctive dma channel via  dma.attachInterrupt(dma0_isr);
  dma.clearInterrupt();
  dma0_isr_counter++;
  //Serial.println(".");
}
/*----------------------------------------------------------------*/

/*DECLARATIONS microSD card files---------------------------------*/
// if SPI mode is used for ext. SD card, uncomment the following two lines
//const uint8_t SD_CS_PIN = 10;                   // set cs pin to 10 for SPI communication with SD card (ext.)
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50))
#define SD_CONFIG SdioConfig(FIFO_SDIO)

SdFs sd;                                          // used to declare the sd.### object (Sdfat); do not use SdFatSdioEX sd
FsFile file;                                      // file object for logging data
FsFile meta;                                      // file object for metadata
String meta_name = ID + "_metadata.txt";
char mname[30];
/*----------------------------------------------------------------*/


// function creates new files for data logging
/*----------------------------------------------------------------*/
void filestuff() {
  DateTime now = rtc.now();
  String Date = String(now.year(), DEC)+"."+String(now.month(), DEC)+"."+String(now.day(), DEC)+"-"+String(now.hour(), DEC)+"."+String(now.minute(), DEC)+"."+String(now.second(), DEC);
  String filename = ID + "_c1_" + Date + ".bin";
  char fname[30];
  filename.toCharArray(fname, 30);
  Serial.print("filename: ");
  Serial.println(filename);

  if (!file.open(fname, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma0 file failed");
  }
}


void setup() 
{
  /*Serial monitor--------------------------------------------------*/
  debug_start = millis();
  Serial.begin(115200);
  while ((millis() - debug_start) <= 5000);
  Serial.println("Begin Setup\n");
  /*----------------------------------------------------------------*/
  
  /*RTC-------------------------------------------------------------*/
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();
  String Date = String(now.year(), DEC)+"."+String(now.month(), DEC)+"."+String(now.day(), DEC)+"-"+String(now.hour(), DEC)+"."+String(now.minute(), DEC)+"."+String(now.second(), DEC);
  /*----------------------------------------------------------------*/
  
  /*SD--------------------------------------------------------------*/
  if (!sd.begin(SD_CONFIG)) {
    sd.errorHalt("SD begin failed");
  }
  /*----------------------------------------------------------------*/
  
  /*Metadata file---------------------------------------------------*/
  meta_name.toCharArray(mname, 30);
  if (!meta.open(mname, O_RDWR | O_CREAT)) {                                  // create SD card files
    sd.errorHalt("open metadata file failed");
  }
  Serial.println("Writing metadata.txt file ... ");
  meta.print("Date-Time (Y.M.D.-H.M.S): ");
  meta.println(Date);
  meta.print("Experiment: ");
  meta.println(experiment_name);
  meta.print("Location: ");
  meta.println(location);
  meta.print("Experimenter: ");
  meta.println(experimenter);
  meta.print("ID: ");
  meta.println(ID);
  meta.print("Sampling frequency (Hz): ");
  meta.println(pdbfreq);
  meta.print("Seconds per file: ");
  meta.println(duration);
  meta.print("Amplifier gain: ");
  meta.println(amp_gain);
  meta.print("Highpass filter (Hz): ");
  meta.println(high_pass);
  meta.print("Lowpass filter (Hz): ");
  meta.println(low_pass);
  meta.print("Record type: ");
  if(single_ended){
    meta.println("Single ended recording");
  } else {
    meta.println("Differential recording");
  }
//  meta.flush(); TEST - unnecessary?
  meta.close();
  /*----------------------------------------------------------------*/
  
  /*FileSetup-------------------------------------------------------*/
  String filename = ID + "_c1_" + Date + ".bin";                        // create filenames
  char fname[30];
  filename.toCharArray(fname, 30);

  Serial.println(filename);
  
  if (!file.open(fname, O_RDWR | O_CREAT)) {                                  // create SD card files
    sd.errorHalt("open dma0 failed");
  }

  delay(100);
  /*----------------------------------------------------------------*/

  /*DurationSetup---------------------------------------------------*/
  bytes = ((duration*1000000)/(1000000/pdbfreq))* 2; 
  preceil = bytes/BUF_DIM;
  scale = ceil(preceil);                                                    // round up preceil value
  FILE_SIZE = (scale+2) * BUF_DIM;                                          // after writing FILE_SIZE uint16_t values to a file a new file is created
  /*----------------------------------------------------------------*/

  /*Mode Setup------------------------------------------------------*/
  pinMode(13, OUTPUT);                                                     // built-in LED is at PIN 13 in Teensy 3.5
  pinMode(diff_pin1, INPUT);
  pinMode(diff_pin2, INPUT);
  /*----------------------------------------------------------------*/

  /*ADC Setup-------------------------------------------------------*/
  adc->adc0->startSingleDifferential(diff_pin1,diff_pin2);

  adc->adc0->setAveraging       (                              1  );       // ADC configuration
  adc->adc0->setResolution      (                              16 );
  adc->adc0->setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );

  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);                         // set analog reference
  /*----------------------------------------------------------------*/

  /* DMA ----------------------------------------------------------*/
  dma.source                 (           ADC0_RA);                         // source is the ADC result register
  dma.transferSize           (                 2);                         // set 2, one uint16_t value are two bytes
  dma.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0);                         // DMAMUX alignes source to DMA channel
  /*----------------------------------------------------------------*/
  
  /*TCD-------------------------------------------------------------*/

  // configure TCD for first dma
  dma.TCD->CITER    =           32 * 512;
  dma.TCD->BITER    =           32 * 512;
  dma.TCD->DOFF     =                  2;                                  // set 2, one uint16_t value are two bytes
  dma.TCD->CSR      =               0x10;
  dma.TCD->CSR |= DMA_TCD_CSR_INTMAJOR;                                     // enable interrupt after major loop completion

  dma.TCD->DADDR        = (volatile void*) &buffer [ 0 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       1]  ;           // points to a 32-byte block that is loaded into the TCD memory of the DMA after major loop completion
  memcpy ( &tcd_mem[0], dma.TCD , 32 ) ;                                   // 32-byte block is transferred to &tcd_mem[0]

  dma.TCD->DADDR        = (volatile void*) &buffer [32 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       2]  ;
  memcpy ( &tcd_mem[1], dma.TCD , 32 ) ;

  dma.TCD->DADDR        = (volatile void*) &buffer [64 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       3]  ;
  memcpy ( &tcd_mem[2], dma.TCD , 32 ) ;

  dma.TCD->DADDR        = (volatile void*) &buffer [96 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       0]  ;
  memcpy ( &tcd_mem[3], dma.TCD , 32 )  ;

  memcpy ( dma.TCD ,  &tcd_mem[0], 32 ) ;                                  // 16-byte block that is transferred into the TCD memory of the DMA
  /*----------------------------------------------------------------*/

  /*Start DMA and ADC-----------------------------------------------*/
  dma.enable();                                                             // enable DMA
  dma.attachInterrupt(dma0_isr);                                            // attach interrupt that is done after major loop completion

  adc->adc0->enableDMA();                                                     // connect DMA and ADC
  adc->adc0->stopPDB();                                                      // start PDB conversion trigger
  adc->adc0->startPDB(pdbfreq);
  /*----------------------------------------------------------------*/


  /*Debug-----------------------------------------------------------*/
  Serial.print("Buffer dimension (bytes): ");
  Serial.println(BUF_DIM);
  Serial.println("----------------------------------");
  /*----------------------------------------------------------------*/

  /*Signal end of Setup method--------------------------------------*/
  for (int i = 0; i < 5; i++){                                             // visual feedback, blink 5 times if the setup was completed
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(300);
  }
  /*----------------------------------------------------------------*/
}


void loop() {  
  if (dma0_isr_counter != old_dma0_isr_counter){                              // check if buffer section can be written on microSD card
    if (BUF_DIM != (uint32_t)file.write( (char*)&buffer[((last/2)&(64*1024-1))], BUF_DIM) ){ 
      sd.errorHalt("write dma0 failed");    
      }
    last += BUF_DIM ;  
    old_dma0_isr_counter++;                                                  // increment counter so that it matches dma0_isr_counter
    if (dma0_isr_counter > old_dma0_isr_counter + 3){                        // check if data has been lost
      Serial.print("lost dma0 data");
      Serial.println( dma0_isr_counter - old_dma0_isr_counter);
    }
  } 
  /*----------------------------------------------------------------*/
  if ( last >= FILE_SIZE ) {                                              // check if end of file is reached
    file.close();
    last = 0;                                                             // reset last
    filestuff();                                                          // create new files for data logging
    digitalWrite(13, HIGH);
    delay(5000);
    digitalWrite(13, LOW);
  }
  /*----------------------------------------------------------------*/
}
