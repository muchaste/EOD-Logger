//*************************************************************************
// Two channel EOD logger programm for testing purposes
// Barebone: no RTC
//*************************************************************************

#include <ADC.h>              //library for easier ADC implementation
#include <DMAChannel.h>       //library for easier DMA implementation
#include <array>              //use C++ array structs
#include <SdFat.h>            // work with SD card

#define SD_FAT_TYPE 3

/*----------------------------------------------------------------*/
const uint32_t pdbfreq = 100000;  // sampling speed [Hz] max ~300MHz - actually the trigger frequency of the programmable delay block
uint32_t duration = 10;           // duration of each measure-cycle [s]
String Name = "Log";              // file name prefix
unsigned long debug_start;
/*----------------------------------------------------------------*/

/*DECLARATIONS adc and pins---------------------------------------*/
ADC *adc = new ADC();                           // Declare adc object. Update: adc as pointer, use adc->adcX

/*PINS------------------------------------------------------------*/
const uint8_t adc_pin0 = A2;                    // A2 is connected to ADC0
const uint8_t adc_pin1 = A22;                   // A22 is connected to ADC1
/*----------------------------------------------------------------*/

/*Buffer declarations----------------------------------------------*/
std::array<volatile uint16_t, (uint32_t)64 * 512> buffer __attribute__ ((aligned (16 * 1024)));      // size of buffer is limited due to the Teensy's program memory
std::array<volatile uint16_t, (uint32_t)64 * 512> buffer1 __attribute__ ((aligned (16 * 1024)));

uint32_t      BUF_DIM       = 32768/2;                                          //size of buffer that holds data from ADC
uint32_t      FILE_SIZE     = 0;                                                  //initial variables for filesize and pointer to...
uint32_t      last          = 0;     
uint32_t      last1         = 0;     
uint32_t      bytes         = 0;
float         preceil       = 0;
float         scale         = 0;
/*----------------------------------------------------------------*/

/*DECLARATIONS dma and tcd----------------------------------------*/
DMAChannel dma;                                 // used to declare the dma.### object for the first channel
DMAChannel dma1;                                // used to declare the dma.### object for the second channel

typeof(*dma.TCD)  tcd_mem[4] __attribute__ ((aligned (32))) ;   // alignment of four 32-byte blocks; needed for four different TCDs
typeof(*dma1.TCD)  tcd1_mem[4] __attribute__ ((aligned (32))) ;
/*----------------------------------------------------------------*/


/*DECLARATIONS microSD card files---------------------------------*/
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

SdFs sd;                                    // used to declare the sd.### object (Sdfat); do not use SdFatSdioEX sd

uint16_t fileNr = 0;                             // after a given duration a new file is created; fileNr is an index used for the filename
uint16_t fileNr1 = 0;

FsFile file;                                       // file object for logging data
FsFile file1;                                      // file object for logging data
/*----------------------------------------------------------------*/


// function creates new files for data logging
/*----------------------------------------------------------------*/
void filestuff() {
  fileNr++;
  String filename = Name + "_dma0_" + fileNr + ".bin";
  char fname[30];
  filename.toCharArray(fname, 30);
  Serial.print("filename: ");
  Serial.println(filename);

  if (!file.open(fname, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma0 file failed");
  }
}

void filestuff1() {
  fileNr1++;
  String filename1 = Name + "_dma1_" + fileNr1 + ".bin";
  char fname1[30];
  filename1.toCharArray(fname1, 30);
  Serial.print("filename: ");
  Serial.println(filename1);
  if (!file1.open(fname1, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma1 file failed");
  }
}

void setup() 
{
  /*Serial monitor--------------------------------------------------*/
  debug_start = millis();
  Serial.begin(115200);
  while ((millis() - debug_start) <= 5000);
  //while (!Serial && ((millis() - debug_start) <= 5000));
  Serial.println("Begin Setup\n");
  /*----------------------------------------------------------------*/

  /*FileSetup-------------------------------------------------------*/
  String filename = Name + "_dma0_" + fileNr + ".bin";                        // create filenames
  char fname[30];
  filename.toCharArray(fname, 30);

  String filename1 = Name + "_dma1_" + fileNr + ".bin";
  char fname1[30];
  filename1.toCharArray(fname1, 30);

  Serial.println(filename);
  Serial.println(filename1);
       
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50)))) {     // Update: initiate SD card with dedicated SPI config, CS pin
    sd.errorHalt("begin failed");
  }
  if (!file.open(fname, O_RDWR | O_CREAT)) {                                  // create SD card files
    sd.errorHalt("open dma0 failed");
  }
  if (!file1.open(fname1, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma1 failed");
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
  pinMode(adc_pin0, INPUT);                                                // configure as analog input pins
  pinMode(adc_pin1, INPUT);
  /*----------------------------------------------------------------*/

  /*ADC Setup-------------------------------------------------------*/
  adc->adc0->startSingleRead(adc_pin0);                                    // start ADC conversion (updated to pointer object)
  adc->adc1->startSingleRead(adc_pin1);

  adc->adc0->setAveraging       (                              1  );       // ADC configuration
  adc->adc0->setResolution      (                              16 );
  adc->adc0->setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );

  adc->adc1->setAveraging       (                                1);
  adc->adc1->setResolution      (                               16);
  adc->adc1->setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc1->setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );

  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);                         // set analog reference
  adc->adc1->setReference(ADC_REFERENCE::REF_3V3);
  /*----------------------------------------------------------------*/

  /* DMA ----------------------------------------------------------*/
  dma.source                 (           ADC0_RA);                         // source is the ADC result register
  dma.transferSize           (                 2);                         // set 2, one uint16_t value are two bytes
  dma.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0);                         // DMAMUX alignes source to DMA channel

  dma1.source                 (           ADC1_RA);
  dma1.transferSize           (                 2);
  dma1.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC1);
  /*----------------------------------------------------------------*/
  
  /*TCD-------------------------------------------------------------*/

  // configure TCD for first dma
  dma.TCD->CITER    =           16 * 512;
  dma.TCD->BITER    =           16 * 512;
  dma.TCD->DOFF     =                  2;                                  // set 2, one uint16_t value are two bytes
  dma.TCD->CSR      =               0x10;

  dma.TCD->DADDR        = (volatile void*) &buffer [ 0 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       1]  ;           // points to a 32-byte block that is loaded into the TCD memory of the DMA after major loop completion
  memcpy ( &tcd_mem[0], dma.TCD , 32 ) ;                                   // 32-byte block is transferred to &tcd_mem[0]

  dma.TCD->DADDR        = (volatile void*) &buffer [16 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       2]  ;
  memcpy ( &tcd_mem[1], dma.TCD , 32 ) ;

  dma.TCD->DADDR        = (volatile void*) &buffer [32 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       3]  ;
  memcpy ( &tcd_mem[2], dma.TCD , 32 ) ;

  dma.TCD->DADDR        = (volatile void*) &buffer [48 * 512]  ;
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[       0]  ;
  memcpy ( &tcd_mem[3], dma.TCD , 32 )  ;

  memcpy ( dma.TCD ,  &tcd_mem[0], 32 ) ;                                  // 16-byte block that is transferred into the TCD memory of the DMA

  // equal configuration for TCD of  second dma1

  dma1.TCD->CITER    =           16 * 512;
  dma1.TCD->BITER    =           16 * 512;
  dma1.TCD->DOFF     =                  2;
  dma1.TCD->CSR      =               0x10;

  dma1.TCD->DADDR        = (volatile void*) &buffer1 [ 0 * 512]    ;
  dma1.TCD->DLASTSGA     = (   int32_t    ) &tcd1_mem[       1]    ;
  memcpy ( &tcd1_mem[0], dma1.TCD , 32 ) ;

  dma1.TCD->DADDR        = (volatile void*) &buffer1 [16 * 512]    ;
  dma1.TCD->DLASTSGA     = (   int32_t    ) &tcd1_mem[       2]    ;
  memcpy ( &tcd1_mem[1], dma1.TCD , 32 ) ;

  dma1.TCD->DADDR        = (volatile void*) &buffer1 [32 * 512]    ;
  dma1.TCD->DLASTSGA     = (   int32_t    ) &tcd1_mem[       3]    ;
  memcpy ( &tcd1_mem[2], dma1.TCD , 32 ) ;

  dma1.TCD->DADDR        = (volatile void*) &buffer1 [48 * 512]    ;
  dma1.TCD->DLASTSGA     = (   int32_t    ) &tcd1_mem[       0]    ;
  memcpy ( &tcd1_mem[3], dma1.TCD , 32 )  ;

  memcpy ( dma1.TCD ,  &tcd1_mem[0], 32 ) ;
  /*----------------------------------------------------------------*/

  /*Start DMA and ADC-----------------------------------------------*/
  dma.enable();                                                             // enable DMA
  dma1.enable();

  adc->adc0->enableDMA();                                                     // connect DMA and ADC
  adc->adc1->enableDMA();

  adc->adc0->stopPDB();                                                      // start PDB conversion trigger
  adc->adc0->startPDB(pdbfreq);

  adc->adc1->stopPDB();
  adc->adc1->startPDB(pdbfreq);

  NVIC_DISABLE_IRQ(IRQ_PDB);                                                // we don't want or need the PDB interrupt

//  adc->adc0->printError();                                                  // print ADC configuration errors
//  adc->adc1->printError();
  /*----------------------------------------------------------------*/


  /*Debug-----------------------------------------------------------*/
  Serial.println(BUF_DIM);
  Serial.println(FILE_SIZE);
  Serial.print("bytes: ");
  Serial.println(bytes);
  Serial.println((uint32_t)&buffer[ 0], HEX);                               // debug: print memory location of buffer
  Serial.println((uint32_t)&buffer[ 16 * 512], HEX);
  Serial.println((uint32_t)&buffer[ 32 * 512], HEX);
  Serial.println((uint32_t)&buffer[ 48 * 512], HEX);
  Serial.println((uint32_t)&buffer1[ 0], HEX);
  Serial.println((uint32_t)&buffer1[ 16 * 512], HEX);
  Serial.println((uint32_t)&buffer1[ 32 * 512], HEX);
  Serial.println((uint32_t)&buffer1[ 48 * 512], HEX);
  Serial.println("----------------------------------");
  /*----------------------------------------------------------------*/

  /*Signal end of Setup method--------------------------------------*/
  for (int i = 0; i < 5; i++){                                             // visual feedback, blink 5 times if the setup was completed
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(300);
  }
}


void loop() {  
  while ( ((64*1024-1) & ( (int)dma.TCD->DADDR - last )) > BUF_DIM ){  
    if (BUF_DIM != (uint32_t)file.write( (char*)&buffer[((last/2)&(32*1024-1))], BUF_DIM) ){ 
      sd.errorHalt("write dma0 failed");    
      }
    last += BUF_DIM ;  
    
    if (BUF_DIM != (uint32_t)file1.write( (char*)&buffer1[((last1/2)&(32*1024-1))], BUF_DIM) ){ 
      sd.errorHalt("write dma1 failed");
      }
    last1 += BUF_DIM ;
  } 
  /*----------------------------------------------------------------*/
  if ( last >= FILE_SIZE ) {                                              // check if end of file is reached
    file.close();
    last = 0;                                                             // reset last
    filestuff();                                                          // create new files for data logging
  }
  if ( last1 >= FILE_SIZE ) {                                              // check if end of file is reached
    file1.close();
    last1 = 0;                                                             // reset last
    filestuff1();                                                          // create new files for data logging
     // blink
    digitalWrite(13, HIGH);
    delay(5000);
    digitalWrite(13, LOW);
  }
  /*----------------------------------------------------------------*/
}
