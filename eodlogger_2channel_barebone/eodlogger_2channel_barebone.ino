//*************************************************************************
// Two channel EOD logger programm for testing purposes
// Barebone: no RTC
//*************************************************************************

#include <ADC.h>              //library for easier ADC implementation
#include <DMAChannel.h>       //library for easier DMA implementation
#include <array>              //use C++ array structs
#include <SdFat.h>            // work with SD card


/*----------------------------------------------------------------*/
const uint32_t pdbfreq = 100000;  // sampling speed [Hz] max ~300MHz - actually the trigger frequency of the programmable delay block
uint32_t duration = 60;           // duration of each measure-cycle [s]
unsigned long pause = 0;          // pause-duration
String Name = "Log";              // file name prefix
unsigned long debug_start;
/*----------------------------------------------------------------*/

/*DECLARATIONS adc and pins---------------------------------------*/
ADC adc;            // used to declare the adc.### object

/*PINS------------------------------------------------------------*/
const uint8_t adc_pin0 = A2;                    // A4 is connected to ADC0
const uint8_t adc_pin1 = A22;                   // A16 is connected to ADC1
/*----------------------------------------------------------------*/

/*Buffer declarations----------------------------------------------*/
std::array<volatile uint16_t, (uint32_t)64 * 512> buffer __attribute__ ((aligned (16 * 1024)));      // size of buffer is limited due to the Teensy's program memory
std::array<volatile uint16_t, (uint32_t)64 * 512> buffer1 __attribute__ ((aligned (16 * 1024)));

uint32_t      BUF_DIM       = 32768 / 2;                                          //size of buffer that holds data from ADC
uint32_t      FILE_SIZE     = 0;                                                  //initial variables for filesize and pointer to...
uint32_t      last          = 0;     
uint32_t      bytes         = ((duration * 1000000) / (1000000 / pdbfreq)) * 2;
float         preceil       = bytes / BUF_DIM;
float         scale         = ceil(preceil);                                      // round up preceil value
const int     times_buffer  = (scale + 2);                                        // creation of a file based on duration
/*----------------------------------------------------------------*/

/*DECLARATIONS dma and tcd----------------------------------------*/
DMAChannel dma;                                 // used to declare the dma.### object for the first channel
DMAChannel dma1;                                // used to declare the dma.### object for the second channel

typeof(*dma.TCD)  tcd_mem[4] __attribute__ ((aligned (32))) ;   // alignment of four 32-byte blocks; needed for four different TCDs
typeof(*dma1.TCD)  tcd1_mem[4] __attribute__ ((aligned (32))) ;
/*----------------------------------------------------------------*/


/*DECLARATIONS microSD card files---------------------------------*/
SdFatSdio sd;                                    // used to declare the sd.### object (Sdfat); do not use SdFatSdioEX sd

uint16_t fileNr = 1;                             // after a given duration a new file is created; fileNr is an index used for the filename

File file;                                       // file object for logging data
File file1;                                      // file object for logging data
/*----------------------------------------------------------------*/


/*DECLARATIONS for Interrupt Service Routine (ISR)----------------*/
// The transfer of data from the buffer to the SD card is initiated via an interrupt service routine

uint32_t dma0_isr_counter = 0;                  // counter that will be incremented after a major loop completion (hardware)
uint32_t dma1_isr_counter = 0;

uint32_t old_dma0_isr_counter = 0;              // counter that is compared to the dma0_isr_counter to register the hardware increment
uint32_t old_dma1_isr_counter = 0;

uint32_t bufPtr = 0;                            // pointer to the buffer section in which the data is currently transferred
uint32_t bufPtr1 = 0;
/*----------------------------------------------------------------*/


//  DMA interrupt service routines
/*----------------------------------------------------------------*/
void dma0_isr(void) {                             // method that deletes interrupt and increments a counter; method is later attached to the resepctive dma channel via  dma.attachInterrupt(dma0_isr);
  dma.clearInterrupt();
  dma0_isr_counter++;
}

void dma1_isr(void) {
  dma1.clearInterrupt();
  dma1_isr_counter++;
}
/*----------------------------------------------------------------*/

/*PDB ISR---------------------------------------------------------*/
void pdb_isr(void) {                           // PDB interrupt routine
  PDB0_SC &= ~PDB_SC_PDBIF  ;                  // clear interrupt
  Serial.println(".");
}
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

  String filename1 = Name + "_dma1_" + fileNr + ".bin";

  filename1.toCharArray(fname, 30);
  Serial.print("filename: ");
  Serial.println(filename);
  if (!file1.open(fname, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma1 file failed");
  }
}

void setup() 
{
  /*Serial monitor--------------------------------------------------*/
  debug_start = millis();
  Serial.begin(115200);
  while (!Serial && ((millis() - debug_start) <= 5000));
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
       
  if (!  sd.begin()) {                                                        // start sdio interface to SD card
    sd.initErrorHalt("SdFatSdio   begin() failed");
  }    sd.chvol();
  if (!file.open(fname, O_RDWR | O_CREAT)) {                                  // create SD card files
    sd.errorHalt("open dma0 failed");
  }
  if (!file1.open(fname1, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma1 failed");
  }

  delay(100);
  /*----------------------------------------------------------------*/

  /*set Filesize-----------------------------------------------------*/
  FILE_SIZE = times_buffer * BUF_DIM;                                        // after writing FILE_SIZE uint16_t values to a file a new file is created
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

  /*Mode Setup------------------------------------------------------*/
  pinMode(13, OUTPUT);                                                     // built-in LED is at PIN 13 in Teensy 3.5
  pinMode                (                 adc_pin0, INPUT  );             // configure as analog input pins
  pinMode                 (                adc_pin1, INPUT  );
  /*----------------------------------------------------------------*/

  /*ADC Setup-------------------------------------------------------*/
  adc.startSingleRead(adc_pin0, ADC_0);                                    // start ADC conversion
  adc.startSingleRead(adc_pin1, ADC_1);


  adc.setAveraging       (                              1  );              // ADC configuration
  adc.setResolution      (                           16, 0  );
  adc.setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );


  adc.setAveraging (1, ADC_1);
  adc.setResolution (16, ADC_1);
  adc.setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_1);
  adc.setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_1  );

  adc.setReference(ADC_REFERENCE::REF_3V3, ADC_0);                         // set analog reference
  adc.setReference(ADC_REFERENCE::REF_3V3, ADC_1);
  /*----------------------------------------------------------------*/

  /* DMA ----------------------------------------------------------*/
  dma.source                 (           ADC0_RA);                         // source is the ADC result register
  dma.transferSize           (                 2);                         // set 2, one uint16_t value are two bytes
  dma.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0);                         // DMAMUX alignes source to DMA channel

  dma1.source                 (           ADC1_RA);
  dma1.transferSize           (                 2);
  dma1.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC1);
  /*----------------------------------------------------------------*/
  
/*Buffer setup ---------------------------*/

  /*TCD-------------------------------------------------------------*/

  // configure TCD for first dma
  dma.TCD->CITER    =           16 * 512;
  dma.TCD->BITER    =           16 * 512;
  dma.TCD->DOFF     =                  2;                                  // set 2, one uint16_t value are two bytes
  dma.TCD->CSR      =               0x10;

  dma.TCD->CSR |= DMA_TCD_CSR_INTMAJOR;                                    // enable interrupt after major loop completion

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

  memcpy ( dma.TCD ,  &tcd_mem[0], 32 ) ;                                  // 32-byte block that is transferred into the TCD memory of the DMA

  // equal configuration for TCD of  second dma1

  dma1.TCD->CITER    =           16 * 512;
  dma1.TCD->BITER    =           16 * 512;
  dma1.TCD->DOFF     =                  2;
  dma1.TCD->CSR      =               0x10;

  dma1.TCD->CSR |= DMA_TCD_CSR_INTMAJOR;

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

  dma.attachInterrupt(dma0_isr);                                            // attach interrupt that is done after major loop completion
  dma1.attachInterrupt(dma1_isr);

  adc.enableDMA(ADC_0);                                                     // connect DMA and ADC
  adc.enableDMA(ADC_1);

  adc.adc0->stopPDB();                                                      // start PDB conversion trigger
  adc.adc0->startPDB(pdbfreq);

  adc.adc1->stopPDB();
  adc.adc1->startPDB(pdbfreq);

  adc.adc0->printError();                                                  // print ADC configuration errors
  adc.adc1->printError();
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
  delay(20);
/*----------------------------------------------------------------*/
  if (dma0_isr_counter != old_dma0_isr_counter)                             // check if buffer section can be written on microSD card
  {
    if (BUF_DIM != (uint32_t)file.write( (char*)&buffer[bufPtr], BUF_DIM))  // write buffer section on SD card;
    {
      sd.errorHalt("write dma0 failed");
    } ;

    last += BUF_DIM ;                                                       // increment last to control for file end

    old_dma0_isr_counter++;                                                 // increment counter so that it matches dma0_isr_counter

    bufPtr = 0x7fff & (bufPtr + 16 * 512);                                  // choose next buffer section

    if (dma0_isr_counter > old_dma0_isr_counter + 3)                        // check if data has been lost
    {
      Serial.print("lost dma0 data");
      Serial.println( dma0_isr_counter - old_dma0_isr_counter);
    }
  }

  /*----------------------------------------------------------------*/     // equal routine for second buffer
  if (dma1_isr_counter != old_dma1_isr_counter)
  {
    if (BUF_DIM != (uint32_t)file1.write( (char*)&buffer1[bufPtr1], BUF_DIM) )
    {
      sd.errorHalt("write dma1 failed");
    } ;

    last += BUF_DIM ;

    old_dma1_isr_counter++;
    bufPtr1 = 0x7fff & (bufPtr1 + 16 * 512);

    if (dma1_isr_counter > old_dma1_isr_counter + 3)
    {
      Serial.print("lost dma1 data");
      Serial.println( dma1_isr_counter - old_dma1_isr_counter);
    }
  }


  /*----------------------------------------------------------------*/
  if ( last >= FILE_SIZE ) {                                              // check if end of file is reached
    file.close();
    file1.close();
    last = 0;                                                             // reset last
    filestuff();                                                          // create new files for data logging
  }
  /*----------------------------------------------------------------*/
}
