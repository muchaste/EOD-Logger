//*************************************************************************
// Eight channel EOD logger programm for testing purposes
// Barebone: no RTC
//*************************************************************************

#include <ADC.h>              //library for easier ADC implementation
#include <DMAChannel.h>       //library for easier DMA implementation
#include <array>              //use C++ array structs
#include <SdFat.h>            // work with SD card

#define BUF_SIZE 256          //for adcbuffer



/*----------------------------------------------------------------*/
const uint32_t pdbfreq = 100000;  // sampling speed [Hz] max ~300MHz - actually the trigger frequency of the programmable delay block
uint32_t duration = 60;           // duration of each measure-cycle [s]
String Name = "Log";              // file name prefix
unsigned long debug_start;

volatile int d2_active;
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE+0))) adcbuffer_0[BUF_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE+0))) adcbuffer_1[BUF_SIZE];
/*----------------------------------------------------------------*/

/*DECLARATIONS adc and pins---------------------------------------*/
  //TO DO: New Object declaration (done)
  // ADC adc;            // used to declare the adc.### object
  ADC *adc = new ADC(); // used to declare the adc object.

/*PINS------------------------------------------------------------*/
  // TO DO Rewrite Pins (done)
  //const uint8_t adc_pin0 = A2;                    // A2 is connected to ADC0
  //const uint8_t adc_pin1 = A22;                   // A22 is connected to ADC1
  // TO DO Current: sc1a numbers for the pins (done)
  // if it's not working hex numbers should be used 0xXX
  const uint8_t ChannelADC_0 []= {6, 7, 15, 4} ; // Pins A6, A7, A8, A9
  const uint8_t ChannelADC_1 []= {4, 5, 6, 7} ; // Pins A16, A17, A18, A19
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
DMAChannel* dma0 = new DMAChannel(false);                                 // used to declare the dma.### object for the first channel
DMAChannel* dma1 = new DMAChannel(false);                                // used to declare the dma.### object for the second channel
//TO DO: new dma channel (done)
DMAChannel* dma2 = new DMAChannel(false);
DMAChannel* dma3 = new DMAChannel(false);
//TO DO: Check TCD for new channels
//typeof(*dma.TCD)  tcd_mem[4] __attribute__ ((aligned (32))) ;   // alignment of four 32-byte blocks; needed for four different TCDs
//typeof(*dma1.TCD)  tcd1_mem[4] __attribute__ ((aligned (32))) ;
/*----------------------------------------------------------------*/


/*DECLARATIONS microSD card files---------------------------------*/
SdFatSdioEX sd;                                    // used to declare the sd.### object (Sdfat); do not use SdFatSdioEX sd

uint16_t fileNr = 0;                             // after a given duration a new file is created; fileNr is an index used for the filename
uint16_t fileNr1 = 0;

File file;                                       // file object for logging data
File file1;                                      // file object for logging data
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

  /*DurationSetup---------------------------------------------------*/
  bytes = ((duration*1000000)/(1000000/pdbfreq))* 2; 
  preceil = bytes/BUF_DIM;
  scale = ceil(preceil);                                                    // round up preceil value
  FILE_SIZE = (scale+2) * BUF_DIM;                                          // after writing FILE_SIZE uint16_t values to a file a new file is created
  /*----------------------------------------------------------------*/

  /*Mode Setup------------------------------------------------------*/
  //TO DO add new Pinmode setup
    pinMode(13, OUTPUT);                                                     // built-in LED is at PIN 13 in Teensy 3.5
  //pinMode(adc_pin0, INPUT);                                                // configure as analog input pins
  //pinMode(adc_pin1, INPUT);
  /*----------------------------------------------------------------*/

  /*ADC Setup-------------------------------------------------------*/
  // TO DO: implement startSynchronizedContinuous or   startSynchronizedSingleRead -> Check with SD library
  //adc.startSingleRead(adc_pin0, ADC_0);                                    // start ADC conversion
  //adc.startSingleRead(adc_pin1, ADC_1);
  //
  
  //TO DO: Upgrade to new ADC library standard. Example adc.setAveraging

  // ADC0:
  adc->adc0->setAveraging       (                              1  );              // ADC configuration
  adc->adc0->setResolution      (                             12  );
  adc->adc0->setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );

  //ADC1:
  adc->adc1->setAveraging       (                              1  );
  adc->adc1->setResolution      (                             12  );
  adc->adc1->setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc1->setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );

  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);                         // set analog reference
  adc->adc1->setReference(ADC_REFERENCE::REF_3V3);

  ADC1_CFG2 |= ADC_CFG2_MUXSEL;                                             //Channel via MUX selection
  /*----------------------------------------------------------------*/

  /* DMA ----------------------------------------------------------*/
  // TO DO: Add new DMA Channel
  //dma.source                 (           ADC0_RA);                         // source is the ADC result register
  //dma.transferSize           (                 2);                         // set 2, one uint16_t value are two bytes
  dma0->begin                  (true              );
  dma0->triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0);                         // DMAMUX alignes source to DMA channel
  dma0->disableOnCompletion    (                  );
  dma0->interruptAtCompletion  (                  );
  dma0->attachInterrupt        (dma0_isr          );

  dma1->begin                  (true              );
  dma1->triggerAtTransfersOf   (*dma0             );
  dma1->triggerAtCompletionOf  (*dma0             );

  dma2->begin                  (true              );
  dma2->triggerAtHardwareEvent (DMAMUX_SOURCE_ADC1);
  dma2->disableOnCompletion    (                  );
  dma2->interruptAtCompletion  (                  );
  dma2->attachInterrupt        (dma2_isr          );
  
  dma3->begin                  (true              );
  dma3->triggerAtTransfersOf   (*dma2             );
  dma3->triggerAtCompletionOf  (*dma2             );
  /*----------------------------------------------------------------*/
  
  /*TCD-------------------------------------------------------------*/
  /*
  //TO DO: ADD new DMA
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
  */
  dma0->TCD->SADDR = &ADC0_RA;    // where to read from
  dma0->TCD->SOFF = 0;            // source increment each transfer
  dma0->TCD->ATTR = 0x101;
  dma0->TCD->NBYTES = 2;     // bytes per transfer
  dma0->TCD->SLAST = 0;
  dma0->TCD->DADDR = &adcbuffer_0[0];// where to write to
  dma0->TCD->DOFF = 2; 
  dma0->TCD->DLASTSGA = -2*BUF_SIZE;
  dma0->TCD->BITER = BUF_SIZE;
  dma0->TCD->CITER = BUF_SIZE;

  dma1->TCD->SADDR = &ChannelADC_0[0];
  dma1->TCD->SOFF = 2;            // source increment each transfer (n bytes)
  dma1->TCD->ATTR = 0x101;
  dma1->TCD->SLAST = -8;          // num ADC0 samples * 2
  dma1->TCD->BITER = 4;           // num of ADC0 samples
  dma1->TCD->CITER = 4;           // num of ADC0 samples
  dma1->TCD->DADDR = &ADC0_SC1A;
  dma1->TCD->DLASTSGA = 0;
  dma1->TCD->NBYTES = 2;
  dma1->TCD->DOFF = 0;

  dma2->TCD->SADDR = &ADC1_RA;    // where to read from
  dma2->TCD->SOFF = 0;            // source increment each transfer
  dma2->TCD->ATTR = 0x101;
  dma2->TCD->NBYTES = 2;     // bytes per transfer
  dma2->TCD->SLAST = 0;
  dma2->TCD->DADDR = &adcbuffer_1[0];// where to write to
  dma2->TCD->DOFF = 2; 
  dma2->TCD->DLASTSGA = -2*BUF_SIZE;
  dma2->TCD->BITER = BUF_SIZE;
  dma2->TCD->CITER = BUF_SIZE;  

  dma3->TCD->SADDR = &ChannelADC_1[0];
  dma3->TCD->SOFF = 2;            // source increment each transfer (n bytes)
  dma3->TCD->ATTR = 0x101;
  dma3->TCD->SLAST = -8;          // num ADC1 samples * 2
  dma3->TCD->BITER = 4;           // num of ADC1 samples
  dma3->TCD->CITER = 4;           // num of ADC1 samples
  dma3->TCD->DADDR = &ADC1_SC1A;
  dma3->TCD->DLASTSGA = 0;
  dma3->TCD->NBYTES = 2;
  dma3->TCD->DOFF = 0;

  
  /*----------------------------------------------------------------*/

  /*Start DMA and ADC-----------------------------------------------*/
  //enable DMA
  //TO DO: ADD new DMA Channel
  dma0->enable(); 
  dma1->enable();
  
  dma2->enable();
  dma3->enable();
  // TO DO : switch adc. to adc -> (done)
  adc->adc0->enableDMA();                                                     // connect DMA and ADC
  adc->adc1->enableDMA();

  adc->adc0->stopPDB();                                                      // start PDB conversion trigger
  adc->adc0->startPDB(pdbfreq);

  adc->adc1->stopPDB();
  adc->adc1->startPDB(pdbfreq);

  NVIC_DISABLE_IRQ(IRQ_PDB);                                                // we don't want or need the PDB interrupt

  //TO DO: Implement printerror();
  //adc.adc0->printError();                                                  // print ADC configuration errors
  //adc.adc1->printError();
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
  }
  /*----------------------------------------------------------------*/
}

void d2_isr(void) {
  if(debounce > 200){
    d2_active = 1;
    debounce = 0;
    }
    else{return;}
}

void dma0_isr(void) {
    dma0->TCD->DADDR = &adcbuffer_0[0];
    dma0->clearInterrupt();
    dma0->enable();
    digitalWriteFast(4, HIGH);
    digitalWriteFast(4, LOW);
}

void dma2_isr(void) {
    dma2->TCD->DADDR = &adcbuffer_1[0];
    dma2->clearInterrupt();
    dma2->enable();
    digitalWriteFast(6, HIGH);
    digitalWriteFast(6, LOW);
}
