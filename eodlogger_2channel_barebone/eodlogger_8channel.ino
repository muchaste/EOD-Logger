//*************************************************************************
// 8 Channel EODLogger
// 
//*************************************************************************

#include <ADC.h>              //library for easier ADC implementation
#include <DMAChannel.h>       //library for easier DMA implementation
#include <array>              //use C++ array structs
#include <SdFat.h>            // work with SD card
#define SD_FAT_TYPE 3
#include <TimeLib.h>          // modified DateTime library for timekeeping functionality
#define TIME_HEADER  "T"      // header tag used to sync time of Teensy's RTC with the computer; Type "T" 1357041600 ( e.g. Jan 1 2013) into the input line of the Serial Monitor 
#define BUF_DIM  16384        //size of buffer that holds data from ADC, size = samples 
#define Major_size 1024       // Major Loop size: 512 Samples * 2 bytes

elapsedMillis TEST;

/*DECLARATIONS ADC and Pins---------------------------------------*/
ADC *adc = new ADC();                                             // used to declare the adc->### object
uint16_t ChannelsCfg_0 [] =  { 0x46, 0x47, 0x4F, 0x44 };          //ADC0: A6, A7, A8, A9 Which pins are used for measurement on ADC0
uint16_t ChannelsCfg_1 [] =  { 0x44, 0x45, 0x46, 0x47 };          //ADC1: A16, A17, A18, A19
const uint16_t ChannelPinNumber0 = 4;                             //Needed for reordering
const uint16_t ChannelPinNumber1 = 4;                             //Both need to be equal, because PDB gets intialized simultanously for ADC0 and ADC1

const uint32_t pdbfreq = 80000;                                   

volatile int buffer_dma0_position = 0;                             //Pointer position in DMA Buffer, used with partition to determine where to write
volatile int buffer_dma1_position = 0;
volatile int amount_SD_writes = 8;                                //How often do you want to write
volatile int partition = BUF_DIM / amount_SD_writes;
volatile int partition1 = BUF_DIM / amount_SD_writes;
volatile int partition_sample_amount = BUF_DIM / amount_SD_writes;

/*WAV-Header structure---------------------------------------------*/
struct fileheader {
  char  mainChunkId[4];                 /* "RIFF"                                                 */
  uint32_t  mainChunkSize;              /* file length in bytes                                   */
  char  mainChunkFormat[4];             /* "WAVE"                                                 */
  char  fmtChunkId[4];                  /* "fmt "                                                 */
  uint32_t  fmtChunkSize;               /* size of FMT chunk in bytes (usually 16 for PCM)        */
  uint16_t format_tag;                  /* 1=PCM, 257=Mu-Law, 258=A-Law, 259=ADPCM                */
  uint16_t num_chans;                   /* Number of channels/pins used                           */
  uint32_t  sample_rate;                /* Sampling rate in samples per second                    */
  uint32_t  byteRate;                   /* Byte rate = SampleRate * NumChannels * BitsPerSample/8 */
  uint16_t blockAlign;                  /* 2=16-bit mono, 4=16-bit stereo                         */
  uint16_t bits_per_samp;               /* Number of bits per sample                              */
  char  SubtwoChunkId[4];               /* "data"                                                 */
  uint32_t  SubtwoChunkSize;            /* data length in bytes (filelength - 44)                 */
} wavheader;

/*DECLARATIONS SD-card and File-----------------------------------*/

/*DECLARATIONS SD-card and File-----------------------------------*/

SdFs sd;                                          // used to declare the sd.### object (Sdfat); do not use SdFatSdioEX sd

uint16_t fileNr = 0;                              // after a given duration a new file is created; fileNr is an index used for the filename
String Name = "EODLog_aa";                        // filename prefix

FsFile file;                                      // file object for logging data

uint32_t      FILE_SIZE       = 0;                // filesize DMA-Buffer*times_buffer
uint32_t      last            = 0;                // coutner uint16_t values that are written to SD card
const int     times_buffer    = 10 * 2;               // creation of a file based on a multiple of the buffer size

uint32_t dma0_isr_counter = 0;                    // counter that will be incremented after a major loop completion (hardware)
uint32_t old_dma0_isr_counter = 0;                // counter that is compared to the dma0_isr_counter to register the hardware increment
uint32_t dma1_isr_counter = 0;
uint32_t old_dma1_isr_counter = 0;

uint32_t bufPtr = 0;                              // pointer to the buffer section in which the data is currently transferred
uint32_t bufPtr1 = 0;

uint16_t numChannels = 8;                         // Amount of channels i.e. number of pins used to measure

// SDCARD_SS_PIN is defined for the built-in SD on some boards. Definition of the pin used for the SD-card
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

/*Buffer declarations and DMA-------------------------------------*/

DMAMEM static volatile uint16_t __attribute__((aligned(BUF_DIM))) buffer[BUF_DIM];    // size of buffer is limited due to the Teensy's program memory
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_DIM))) buffer1[BUF_DIM];
unsigned long debug_start;

DMAChannel dma0;                                                                       // used to declare the dma.### object for the first channel
DMAChannel dma0_switch;                                                                //DMA channel linked to DMA to switch pins
DMAChannel dma1;
DMAChannel dma1_switch;

/*----------------------------------------------------------------*/

void setup() 
{
 /*WAV-Header-----------------------------------------------------*/ 
  FILE_SIZE = times_buffer * BUF_DIM;                                                 // after writing FILE_SIZE uint16_t values to a file a new file is created. *2 because we use 2 DMA channels
  setupWAVHeader();
/*Reorder---------------------------------------------------------*/                  // Reorders Pin arrays to match order in WAV-FILE
  reorder(ChannelsCfg_0, ChannelPinNumber0);
  reorder(ChannelsCfg_1, ChannelPinNumber1);
/*Serial monitor--------------------------------------------------*/
  debug_start = millis();
  Serial.begin(115200);
  while (!Serial && ((millis() - debug_start) <= 5000));
  Serial.println("Begin Setup\n");
/*Time and File setup---------------------------------------------*/
  setupRTC();                  
  setupFile();
/*LED pin---------------------------------------------------------*/
  pinMode(13, OUTPUT);                                                                // built-in LED is at PIN 13 in Teensy 3.5
  
  for (int j = 0; j < BUF_DIM; ++j){                                                  //Set values of the DMA buffer to a default value.
      buffer[j] = 30000;}
  for(int k = 0; k < BUF_DIM; ++k){
    buffer1[k] = 30000;}
/*DMA ADC Setup---------------------------------------------------*/ 
  setupADC();
  startPDB(); 
  setupDMA(); 
/*Debug-----------------------------------------------------------*/
  Serial.print("DMA Buffer sample size: ");
  Serial.println(BUF_DIM);                                                            //Prints Amount of samples per DMA
  Serial.print("File sample size: ");
  Serial.println(FILE_SIZE);
  Serial.println("End of setup");
  Serial.println("----------------------------------");
/*----------------------------------------------------------------*/
}
void loop() { 
  if (dma0_isr_counter != old_dma0_isr_counter && dma1_isr_counter != old_dma1_isr_counter)                              // check if buffer section can be written on microSD card
  {
    
    conversionWrite();                                                      // converts unsigned to signed integer and writes it in the file 

    if (dma0_isr_counter > old_dma0_isr_counter + (amount_SD_writes-1))     // check if data has been lost
    {
      Serial.print("lost dma0 data");
      Serial.println( dma0_isr_counter - old_dma0_isr_counter);
    }

    if (dma1_isr_counter > old_dma1_isr_counter + (amount_SD_writes-1))     // check if data has been lost
    {
      Serial.print("lost dma1 data");
      Serial.println( dma0_isr_counter - old_dma0_isr_counter);
    }
    
  }

  if ( last >= FILE_SIZE ) {                                                // check if end of file is reached
    file.close();
    last = 0;                                                               // reset last
    createNewFile();                                                        // create new files for data logging
  }
}
/*----------------------------------------------------------------*/

/*Functions-------------------------------------------------------*/

void setupADC() {

  adc->adc0->setResolution      (                  16  );
  adc->adc0->setReference       (ADC_REFERENCE::REF_3V3);
  adc->adc0->enableDMA();                                                   // connect DMA and ADC
  adc->adc0->setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);      
  adc->adc0->setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );
  adc->adc0->stopPDB();                                                     // start PDB conversion trigger


  adc->adc1->setResolution      (                  16  );
  adc->adc1->setReference       (ADC_REFERENCE::REF_3V3);
  adc->adc1->enableDMA();                                                   // connect DMA and ADC
  adc->adc1->setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);      
  adc->adc1->setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  );
  adc->adc1->stopPDB();                                                     // start PDB conversion trigger   
                                                             
}

void setupDMA(){
  //Beware order important i.e. transferSize can not be used everywhere, needs to be after source and destination declaration

  dma0.begin(true);
  dma0.source(ADC0_RA);
  dma0.destinationBuffer(&buffer[0], Major_size);                            //Major Size should be changed if less than 512 samples are used
  dma0.transferSize(2);
  dma0.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0); 
  dma0.disableOnCompletion();
  dma0.interruptAtCompletion();
  dma0.attachInterrupt(dma0ISR); 
 
  dma0_switch.sourceBuffer(&ChannelsCfg_0[0], sizeof(ChannelsCfg_0));
  dma0_switch.destination(ADC0_SC1A);
  dma0_switch.transferSize(2);
  dma0_switch.triggerAtTransfersOf(dma0);
  dma0_switch.triggerAtCompletionOf(dma0);

  dma1.begin(true);
  dma1.source(ADC1_RA);
  dma1.destinationBuffer(&buffer1[0], Major_size);
  dma1.transferSize(2);
  dma1.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC1); 
  dma1.disableOnCompletion();
  dma1.interruptAtCompletion();
  dma1.attachInterrupt(dma1ISR); 
 
  dma1_switch.sourceBuffer(&ChannelsCfg_1[0], sizeof(ChannelsCfg_1));
  dma1_switch.destination(ADC1_SC1A);
  dma1_switch.transferSize(2);
  dma1_switch.triggerAtTransfersOf(dma1);
  dma1_switch.triggerAtCompletionOf(dma1);

  dma0.enable();                                                         
  dma0_switch.enable();

  dma1.enable();                                                         
  dma1_switch.enable();
}



void dma0ISR() {                                                             // method that deletes interrupt and increments a counter; method is later attached to the resepctive dma channel via  dma.attachInterrupt(dma0_isr);
    buffer_dma0_position = buffer_dma0_position + 512;
    if(buffer_dma0_position == partition){
      dma0_isr_counter++;
      partition = partition + partition_sample_amount;
      if(partition == BUF_DIM + partition_sample_amount){
        partition = BUF_DIM / amount_SD_writes;
      }
    }
    if(buffer_dma0_position == BUF_DIM){
      dma0.TCD->DADDR = &buffer[0];
      buffer_dma0_position = 0;
    }
    else{
    dma0.TCD->DADDR = &buffer[buffer_dma0_position];
    }
    dma0.clearInterrupt();
    dma0.enable();
}

void dma1ISR() {                                                             // method that deletes interrupt and increments a counter; method is later attached to the resepctive dma channel via  dma.attachInterrupt(dma0_isr);
    buffer_dma1_position = buffer_dma1_position + 512;
    if(buffer_dma1_position == partition1){
      dma1_isr_counter++;
      partition1 = partition1 + partition_sample_amount;
      if(partition1 == BUF_DIM + partition_sample_amount){
        partition1 = BUF_DIM / amount_SD_writes;
      }
    }
    if(buffer_dma1_position == BUF_DIM){
      dma1.TCD->DADDR = &buffer1[0];
      buffer_dma1_position = 0;
    }
    else{
    dma1.TCD->DADDR = &buffer1[buffer_dma1_position];
    }
    dma1.clearInterrupt();
    dma1.enable();
}

void setupRTC(){
    setSyncProvider(getTeensy3Time);                                           //  set RTC of Teensy's 3.x
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600;                               // default Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) {                                              // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L;                                                             // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

void digitalClockDisplay() {
  Serial.print(hour());
  Serial.print(minute());
  Serial.print(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

 void setupFile(){
  
  String Date = String(year()) + "." + String(month()) + "." + String(day()) + "-" + String(hour()) + "." + String(minute()) + "." + String(second());
  String filename = Date + "_dma0_" + fileNr + ".wav";                                // create filenames
  char fname[30];
  filename.toCharArray(fname, 30);

  Serial.println(filename);

  if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50)))) {            // start sdio interface to SD card
    sd.initErrorHalt("SdFatSdio   begin() failed");
  }    
  sd.chvol();
  if (!file.open(fname, O_RDWR | O_CREAT)) {                                         // create SD card files
    sd.errorHalt("open dma0 failed");
  }

  file.write ((byte *)&wavheader,44);
  
 }

 void createNewFile() {
  String Date = String(year()) + "." + String(month()) + "." + String(day()) + "-" + String(hour()) + "." + String(minute()) + "." + String(second());
  fileNr++;
  String filename = Date + "_dma0_" + fileNr + ".wav";                               // create filenames
  char fname[30];
  filename.toCharArray(fname, 30);
  Serial.print("filename: ");
  Serial.print(filename);
  Serial.print("    ");
  Serial.println(TEST);
  
  if (!file.open(fname, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma0 file failed");
  }

  file.write ((byte *)&wavheader,44);
}

void setupWAVHeader(){
  uint16_t resolution0 = adc->adc0->getResolution();
  uint32_t SubtwoChunkSizeCalc = (FILE_SIZE*2)  * (resolution0/8);                                                           // =NumSamples * NumChannels * BitsPerSample/8.
  
  //RIFF chunk descriptor
  char riff[4] = {'R', 'I', 'F', 'F'};
  strncpy(wavheader.mainChunkId,riff,4);
  wavheader.mainChunkSize = 36 + SubtwoChunkSizeCalc;                                                                        // Size of the entire File -8 bytes for the two fields not included in this count (ChunkID and ChunkSize) 
  char wav[4] = {'W', 'A', 'V', 'E'};
  strncpy(wavheader.mainChunkFormat,wav,4);

  //Subchunk1 --> fmt sub-chunk

  char fmt[4] = {'f', 'm', 't', ' '};
  strncpy(wavheader.fmtChunkId,fmt,4);
  wavheader.fmtChunkSize = 16;                                                                                               //16 for PCM
  wavheader.format_tag = 1;                                                                                                  // 1 is PCM (Pulse-code modulation used for sampled analog signals)

  //Subchunk1 sound attributes
 
  wavheader.num_chans = numChannels;
  wavheader.sample_rate = pdbfreq / ChannelPinNumber0;
  wavheader.byteRate = pdbfreq / ChannelPinNumber0 * (resolution0/8) * numChannels;                                                                      
  wavheader.blockAlign = numChannels * (resolution0/8);
  wavheader.bits_per_samp = resolution0; 
  //ExtraParamSize = ... doesn't exist when using PCM

  //Subchunk2 contains size of data and actual sound:

  char data[4] = {'d', 'a', 't', 'a'};
  strncpy(wavheader.SubtwoChunkId,data,4);
  wavheader.SubtwoChunkSize = SubtwoChunkSizeCalc;
  
   //This header makes a total of 44 bytes in size
   //for more information visit http://soundfile.sapp.org/doc/WaveFormat/
   //it follows the actual data written in the loop 
  
}


void startPDB()
{
    if (!(SIM_SCGC6 & SIM_SCGC6_PDB))
    {                               // setup PDB
        SIM_SCGC6 |= SIM_SCGC6_PDB; // enable pdb clock
    }

    if (pdbfreq > ADC_F_BUS)
        return; // too high
    if (pdbfreq < 1)
        return; // too low

    // mod will have to be a 16 bit value
    // we detect if it's higher than 0xFFFF and scale it back accordingly.
    uint32_t mod = (ADC_F_BUS / pdbfreq);

    uint8_t prescaler = 0; // from 0 to 7: factor of 1, 2, 4, 8, 16, 32, 64 or 128
    uint8_t mult = 0;      // from 0 to 3, factor of 1, 10, 20 or 40

    // if mod is too high we need to use prescaler and mult to bring it down to a 16 bit number
    const uint32_t min_level = 0xFFFF;
    if (mod > min_level)
    {
        if (mod < 2 * min_level)
        {
            prescaler = 1;
        }
        else if (mod < 4 * min_level)
        {
            prescaler = 2;
        }
        else if (mod < 8 * min_level)
        {
            prescaler = 3;
        }
        else if (mod < 10 * min_level)
        {
            mult = 1;
        }
        else if (mod < 16 * min_level)
        {
            prescaler = 4;
        }
        else if (mod < 20 * min_level)
        {
            mult = 2;
        }
        else if (mod < 32 * min_level)
        {
            prescaler = 5;
        }
        else if (mod < 40 * min_level)
        {
            mult = 3;
        }
        else if (mod < 64 * min_level)
        {
            prescaler = 6;
        }
        else if (mod < 128 * min_level)
        {
            prescaler = 7;
        }
        else if (mod < 160 * min_level)
        { // 16*10
            prescaler = 4;
            mult = 1;
        }
        else if (mod < 320 * min_level)
        { // 16*20
            prescaler = 4;
            mult = 2;
        }
        else if (mod < 640 * min_level)
        { // 16*40
            prescaler = 4;
            mult = 3;
        }
        else if (mod < 1280 * min_level)
        { // 32*40
            prescaler = 5;
            mult = 3;
        }
        else if (mod < 2560 * min_level)
        { // 64*40
            prescaler = 6;
            mult = 3;
        }
        else if (mod < 5120 * min_level)
        { // 128*40
            prescaler = 7;
            mult = 3;
        }
        else
        { // frequency too low
            return;
        }

        mod >>= prescaler;
        if (mult > 0)
        {
            mod /= 10;
            mod >>= (mult - 1);
        }
    }

    adc->adc0->setHardwareTrigger();
    adc->adc1->setHardwareTrigger(); // trigger ADC with hardware

    //                                   software trigger    enable PDB     PDB interrupt  continuous mode load immediately
    constexpr uint32_t ADC_PDB_CONFIG = PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_LDMOD(0);

    constexpr uint32_t PDB_CHnC1_TOS_1 = 0x0100;
    constexpr uint32_t PDB_CHnC1_EN_1 = 0x01;

    PDB0_IDLY = 1; // the pdb interrupt happens when IDLY is equal to CNT+1

    PDB0_MOD = (uint16_t)(mod - 1);

    PDB0_SC = ADC_PDB_CONFIG | PDB_SC_PRESCALER(prescaler) | PDB_SC_MULT(mult) | PDB_SC_LDOK; // load all new values

    PDB0_SC = ADC_PDB_CONFIG | PDB_SC_PRESCALER(prescaler) | PDB_SC_MULT(mult) | PDB_SC_SWTRIG; // start the counter!

    PDB0_CH0C1 = PDB_CHnC1_TOS_1 | PDB_CHnC1_EN_1; // enable pretrigger 0 (SC1A)
    PDB0_CH1C1 = PDB_CHnC1_TOS_1 | PDB_CHnC1_EN_1; // enable pretrigger 0 (SC1A)

    //NVIC_ENABLE_IRQ(IRQ_PDB);
}

void reorder(uint16_t channel[], uint16_t pins){
  uint16_t temp = channel[0];

  for(int i = 1; i < pins; i++){
    channel[i-1] = channel[i];}
    
  channel[pins-1] = temp;
}

void conversionWrite(){
/*Conversion------------------------------------------------------*/
  int conversionBufPtr = bufPtr;
  int conversionBufPtr1 = bufPtr1;
  uint16_t tempbuffer[partition_sample_amount*2];
  for (int i = 0; i < partition_sample_amount*2; i+=2){
    tempbuffer[i] = buffer[conversionBufPtr];
    tempbuffer[i] += 0x8000;
    tempbuffer[i + 1] = buffer1[conversionBufPtr1];
    tempbuffer[i + 1] += 0x8000;
    conversionBufPtr++;
    conversionBufPtr1++;
  }
/*Increment Point, Counter------------------------------------------*/
  file.write(tempbuffer, sizeof(tempbuffer));                             // write buffer section on SD card;  
  last += partition_sample_amount * 2 ;                                   // increment last to control for file end
  old_dma0_isr_counter++;                                                 // increment counter so that it matches dma0_isr_counter
  old_dma1_isr_counter++;

  bufPtr = (BUF_DIM - 1) & (bufPtr + partition_sample_amount);            // choose next buffer section
  bufPtr1 = (BUF_DIM - 1) & (bufPtr1 + partition_sample_amount);
  
}

/*----------------------------------------------------------------*/
