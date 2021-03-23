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

const uint32_t pdbfreq = 10000;                                   //Sample Frequency If you use 4 Pins on 1 ADC max 12kHz

volatile int buffer_dma_position = 0;                             //Pointer position in DMA Buffer, used with partition to determine where to write
volatile int buffer_dma1_position = 0;
volatile int amount_SD_writes = 4;                                //How often do you want to write
volatile int partition = BUF_DIM / amount_SD_writes;
volatile int partition1 = BUF_DIM / amount_SD_writes;
volatile int partition_sample_amount = BUF_DIM / amount_SD_writes;

/*DECLARATIONS SD-card and File-----------------------------------*/

SdFs sd;                                          // used to declare the sd.### object (Sdfat); do not use SdFatSdioEX sd

uint16_t fileNr = 0;                              // after a given duration a new file is created; fileNr is an index used for the filename
String Name = "EODLog_aa";                        // filename prefix

FsFile file;                                      // file object for logging data

uint32_t      FILE_SIZE       = 0;                // filesize DMA-Buffer*times_buffer
uint32_t      last            = 0;                // coutner uint16_t values that are written to SD card
const int     times_buffer    = 1000;               // creation of a file based on a multiple of the buffer size

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

DMAChannel dma;                                                                       // used to declare the dma.### object for the first channel
DMAChannel dma_switch;                                                                //DMA channel linked to DMA to switch pins
DMAChannel dma1;
DMAChannel dma1_switch;

/*----------------------------------------------------------------*/

void setup() 
{
/*Reorder---------------------------------------------------------*/                  // Reorders Pin arrays to match order in WAV-FILE
  reorder(ChannelsCfg_0, ChannelPinNumber0);
  reorder(ChannelsCfg_1, ChannelPinNumber1);
/*Serial monitor--------------------------------------------------*/
  debug_start = millis();
  Serial.begin(115200);
  while (!Serial && ((millis() - debug_start) <= 5000));
  Serial.println("Begin Setup\n");
/*Time and File setup---------------------------------------------*/
  setup_RTC(); 
  FILE_SIZE = times_buffer * BUF_DIM;                                 // after writing FILE_SIZE uint16_t values to a file a new file is created. *2 because we use 2 DMA channels                 
  setup_File();
/*LED pin, Default Buffer-----------------------------------------*/
  pinMode(13, OUTPUT);                                                // built-in LED is at PIN 13 in Teensy 3.5
    // clear buffer 
    
  for (int j = 0; j < BUF_DIM; ++j){                                  //Set values of the DMA buffer to a default value.
      buffer[j] = 30000;}
  for(int k = 0; k < BUF_DIM; ++k){
    buffer1[k] = 30000;}
/*DMA ADC Setup---------------------------------------------------*/ 
  setup_adc();
  setup_dma(); 
/*Debug-----------------------------------------------------------*/
  Serial.print("DMA Buffer sample size: ");
  Serial.println(BUF_DIM);                                                 //Prints Amount of samples
  Serial.print("File sample size: ");
  Serial.println(FILE_SIZE);
  Serial.println("End of setup");
  Serial.println("----------------------------------");
/*Signal end of Setup method--------------------------------------*/
  for (int i = 0; i < 5; i++){                                             // visual feedback, blink 5 times if the setup was completed
    digitalWrite(13, HIGH);
    delay(30);
    digitalWrite(13, LOW);
    delay(30);
  }
}
/*----------------------------------------------------------------*/

void loop() { 
  if (dma0_isr_counter != old_dma0_isr_counter && dma1_isr_counter != old_dma1_isr_counter)                              // check if buffer section can be written on microSD card
  {
/*Conversion------------------------------------------------------*/
  //Conversion from unsigned to signed --> switches the left most bit in hex
  uint16_t tempbuffer[partition_sample_amount*2];
  int conversionBufPtr = bufPtr;
  int conversionBufPtr1 = bufPtr1;
  for (int i = 0; i < partition_sample_amount*2; i+=2){
    tempbuffer[i] = buffer[conversionBufPtr];
    tempbuffer[i] += 0x8000;
    tempbuffer[i + 1] = buffer1[conversionBufPtr1];
    tempbuffer[i + 1] += 0x8000;
    conversionBufPtr++;
    conversionBufPtr1++;
  }
/*----------------------------------------------------------------*/
    file.write(tempbuffer, sizeof(tempbuffer));                             // write buffer section on SD card;
    
    last += partition_sample_amount * 2 ;                                                 // increment last to control for file end

    old_dma0_isr_counter++;                                                 // increment counter so that it matches dma0_isr_counter
    old_dma1_isr_counter++;

    bufPtr = (BUF_DIM - 1) & (bufPtr + partition_sample_amount);                          // choose next buffer section
    bufPtr1 = (BUF_DIM - 1) & (bufPtr1 + partition_sample_amount);

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
    filestuff();                                                            // create new files for data logging
  }
}
/*----------------------------------------------------------------*/

/*Functions-------------------------------------------------------*/

void setup_adc() {

  adc->adc0->setResolution      (                  16  );
  adc->adc0->setReference       (ADC_REFERENCE::REF_3V3);
  adc->adc0->enableDMA();                                                   // connect DMA and ADC
  adc->adc0->stopPDB();                                                     // start PDB conversion trigger
  adc->adc0->setHardwareTrigger();

  adc->adc1->setResolution      (                  16  );
  adc->adc1->setReference       (ADC_REFERENCE::REF_3V3);
  adc->adc1->enableDMA();                                                   // connect DMA and ADC
  adc->adc1->stopPDB();                                                     // start PDB conversion trigger
  adc->adc1->setHardwareTrigger();

  // enable PDB    
   SIM_SCGC6 |= SIM_SCGC6_PDB;
   setPdbFreq();                                                            // Modified Version of startPDB to trigger both ADC's at once
   startPdb();    
                                                             
}

void setup_dma(){
  //Beware order important i.e. transferSize can not be used everywhere, needs to be after source and destination declaration

  dma.begin(true);
  dma.source(ADC0_RA);
  dma.destinationBuffer(&buffer[0], Major_size);                            //Major Size should be changed if less than 512 samples are used
  dma.transferSize(2);
  dma.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0); 
  dma.disableOnCompletion();
  dma.interruptAtCompletion();
  dma.attachInterrupt(dma0_isr); 
 
  dma_switch.sourceBuffer(&ChannelsCfg_0[0], sizeof(ChannelsCfg_0));
  dma_switch.destination(ADC0_SC1A);
  dma_switch.transferSize(2);
  dma_switch.triggerAtTransfersOf(dma);
  dma_switch.triggerAtCompletionOf(dma);

  dma1.begin(true);
  dma1.source(ADC1_RA);
  dma1.destinationBuffer(&buffer1[0], Major_size);
  dma1.transferSize(2);
  dma1.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC1); 
  dma1.disableOnCompletion();
  dma1.interruptAtCompletion();
  dma1.attachInterrupt(dma1_isr); 
 
  dma1_switch.sourceBuffer(&ChannelsCfg_1[0], sizeof(ChannelsCfg_1));
  dma1_switch.destination(ADC1_SC1A);
  dma1_switch.transferSize(2);
  dma1_switch.triggerAtTransfersOf(dma1);
  dma1_switch.triggerAtCompletionOf(dma1);

  dma.enable();                                                         
  dma_switch.enable();

  dma1.enable();                                                         
  dma1_switch.enable();
}



void dma0_isr() {                                                             // method that deletes interrupt and increments a counter; method is later attached to the resepctive dma channel via  dma.attachInterrupt(dma0_isr);
    buffer_dma_position = buffer_dma_position + 512;
    if(buffer_dma_position == partition){
      dma0_isr_counter++;
      partition += partition;
      if(partition == BUF_DIM){
        partition = BUF_DIM / amount_SD_writes;
      }
    }
    if(buffer_dma_position == BUF_DIM){
      dma.TCD->DADDR = &buffer[0];
      buffer_dma_position = 0;
    }
    else{
    dma.TCD->DADDR = &buffer[buffer_dma_position];
    }
    dma0_isr_counter++;
    dma.clearInterrupt();
    dma.enable();
}

void dma1_isr() {                                                             // method that deletes interrupt and increments a counter; method is later attached to the resepctive dma channel via  dma.attachInterrupt(dma0_isr);
    buffer_dma1_position = buffer_dma1_position + 512;
    if(buffer_dma1_position == partition1){
      dma1_isr_counter++;
      partition1 += partition1;
      if(partition1 == BUF_DIM){
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
    dma1_isr_counter++;
    dma1.clearInterrupt();
    dma1.enable();
}

void setup_RTC(){
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

 void setup_File(){
  
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

  write_wav_header();
  
 }

 void filestuff() {
  String Date = String(year()) + "." + String(month()) + "." + String(day()) + "-" + String(hour()) + "." + String(minute()) + "." + String(second());
  fileNr++;
  String filename = Date + "_dma0_" + fileNr + ".wav";                               // create filenames
  char fname[30];
  filename.toCharArray(fname, 30);
  Serial.print("filename: ");
  Serial.println(filename);
  
  if (!file.open(fname, O_RDWR | O_CREAT)) {
    sd.errorHalt("open dma0 file failed");
  }

  write_wav_header();
}

void write_wav_header(){

  uint16_t resolution0 = adc->adc0->getResolution();
  uint32_t SubtwoChunkSize = FILE_SIZE  * (resolution0/8);                        // =NumSamples * NumChannels * BitsPerSample/8. This is the number of bytes in the data. Number of samples is per Channel, we therefore divide the total FILE_SIZE by the number of Channels --> makes calculation redundant but kept for clarity.

  
  //RIFF chunk descriptor
    uint8_t mainChunkId[4] = {'R', 'I', 'F', 'F'};                                // Contains the letters "RIFF" in ASCII form
    file.write(mainChunkId, sizeof(mainChunkId));

    uint32_t mainChunkSize = 36 + SubtwoChunkSize;                                // Size of the entire File -8 bytes for the two fields not included in this count (ChunkID and ChunkSize) 
    file.write(&mainChunkSize, 4);
    
    uint8_t mainChunkFormat[4] = {'W', 'A', 'V', 'E'};
    file.write(mainChunkFormat, sizeof(mainChunkFormat));

  //Subchunk1 --> fmt sub-chunk

    uint8_t fmtChunkId[4] = {'f', 'm', 't', ' '};
    file.write(fmtChunkId, sizeof(fmtChunkId));
    
    uint32_t fmtChunkSize = 16; //16 for PCM
    file.write(&fmtChunkSize, 4);

    uint16_t format = 1;                                                          // 1 is PCM (Pulse-code modulation used for sampled analog signals)
    file.write(&format, 2);

  //Subchunk1 sound attributes
  
    file.write(&numChannels, 2); //number of channels

    file.write(&pdbfreq, 4); //sample rate
   
    uint32_t byteRate = pdbfreq * (resolution0/8) * numChannels;                 //Byte rate = SampleRate * NumChannels * BitsPerSample/8. pdbfreq is determined in the beginning
    file.write(&byteRate, 4);
    
    uint16_t blockAlign = numChannels * (resolution0/8);                         //Block align: number of bytes for one sample including all channels
    file.write(&blockAlign, 2);

    file.write(&resolution0, 2);                                                 //bits per sample

    //ExtraParamSize = ... doesn't exist when using PCM

  //Subchunk2 contains size of data and actual sound: 
  
    uint8_t SubtwoChunkId[4] = {'d', 'a', 't', 'a'};                             //contains letters data
    file.write(SubtwoChunkId, sizeof(SubtwoChunkId));
    
    file.write(&SubtwoChunkSize, 4);
    
   //This header makes a total of 44 bytes in size
   //for more information visit http://soundfile.sapp.org/doc/WaveFormat/
   //it follows the actual data written in the loop 
  
}

void startPdb() {
    PDB0_SC |= PDB_SC_PDBEN | PDB_SC_SWTRIG;
}

void setPdbFreq() {
    constexpr uint32_t prescaler = 7;                                                                                             // from 0 to 7: factor of 1, 2, 4, 8, 16, 32, 64 or 128
    constexpr uint32_t prescaler_multiplier = 1;                                                                                  // from 0 to 3: factor of 1, 10, 20 or 40
    constexpr uint32_t pdb_trigger_frequency = pdbfreq;
    constexpr uint32_t amount_pins_channel = ChannelPinNumber0;
    constexpr uint32_t mod = (F_BUS / 128 / 10 / pdb_trigger_frequency / amount_pins_channel);
    Serial.print("MOD : ");
    Serial.println(mod);
    static_assert(mod <= 0x10000, "Prescaler insufficient.");
    PDB0_MOD = (uint16_t)(mod-1);
    constexpr uint32_t PDB_CHnC1_TOS_1 = 0x0100;
    constexpr uint32_t PDB_CHnC1_EN_1 = 0x01;
    PDB0_CH0C1 = PDB_CHnC1_TOS_1 | PDB_CHnC1_EN_1;                                                                                 // PDB triggers ADC0 SC1A
    PDB0_CH1C1 = PDB_CHnC1_TOS_1 | PDB_CHnC1_EN_1;                                                                                 // PDB triggers ADC1 SC1A
    PDB0_SC = PDB_SC_PRESCALER(prescaler) | PDB_SC_MULT(prescaler_multiplier) | PDB_SC_LDOK | 
              PDB_SC_TRGSEL(0b1111) | PDB_SC_PDBEN | PDB_SC_LDMOD(0) | PDB_SC_CONT;                                                // PDB triggers ADC1 SC1A
}

void reorder(uint16_t channel[], uint16_t pins){
  uint16_t temp = channel[0];

  for(int i = 1; i < pins; i++){
    channel[i-1] = channel[i];}
    
  channel[pins-1] = temp;
}