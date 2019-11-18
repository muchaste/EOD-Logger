//*************************************************************************
//*      Hybrid program from:                                             *
//*       Tni, Greiman, Yannick, Leon and Stefan                          *
//*                                                                       *
//*       for analog measurements and datalogging                         *
//*************************************************************************

#include <ADC.h>              //library for easier ADC implementation
#include <DMAChannel.h>       //library for easier DMA implementation
#include <array>              //use C++ array structs
#include "SdFat.h"            //work with SD card
#include <TimeLib.h>          //modified DateTime library for timekeeping functionality
#include <DS1307RTC.h>        //library to read DS1307 rtc
#include <Wire.h>             //library for I2C-communication (via SCL - serial clock and SDA - serial data)


/*----------------------------------------------------------------*/
const uint32_t pdbfreq = 100000;  // sampling speed [Hz] max ~300MHz - actually the trigger frequency of the programmable delay block
uint32_t duration = 600;          // duration of each measure-cycle [s]
unsigned long pause = 0;          // pause-duration
String Name = "Log13";           // filename
unsigned long debug_start;
/*----------------------------------------------------------------*/


/*PRÄ-------------------------------------------------------------*/
uint32_t      BUF_DIM         = 32768 ;           //size of buffer that holds data from ADC
uint32_t      FILE_SIZE       = 0 , last = 0;     //initial variables for filesize and pointer to...
volatile size_t write_pos = 0;                    //points to last item in ring buffer
volatile uint16_t adc_val = 0;                    //holds data from adc
File file;                                        //file object for logging data
uint32_t bytes = 0;
float preceil = 0;
float scale = 0;
/*----------------------------------------------------------------*/



/*PINS------------------------------------------------------------*/
const uint8_t adc_pin = A9;     // digital pin 23 for singleended
const uint8_t diff_pin1 = A10; // digital pin A10 for differential
const uint8_t diff_pin2 = A11; // digital pin A11 for differential
/*----------------------------------------------------------------*/



/*DECLARATIONS----------------------------------------------------*/
SdFatSdioEX sd ;    // used to declare the sd.### object (Sdfat)
ADC adc;            // used to declare the adc.### object
DMAChannel dma;     // used to declare the dma.### object
std::array<volatile uint16_t, (uint32_t)128*512> buffer __attribute__ ((aligned (32*1024)));  
char str[128] ;
typeof(*dma.TCD)  tcd_mem[4] __attribute__ ((aligned (32))) ;
tmElements_t tm;
/*----------------------------------------------------------------*/


void setup() 
{
  pinMode(13, OUTPUT);
  debug_start = millis();
  Serial.begin(9600);  
  while (!Serial && ((millis() - debug_start) <= 5000));
  /*TimeSetup-------------------------------------------------------*/
  if (RTC.read(tm)) {
    Serial.println("Begin Setup\n");
  }else{
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
      while(1);
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
      while(1);
    }
  }
/*----------------------------------------------------------------*/

/*ModeSetup-------------------------------------------------------*/
  adc.adc0->startSingleDifferential(diff_pin1,diff_pin2);
  
/*----------------------------------------------------------------*/

/*FileSetup-------------------------------------------------------*/
  String Date = String(tmYearToCalendar(tm.Year))+"."+String(tm.Month)+"."+String(tm.Day)+"-"+String(tm.Hour)+"."+String(tm.Minute)+"."+String(tm.Second);
  String filename = Name + "_" + Date + ".bin";
  char fname[30];
  filename.toCharArray(fname,30);

       
  if (!  sd.begin()) { sd.initErrorHalt("SdFatSdio   begin() failed"); }    sd.chvol();
  if (!file.open(fname, O_RDWR | O_CREAT)) {  sd.errorHalt("open failed");  } 
  delay(100);
/*----------------------------------------------------------------*/

/*DurationSetup---------------------------------------------------*/
  bytes = ((duration*1000000)/(1000000/pdbfreq))* 2; 
  preceil = bytes/BUF_DIM;
  scale = ceil(preceil);                             //round up preceil value
  FILE_SIZE = (scale+2) * BUF_DIM;
/*----------------------------------------------------------------*/

/*HardcodedSetup-could be made variable---------------------------*/
//  pinMode                (             LED_BUILTIN, OUTPUT );
  pinMode                (                 adc_pin, INPUT  );
  pinMode                (               diff_pin1, INPUT  ); 
  pinMode                (               diff_pin2, INPUT  ); 
  //pinMode                (              button_pin, INPUT  ); 
  //pinMode                (                 red_pin, OUTPUT ); 
  //pinMode                (                blue_pin, OUTPUT ); 
  //pinMode                (               green_pin, OUTPUT ); 
  adc.setAveraging       (                              1  );
  adc.setResolution      (                           16,0  );
  adc.setConversionSpeed ( ADC_CONVERSION_SPEED::HIGH_SPEED);      
  adc.setSamplingSpeed   ( ADC_SAMPLING_SPEED::HIGH_SPEED  ); 
  
  adc.setReference(ADC_REFERENCE::REF_3V3, ADC_0);                //set ADC reference voltage
  dma.source                 (           ADC0_RA);   
  dma.transferSize           (                 2);
  dma.triggerAtHardwareEvent (DMAMUX_SOURCE_ADC0); 
/*----------------------------------------------------------------*/
  
/*Buffer setup ---------------------------*/

  dma.TCD->CITER    =           32*1024/2   ;      dma.TCD->BITER =  32*1024/2 ;   dma.TCD->DOFF = 2 ;    dma.TCD->CSR   =  0x10; 
  
  dma.TCD->DADDR        = (volatile void*) &buffer [ 0*512]    ;   
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[     1]    ;   
  memcpy ( &tcd_mem[0], dma.TCD , 32 ) ;   
                                                                                       
  dma.TCD->DADDR        = (volatile void*) &buffer [32*512]    ;   
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[     2]    ;   
  memcpy ( &tcd_mem[1], dma.TCD , 32 ) ;   
                                                                                     
  dma.TCD->DADDR        = (volatile void*) &buffer [64*512]    ;   
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[     3]    ;   
  memcpy ( &tcd_mem[2], dma.TCD , 32 ) ;   
                                                                                     
  dma.TCD->DADDR        = (volatile void*) &buffer [96*512]    ;    
  dma.TCD->DLASTSGA     = (   int32_t    ) &tcd_mem[     0]    ;   
  memcpy ( &tcd_mem[3], dma.TCD , 32 )  ;   
                                                                                   
  memcpy ( dma.TCD ,  &tcd_mem[0], 32 ) ;                          
                                                                                             
                                                                                                                                                                             
//  *************       


  dma.enable();      

  adc.enableDMA(ADC_0);     
   
  adc.adc0->stopPDB();         
  adc.adc0->startPDB(pdbfreq);
  NVIC_DISABLE_IRQ(IRQ_PDB); // we don't want or need the PDB interrupt

  for (int i = 0; i < 5; i++){ // just as a visual feedback, blink 5 times if the setup was completed
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(300);
  }
}




void loop() {  
while ( ((128*1024-1) & ( (int)dma.TCD->DADDR - last )) > BUF_DIM ){  
  if (BUF_DIM != (uint32_t)file.write( (char*)&buffer[((last/2)&(64*1024-1))], BUF_DIM) )       { sd.errorHalt("write failed");    } ; last += BUF_DIM ;  
} 

  if ( last >= FILE_SIZE ) {   
    file.close();
    last = 0;
    filestuff();
  }
}

void pdb_isr(void) {  PDB0_SC &=~PDB_SC_PDBIF  ;  Serial.println("."); }; 

void filestuff(){
  RTC.read(tm);
  String Date = String(tmYearToCalendar(tm.Year))+"."+String(tm.Month)+"."+String(tm.Day)+"-"+String(tm.Hour)+"."+String(tm.Minute)+"."+String(tm.Second);
  String filename = Name + "_" + Date + ".bin";
  char fname[30];
  filename.toCharArray(fname,30);
  Serial.println("filename");
  if (!file.open(fname, O_RDWR | O_CREAT)) {  sd.errorHalt("open failed");  } 
  Serial.println(filename);
}
