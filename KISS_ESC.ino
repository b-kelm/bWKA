#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"

#define ESCPID_USB_UART_SPEED 115200  // Uart Speed
#define ESCPID_NB_ESC 1               // No. of ESCs (here 1)

const float LOGGING_CYCLE = 0.01;     // Logging cycle (probably not needed)[s]

IntervalTimer Timer;                  //10ms Interval Timer to update the ESC command

int16_t Dshot_val = 0;                      // dshot_cmd value

// PWM Servo Tester
volatile int pwm_value = 0;
volatile int prev_time = 0;
// Logging File Frequency
uint32_t prev_time_log = 0;

byte pinPWM = 14;

typedef struct {
  uint32_t      magic;                       // Magic number
  int8_t        err[ESCPID_NB_ESC];          // Last error number
  uint8_t       deg[ESCPID_NB_ESC];          // ESC temperature (Â°C)
  int16_t       cmd[ESCPID_NB_ESC];          // Current ESC command value
  uint16_t      volt[ESCPID_NB_ESC];         // Voltage of the ESC power supply (0.01V)
  uint16_t      amp[ESCPID_NB_ESC];          // ESC current (0.01A)
  uint16_t      rpm[ESCPID_NB_ESC];          // Motor rpm (10 rpm)
  
} ESCPIDcomm_struct_t;

ESCPIDcomm_struct_t ESCPID_comm = {
  {},
  {},
  {},
  {},
  {},
  {},
  {}
};

// SD Card Logging
// Taken From:
/************************************************************
   Demo program to log time and 5 channels of  ADC data
   to the SD Card on a Teensy 4.1
   MBorgerson   2/5/2021
********************************************************/
#include <ADC.h>
#include <SD.h>
#include <TimeLib.h>

const int ledpin  = 13;
#define  LEDON   digitalWriteFast(ledpin, HIGH);
#define  LEDOFF  digitalWriteFast(ledpin,  LOW);

// specify the number of channels and which pins to use
#define NUMCHANNELS 5

// Specify how fast to collect samples
#define SAMPRATE  50 // Sample Rate
int logFileNo = 0;
// Close file and start a new one, if time elapsed
int delta_time_ms = 5*60*1000; // in ms


// We will save a 32-bit Unix seconds value, 16-bit spare and NUMCHANNELS 16-bit ADC counts for each record
#define RECORDSIZE (4 +2 + NUMCHANNELS * 2)// size in bytes of each record 


// define a new data type for the samples
// with default packing, the structure will be a 
// multiple of 4 byte--so I added spare to make  it come
// out to 16 bytes.
typedef struct tSample {
  uint32_t useconds;
  uint16_t spare;   
  uint16_t avals[NUMCHANNELS];
} sampletype;  // each record is 14 bytes long for now

// now define two buffers, each holding 1024 samples.
// For efficiency, the number of samples in each buffer
// is a multiple of 512, which means that complete sectors
// are writen to the output file each time a buffer is written.
#define SAMPLESPERBUFFER 1024
tSample buffer1[SAMPLESPERBUFFER];
tSample buffer2[SAMPLESPERBUFFER];


// Define pointers to buffers used for ADC Collection
// and SD card file writing.
tSample *adcptr = NULL;
tSample *sdptr = NULL;

static uint32_t samplecount = 0;
volatile uint32_t bufferindex = 0;
volatile uint32_t overflows = 0;

const char compileTime [] = "\n\n5-channel KISS Controller Test Compiled on " __DATE__ " " __TIME__;

IntervalTimer CollectionTimer;

// FILE OBJECT
File adcFile;

/* SETUP ROUTINE */
void setup() {
  // PWM Handling
  attachInterrupt(pinPWM, rising, RISING); // Servo Tester on Pin 14 Teensy 4.1
  
  // SERIAL
  Serial.begin( ESCPID_USB_UART_SPEED );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC );

  // Arming ESCs
  ESCCMD_arm_all( );

  // Switch 3D mode on
  ESCCMD_3D_off( );

  // Arming ESCs
  ESCCMD_arm_all( );
  delay(1000);

  Dshot_val = -1;
  ESCCMD_throttle(0, Dshot_val);
  delay(500);
  
  // Start periodic loop
  ESCCMD_start_timer( );

  // Stop all motors
  for (int i = 0; i < ESCPID_NB_ESC; i++ ) {
   // ESCCMD_stop( i );
  }

  Timer.priority(200);
  Timer.begin(interrupt_10ms, LOGGING_CYCLE * 1000000);  //10msでタイマー割り込み開始

  // SD Card Logging Prep

   Serial.printf("Size of tSample is %lu\n", sizeof(tSample));
  pinMode(ledpin, OUTPUT);
  // CMSI(); / command line code
  Serial.print("Initializing SD card...");

// Indicate, that SD logging is not ready
  if (!StartSDCard()) {
    Serial.println("initialization failed!");
    while (1) { // Fast blink LED if SD card not ready
      LEDON; delay(100);
      LEDOFF; delay(100);
    }
  }
  
  // Start the timer that controls ADC and DAC
  CollectionTimer.begin(ADC_ISR, 1000000 / SAMPRATE);

  Serial.println("Waiting for commands.");
  setSyncProvider(getTeensy3Time); // helps put time into file directory data

  // Open the file
  if (adcFile) adcFile.close();
  adcFile = SD.open(NewFileName(), FILE_WRITE);
  samplecount = 0;
  Serial.printf("Opened file");
  bufferindex = 0;
  overflows = 0;
  
  adcptr = &buffer1[0];
  sdptr = NULL;

  

} // End of Setup(){}

void interrupt_10ms(void)
{
  //シリアルモニタに表示
  for (int i = 0; i < ESCPID_NB_ESC; i++ ) {
    Serial.print("Commanded PWM \t ");
    Serial.println(pwm_value);
//    Serial.print("Channel:\t");
//    Serial.println(i);
//    Serial.print("Error:\t\t");
//    Serial.println(ESCPID_comm.err[i]);
    Serial.print("Command:\t");
    Serial.println(ESCPID_comm.cmd[i]);
    Serial.print("Temperature:\t");
    Serial.println(ESCPID_comm.deg[i]);
    Serial.print("Voltage:\t");
    Serial.println(ESCPID_comm.volt[i]*0.01);
    Serial.print("Current:\t");
    Serial.println(ESCPID_comm.amp[i]*0.01);
   // Serial.print("Capacity:\t");
   // Serial.println(ESCPID_comm.mah[i]);
    Serial.print("rpm:\t\t");
    Serial.println(ESCPID_comm.rpm[i]*11.11); // RPM (*100/(18 poles / 2))
    Serial.println();
  }

}

/* LOOP ROUTINE */
void loop() {
  static int ret;

  Dshot_val = map(pwm_value, 900, 2090, 0, 550); // 550 Corresponding to 700 RPM 
  // val = 400; // 400 = 500 U/min

  ret = ESCCMD_tic();//テレメトリ情報の更新

  //テレメトリ情報の取得
  for (int i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_read_err(i, &ESCPID_comm.err[i]);
    ESCCMD_read_cmd(i, &ESCPID_comm.cmd[i]);
    ESCCMD_read_deg(i, &ESCPID_comm.deg[i]);
    ESCCMD_read_volt(i, &ESCPID_comm.volt[i]);
    ESCCMD_read_amp(i, &ESCPID_comm.amp[i]);
    ESCCMD_read_rpm(i, &ESCPID_comm.rpm[i]);
  }

  //ESCへ指令値送信
  if (ret == ESCCMD_TIC_OCCURED)
  {
    for (int i = 0; i < ESCPID_NB_ESC; i++)
    {
      // Serial.print("**NEW THROTTLE SIGNAL written**");
      ret = ESCCMD_throttle(i, Dshot_val);
    }
  }


  
  if(millis() > (prev_time_log + delta_time_ms)){ // Time Sging 10s = 10,000 ms
    
    prev_time_log = millis(); // Update time
    bufferindex = 0;
    if (adcFile) {
      adcFile.close();
      Serial.println("File Collection halted.");
      Serial.printf("Buffer overflows = %lu\n", overflows);
    }
    //sdptr = NULL;
    //adcptr = NULL;
    //bufferindex = 0;

    Serial.println("#### New Data Record File ####");
   
    // Start a new sample
    adcFile = SD.open(NewFileName(), FILE_WRITE);
    samplecount = 0;
    Serial.printf("Opened new file");
    bufferindex = 0;
    overflows = 0;
    
    adcptr = &buffer1[0];
    sdptr = NULL;
  }

  // LOG All the Data
  CheckFileState();

}


// Interrupt Routines for PWM Reader functions
void rising() {
  attachInterrupt(pinPWM, falling, FALLING);
  prev_time = micros();
}
 
void falling() {
  attachInterrupt(pinPWM, rising, RISING);
  pwm_value = micros()-prev_time;
}


// SD Logging

// This is the interrupt handler called by the collection interval timer
void ADC_ISR(void) {
  tSample *sptr;
  static uint32_t lastmicros;
  if (adcptr == NULL) return; // don't write unless adcptr is valid
  if (bufferindex >= SAMPLESPERBUFFER) {  // Switch buffers and signal write to SD
    if(sdptr != NULL) overflows++; // foreground didn't write buffer in time
    sdptr = adcptr; // notify foreground to write buffer to SD
    bufferindex = 0;
    if (adcptr == buffer1) {
      adcptr = buffer2;   // collect to buffer2 while buffer1 is written to SD
    } else { // use buffer 1 for collection
      adcptr = buffer1;
    }
  }
  
  // now we know that bufferindex is less than SAMPLESPERBUFFER
  // Please forgive the mix of pointers and array indices in the next line.
  sptr = (tSample *)&adcptr[bufferindex];
  // pure pointer arithmetic MIGHT be faster--depending on how well the compiler optimizes.
  sptr->useconds = now();
  sptr->spare = uint16_t(micros() - lastmicros);
  lastmicros =  micros();  // we can use this later to check for sampling jitter
  byte  i = 0;
  sptr->avals[0] = (uint16_t)ESCPID_comm.cmd[i];
  sptr->avals[1] = (uint16_t)(ESCPID_comm.deg[i]*10);
  sptr->avals[2] = (uint16_t)ESCPID_comm.volt[i]; // centi-volt *0.01
  sptr->avals[3] = (uint16_t)ESCPID_comm.amp[i]; // centi amp *0.01
  sptr->avals[4] = (uint16_t)ESCPID_comm.rpm[i]*11.11;
  samplecount++;
  bufferindex++;
}




void CMSI(void) {
  Serial.println();
  Serial.println(compileTime);
  if (adcFile) {
    Serial.printf("adcFile is open and contains %lu samples.\n", samplecount);
  } else {
    Serial.println("adcFile is closed.");
  }
  Serial.println("Valid commands are:");
  Serial.println("   s : Show this message");
  Serial.println("   d : Show SD Card Directory");
  Serial.println("   v : Show approximate volts");
  Serial.println("   c : Start data collection");
  Serial.println("   q : Stop data collection");
  Serial.println();
}

// Add data buffer to output file when needed
void CheckFileState(void) {

  // ADC ISR sets sdptr to a buffer point in ISR
  if (sdptr != NULL) { // write buffer to file
    if (adcFile) { // returns true when file is open
      LEDON
      adcFile.write(sdptr, SAMPLESPERBUFFER * sizeof(tSample));
      adcFile.flush();  // update directory and reduce card power
      LEDOFF
    } // end of if(adcfile)
    sdptr = NULL;  // reset pointer after file is written
  }  // end of if(sdptr != NULL)

}

/*****************************************************************************
   Read the Teensy RTC and return a time_t (Unix Seconds) value

 ******************************************************************************/
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

//------------------------------------------------------------------------------
//User provided date time callback function.
//   See SdFile::dateTimeCallback() for usage.
//
void dateTime(uint16_t* date, uint16_t* time) {
  // use the year(), month() day() etc. functions from timelib
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}

bool StartSDCard() {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("\nSD File initialization failed.\n");
    return false;
  } else  Serial.println("initialization done.");
  // set date time callback function for file dates
  SdFile::dateTimeCallback(dateTime);
  return true;
}


// make a new file name based on time and date
char* NewFileName(void) {
  static char fname[36];
  time_t nn;
  nn = now();
  int mo = month(nn);
  int dd = day(nn);
  int hh = hour(nn);
  int mn = minute(nn);
  int ss = second(nn);
  sprintf(fname, "bWKA_%02d%02d%02d%02d%02d_%03d.dat", mo, dd, hh, mn, ss,logFileNo);
  logFileNo = logFileNo + 1; // Increment the File Number
  return &fname[0];
  
}
