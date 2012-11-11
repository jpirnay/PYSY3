/*
 Interrupt Driven RTTY TX Demo
  
 Transmits data via RTTY with an interupt driven subroutine.
  
 By Anthony Stirk M0UPU
  
 October 2012 Version 5
  
 Thanks and credits :
 Evolved from Rob Harrison's RTTY Code.
 Compare match register calculation by Phil Heron.
 Thanks to : http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)

 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> // needed for bmp_
#include <util/crc16.h> // needed for Checksum
// Logging might still be optimized...
#include <SdFat.h>                      // the complete SD library is too fat (pun intended)  
#include <Time.h>                       // time functions

#include <SPI.h>
/// BMP085 Pressure sensor
#include <Wire.h>                       // Two Wire interface
/// ATTACHED VENUS (=SKYTRAQ GPS)
#include <SoftwareSerial.h>             // Had too much troubles with HW TX/RX so had to bypass that issue...
////// DEFINES FOR GEIGER BOARD
#define BUZZER_PIN       16             // sw jumper for buzzer
#define LED_PIN          14             // L1 LED on Logger Shield

////// DEFINES FOR logging periods
#define FREQ_LOG_SD    5000  // every 5 seconds write data to file
#define FREQ_LOG_RADIO 10000 // every 10 seconds transmit data over radio
// #define PRI_RATIO       175             // defaults to SBM-20 ratio
#define PRI_RATIO       122             // defaults to SI-29BG ratio

////// DEFINES FOR RADIO
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 100    // Baud rate for use with RFM22B Max = 600
#define REVERSE false  

#define COMPUTEFREQ false // if false take parameter from previous calc, saves some bytes
#define RADIO_FREQUENCY 434.198

#define RFM22B_PIN 9

/// Communication with GPS
SoftwareSerial GPSModul(6, 7);          // 6 is RX, 7 is Tx
unsigned long _new_time;
unsigned long _new_date;
long _new_latitude;
long _new_longitude;
long _new_altitude;
unsigned long  _new_speed;
unsigned short _new_numsats;
unsigned long _last_time_fix, _new_time_fix;
unsigned long _last_position_fix, _new_position_fix;

// parsing state variables
byte _parity;
bool _is_checksum_term;
char _term[15];
byte _sentence_type;
byte _term_number;
byte _term_offset;
bool _gps_data_good;
  
/// VARIABLES FOR RADIO TRANSMISSION, TODO: ADAPT SIZE OF ARRAY
#define RADIO_MAXTRANSMIT_LEN 80
char datastring[RADIO_MAXTRANSMIT_LEN];
char txstring[RADIO_MAXTRANSMIT_LEN];
char dummy[12], timestr[12];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
bool InterruptAllowed = true;

/// COMMON VARIABLES
boolean SD_OK = true;                   // assume SD is OK until init
unsigned long counter;                  // counter for logcount
boolean lowVcc = false;                 // true when Vcc < LOW_VCC

unsigned long msec;

unsigned long last_log_SD = 0;
unsigned long last_log_Radio = 0;

struct Telemetry
{
   unsigned long counter;
   long           date;       // DDMMYY
   long           time;
   long           latitude;   // / 100000.0 to get decimal lat
   long           longitude;  // / 100000.0 to get decimal lon
   long           altitude;   // / 100.0 to get altitude in m
   long           speed_horiz; // in knots to get speed kmph : / speed / 100.0 * 1852 
   double         speed_vert;
   unsigned short satellites;
   long           last_time_fix;
   long           last_position_fix;
  // Geiger 
   unsigned long  geigerticks;
   unsigned long  startticks;
  // Temperatur 
   double         temperature_int;
   double         temperature_ext;
  // Druck 
   long           pascal;
   long           pascal_raw;
};
Telemetry TelemetryData;
volatile Telemetry TelemetryTx;

// Communication with SD card reader
SdFat sd;
SdFile logfile;
SdFile gpsfile;

void CopyTelemetry()
{
 TelemetryTx.counter = TelemetryData.counter;
 TelemetryTx.date = TelemetryData.date;
 TelemetryTx.time = TelemetryData.time;
 TelemetryTx.latitude = TelemetryData.latitude;
 TelemetryTx.altitude = TelemetryData.altitude;
 TelemetryTx.speed_horiz = TelemetryData.speed_horiz;
 TelemetryTx.speed_vert = TelemetryData.speed_vert;
 TelemetryTx.satellites = TelemetryData.satellites;
 TelemetryTx.last_time_fix = TelemetryData.last_time_fix;
 TelemetryTx.last_position_fix = TelemetryData.last_position_fix;
 TelemetryTx.geigerticks = TelemetryData.geigerticks;
 TelemetryTx.startticks = millis() - TelemetryData.startticks; // logging period in msec
 TelemetryTx.temperature_int = TelemetryData.temperature_int;
 TelemetryTx.temperature_ext = TelemetryData.temperature_ext;
 TelemetryTx.pascal = TelemetryData.pascal;
 TelemetryTx.pascal_raw = TelemetryData.pascal_raw;
}  

void ResetTelemetry()
{
  TelemetryData.startticks = millis();
  TelemetryData.geigerticks = 0;
  // TelemetryData = {};
}  

void FillTelemetry()
{
  TelemetryData.temperature_int = bmp_readTemperature();
  TelemetryData.pascal = bmp_readPressure();
  TelemetryData.pascal_raw = bmp_readRawPressure();  
  TelemetryData.counter++;
}

/// SOME UTILITY FUNCTIONS
int AvailRam(){ 
  int memSize = 2048;                   // if ATMega328
  byte *buf;
  while ((buf = (byte *) malloc(--memSize)) == NULL);
  free(buf);
  return memSize;
} 

void addLong(long valu)
{
   emptyDummy();
   strcat(datastring, ",");
   ltoa(valu, dummy, 10);
   strcat(datastring, dummy);
}  

void addUInt(unsigned int valu)
{
   emptyDummy();
   strcat(datastring, ",");
   itoa(valu, dummy, 10);
   strcat(datastring, dummy);
}  

void addDouble(double valu, int postfix)
{
   emptyDummy();
   strcat(datastring, ",");
   dtostrf(valu, postfix+2, postfix, dummy);
   strcat(datastring, dummy);
}  
 
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring all $s
  for (i = 0; i < strlen(string); i++)
  {
    c = string[i];
    if (c!='$') crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}   

//// Setup and Main Loop  
void setup()
{
  Serial.begin(9600);
  Serial.print("Ram 1: ");
  Serial.println(AvailRam());

  GPSModul.begin(9600);
  Serial.println("reset");
  ResetTelemetry();
  Serial.println("card");
  initCard();                           // init the SD card
  Serial.println("time");
  initTime();				// read time from SD card 
  Serial.println("log");
  initLog();                            // prepare log files
  Serial.println("bmp");
  bmp_begin();                          // prepare BMP085 sensor
  Serial.print("Ram 2: ");
  Serial.println(AvailRam());
  
  setupRadio();                         // setup RFM22 radio module
  initialise_interrupt();               // allow Timer Interrupts for transmission as well as 
}

void emptyDummy()
{
  for (byte i=0; i<sizeof(dummy); i++) dummy[i] = '\0';
}

void AppendToString (int iValue, char *pString){ // appends a byte to string passed
  emptyDummy();
  itoa(iValue,dummy,10);
  strcat(pString,dummy);
}

void FormatTime(){  // get the time and date from the GPS and format it
  int hr, mi, se;
  hr = (int) TelemetryTx.time / 1000000;
  mi = (int) (TelemetryTx.time / 10000) % 100;
  se = (int) (TelemetryTx.time / 100) % 100;
  Serial.println("FT: ");
  Serial.println(hr);
  Serial.println(mi);
  Serial.println(se);
  
  timestr[0]='\0';
  AppendToString (hr, timestr);         // add 24 hour time to string
  strcat(timestr, ":");
  if (mi < 10) strcat(timestr,"0");
  AppendToString (mi,timestr);       // add MINUTES to string
  strcat(timestr,":");
  if (se < 10) strcat(timestr,"0");
  AppendToString (se,timestr);     // add SECONDS to string
  Serial.println(timestr);
}

void HAppendToString (int iValue, char *pString){ // appends a hex to string passed
  emptyDummy();//  if (iValue<0x100) strcat(pString, "0");
  if (iValue<0x10) strcat(pString, "0");
  itoa(iValue, dummy, 16);
  strcat(pString,dummy);
}

void logString() // just basic data
{
   int mylen;
   uint16_t crc;
   double logCPM, uSv;
   long msc;
   InterruptAllowed = false;
   datastring[0];
   strcat(datastring, "$$PYSY3");
   addLong(TelemetryTx.counter);
   FormatTime();
   strcat(datastring, ",");
   strcat(datastring, timestr);
   addDouble(TelemetryTx.latitude / 100000.0, 5);
   addDouble(TelemetryTx.longitude / 100000.0, 5);
   addDouble(TelemetryTx.altitude / 100.0, 1);
   addDouble(TelemetryTx.temperature_int, 1);
   addDouble(TelemetryTx.temperature_ext, 1);
   addDouble(TelemetryTx.speed_vert, 1);
// store length of data as we will be truncating the string to this length later   
   mylen = strlen(datastring);
   addDouble(TelemetryTx.speed_horiz, 2);
   addLong(TelemetryTx.pascal);
   addLong(TelemetryTx.pascal_raw);
   addLong(TelemetryTx.geigerticks);
   logCPM = float(TelemetryTx.geigerticks) / (float(TelemetryTx.startticks) / 60000);
   uSv = logCPM / PRI_RATIO;         // make uSV conversion
   addDouble(logCPM, 2);
   addDouble(uSv, 4);
   
  /// FILELOGGING
  Serial.println(datastring);
  Serial.println(strlen(datastring));
  
  logfile.println(datastring);
  logfile.sync();
// reset string to basic data and add checksum so radio interrupt can pick up a valid string at any time
   datastring[mylen] = 0;
   crc = gps_CRC16_checksum(datastring);
   strcat(datastring, "*");
   HAppendToString(crc, datastring);
   strcat(datastring, "\x0d");   
   Serial.println(datastring);
   Serial.println(strlen(datastring));
   InterruptAllowed = true;
}


void loop()
{
  msec = millis();
  readGPS();
  if (msec - last_log_SD >= FREQ_LOG_SD)
  {
    FillTelemetry();   // get additional data, GPS and geigerticks are propagated in the background...
    CopyTelemetry();   // copy telemetry data to transfer buffer

    logString();
    ResetTelemetry();
    last_log_SD = msec;
  }  
  if (msec - last_log_Radio >= FREQ_LOG_RADIO)
  {
//    logRadio();
    last_log_Radio = msec;
  }  
}

//// GPScode
bool readGPS(){  //  request and get a sentence from the GPS
char c;
boolean res;
  res = false;
  while (GPSModul.available()){           // Get sentence from GPS
    c = GPSModul.read();
/*    
    if (SD_OK) {
       gpsfile.print(c);
    }   
*/    
    if (GPS_encode(c)) res = true;
  }
  return res;
}
//// INTERRUPT ROUTINES FOR RADIO AND GEIGER COUNTER GO HERE
void initialise_interrupt()
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);

  attachInterrupt(0,GetEvent,FALLING);  // Geiger event on D2 triggers interrupt

  sei();          // enable global interrupts
}

void GetEvent(){   // ISR triggered for each new event (count)
  TelemetryData.geigerticks++;
}


ISR(TIMER1_COMPA_vect)
{
  if (InterruptAllowed) 
  {
    switch(txstatus) {
    case 0: // This is the optional delay between transmissions.
      txj++;
      if(txj>(TXDELAY*RTTY_BAUD)) {
        txj=0;
        txstatus=1;
  //      Serial.println("Delay");
      }
      break;
    case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission.
  //    Serial.println("Start");
      strcpy(txstring,datastring);
      txstringlength=strlen(txstring);
      txstatus=2;
      txj=0;
      break;
    case 2: // Grab a char and lets go transmit it.
      if ( txj < txstringlength)
      {
        txc = txstring[txj];
  //      Serial.print(txc);
        txj++;
        txstatus=3;
        rtty_txbit (0); // Start Bit;
        txi=0;
      }
      else
      {
        txstatus=0; // Should be finished
        txj=0;
      }
      break;
    case 3:
      if(txi<ASCII)
      {
        txi++;
        if (txc & 1) rtty_txbit(1);
        else rtty_txbit(0);  
        txc = txc >> 1;
        break;
      }
      else
      {
        rtty_txbit (1); // One stop Bit in any case, let "case 4" decide, whether we need another one...
        txstatus=4;
        txi=0;
        break;
      }
    case 4:
      if(STOPBITS==2)
      {
        rtty_txbit (1); // Stop Bit
        txstatus=2;
        break;
      }
      else
      {
        txstatus=2;
        break;
      }
    }
  }
}


void initCard(){   // initialize the SD card
  SD_OK = false;                        // don't try to write to the SD card
  pinMode(10, OUTPUT);                  // must set DEFAULT CS pin output, even if not used
  if (!sd.begin(10, SPI_HALF_SPEED))
 {  
    error("Card");
    sd.initErrorHalt();
  }
  SD_OK = true;
//  SdFile::dateTimeCallback(SDdateTime); 
}

void initLog(){
  char filename[]="py000.csv";
  if ( gpsfile.open("pysy.log", O_WRONLY | O_CREAT) ) {
    gpsfile.println("PYSY-Startup");
    FormatTime();
    gpsfile.println(timestr);
    gpsfile.print("RAM:");
    gpsfile.println(AvailRam());
    gpsfile.sync();
  }
  else  
  {
    sd.errorHalt("pysy.log not writeable");
  }  
  
  for (uint8_t i = 0; i < 250; i++) {
    filename[2] = i/100 + '0';
    filename[3] = (i%100)/10 + '0';
    filename[4] = i%10 + '0';
/*  */
    Serial.print("Trying: ");
    Serial.println(filename);
/* */
    if (! sd.exists(filename)) {
        break;  // leave the loop!
    }  
  }
  Serial.print("Final test:"); Serial.println(filename);
  if (logfile.open(filename, O_WRONLY | O_CREAT) ) {
    Serial.println("SD OK, now write...");
    logfile.println("NR,TIME,LAT,LON,ALT,FIX,SATS,CPM,uSv,Vcc,TEMP,PR1,PR2");
  }
  else
  {
     sd.errorHalt("opening logfile for write failed");
  }  

}

void error(char *str){
  Serial.println("CARD!");
  Serial.println(str);                       // display this error or status
  digitalWrite(LED_PIN, HIGH);          // red LED indicates error
}

/* Reads default time before any fix from GPS forces the right one, 
   tries to read time.txt from SD-card 
   Attention: uses logfile variable ! */
void initTime()
{
  char ch;
  int tpos[6];
  int valu, ct, rd;
  tpos[6]=2012; // YEAR
  tpos[5]=11;   // MONTH
  tpos[4]=17;   // DAY
  tpos[3]=0;    // SEC
  tpos[2]=0;    // MIN
  tpos[1]=14;   // HOUR
#if false  
  Serial.println("Time variables set...");
  if (SD_OK)
  {
    ct = 1;
    valu = 0;
    if (sd.exists("time.txt"))
    {
      if (logfile.open("time.txt", O_READ))
      {
         Serial.println("Time open okay");
         while ((rd = logfile.read()) >= 0){
           ch = (char)rd;
           Serial.print(ch);
           switch (ch) {
           case '\n':
             if (ct<=6) tpos[ct] = valu;
             ct = ct + 1;
             valu = 0;
             break;
           case '0': valu = valu * 10 + 0; break;
           case '1': valu = valu * 10 + 1; break;
           case '2': valu = valu * 10 + 2; break;
           case '3': valu = valu * 10 + 3; break;
           case '4': valu = valu * 10 + 4; break;
           case '5': valu = valu * 10 + 5; break;
           case '6': valu = valu * 10 + 6; break;
           case '7': valu = valu * 10 + 7; break;
           case '8': valu = valu * 10 + 8; break;
           case '9': valu = valu * 10 + 9; break;
           default:
             break;  
           }  
         } 
        Serial.println(); 
        logfile.close(); 
      }
      else 
        Serial.println("Could not open time.txt"); 
    }
    else
      Serial.println("No time file");
  }
/**/
  Serial.print("Set Time:");
  for (ct=1;ct<4;ct++){
     Serial.print(tpos[ct]); 
     Serial.print(":");
  }   
  Serial.print(" ");
  for (ct=4;ct<7;ct++){
     Serial.print(tpos[ct]); 
     Serial.print(".");
  }  
  Serial.println();
/**/
#endif
  setTime(tpos[1], tpos[2], tpos[3], tpos[4], tpos[5], tpos[6]);
}

// call back for file timestamps
void SDDateTime(uint16_t* date, uint16_t* time) {
  time_t nw;
  nw = gpsTimeSync();
  if (nw==0) nw = now();
/*  
  Serial.print("SDDate:" );
  Serial.print(day(nw));  
  Serial.print(".");
  Serial.print(month(nw));  
  Serial.print(".");
  Serial.println(year(nw));
*/
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(nw), month(nw), day(nw));

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(nw), minute(nw), second(nw));
}
 
 time_t gpsTimeSync(){  //  returns time if avail from gps, else returns 0
  unsigned long fix_age = 0 ;
  GPS_get_datetime(NULL,NULL, &fix_age);

  if(fix_age < 1000) return gpsTimeToArduinoTime();   // only return time if recent fix  
  return 0;
}


time_t gpsTimeToArduinoTime(){  // returns time_t from GPS with GMT_Offset for hours
  tmElements_t tm;
  int year;
  readGPS();                    // make sure we process te most recent data
  GPS_crack_datetime(&year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, NULL);
  tm.Year = year - 1970; 
  time_t time = makeTime(tm);
//  if (InDST()) time = time + SECS_PER_HOUR;
  return time;
 // + (GMT_Offset * SECS_PER_HOUR);
}



