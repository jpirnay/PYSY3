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

////// DEFINES FOR RADIO
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 50    // Baud rate for use with RFM22B Max = 600
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
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
bool InterruptAllowed = true;

/// COMMON VARIABLES
boolean SD_OK = true;                   // assume SD is OK until init
char timeString[10];                    // time format determined by #defines above
char dateString[10];                    // time format determined by #defines above
byte day_of_week,DST;                   // if daylight savings time, DST == true
int GMT_Offset;                         // defaults to 13 = GMT+1
unsigned long counter;                  // counter for logcount
boolean lowVcc = false;                 // true when Vcc < LOW_VCC

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
volatile Telemetry TelemetryData;
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
 TelemetryTx.startticks = TelemetryData.startticks;
 TelemetryTx.temperature_int = TelemetryData.temperature_int;
 TelemetryTx.temperature_ext = TelemetryData.temperature_ext;
 TelemetryTx.pascal = TelemetryData.pascal;
 TelemetryTx.pascal_raw = TelemetryData.pascal_raw;

// won't work
// TelemetryTx = TelemetryData;
 ResetTelemetry(); 
}  

void ResetTelemetry()
{
  memset((void *)&TelemetryData, 0, sizeof(TelemetryData));
  TelemetryData.startticks = millis();
  // TelemetryData = {};
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
   char dummy[12];
   strcat(datastring, ",");
   ltoa(valu, dummy, 10);
   strcat(datastring, dummy);
}  

void addUInt(unsigned int valu)
{
   char dummy[12];
   strcat(datastring, ",");
   itoa(valu, dummy, 10);
   strcat(datastring, dummy);
}  

void addDouble(double valu, int postfix)
{
   char dummy[12];
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
//  Serial.begin(9600);
  GPSModul.begin(9600);

  ResetTelemetry();
    
  initCard();                           // init the SD card
  initTime();				// read time from SD card 
  initLog();                            // prepare log files
  bmp_begin();                          // prepare BMP085 sensor
  
  setupRadio();                         // setup RFM22 radio module
  initialise_interrupt();               // allow Timer Interrupts for transmission as well as 
//  Serial.print("Avail Ram: ");
//  Serial.println(AvailRam());
}
  
void loop()
{
  long bmp_pascal;
  double bmp_Temp;
  readGPS();
/// GET TELEMETRY DATA
/*
  Serial.print("Pascal=");
  Serial.println(bmp_pascal);
  Serial.print("Raw Pascal=");
  Serial.println(bmp_raw_pascal);
*/  
/// CREATE STRING FOR TRANSMISSION
  datastring[0]=0;
  strcat(datastring, "$$PYSY3");
/// COUNTER AND RAM
  TelemetryData.counter = counter;
  TelemetryData.pascal = bmp_readPressure();
  TelemetryData.pascal_raw = bmp_readRawPressure();

/// ADD CHECKSUM
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);
/// THATS IT FOR TRANSMISSION, NOW ADD SOME ADDITIONAL DETAILS FOR SD CARD WRITING
/// Now we do write our data on the SD card, let's make sure no one is interferring on the SPI bus during that time, 
/// this may hurt the timing somewhat, but lets assume the writing is quick enough...
  InterruptAllowed = false;
  /// FILELOGGING
  InterruptAllowed = true;
  delay(1000);
  counter++;
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

void FormatDateTime(){  // get the time and date from the GPS and format it
  int dispYear;
  int i;
  memset(timeString,0,sizeof(timeString));  // initialize the strings
  memset(dateString,0,sizeof(dateString));

  // convert Sun=1 format to Sun=7 format (DST calc is based on this)
  day_of_week = (weekday()==1) ? 7: (weekday() - 1);

  // (The time lib will deal with AM/PM and 12 hour clock)
  // make time string
  AppendToString (hour(),timeString);         // add 24 hour time to string
  strcat(timeString,":");
  if (minute() < 10) strcat(timeString,"0");
  AppendToString (minute(),timeString);       // add MINUTES to string
  strcat(timeString,":");
  if (second() < 10) strcat(timeString,"0");
  AppendToString (second(),timeString);     // add SECONDS to string

  // make date string
  i=day();                              // add DAY to string
  if (i < 10) strcat(dateString,"0");
  AppendToString (i,dateString);  
  strcat(dateString, ".");

  i = month();                          // add MONTH to string  
  if (i < 10) strcat(dateString,"0");
  AppendToString (i,dateString);
  strcat(dateString, ".");

  i = year();                          // add YEAR to string
  AppendToString (i,dateString);                            
}


void AppendToString (int iValue, char *pString){ // appends a byte to string passed
  char tempStr[6];
  memset(tempStr,'\0',sizeof(tempStr));
  itoa(iValue,tempStr,10);
  strcat(pString,tempStr);
}


void initCard(){   // initialize the SD card
  SD_OK = false;                        // don't try to write to the SD card
  pinMode(10, OUTPUT);                  // must set DEFAULT CS pin output, even if not used

  if (!sd.begin(10, SPI_HALF_SPEED))
 {  
    sd.initErrorHalt();
    error("Card");
  }
  SD_OK = true;
//  SdFile::dateTimeCallback(SDdateTime); 
  SdFile::dateTimeCallback(SDDateTime);
}

void initLog(){
  char filename[]="py000.csv";
  if ( gpsfile.open("pysy.log", O_WRONLY | O_CREAT) ) {
    gpsfile.println("PYSY-Startup");
    FormatDateTime();
    gpsfile.println(dateString);
    gpsfile.println(timeString);
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
/*    
    Serial.print("Trying: ");
    Serial.println(filename);
*/
    if (! sd.exists(filename)) {
        break;  // leave the loop!
    }  
  }
//  Serial.print("Final test:"); Serial.println(filename);
  if (logfile.open(filename, O_WRONLY | O_CREAT) ) {
//    Serial.println(F("SD OK, now write..."));
    logfile.println(F("NR,DATE,TIME,LAT,LON,KMPH,ALT,FIX,SATS,CPM,uSv,Vcc,TEMP,PR1,PR2"));
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
  tpos[5]=10;   // MONTH
  tpos[4]=20;   // DAY
  tpos[3]=0;    // SEC
  tpos[2]=0;    // MIN
  tpos[1]=14;   // HOUR
  if (SD_OK)
  {
    ct = 1;
    valu = 0;
    if (logfile.open("time.txt", O_READ))
    {
       while ((rd = logfile.read()) >= 0){
         ch = (char)rd;
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
      logfile.close(); 
    }
  }
/*
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
*/
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
  if (InDST()) time = time + SECS_PER_HOUR;
  return time + (GMT_Offset * SECS_PER_HOUR);
}


bool InDST(){  // Returns true if in DST - Caution: works for central europe only
  // DST starts the last Sunday in March and ends the last Sunday in Octobre
  bool res;
  byte DOWinDST, nextSun;
  int Dy, Mn;  

  Dy = tmDay;
  Mn = tmMonth;
  //Dy = 27;  // FOR TESTING
  //Mn = 7;
  res = false;

  // Pre-qualify for in DST in the widest range (any date between  and 23.10) 
  // Earliest date in March is 25th
  // Earliest date in October is 25th
  if ((Mn == 3 && Dy >= 25) || (Mn > 3 && Mn < 10) || (Mn == 10 && Dy <= 23) ){
    DOWinDST = true;                    // assume it's in DST until proven otherwise
    nextSun = Dy + (7 - day_of_week);   // find the date of the last Sunday
  while (nextSun<24) nextSun += 7;
    if (nextSun > 31) nextSun -= 7;     // come back to month
    if (Mn == 3 && (Dy < nextSun)) DOWinDST = false;     // it's before the last Sun in March
    if (Mn == 10 && (Dy >= nextSun)) DOWinDST = false; // it's after the 1ast Sun in Oct.
    if (DOWinDST) res = true;           // DOW is still OK so it's in DST
  }
  return res;                         // Nope, not in DST
}

