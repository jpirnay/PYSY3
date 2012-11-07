
#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"
#define GPS_INVALID_AGE 0xFFFFFFFF
#define GPS_INVALID_ANGLE 999999999 
#define GPS_INVALID_ALTITUDE  999999999
#define GPS_INVALID_DATE 0
#define GPS_INVALID_TIME 0xFFFFFFFF
#define GPS_INVALID_SPEED  999999999
#define GPS_INVALID_FIX_TIME 0xFFFFFFFF
#define GPS_INVALID_SATELLITES 0xFF
#define GPS_INVALID_HDOP 0xFFFFFFFF
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
#define _GPS_SENTENCE_GPGGA 0
#define _GPS_SENTENCE_GPRMC 1
#define _GPS_SENTENCE_OTHER 2

const float GPS_INVALID_F_ALTITUDE = 0.0;
const float GPS_INVALID_F_SPEED = 0.0;
const float GPS_INVALID_F_ANGLE = 1000.0;

bool GPS_encode(char c)
{
  bool valid_sentence = false;

  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = GPS_term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}

//
// internal utilities
//
int GPS_from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

unsigned long GPS_parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * GPS_gpsatol(p);
  while (GPS_gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (GPS_gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (GPS_gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

unsigned long GPS_parse_degrees()
{
  char *p;
  unsigned long left = GPS_gpsatol(_term);
  unsigned long tenk_minutes = (left % 100UL) * 10000UL;
  for (p=_term; GPS_gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 1000;
    while (GPS_gpsisdigit(*++p))
    {
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 100000 + tenk_minutes / 6;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool GPS_term_complete()
{
  if (_is_checksum_term)
  {
    byte checksum = 16 * GPS_from_hex(_term[0]) + GPS_from_hex(_term[1]);
    if (checksum == _parity)
    {
      if (_gps_data_good)
      {
        if (TelemetryData.last_time_fix != 0 && _sentence_type==_GPS_SENTENCE_GPGGA )
        {
           TelemetryData.speed_vert = (_new_altitude - TelemetryData.altitude)/(_new_time_fix - TelemetryData.last_time_fix)*10; // Time in ms
        }  
        TelemetryData.last_time_fix = _new_time_fix;
        TelemetryData.last_position_fix = _new_position_fix;

        switch(_sentence_type)
        {
        case _GPS_SENTENCE_GPRMC:
          TelemetryData.time      = _new_time;
          TelemetryData.date      = _new_date;
          TelemetryData.latitude  = _new_latitude;
          TelemetryData.longitude = _new_longitude;
          TelemetryData.speed_horiz = _new_speed;
          break;
        case _GPS_SENTENCE_GPGGA:
          TelemetryData.altitude  = _new_altitude;
          TelemetryData.speed_vert= 0;
          TelemetryData.time      = _new_time;
          TelemetryData.latitude  = _new_latitude;
          TelemetryData.longitude = _new_longitude;
          TelemetryData.satellites  = _new_numsats;
//          _hdop      = _new_hdop;
//          _fixtype   = _new_fixtype;
          break;
        }

        return true;
      }
    }

    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!GPS_gpsstrcmp(_term, _GPRMC_TERM))
      _sentence_type = _GPS_SENTENCE_GPRMC;
    else if (!GPS_gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGGA;
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
    switch(COMBINE(_sentence_type, _term_number))
  {
    case COMBINE(_GPS_SENTENCE_GPRMC, 1): // Time in both sentences
    case COMBINE(_GPS_SENTENCE_GPGGA, 1):
      _new_time = GPS_parse_decimal();
      _new_time_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
      _gps_data_good = _term[0] == 'A';
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 2):
      _new_latitude = GPS_parse_degrees();
      _new_position_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(_GPS_SENTENCE_GPGGA, 3):
      if (_term[0] == 'S')
        _new_latitude = -_new_latitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 4):
      _new_longitude = GPS_parse_degrees();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(_GPS_SENTENCE_GPGGA, 5):
      if (_term[0] == 'W')
        _new_longitude = -_new_longitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      _new_speed = GPS_parse_decimal();
      break;
/*
      case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
      _new_course = GPS_parse_decimal();
      break;
*/      
    case COMBINE(_GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
      _new_date = GPS_gpsatol(_term);
      break;
/*
    case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      _gps_data_good = _term[0] > '0';
	  _new_fixtype = (unsigned char)atoi(_term);
      break;
*/
    case COMBINE(_GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
      _new_numsats = (unsigned char)atoi(_term);
      break;
/*      
    case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
      _new_hdop = GPS_parse_decimal();
      break;
*/      
    case COMBINE(_GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
      _new_altitude = GPS_parse_decimal();
      break;
  }

  return false;
}

long GPS_gpsatol(const char *str)
{
  long ret = 0;
  while (GPS_gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

int GPS_gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}


/*
// lat/long in hundred thousandths of a degree and age of fix in milliseconds
void GPS_get_position(long *latitude, long *longitude, unsigned long *fix_age)
{
  if (latitude) *latitude = TelemetryData.latitude;
  if (longitude) *longitude = TelemetryData.longitude;
  if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
GPS_INVALID_AGE : millis() - _last_position_fix;
}

void GPS_f_get_position(float *latitude, float *longitude, unsigned long *fix_age)
{
  long lat, lon;
  GPS_get_position(&lat, &lon, fix_age);
  *latitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (TelemetryData.latitude / 100000.0);
  *longitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (TelemetryData.longitude / 100000.0);
}
*/

// date as ddmmyy, time as hhmmsscc, and age in milliseconds
void GPS_get_datetime(unsigned long *date, unsigned long *time, unsigned long *age)
{
  if (date) *date = TelemetryData.date;
  if (time) *time = TelemetryData.time;
  if (age) *age = _last_time_fix == GPS_INVALID_FIX_TIME ? 
GPS_INVALID_AGE : millis() - _last_time_fix;
}

void GPS_crack_datetime(int *year, byte *month, byte *day, 
  byte *hour, byte *minute, byte *second, byte *hundredths, unsigned long *age)
{
  unsigned long date, time;
  GPS_get_datetime(&date, &time, age);
  if (year) 
  {
    *year = date % 100;
    *year += *year > 80 ? 1900 : 2000;
  }
  if (month) *month = (date / 100) % 100;
  if (day) *day = date / 10000;
  if (hour) *hour = time / 1000000;
  if (minute) *minute = (time / 10000) % 100;
  if (second) *second = (time / 100) % 100;
  if (hundredths) *hundredths = time % 100;
}

/*
float GPS_f_altitude()    
{
  return _altitude == GPS_INVALID_ALTITUDE ? GPS_INVALID_F_ALTITUDE : _altitude / 100.0;
}


float GPS_f_speed_knots() 
{
  return _speed == GPS_INVALID_SPEED ? GPS_INVALID_F_SPEED : speed / 100.0;
}

float GPS_f_speed_kmph()  
{ 
  float sk = GPS_f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * GPS_f_speed_knots(); 
}
*/

bool GPS_gpsisdigit(char c) { return c >= '0' && c <= '9'; }

 
