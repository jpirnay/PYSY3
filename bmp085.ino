///// DEFINES FOR BMP085 SENSOR

#define BMP085_I2CADDR 0x77

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD          0x2E
#define BMP085_READPRESSURECMD            0x34

/// BAROMETRIC PRESSURE
// BMP085 barometric sensor
uint8_t oversampling;

int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;


////////////////////////// BMP085 code - based on Adafruit library
  
boolean bmp_begin() {
uint8_t mode;
  mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;

  Wire.begin();

  if (bmp_read8(0xD0) != 0x55) return false;

  /* read calibration data */
  ac1 = bmp_read16(BMP085_CAL_AC1);
  ac2 = bmp_read16(BMP085_CAL_AC2);
  ac3 = bmp_read16(BMP085_CAL_AC3);
  ac4 = bmp_read16(BMP085_CAL_AC4);
  ac5 = bmp_read16(BMP085_CAL_AC5);
  ac6 = bmp_read16(BMP085_CAL_AC6);

  b1 = bmp_read16(BMP085_CAL_B1);
  b2 = bmp_read16(BMP085_CAL_B2);

  mb = bmp_read16(BMP085_CAL_MB);
  mc = bmp_read16(BMP085_CAL_MC);
  md = bmp_read16(BMP085_CAL_MD);
}

uint16_t bmp_readRawTemperature(void) {
  bmp_write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  _delay_ms(5);
  return bmp_read16(BMP085_TEMPDATA);
}

uint32_t bmp_readRawPressure(void) {
  uint32_t raw;

  bmp_write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER) 
    _delay_ms(5);
  else if (oversampling == BMP085_STANDARD) 
    _delay_ms(8);
  else if (oversampling == BMP085_HIGHRES) 
    _delay_ms(14);
  else 
    _delay_ms(26);

  raw = bmp_read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= bmp_read8(BMP085_PRESSUREDATA+2);
  raw >>= (8 - oversampling);

 /* this pull broke stuff, look at it later?
  if (oversampling==0) {
    raw <<= 8;
    raw |= bmp_read8(BMP085_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
  }
 */

  return raw;
}

int32_t bmp_readPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = bmp_readRawTemperature();
  UP = bmp_readRawPressure();

#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  UP = 23843;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
  b1 = 6190;
  b2 = 4;
  ac3 = -14383;
  ac2 = -72;
  ac1 = 408;
  ac4 = 32741;
  oversampling = 0;
#endif

  // do temperature calculations
  X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
  X2 = ((int32_t)mc << 11) - (X1 + md)/2;     // round up
  X2 /= (X1 + md);
  B5 = X1 + X2;

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791)>>4);
  return p;
}


float bmp_readTemperature(void) {
  int32_t UT, X1, X2, B5;     // following ds convention
  float temp;

  UT = bmp_readRawTemperature();

  // step 1
  X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
  X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  B5 = X1 + X2;
  temp = (B5 + 8) >> 4;
  temp /= 10;

  return temp;
}

/*********************************************************************/

uint8_t bmp_read8(uint8_t a) {
  uint8_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
#if (ARDUINO >= 100)
  ret = Wire.read(); // receive DATA
#else
  ret = Wire.receive(); // receive DATA
#endif
  Wire.endTransmission(); // end transmission

  return ret;
}

uint16_t bmp_read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
#if (ARDUINO >= 100)
  ret = Wire.read(); // receive DATA
  ret <<= 8;
  ret |= Wire.read(); // receive DATA
#else
  ret = Wire.receive(); // receive DATA
  ret <<= 8;
  ret |= Wire.receive(); // receive DATA
#endif
  Wire.endTransmission(); // end transmission

  return ret;
}

void bmp_write8(uint8_t a, uint8_t d) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
  Wire.write(d);  // write data
#else
  Wire.send(a); // sends register address to read from
  Wire.send(d);  // write data
#endif
  Wire.endTransmission(); // end transmission
}

