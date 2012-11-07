
/////////////////////////// RMF 22 CODE TAKEN FROM RFM - Library
//// RTTY TRANSMISSION
 
void rtty_txbit (int bit)
{
  if (bit)
  {
#if REVERSE
    radio_write(0x73,0x00); // Low
#else
    radio_write(0x73,0x03); // High
#endif
}
  else
  {
#if REVERSE
    radio_write(0x73,0x03); // High
#else
    radio_write(0x73,0x00); // Low
#endif
  }
}


void setupRadio(){
//  pinMode(RFM22B_SDN, OUTPUT);    // RFM22B SDN is on ARDUINO A3
//  digitalWrite(RFM22B_SDN, LOW);
  pinMode(RFM22B_PIN, OUTPUT);
  digitalWrite(RFM22B_PIN, HIGH);
  delay(1000);
  radio_initSPI();
  radio_init();
}

uint8_t radio_read(uint8_t addr)  {
	//write ss low to start
	digitalWrite(RFM22B_PIN, LOW);
	
	// make sure the msb is 0 so we do a read, not a write
	addr &= 0x7F;
	SPI.transfer(addr);
	uint8_t val = SPI.transfer(0x00);
	
	//write ss high to end
	digitalWrite(RFM22B_PIN, HIGH);
	
	return val;
}

void radio_write(uint8_t addr, uint8_t data)  {
	//write ss low to start
	digitalWrite(RFM22B_PIN, LOW);
	
	// make sure the msb is 1 so we do a write
	addr |= 0x80;
	SPI.transfer(addr);
	SPI.transfer(data);
	
	//write ss high to end
	digitalWrite(RFM22B_PIN, HIGH);
}

void radio_read(uint8_t start_addr, uint8_t buf[], uint8_t len) {
	//write ss low to start
	digitalWrite(RFM22B_PIN, LOW);

	// make sure the msb is 0 so we do a read, not a write
	start_addr &= 0x7F;
	SPI.transfer(start_addr);
	for (int i = 0; i < len; i++) {
		buf[i] = SPI.transfer(0x00);
	}

	//write ss high to end
	digitalWrite(RFM22B_PIN, HIGH);
}
void radio_write(uint8_t start_addr, uint8_t data[], uint8_t len) {
	//write ss low to start
	digitalWrite(RFM22B_PIN, LOW);

	// make sure the msb is 1 so we do a write
	start_addr |= 0x80;
	SPI.transfer(start_addr);
	for (int i = 0; i < len; i++) {
		SPI.transfer(data[i]);
	}

	//write ss high to end
	digitalWrite(RFM22B_PIN, HIGH);
}

void radio_resetFIFO() {
	radio_write(0x08, 0x03);
	radio_write(0x08, 0x00);
}
#if COMPUTEFREQ
// Returns true if centre + (fhch * fhs) is within limits
// Caution, different versions of the RF22 suport different max freq
// so YMMV
boolean radio_setFrequency(float centre)
{
    uint8_t fbsel = 0x40;
    if (centre < 240.0 || centre > 960.0) // 930.0 for early silicon
		return false;
    if (centre >= 480.0)
    {
		centre /= 2;
		fbsel |= 0x20;
    }
    centre /= 10.0;
    float integerPart = floor(centre);
    float fractionalPart = centre - integerPart;
	
    uint8_t fb = (uint8_t)integerPart - 24; // Range 0 to 23
    fbsel |= fb;
    uint16_t fc = fractionalPart * 64000;
    radio_write(0x73, 0);  // REVISIT
    radio_write(0x74, 0);
    radio_write(0x75, fbsel);
    radio_write(0x76, fc >> 8);
    radio_write(0x77, fc & 0xff);
/*
    char info[80];
    sprintf(info, "Parameters for SetFreq: %02X %02X %02X %02X %02X", 0, 0, fbsel, fc>>8, fc&0xff);
    Serial.println(info);
*/    
}
#endif

void radio_init() {
	// disable all interrupts
	radio_write(0x06, 0x00);
	
	// move to ready mode
	radio_write(0x07, 0x01);
	
	// set crystal oscillator cap to 12.5pf (but I don't know what this means)
	radio_write(0x09, 0x7f);
	
	// GPIO setup - not using any, like the example from sfi
	// Set GPIO clock output to 2MHz - this is probably not needed, since I am ignoring GPIO...
	radio_write(0x0A, 0x05);//default is 1MHz
	
	// GPIO 0-2 are ignored, leaving them at default
	radio_write(0x0B, 0x00);
	radio_write(0x0C, 0x00);
	radio_write(0x0D, 0x00);
	// no reading/writing to GPIO
	radio_write(0x0E, 0x00);
	
	// ADC and temp are off
	radio_write(0x0F, 0x70);
	radio_write(0x10, 0x00);
	radio_write(0x12, 0x00);
	radio_write(0x13, 0x00);
	
	// no whiting, no manchester encoding, data rate will be under 30kbps
	// subject to change - don't I want these features turned on?
	radio_write(0x70, 0x20);
	
	// RX Modem settings (not, apparently, IF Filter?)
	// filset= 0b0100 or 0b1101
	// fuck it, going with 3e-club.ru's settings
	radio_write(0x1C, 0x04);
	radio_write(0x1D, 0x40);//"battery voltage" my ass
	radio_write(0x1E, 0x08);//apparently my device's default
	
	// Clock recovery - straight from 3e-club.ru with no understanding
	radio_write(0x20, 0x41);
	radio_write(0x21, 0x60);
	radio_write(0x22, 0x27);
	radio_write(0x23, 0x52);
	// Clock recovery timing
	radio_write(0x24, 0x00);
	radio_write(0x25, 0x06);
	
	// Tx power to max
	radio_write(0x6D, 0x04);//or is it 0x03?
	
	// Tx data rate (1, 0) - these are the same in both examples
	radio_write(0x6E, 0x27);
	radio_write(0x6F, 0x52);
	
	// "Data Access Control"
	// Enable CRC
	// Enable "Packet TX Handling" (wrap up data in packets for bigger chunks, but more reliable delivery)
	// Enable "Packet RX Handling"
	radio_write(0x30, 0x8C);
	
	// "Header Control" - appears to be a sort of 'Who did i mean this message for'
	// we are opting for broadcast
	radio_write(0x32, 0xFF);
	
	// "Header 3, 2, 1, 0 used for head length, fixed packet length, synchronize word length 3, 2,"
	// Fixed packet length is off, meaning packet length is part of the data stream
	radio_write(0x33, 0x42);
	
	// "64 nibble = 32 byte preamble" - write this many sets of 1010 before starting real data. NOTE THE LACK OF '0x'
	radio_write(0x34, 64);
	// "0x35 need to detect 20bit preamble" - not sure why, but this needs to match the preceeding register
	radio_write(0x35, 0x20);
	
	// synchronize word - apparently we only set this once?
	radio_write(0x36, 0x2D);
	radio_write(0x37, 0xD4);
	radio_write(0x38, 0x00);
	radio_write(0x39, 0x00);
	
	// 4 bytes in header to send (note that these appear to go out backward?)
	radio_write(0x3A, 's');
	radio_write(0x3B, 'o');
	radio_write(0x3C, 'n');
	radio_write(0x3D, 'g');
	
	// Packets will have 1 bytes of real data
	radio_write(0x3E, 1);
	
	// 4 bytes in header to recieve and check
	radio_write(0x3F, 's');
	radio_write(0x40, 'o');
	radio_write(0x41, 'n');
	radio_write(0x42, 'g');
	
	// Check all bits of all 4 bytes of the check header
	radio_write(0x43, 0xFF);
	radio_write(0x44, 0xFF);
	radio_write(0x45, 0xFF);
	radio_write(0x46, 0xFF);
	
	//No channel hopping enabled
	radio_write(0x79, 0x00);
	radio_write(0x7A, 0x00);
	
	// FSK, fd[8]=0, no invert for TX/RX data, FIFO mode, no clock
	radio_write(0x71, 0x22);
	
	// "Frequency deviation setting to 45K=72*625"
	radio_write(0x72, 0x48);
	
	
// Custom Code here 
#if COMPUTEFREQ
        radio_setFrequency(RADIO_FREQUENCY);
#else
	// "No Frequency Offet" - channels?
	radio_write(0x73, 0x00);
	radio_write(0x74, 0x00);
	// "frequency set to 434.19MHz" board default
	radio_write(0x75, 0x53);		
	radio_write(0x76, 0x68);
	radio_write(0x77, 0xF3);
#endif
        radio_write(0x71, 0x00); // unmodulated carrier
        //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
        radio_write(0x0b,0x12);
        radio_write(0x0c,0x15);
        radio_write(0x6D, 0x07);// turn tx high power 17/20db
//        radio_write(0x6D, 0x02);// turn tx low power 8db
        radio_write(0x07, 0x08);
        delay(500);

	radio_resetFIFO();
}

void radio_initSPI() {
	SPI.begin();
	// RFM22 seems to speak spi mode 0
	SPI.setDataMode(SPI_MODE0);
	// Setting clock speed to 8mhz, as 10 is the max for the rfm22
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	// MSB first
	//SPI.setBitOrder(MSBFIRST);
}
