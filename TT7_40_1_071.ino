//INFO
//The code derives from Anthony Stirk's interrupt routine and code examples published on UKHAS website.
//The board consists of ATMEGA328P running at 4MHz, RFM22B transmitter, UBLOX MAX7-C GPS module.

//LIBRARIES
#include <SPI.h>
#include <RFM22.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>


//DEFINE
#define STOPBITS 2 // Either 1 or 2
#define ASCII 7 // 7 or 8 bits
#define TXDELAY 0 // 1=1sec
#define TX_BAUD 100 // 50, 100, 200, 300, 600, 1200 - 200+ increase 0 and 1 frequency spacing
#define RFM22_RESET 15 // reset every
#define TX_POWER 0x04
/*
0x02 5db (3mW)
0x03 8db (6mW)
0x04 11db (12mW)
0x05 14db (25mW)
0x06 17db (50mW)
0x07 20db (100mW)
*/
rfm22 radio1(10);//NSEL - PIN10
/*
SS - D10 - NSEL pull low to enable communication
MOSI - D11 - SDI
MISO - D12 - SDO
SCK - D13 - SCLK
SDN - D9 RFM22B shutdown when pulled high
*/


//VARIABLES
char TXbuffer[80];
char battVs[10] = "0";
int GPSerror=0;
int n;
int adcTemp=0;
int rfmTemp=0;
int lat_int=0, lon_int=0;
int navmode=0;
int countreset=0;
int lastCount=0;
float battV=0.00;
uint8_t GPSbuffer[80];
uint8_t hour=0, minute=0, second=0;
uint8_t lock=0, sats=0;
int32_t lat=0, lon=0, alt=0, lat_dec = 0, lon_dec =0;
/*
GPSfix Type, range 0..5 - LOCK
0x00 = No Fix
0x01 = Dead Reckoning only
0x02 = 2D-Fix
0x03 = 3D-Fix
0x04 = GPS + dead reckoning combined
0x05 = Time only fix
0x06..0xff: reserved

Dynamic Platform model:
0 Portable
2 Stationary
3 Pedestrian
4 Automotive
5 Sea
6 Airborne with <1g Acceleration
7 Airborne with <2g Acceleration
8 Airborne with <4g Acceleration
*/

volatile int txstatus=1;//Interrupt
volatile int txj;
volatile int txstringlength=0;
volatile int count=1;
volatile int txi;
volatile char txc;
volatile boolean lockvariables = 0;
volatile boolean sentenceReady=0;


void setup()
{
  //GPS setup
  Serial.begin(9600);
  resetGPS();
  wait(100);
  setupGPS();
  
  //radio setup
  setupRadio();
  
  //interrupt setup
  initialise_interrupt();
}


void loop()
{
  if(count>lastCount)//duplicate to shorten the time between transmissions
  {
    //Reset Radio in regular cycles - duplicate
    if(countreset==RFM22_RESET)
    {
      digitalWrite(9, HIGH);//pull SDN high
      wait(2000);
      setupRadio();
      wait(500);
      countreset=0;
    }
    
    sprintf(TXbuffer, "$$TT7-40,%i,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%d,%s,%i",count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, alt, sats, lock, battVs, rfmTemp);
    sprintf(TXbuffer, "%s*%04X\n", TXbuffer, CRC16_checksum(TXbuffer));
    txstringlength=strlen(TXbuffer);
    lastCount=count;
    sentenceReady=1;
  }
  
  //Check Airborne Mode - 6
  gps_check_nav();
  if(navmode!=6)
  {
    setGPS_DynamicModel6();
  }
  
  //GPS Data
  gps_get_time();
  gps_get_lock();
  gps_get_position();
    
  if(count>lastCount)//duplicate to shorten the time between transmissions
  {
    //Reset Radio in regular cycles - duplicate
    if(countreset==RFM22_RESET)
    {
      digitalWrite(9, HIGH);//pull SDN high
      wait(2000);
      setupRadio();
      wait(500);
      countreset=0;
    }
    
    sprintf(TXbuffer, "$$TT7-40,%i,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%d,%s,%i",count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, alt, sats, lock, battVs, rfmTemp);
    sprintf(TXbuffer, "%s*%04X\n", TXbuffer, CRC16_checksum(TXbuffer));
    txstringlength=strlen(TXbuffer);
    lastCount=count;
    sentenceReady=1;
  }

  //Battery Voltage
  battV = ((1.98 / 1024)* analogRead(A0)); // ((vcc / maxADC)* adcreading)
  dtostrf(battV,3,2,battVs); // convert lat from float to string
    
  //RFM22B Temperature
  radio1.write(0x0F, 0x00); // RF22_REG_0F_ADC_CONFIGURATION 0x0f : RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR 0x00 Temprtature sensor, oo, yes plz
  radio1.write(0x12, 0x00); // set temp range (-64 - +64 degC)
  radio1.write(0x12, 0x20); // set ENTSOFF (wtf is that?)
  radio1.write(0x0F, 0x80); // RF22_REG_0F_ADC_CONFIGURATION 0x0f : RF22_ADCSTART 0x80 After reading the manual you must set this self clearing bit to get the ADC to take a reading
  delayMicroseconds(400); //wait > 350 us for ADC converstion
  adcTemp = radio1.read(0x11); //Register 11h. ADC Value. What units this returns in I have no idea. Degrees bannana? - Oh its an ADC value, so probably 0-255
  rfmTemp = adcTemp * 0.5 - 64 - 7;
    
  if(count>lastCount)//duplicate to shorten the time between transmissions
  {
    //Reset Radio in regular cycles - duplicate
    if(countreset==RFM22_RESET)
    {
      digitalWrite(9, HIGH);//pull SDN high
      wait(2000);
      setupRadio();
      wait(500);
      countreset=0;
    }
    
    sprintf(TXbuffer, "$$TT7-40,%i,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%d,%s,%i",count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, alt, sats, lock, battVs, rfmTemp);
    sprintf(TXbuffer, "%s*%04X\n", TXbuffer, CRC16_checksum(TXbuffer));
    txstringlength=strlen(TXbuffer);
    lastCount=count;
    sentenceReady=1;
  }
}


//
//
//RADIO FUNCTIONS
//
//
void setupRadio()
{
  pinMode(9,OUTPUT);//SDN - RFM22B shutdown when pulled high
  digitalWrite(9,LOW);
  wait(500);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71,0x00);//unmodulated carrier
  radio1.write(0x0b,0x12);//setting automatic antenna switching
  radio1.write(0x0c,0x15);//
  radio1.setFrequency(434.301);
  radio1.write(0x6D, TX_POWER);
  radio1.write(0x07,0x08);//TX on
}


void rtty_txbit (int bit)
{
  if (bit)
  {
    //radio1.write(0x73,0x03); // High - shift register AVA
    radio1.setFrequency(434.3015);//50, 100 baud
    //radio1.setFrequency(434.30175);//300 baud - didn't decode properly, 200 baud - ok
  }
  else
  {
    //radio1.write(0x73,0x00); // Low - shift register AVA
    radio1.setFrequency(434.3010);
  }
}


//
//
//GPS FUNCTIONS 
//
//
void setupGPS()
{
  int gps_set_sucess=0;
  
  //UART configuration - Baud Rate, Output Sentences
  uint8_t setNMEAoff[]={0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  while(!gps_set_sucess)//NEW
  {
    sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNMEAoff);
  }
  wait(100);
  
  //Airborne Mode 6 - equivalent to DynamicMode16 (DynamicMode13 - Pedestrian instead of Airborne)
  setGPS_DynamicModel6();
  wait(100);
  
  //Continuous Mode
  setGps_MaxPerformanceMode();
  wait(100);
}


void resetGPS() 
{
  uint8_t set_reset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5 };
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}


void setGps_MaxPerformanceMode()// NORMAL MODE
{
  /*
  0: Continous Mode
  1: Power Save Mode
  2-3: reserved
  4: Continuous Mode
  */
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91 }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}


void setGPS_PowerSaveMode()//Power Mode selection
{
  // Power Save Mode
  uint8_t setPSM[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

/*
void setGPS_Cyclic()//Power Save Mode used when position fixes required in short periods 1-10s !!! think it's wrong !!! ==>> REPAIR
{
  // Update Period 10 seconds , do not enter 'inactive for search' state when no fix unchecked
  uint8_t setCyclic[] = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x02, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0xE3, 0x65};
  sendUBX(setCyclic, sizeof(setCyclic)/sizeof(uint8_t));
}
*/

void setGPS_Cyclic()//Power Mode configuration - Power Save Mode used when position fixes required in short periods 1-10s - REPAIRED based on Ublox Manual - NOT TESTED 
{
  // Update Period 10 seconds , do not enter 'inactive for search' state when no fix unchecked - (1B - nic, 2B - jen update EPH, 3B - jen cyclic tracking, 4B - nic) CHECKSUM vyřešen v Excelu
  uint8_t setCyclic[] = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x43};
  sendUBX(setCyclic, sizeof(setCyclic)/sizeof(uint8_t));
}


void setGPS_DynamicModel6()//set Airborne Mode - 6
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
}


void sendUBX(uint8_t *MSG, uint8_t len)
{
  Serial.flush();
  Serial.write(0xFF);// new - AVA
  
  for(int i=0; i<len; i++)
  {
    Serial.write(MSG[i]);
  }
}


boolean getUBX_ACK(uint8_t *MSG)
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  
  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;	// CK_A
  ackPacket[9] = 0;	// CK_B
  
   // Calculate the checksums
  for(uint8_t ubxi=2; ubxi<8; ubxi++)
  {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  
  while (1)
  {
    // Test for success
    if (ackByteID > 9)
    {
      // All packets in order!
      return true;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000)
    {
      return false;
    }
    // Make sure data is available to read
    if (Serial.available())
    {
      b = Serial.read();
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID])
      {
        ackByteID++;
      }
      else
      {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
}


boolean _gps_verify_checksum(uint8_t* data, uint8_t len)//verifies the checksum of the data polled from Ublox
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  
  if(a != *(data+len) || b != *(data+len+1))
    return false;
  else
    return true;
}


void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb)//verifies the checksum of the data polled from Ublox
{
  *cka=0;
  *ckb=0;
  
  for(uint8_t i=0; i<len; i++)
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}


void gps_get_data()
{
  Serial.flush();
  
  for(int i=0;i<80;i++)
  {
    GPSbuffer[i]=0;
  }
  
  int i=0;
  unsigned long startTime=millis();
  
  while((i<80) && (millis()-startTime)<1000)
  {
    if(Serial.available())
    {
      GPSbuffer[i]=Serial.read();
      i++;
    }
  }
}


void gps_get_lock()
{
  GPSerror=0;
  Serial.flush();
  
  uint8_t request[8]={0xB5, 0x62, 0x01, 0x06, 0x00, 0x00, 0x07, 0x16};
  sendUBX(request, 8);
  
  gps_get_data();
  
  if(GPSbuffer[0] != 0xB5 || GPSbuffer[1] != 0x62)
  {
    GPSerror=11;
  }
  
  if(GPSbuffer[2] != 0x01 || GPSbuffer[3] != 0x06)
  {
    GPSerror=12;
  }
  
  if(!_gps_verify_checksum(&GPSbuffer[2], 56))
  {
    GPSerror=13;
  }
  
  if(GPSerror==0)
  {
    if(GPSbuffer[17] & 0x01)
    {
      lock=GPSbuffer[16];
    }
    else
    {
      lock=0;
    }
    
    sats=GPSbuffer[53];
  }
  else
  {
    lock=0;
    sats=0;// in AVA,the previous value remains
    
    //getLockTrouble++;//debugging
  }
}


void gps_get_time()
{
  //hour=0;//to see whether the comunication with Ublox broke down
  //minute=0;
  //second=0;
  
  GPSerror=0;
  Serial.flush();
  
  uint8_t request[8]={0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67};
  sendUBX(request, 8);
  
  gps_get_data();
  
  if(GPSbuffer[0] != 0xB5 || GPSbuffer[1] != 0x62)
  {
    GPSerror=31;
  }
  
  if(GPSbuffer[2] != 0x01 || GPSbuffer[3] != 0x21)
  {
    GPSerror=32;
  }
  
  if(!_gps_verify_checksum(&GPSbuffer[2], 24))
  {
    GPSerror=33;
  }
  
  if(GPSerror==0)
  {
    if(hour>23 || minute>59 || second>59)
    {
      GPSerror=34;
    }
    else
    {
      hour=GPSbuffer[22];
      minute=GPSbuffer[23];
      second=GPSbuffer[24];
    }
  }
  //if(GPSerror>0) getTimeTrouble++;//debugging
}


void gps_get_position()
{
  //lat=0;//to see whether the comunication with Ublox broke down
  //lon=0;
  //alt=0;
  
  GPSerror=0;
  Serial.flush();
  
  uint8_t request[8]={0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A};
  sendUBX(request, 8);
  
  gps_get_data();
  
  if(GPSbuffer[0] != 0xB5 || GPSbuffer[1] != 0x62)
  {
    GPSerror=21;
  }
  
  if(GPSbuffer[2] != 0x01 || GPSbuffer[3] != 0x02)
  {
    GPSerror=22;
  }
  
  if(!_gps_verify_checksum(&GPSbuffer[2], 32))
  {
    GPSerror=23;
  }
  
  if(GPSerror==0)
  {
    lon= (int32_t)GPSbuffer[10] | (int32_t)GPSbuffer[11] << 8 | (int32_t)GPSbuffer[12] << 16 | (int32_t)GPSbuffer[13] << 24;
    
    lon_int=abs(lon/10000000);//AVA
    lon_dec=(labs(lon) % 10000000)/100;//AVA
    
    lat= (int32_t)GPSbuffer[14] | (int32_t)GPSbuffer[15] << 8 | (int32_t)GPSbuffer[16] << 16 | (int32_t)GPSbuffer[17] << 24;
    
    lat_int=abs(lat/10000000);//AVA
    lat_dec=(labs(lat) % 10000000)/100;//AVA
    
    alt= (int32_t)GPSbuffer[22] | (int32_t)GPSbuffer[23] << 8 | (int32_t)GPSbuffer[24] << 16 | (int32_t)GPSbuffer[25] << 24;
    alt/=1000;    
  }
  //else//debugging
  //{
  //  getPosTrouble++;
  //}
}


uint8_t gps_check_nav(void)
{
  GPSerror=0;
  Serial.flush();
  
  uint8_t request[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 };
  sendUBX(request, 8);
  // Get the message back from the GPS
  gps_get_data();
  // Verify sync and header bytes
  if(GPSbuffer[0] != 0xB5 || GPSbuffer[1] != 0x62 )
  {
    GPSerror = 41;
  }
  if( GPSbuffer[2] != 0x06 || GPSbuffer[3] != 0x24 )
  {
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&GPSbuffer[2], 40) ) 
  {
    GPSerror = 43;
  }
  // Return the navigation mode and let the caller analyse it
  navmode=GPSbuffer[8];
  
  //if(GPSerror>0) getNavTrouble++;
}


//
//
//GENERAL FUNCTIONS
//
//
void wait(unsigned long delaytime)
{
  unsigned long _delaytime=millis();
  
  while((_delaytime+delaytime)>=millis()){}
}


uint16_t CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
  crc = 0xFFFF;
  // Calculate checksum ignoring the first two $s - original 5, AVA uses $$$$$AVA
  for (i=2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
  return crc;
}


//
//
//INTERRUPT FUNCTIONS
//
//
void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / (TX_BAUD - 1);  // set compare match register to desired timer count
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


ISR(TIMER1_COMPA_vect)// RTTY Interrupt Routine - ANTHONY STIRK-BETTER
{
  switch (txstatus)
  {
    case 0: // This is the optional delay between transmissions.
      txj++;
      if(txj>(TXDELAY*TX_BAUD))//Delay*Baud Rate
      {
        txj=0;
        txstatus=1;
      }
      break;
    case 1: // Initialise transmission
      if(sentenceReady)//ADDED
      {
        txstringlength=strlen(TXbuffer);
        txstatus=2;
        txj=0;
        sentenceReady=0;
      }
      break;
    case 2: // Grab a char and lets go transmit it.
      if ( txj < txstringlength)
      {
        txc = TXbuffer[txj];
        txj++;
        txstatus=3;
        rtty_txbit (0); // Start Bit;
        txi=0;
      }
      else
      {
        txstatus=1; // Should be finished - CHANGED go straight to case 1
        txj=0;
        count++;
        countreset++;
        //lockvariables=0; // ADDED
      }
      break;
    case 3:
      if(txi<ASCII)//txi<ASCII - testuju 8 kvůli °
      {
        txi++;
        if (txc & 1) rtty_txbit(1);
        else rtty_txbit(0);
        txc = txc >> 1;
        break;
      }
      else
      {
        rtty_txbit (1); // Stop Bit
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

