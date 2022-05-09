#include <Arduino.h>
#include <elapsedMillis.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <time.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <SPIMemory.h>
#include <MCP79412RTC.h>
#include <SparkFun_ADXL345.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <RadioLib.h>
#include <LowPower.h>

///// LIBRARY DECLARATIONS /////
TinyGPSPlus gps;
SPIFlash flash(1);
ADXL345 adxl = ADXL345();
elapsedMillis mTime;
SoftwareSerial gps_serial(19,20);
SX1276 radio = new Module(4,0,3,RADIOLIB_NC);
FSK4Client fsk4(&radio);

///// PIN DEFINITIONS /////
#define GPS_PIN A0
#define RTC_PIN 21
const int PROGMEM RINT = 2;
const int PROGMEM AINT = 11;
const int PROGMEM LCS = 4;
const int PROGMEM LRST = 3;
const int PROGMEM LDIO = 0;

///// DEVICE DEFINITIONS /////

const uint16_t tag = 10112;
const uint8_t devType = 107;

///// VARIABLES /////

// Flash Adressess //
uint32_t wAdd = 0;                // Write Address Parameter
uint32_t rAdd = 0;                // Read Address Parameter


// Device Variables //
uint16_t cnt;                     // No. of data points available for download

// Time Variable //
time_t strtTime = 1640841540;     // Device Time Set During Start Up
time_t pingTime;                  // Ping Alram set in code
time_t gpsTime;                   // GPS Alarm Set in code
time_t PingAlarmTime = 1640841600;             // Stores scheduled ping time in scheduled mode *** USER CONFIG *** x

// Volatile Variables //
volatile bool Alarm_Trig = false; // Alarm 1
volatile bool trigger = false;    // Activity Trigger

// Other Variables //
bool wipe = true;                // Enable/Disable wiping of Flash Memory *** USER CONFIG *** x
bool act_mode = false;             // Activty Mode Parameter
bool scheduled = false;            // Enable or diable schedule mode *** USER CONFIG *** x
bool window = false;              // Schedule window on/off parameter
bool activity_enabled = false;     // Enable/Disable Activity mode *** USER CONFIG *** x

// Accelerometer Variables //
int act_treshold = 30;            // Activity threshold 0-255 *** USER CONFIG *** 
int act_gpsFrequency = 15;         // Activity mode GPS Frequency *** USER CONFIG ***
int act_duration = 60;             // Activity mode duration in minutes *** USER CONFIG ***
time_t act_start;                 // activity mode start time
time_t act_end;                   // activity mode end time

// GPS Control Variables //
int gpsFrequency = 60;            // GPS Frequency in minutes *** USER CONFIG ***
int gpsTimeout = 60;              // GPS Timesout after 'x' seconds *** USER CONFIG ***
int gpsHdop = 5;                  // GPS HODP Parameter *** USER CONFIG ***

// GPS Storage Variables //
float lat;                        // Storing last known Latitude
float lng;                        // Storign last known Longitude

// Radio Variables //
int radioFrequency = 1;           //Frequency of Pings in minutes *** USER CONFIG ***
int rcv_duration = 5;             // Receive Window Duration in seconds *** USER CONFIG ***
time_t sch_start;                 // Last Schedule Start Time 
time_t sch_end;                   // Last Schedule End Time
int sch_duration = 5;             // Schedule Window Duration in mins *** USER CONFIG ***
int sch_rpt_duration = 10;         // Schedule repeat time in days *** USER CONFIG ***

// VHF Packets //
int horusPacketLen = 4;
byte horusPacket[] = {
  0x45, 0x24, 0x1B, 0X1A
};

//...................................//
//           FUNCTIONS               //
//...................................//

void activationPing(){

  struct ping{
    uint16_t tag;
    byte request;
  }px1;

  struct resp{
    uint16_t tag;
    byte resp;
  }rs1;

  int x;

  px1.tag = tag;
  px1.request = (byte)73;


  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&px1, sizeof(px1));
  LoRa.endPacket();

  mTime = 0;
  while (mTime < 30000)
  {
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(F("Incoming1"));
      Serial.println(x);
    }
    
    if (x == 3)
    {
      while (LoRa.available())
      {
        Serial.println(F("Incoming"));
        LoRa.readBytes((uint8_t*)&rs1, sizeof(rs1));
      }
      break;      
    }   

    
  } 
  LoRa.sleep();

  if (rs1.tag == tag && rs1.resp == (byte)70)
  {
    Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
    Serial.print(F("System Initialising"));
     /// Begin GPS and Acquire Lock ////
    // digitalWrite(GPS_PIN, HIGH);
    //   do{ 
    //     while (gps_serial.available() > 0)
    //     {
    //       if (gps.encode(gps_serial.read()))
    //       {
    //         if (!gps.location.isValid())
    //         {
    //           Serial.println(F("Not Valid"));
    //         }else{
    //           Serial.println(gps.location.isUpdated());
    //           Serial.print("Location Age:");
    //           Serial.println(gps.location.age());
    //           Serial.print("Time Age:");
    //           Serial.println(gps.time.age());
    //           Serial.print("Date Age:");
    //           Serial.println(gps.date.age());
    //           Serial.print("Satellites:");
    //           Serial.println(gps.satellites.value());
    //           Serial.print("HDOP:");
    //           Serial.println(gps.hdop.hdop());
    //         }
    //       }
    //     }
    //   }while(!gps.location.isValid());
    // if (gps.location.age() < 60000)
    // {
    //   //pack data into struct
    //   lat = gps.location.lat();
    //   lng = gps.location.lng();
    // }
    // if (gps.time.isValid())
    // {
    //   setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
    //   time_t n = now();
    //   strtTime = n;
    //   Serial.print(F("START TIME")); Serial.println(strtTime);
    // }
    
    // digitalWrite(GPS_PIN, LOW);
    
    wipe = true;

    px1.request = (byte)106;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
  }

  if (rs1.tag == tag && rs1.resp == (byte)71)
  {
    Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    digitalWrite(GPS_PIN, HIGH);
      do{ 
        while (gps_serial.available())
        {
          if (gps.encode(gps_serial.read()))
          {
            if (!gps.location.isValid())
            {
              Serial.println(F("Not Valid"));
            }else{
              Serial.println(gps.location.isUpdated());
              Serial.print("Location Age:");
              Serial.println(gps.location.age());
              Serial.print("Time Age:");
              Serial.println(gps.time.age());
              Serial.print("Date Age:");
              Serial.println(gps.date.age());
              Serial.print("Satellites:");
              Serial.println(gps.satellites.value());
              Serial.print("HDOP:");
              Serial.println(gps.hdop.hdop());
            }
          }
        }
      }while(!gps.location.isValid());
    if (gps.location.age() < 60000)
    {
      //pack data into struct
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      time_t n = now();
      strtTime = n;
      Serial.print(F("START TIME")); Serial.println(strtTime);
    }
    digitalWrite(GPS_PIN, LOW);

    wipe = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();

  }

  if (rs1.tag == tag && rs1.resp == (byte)115){
        Serial.print(F("Indefinite Sleep"));
        EEPROM.put(1, false);
        Serial.println(EEPROM.read(1));
        delay(50);
        disablePower(POWER_ADC);
        disablePower(POWER_SERIAL0);
        disablePower(POWER_SERIAL1);
        disablePower(POWER_SPI);
        disablePower(POWER_WIRE);
        sleepMode(SLEEP_POWER_DOWN);
        sleep();
  }

  Serial.println(EEPROM.read(1));

  if (EEPROM.read(1) == false){
    Serial.println(F("SLEEP1"));
    delay(50);
    disablePower(POWER_ADC);
    disablePower(POWER_SERIAL0);
    disablePower(POWER_SERIAL1);
    disablePower(POWER_SPI);
    disablePower(POWER_WIRE);
    sleepMode(SLEEP_POWER_DOWN);
    sleep();
  }else{
    Serial.println(F("Reset"));
  }
    
}

void isr(){
  trigger = true;
  noInterrupts();
  noSleep();
}

void risr(){
  Alarm_Trig = true;
  noInterrupts();
  noSleep();
}

void recGPS(){
  mTime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println(gpsTimeout*1000);
  while (mTime <= (unsigned)gpsTimeout*1000)
  {
    while (gps_serial.available())
    {
      if (gps.encode(gps_serial.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println("Acquiring");
        }else{
          Serial.println(gps.location.isUpdated());
          Serial.print("Location Age:");
          Serial.println(gps.location.age());
          Serial.print("Time Age:");
          Serial.println(gps.time.age());
          Serial.print("Date Age:");
          Serial.println(gps.date.age());
          Serial.print("Satellites:");
          Serial.println(gps.satellites.value());
          Serial.print("HDOP:");
          Serial.println(gps.hdop.hdop());
        }       
      }      
    }
    if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000)
    {
      break;
    }  
  }   

  digitalWrite(GPS_PIN, LOW);
  struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
    bool act;
    }dat;

  if (gps.location.age() < 60000)
  {
    //pack data into struct
    lat = gps.location.lat();
    lng = gps.location.lng();
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
  }else{
    // pack data into struct with lat long = 0
    dat.lat = 0;
    dat.lng = 0;
  }
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    dat.datetime = (uint32_t)now();
    dat.locktime = mTime/1000;
    dat.hdop = gps.hdop.hdop();
    if(act_mode == true){
      dat.act = true;
    }else{
      dat.act = false;
    }
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);
    Serial.println(dat.act);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    wAdd = flash.getAddress(sizeof(dat));
    // Serial.println(sizeof(dat));
    Serial.println(wAdd);
    if (flash.writeAnything(wAdd, dat))
    {
      Serial.println(F("Write Successful"));
      cnt = cnt + 1;
    }else
    {
      Serial.println(F("Write Failed"));
      Serial.println(flash.error(VERBOSE));
    }     
  }else
  {
    Serial.println(F("Power Up Failed"));
  }   
  flash.powerDown();

}

void read_send(){ 
  struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
    bool act;
    }dat;

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      // dat.id = tag;
      Serial.println(F("Read Successful"));
      Serial.println(dat.datetime);
      Serial.println(dat.hdop);
      Serial.println(dat.lat);
      Serial.println(dat.lng);
      Serial.println(dat.locktime);
      Serial.println(dat.act);
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }
      LoRa.idle();
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&dat, sizeof(dat));
      LoRa.endPacket();
      LoRa.sleep();
}

void Ping(float x, float y, uint16_t a, uint16_t c, byte d){

  struct ping{
    uint16_t ta;    
    uint16_t cnt;
    float la;
    float ln;
    uint8_t devtyp;
  }p;
  p.devtyp = d;
  p.ta = a;
  p.la = x;
  p.ln = y;
  p.cnt = c;
  Serial.print("Size"); Serial.println((int)sizeof(p));
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.sleep();
  
}

void receive(unsigned int time){
  Serial.println(F("Receiving"));
  LoRa.idle();
  mTime = 0;
  int x;
  do
  {  
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(x);
    }
    
    
    if (x == 3)
    { 
      Serial.print(F("int : ")); Serial.println(x);
      struct request{
      uint16_t tag;
      byte request;
      }r;
      while (LoRa.available())
      {
        Serial.println(F("Reading in"));
        LoRa.readBytes((uint8_t*)&r, sizeof(r));
      }
      Serial.println(r.tag);
      Serial.println(r.request);
      if (r.tag == tag && r.request == (byte)82)
      {
        
        do
        {
          Serial.println("Init Stream");
          read_send();
          rAdd = rAdd + 16;
          Serial.println(rAdd);
        
        } while (rAdd <= wAdd);

        delay(1000);
        struct resp{
        uint16_t tag;
        byte res;
        }r;
        r.res = (byte) 68;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
      }            
    }

    if (x == 21)
    {
      struct setttings{
        uint32_t pingTime;
        uint16_t act_trsh;
        uint16_t act_gps_frq;
        uint16_t act_duration;
        uint16_t gpsFrq;
        uint16_t gpsTout;
        uint8_t hdop;
        uint8_t radioFrq;
        uint8_t rcv_dur;
        uint8_t sch_dur;
        uint8_t sch_rpt_dur;
        bool act_enabled;
        bool sch_enabled;
      } __attribute__((__packed__)) set;

      while (LoRa.available())
      {
        Serial.println(F("Incoming Settings"));
        LoRa.readBytes((uint8_t*)&set, sizeof(set));
      }

      PingAlarmTime = set.pingTime;
      act_treshold = set.act_trsh;
      act_gpsFrequency = set.act_gps_frq;
      act_duration = set.act_duration;
      gpsFrequency = set.gpsFrq;
      gpsTimeout = set.gpsTout;
      gpsHdop = set.hdop;
      radioFrequency = set.radioFrq;
      rcv_duration = set.rcv_dur;
      sch_duration = set.sch_dur;
      sch_rpt_duration = set.sch_rpt_dur;
      activity_enabled = set.act_enabled;
      scheduled = set.sch_enabled;

      Serial.println(set.pingTime);
      Serial.println(set.act_trsh);
      Serial.println(set.act_gps_frq);
      Serial.println(set.act_duration);
      Serial.println(set.gpsFrq);
      Serial.println(set.gpsTout);
      Serial.println(set.hdop);
      Serial.println(set.radioFrq);
      Serial.println(set.rcv_dur);
      Serial.println(set.sch_dur);
      Serial.println(set.sch_rpt_dur);
      Serial.println(set.act_enabled);
      Serial.println(set.sch_enabled);
      delay(100);

      if (activity_enabled == true)
      {
        adxl.ActivityINT(1);
        adxl.setActivityThreshold(act_treshold);
        Serial.print(F("ACTIVITY On"));
      }else{
        adxl.ActivityINT(0);
        if (act_mode == true)
        {
          act_mode = false;
        }        
        Serial.print(F("ACTIVITY OFF"));
      }

      if (scheduled == true)
      {
        digitalWrite(RTC_PIN, HIGH);
        RTC.alarmPolarity(HIGH);
        RTC.setAlarm(1, PingAlarmTime);
        RTC.enableAlarm(1, ALM_MATCH_DATETIME);
        sch_start = PingAlarmTime;
        sch_end = sch_start + sch_duration*60;
      }
      struct resp{
        uint16_t tag;
        byte res;
        }r;
        r.res = (byte)83;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
    }
  }while(mTime <= time);
  LoRa.sleep();
  delay(50);
}

void fsk(){
  
    int state = radio.beginFSK();
    if(state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while(true);
    }

    Serial.print(F("[FSK4] Initializing ... "));
    // low ("space") frequency:     434.0 MHz
    // frequency shift:             270 Hz
    // baud rate:                   100 baud
    state = fsk4.begin(150.5, 270, 100);
    if(state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while(true);
    }

    fsk4.idle();
    delay(10);

    fsk4.write(horusPacket, horusPacketLen);
    // Begin LoRa Radio//
    LoRa.setPins(LCS, LRST, LDIO);
    if(!LoRa.begin(867E6)){
      Serial.println(F("LoRa Failed Init"));
    }
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSpreadingFactor(12);
    // LoRa.setSignalBandwidth(62.5E3);
    LoRa.sleep();
}

void setup() {
  // put your setup code here, to run once:

  enablePower(POWER_ADC);
  enablePower(POWER_SERIAL0);
  enablePower(POWER_SERIAL1);
  enablePower(POWER_SPI);
  enablePower(POWER_WIRE);

  pinMode(RTC_PIN, OUTPUT);
  pinMode(AINT, INPUT);
  pinMode(RINT, INPUT);
  digitalWrite(RINT, LOW);

  Serial.begin(9600);
  gps_serial.begin(9600);
  delay(1000);

  Serial.println(F("SYSTEM INIT..."));

  // Begin LoRa Radio//
  LoRa.setPins(LCS, LRST, LDIO);
  if(!LoRa.begin(867E6)){
    Serial.println(F("LoRa Failed Init"));
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(12);
  // LoRa.setSignalBandwidth(62.5E3);
  LoRa.sleep();

  // Activation Ping //
  // activationPing();

  // Enable & Start Flash //
  
  if(flash.powerUp()){
    Serial.println(F("Powered Up1"));
  }
  if(!flash.begin()){
    Serial.println(F("Flash again"));
    Serial.println(flash.error(VERBOSE));
  } 
  Serial.println(flash.getManID());
  if(flash.powerUp()){
    Serial.println(F("Powered Up"));
  }else
  {
    Serial.println(F("PWR UP Failed!"));
  }
  if (wipe == true)
  {
    Serial.println(F("WIPING FLASH"));
    if(flash.eraseChip()){
    Serial.println(F("Memory Wiped"));  
    }else
    {
      Serial.println(flash.error(VERBOSE));
    }
  }else{
    rAdd = flash.getAddress(16);
  }    
  if(flash.powerDown()){
    Serial.println("Powered Down");
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

  // Set Up Accelerometer //
  adxl.powerOn();
  adxl.setRangeSetting(8);
  adxl.set_bw(ADXL345_BW_0_78);
  adxl.setLowPower(true);
  // Config Interrupts //  
  adxl.setImportantInterruptMapping(1,1,1,1,1);
  adxl.setActivityXYZ(1,1,1);
  adxl.setActivityThreshold(act_treshold);
  if (activity_enabled == true)
  {
    adxl.ActivityINT(1);
  }else{
    adxl.ActivityINT(0);
  }
  
  adxl.ActivityINT(1);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);
  adxl.InactivityINT(0);
  adxl.FreeFallINT(0);

  adxl.getInterruptSource();
  
  digitalWrite(RTC_PIN, HIGH);
  RTC.set(strtTime);
  Serial.println(RTC.get());
  delay(100);
  RTC.alarmPolarity(HIGH);
  RTC.setAlarm(0, strtTime + 60*gpsFrequency);
  if (scheduled == true)
  {
    RTC.setAlarm(1, PingAlarmTime);
  }else{
    RTC.setAlarm(1, strtTime + 60*radioFrequency);
    pingTime = strtTime + 60*radioFrequency;
  }
  
  gpsTime = strtTime + 60*gpsFrequency;
  RTC.enableAlarm(0, ALM_MATCH_DATETIME);
  RTC.enableAlarm(1, ALM_MATCH_DATETIME);
  digitalWrite(RTC_PIN, LOW);
  delay(100);
  
  // // Attach Interupt //
  attachInterrupt(digitalPinToInterrupt(AINT), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RINT), risr, CHANGE);

  Serial.println(F("SYSTEM READY"));
  delay(100);
  disablePower(POWER_ADC);
  disablePower(POWER_SERIAL0);
  disablePower(POWER_SERIAL1);
  disablePower(POWER_SPI);
  disablePower(POWER_WIRE); 
  
  // sleepMode(SLEEP_POWER_DOWN);
  // sleep();

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  

}

void loop() {
  // put your main code here, to run repeatedly:

  enablePower(POWER_ADC);
  enablePower(POWER_SERIAL0);
  enablePower(POWER_SERIAL1);
  enablePower(POWER_SPI);
  enablePower(POWER_WIRE);

  Serial.println(F("AWAKE"));
  if(trigger == true){                                            
    byte interrupts = adxl.getInterruptSource();
    if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
    Serial.println(F("*** ACTIVITY ***")); 
    digitalWrite(RTC_PIN, HIGH);
    act_start = RTC.get();
    Serial.println(act_start);
    RTC.setAlarm(0, act_start + 10);
    gpsTime = act_start + 10;
    RTC.alarmPolarity(HIGH);
    digitalWrite(RTC_PIN, LOW);
    RTC.enableAlarm(0, ALM_MATCH_DATETIME);
    act_end = act_start + act_duration*60;  
    act_mode = true;  
    }
    
    attachInterrupt(digitalPinToInterrupt(AINT), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RINT), risr, CHANGE);
    delay(1000);
    adxl.getInterruptSource();
  }
  if (Alarm_Trig == true)
  {
    digitalWrite(RTC_PIN, HIGH);
    if(RTC.alarm(0)){
      Serial.println(F("ALARM 0"));
      time_t prevTime = RTC.get();
      if (act_mode == true && prevTime >= act_end)
      {
        act_mode = false;
        Serial.println(F("ACTIVITY MODE ENDING"));
      }      
      Serial.println(RTC.get());
      delay(10);
      RTC.alarmPolarity(HIGH);
      if (act_mode ==true)
      {
        RTC.setAlarm(0, gpsTime + 60*act_gpsFrequency);
        gpsTime = gpsTime + 60*act_gpsFrequency;
      }else{
        RTC.setAlarm(0, gpsTime + 60*gpsFrequency);
        gpsTime = gpsTime + 60*gpsFrequency;
      }      
      RTC.enableAlarm(0, ALM_MATCH_DATETIME);
      recGPS();
      ////////////////////////////////////////////////
      prevTime = RTC.get();
      RTC.alarmPolarity(HIGH);
      RTC.setAlarm(1, prevTime + 60*radioFrequency);
      pingTime = prevTime + 60*radioFrequency;    
      RTC.enableAlarm(1, ALM_MATCH_DATETIME);   
      /////////////////////////////////////////////// 
    }
    if(RTC.alarm(1)){
      Serial.println(F("ALARM 1"));
      Serial.println(RTC.get());
      delay(10);
      if (scheduled == false)
      {
        Serial.println(F("Normal Mode"));  
        RTC.alarmPolarity(HIGH);
        RTC.setAlarm(1, pingTime + 60*radioFrequency);
        pingTime = pingTime + 60*radioFrequency;
      } 

      if (scheduled == true && window == false)
      {
        Serial.println(F("Entering Scheduled Mode"));
        sch_start = RTC.get();
        sch_end = sch_start + sch_duration*60;
        PingAlarmTime = sch_start + 86400*sch_rpt_duration;
        Serial.print(F("PingAlmTime")); Serial.println(PingAlarmTime);
        window = true;
        pingTime = sch_start;
        delay(50);
      }
      if (scheduled == true && window == true)
      {
        time_t n = RTC.get();
        if (n >= sch_end)
        {
          Serial.println(F("Exiting Scheduled Mode"));
          window = false;
          RTC.setAlarm(1, PingAlarmTime);
        }else{
          RTC.setAlarm(1, pingTime + 60*radioFrequency);
          pingTime = pingTime + 60*radioFrequency;
          Serial.println(pingTime);
        }        
      }
      RTC.enableAlarm(1, ALM_MATCH_DATETIME);
      Ping(lat,lng,tag, cnt, devType);
      receive(rcv_duration*1000);
      
        
    }
    digitalWrite(RTC_PIN, LOW);

    adxl.getInterruptSource();
    attachInterrupt(digitalPinToInterrupt(AINT), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RINT), risr, CHANGE);
  }
  delay(10); 

  fsk();
  
  Serial.println(F("SLEEPING..."));
  delay(50); 
  disablePower(POWER_ADC);
  disablePower(POWER_SERIAL0);
  disablePower(POWER_SERIAL1);
  disablePower(POWER_SPI);
  disablePower(POWER_WIRE); 

  // sleepMode(SLEEP_POWER_DOWN);
  // sleep();
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

}




