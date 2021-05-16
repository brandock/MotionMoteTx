// Brandon Baldock July 2018: I modified Felix Rusu's sketch for the MotionMote to use the Open Energy Monitor (JeeLib)
// style of communication (using Nathan Chantrell's tinyTx sketch for as a basis for that).
//
// Credits:
//----------------------------------------------------------------------------------------------------------------------
// TinyTX Simple Receive Example
// By Nathan Chantrell. 
//
// https://github.com/nathanchantrell/TinyTX
// Licenced under the Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0) licence:
// http://creativecommons.org/licenses/by-sa/3.0/
//----------------------------------------------------------------------------------------------------------------------
// Sample RFM69 sender/node sketch for the MotionMote
// By Felix Rusu
//
// https://lowpowerlab.com/guide/motionmote/
// PIR motion sensor connected to D3 (INT1)
// When RISE happens on D3, the sketch transmits a "MOTION" msg to receiver Moteino and goes back to sleep
// In sleep mode, Moteino + PIR motion sensor use about ~60uA
// With Panasonic PIRs it's possible to use only around ~9uA, see guide link above for details
// IMPORTANT: adjust the settings in the configuration section below !!!
// **********************************************************************************
// Copyright Felix Rusu of LowPowerLab.com, 2018
// RFM69 library and sample code by Felix Rusu - lowpowerlab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#define RF69_COMPAT 1
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include "EmonLib.h"

#include <LowPower.h>      //get library from: https://github.com/lowpowerlab/lowpower
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SparkFunBME280.h>//get it here: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/master/src
#include <Wire.h>          //comes with Arduino

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define myNodeID 9      // RF12 node ID in the range 1-30
#define network 210      // RF12 Network group
#define freq RF12_433MHZ // Frequency of RFM12B module


#define ENABLE_BME280 //uncomment to allow reading the BME280 (if present)
//*********************************************************************************************
//#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

#define ONBOARDLED     9  // Moteinos have LEDs on D9
#define LED            5  // MotionOLEDMote has an external LED on D5
#define MOTION_PIN     3  // D3
#define MOTION_IRQ     1  // hardware interrupt 1 (D3) - where motion sensors OUTput is connected, this will generate an interrupt every time there is MOTION
#define BATT_MONITOR  A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 883 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
#define BATT_FORMULA(reading) reading * 0.00322 * 1.49 // >>> fine tune this parameter to match your voltage when fully charged
                                                       // details on how this works: https://lowpowerlab.com/forum/index.php/topic,1206.0.html
#define DUPLICATE_INTERVAL 5000 //avoid duplicates in 55second intervals (ie mailman sometimes spends 30+ seconds at mailbox)
#define BATT_INTERVAL  300000  // Battery reads are hard-coded in the read function to 30 seconds. This is the temp sensor interval.

#define SERIAL_EN             //comment this out when deploying to an installed Mote to save a few KB of sketch size
#define SERIAL_BAUD    115200
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#define DEBUGFlush() { Serial.flush(); }
#else
#define DEBUG(input);
#define DEBUGln(input);
#define DEBUGFlush();
#endif

#define FLASH_SS      8 // and FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

#ifdef ENABLE_BME280
  BME280 bme280;
#endif

//########################################################################################################################
//Data Structure to be sent
//########################################################################################################################

 typedef struct {
      int motion;
      int temp; 
      int humidity; 
      int pressure; 
      int supplyV;  // Supply voltage
 } Payload;

Payload tinytx;

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//-------------------------------------------------------------------------------------------------
 static void rfwrite(){
  #ifdef USE_ACK
   for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
      rf12_sleep(-1);              // Wake up RF module
      while (!rf12_canSend())
      rf12_recvDone();
      rf12_sendStart(RF12_HDR_ACK, &tinytx, sizeof tinytx); 
      rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
      byte acked = waitForAck();  // Wait for ACK
      rf12_sleep(0);              // Put RF module to sleep
      if (acked) { return; }      // Return if ACK received
  
   Sleepy::loseSomeTime(RETRY_PERIOD * 1000);     // If no ack received wait and try again
   }
  #else
     rf12_sleep(-1);              // Wake up RF module
     rf12_sendNow(0, &tinytx, sizeof tinytx); 
     rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
     rf12_sleep(0);              // Put RF module to sleep
     return;
  #endif
 }
 
volatile boolean motionDetected=false;
float batteryVolts = 5;
char BATstr[10]; //longest battery voltage reading message = 9chars
char sendBuf[32];
byte sendLen;
#ifdef ENABLE_BME280
  double F,P,H;
#endif

void motionIRQ(void);
void checkBattery(void);

void setup() {
  Serial.begin(SERIAL_BAUD);
  rf12_initialize(myNodeID,freq,network); // Initialize RFM12 with settings defined above 
  rf12_sleep(0);                          // Put the RFM12 to sleep
  
  pinMode(MOTION_PIN, INPUT);
  attachInterrupt(MOTION_IRQ, motionIRQ, RISING);

  pinMode(ONBOARDLED, OUTPUT);
  pinMode(LED, OUTPUT);

  if (flash.initialize()) flash.sleep(); //if Moteino has FLASH-MEM, make sure it sleeps

#ifdef ENABLE_BME280
  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  //initialize weather shield BME280 sensor
  bme280.setI2CAddress(0x77); //0x76,0x77 is valid.
  bme280.beginI2C();
  bme280.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
  bme280.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
  bme280.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
  bme280.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  bme280.setPressureOverSample(1); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  bme280.setHumidityOverSample(1); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
  F = bme280.readTempF();
  H = bme280.readFloatHumidity();
  bme280.setMode(MODE_SLEEP);
#endif
}

void motionIRQ()
{
  motionDetected=true;
  DEBUGln("IRQ");
}

uint16_t batteryReportCycles=0;
uint32_t time=0, now=0, MLO=0, BLO=0;
byte motionRecentlyCycles=0;

void loop() {
  now = millis();
  checkBattery();
  //DEBUG("Slept: ");DEBUG(now-lastSleepTime);DEBUGln("ms");
  
  if (motionDetected && (time-MLO > DUPLICATE_INTERVAL))
  {
    digitalWrite(LED, HIGH);
    MLO = time; //save timestamp of event

#ifdef ENABLE_BME280
    //read BME sensor
    bme280.setMode(MODE_FORCED); //Wake up sensor and take reading
    P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
    F = bme280.readTempF();
    H = bme280.readFloatHumidity();
    bme280.setMode(MODE_SLEEP);
    tinytx.temp = round(F * 10);
    tinytx.humidity = round(H * 10);
    tinytx.pressure = round(P * 100);
#endif

    tinytx.motion = motionDetected;

    rfwrite(); // Send data via RF 

    digitalWrite(LED, LOW);
  }
  else if (time-BLO > BATT_INTERVAL)
  {
#ifdef ENABLE_BME280
    //read BME sensor
    bme280.setMode(MODE_FORCED); //Wake up sensor and take reading
    P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
    F = bme280.readTempF();
    H = bme280.readFloatHumidity();
    bme280.setMode(MODE_SLEEP); 
    tinytx.temp = round(F * 10);
    tinytx.humidity = round(H * 10);
    tinytx.pressure = round(P * 100);
#endif
    tinytx.motion = motionDetected;


    rfwrite(); // Send data via RF 
    
    BLO = time;
  }
  
  DEBUGFlush();

  //while motion recently happened sleep for small slots of time to better approximate last motion event
  //this helps with debouncing a "MOTION" event more accurately for sensors that fire the IRQ very rapidly (ie panasonic sensors)
  if (motionDetected || motionRecentlyCycles>0)
  {
    if (motionDetected) motionRecentlyCycles=8;  //BB: this line may be necessary using the fast sensor
    else motionRecentlyCycles--;
    motionDetected=false; //do NOT move this after the SLEEP line below or motion will never be detected
    time = time + 250 + millis()-now;
    rf12_sleep(0);
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    DEBUGln("WAKEUP250ms");
  }
  else
  {
    time = time + 8000 + millis()-now;
    rf12_sleep(0);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    DEBUGln("WAKEUP8s");
  }
  batteryReportCycles++;
}

uint32_t BLR=0;
void checkBattery()
{
  if (time-BLR > 30000) //only read battery every 30s or so
  {
    unsigned int readings=0;
    BLR = time;
    for (byte i=0; i<10; i++) //take 10 samples, and average
      readings+=analogRead(BATT_MONITOR);
    batteryVolts = round(BATT_FORMULA(readings / 10.0) * 100); //Divided by 10 to get the average of 10 samples, multiply by 100 to send
    tinytx.supplyV = batteryVolts; //update the supplyV which gets sent every BATT_CYCLES or along with the MOTION message
  }
}
