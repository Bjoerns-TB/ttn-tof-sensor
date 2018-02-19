/*  
 * ------------------------------------------------------------------------
 * "PH2LB LICENSE" (Revision 1) : (based on "THE BEER-WARE LICENSE" Rev 42) 
 * <lex@ph2lb.nl> wrote this file. As long as you retain this notice
 * you can do modify it as you please. It's Free for non commercial usage 
 * and education and if we meet some day, and you think this stuff is 
 * worth it, you can buy me a beer in return
 * Lex Bolkesteijn 
 * ------------------------------------------------------------------------ 
 * Modified by Björn Amann
 * Code for BME280 deletetd and for  VL53L0X integrated
 * bjoern@bjoernamann.de
 * ------------------------------------------------------------------------ 
 * Filename : LoRaWAN_TTN_ToF_Node_OTAA.ino  
 * Version  : 1.0 (BETA)
 * ------------------------------------------------------------------------
 * Description : A low power VL53L0X based ToF sensor for the ThingsNetwork.
 *  with deepsleep support and variable interval
 * ------------------------------------------------------------------------
 * Revision : 
 *  - 2018-feb-18 1.0
 * ------------------------------------------------------------------------
 * Hardware used : 
 *  - Arduino Pro-Mini 3.3V 
 *  - RFM95W
 *  - VL53L0X sensor.
 * ------------------------------------------------------------------------
 * Software used : 
 *  - LMIC https://github.com/matthijskooijman/arduino-lmic 
 *  - LowPower library https://github.com/rocketscream/Low-Power
 *  - special adcvcc library from Charles (see : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059/32?u=lex_ph2lb )
 *  - VL53L0X library https://github.com/pololu/vl53l0x-arduino
 *  
 * For licenses of the used libraries, check the links above.
 * ------------------------------------------------------------------------ 
 *  
 *  
 **************************************************************************************** 
 * TheThingsNetwork Payload functions : 
    function Decoder(bytes, port)  
    {
      var retValue =   { 
        bytes: bytes
      };
      
      retValue.batt = bytes[0] / 10.0;

    
      if (bytes.length >= 2)
      {
        retValue.distance = ((bytes[1] << 8) | bytes[2]);
      } 
       
      return retValue; 
    }
 */

 
/*
*****************************************************************************************
* INCLUDE FILES
*****************************************************************************************
*/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h> 
#include  "adcvcc.h"  
#include <VL53L0X.h>


VL53L0X sensor;
int val;

/*
 * Defines
 */ 

#define debugSerial Serial  
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); } 

// use low power sleep; comment next line to not use low power sleep
#define SLEEP

#ifdef SLEEP
  #include "LowPower.h"
  bool next = false;
#endif

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 304; //multiple of 8

// Pin mapping CH2I (check out : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059 ) 
#define LMIC_NSS    10
#define LMIC_RXTX   LMIC_UNUSED_PIN
#define LMIC_RST    LMIC_UNUSED_PIN
#define LMIC_DIO0   2
#define LMIC_DIO1   7
#define LMIC_DIO2   8


const lmic_pinmap lmic_pins = {
    .nss = LMIC_NSS,
    .rxtx = LMIC_RXTX,   
    .rst = LMIC_RST,
    .dio = {LMIC_DIO0, LMIC_DIO1, LMIC_DIO2},  
}; 

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0xFB, 0x9E, 0x00, 0xD0, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x97, 0x63, 0x65, 0x6E, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x01, 0xFD, 0xF0, 0xFE, 0xED, 0xD3, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
 
static osjob_t sendjob; 

   
ISR(ADC_vect)  
{
  // Increment ADC counter
  _adc_irq_cnt++;
}


 

void onEvent (ev_t ev) 
{
    debugPrint(os_getTime());
    debugPrint(": ");
    debugPrintLn(ev);
    switch(ev) 
    {
        case EV_SCAN_TIMEOUT:
            //debugPrintLn(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            //debugPrintLn(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            //debugPrintLn(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            //debugPrintLn(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            //debugPrintLn(F("EV_JOINING"));
            break;
        case EV_JOINED:
            //debugPrintLn(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            //debugPrintLn(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            //debugPrintLn(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            //debugPrintLn(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            debugPrintLn(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
              debugPrintLn(F("R ACK")); // Received ack
            if (LMIC.dataLen) 
            {
              debugPrintLn(F("R "));
              debugPrintLn(LMIC.dataLen);
              debugPrintLn(F(" bytes")); // of payload
            }            
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            next = true; 
            break;
        case EV_LOST_TSYNC:
            //debugPrintLn(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //debugPrintLn(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            //debugPrintLn(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //debugPrintLn(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //debugPrintLn(F("EV_LINK_ALIVE"));
            break;
         default:
            //debugPrintLn(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        debugPrintLn(F("OP_TXRXPEND")); //P_TXRXPEND, not sending
    } 
    else 
    {
            val = sensor.readRangeSingleMillimeters();
            Serial.print(val);
            if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
            Serial.println();
        
        int batt = (int)(readVcc() / 100);  // readVCC returns  mVolt need just 100mVolt steps
        byte batvalue = (byte)batt; // no problem putting it into a int. 


        unsigned char mydata[3];
        mydata[0] = batvalue;
        mydata[1] = val >> 8;      
        mydata[2] = val & 0xFF; 

       LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
       debugPrintLn(F("PQ")); //Packet queued
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setup() {
    Serial.begin(115200);
    
	Wire.begin();
  sensor.init();
  sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

//    Serial.print("Firmware Version = ");
//    Serial.println(gas.getVersion());
  
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    do_send(&sendjob);
  
}

void loop() 
{ 
  extern volatile unsigned long timer0_overflow_count;

  if (next == false) {

    os_runloop_once();

  } else {

    int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
    Serial.flush(); // give the serial print chance to complete
    for (int i=0; i<sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

      // LMIC uses micros() to keep track of the duty cycle, so
      // hack timer0_overflow for a rude adjustment:
      cli();
      timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
      sei();
    }
    next = false;
    // Start job
    do_send(&sendjob);

    }
 


}
