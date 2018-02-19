Description :
A low power VL53L0X based ToF sensor for the ThingsNetwork with deepsleep support and variable interval.

Revision :
2018-feb-18 1.0 first "beta"

Hardware used :
Arduino Pro-Mini 3.3V
RFM95W
VL53L0X sensor.

Software and libraries used :

LMIC https://github.com/matthijskooijman/arduino-lmic	
LowPower library https://github.com/rocketscream/Low-Power
special adcvcc library from Charles (see : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059/32?u=lex_ph2lb )
VL53L0X library https://github.com/pololu/vl53l0x-arduino
For licenses of the used libraries, check the links above.

Aditional note :

I use a HTTP integration on the TTN with the decoder / payload function below. :

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
