/**************************************************************************/
/*!
    @file     controller.ino
    @author   ladyada, ktown (Adafruit Industries)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2015, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BLE_HWSPI.h"
#include "Adafruit_BLE_SWUART.h"

/*=========================================================================
    DATA TRANSPORT SELECTION
    Set TRANSPORT to an appropriate target, depending on the board you are
    using and the transport protocol it is based on
    -----------------------------------------------------------------------*/
    #define TRANSPORT_SWSERIAL              (1)
    #define TRANSPORT_HWSPI                 (2)
    #define TRANSPORT                       (TRANSPORT_SWSERIAL)
/*=========================================================================*/

/*=========================================================================
    SPI SETTINGS
    The following macros declare the pins to use for SPI communication
    -----------------------------------------------------------------------*/
    #define BLUEFRUIT_SPI_RST_PIN           (9)
    #define BLUEFRUIT_SPI_IRQ_PIN           (3)  // MUST be an interrupt pin (pin 2 or 3 on an Uno)!
    #define BLUEFRUIT_SPI_CS_PIN            (10)
    /* Use HW SPI pins for the other SPI functions. On an Uno this means:
         SCK  = 13
         MISO = 12
         MOSI = 11 */
/*=========================================================================*/

/*=========================================================================
    SOFTWARE SERIAL SETTINGS
    The following macros declare the pins used for SW serial
    -----------------------------------------------------------------------*/
    #define BLUEFRUIT_UART_RTS_PIN          (8)
    #define BLUEFRUIT_UART_RXD_PIN          (9)
    #define BLUEFRUIT_UART_TXD_PIN          (10)
    #define BLUEFRUIT_UART_CTS_PIN          (11)
    #define BLUEFRUIT_UART_MODE_PIN         (12)
/*=========================================================================*/

/*=========================================================================
    APPLICATION SETTINGS

    READ_BUFSIZE            Size of the read buffer for incoming data
    VERBOSE_MODE            If set to 1 enables full data output (for
                            debugging), otherwise set it to 0 to disable
                            verbose output
    BLE_READPACKET_TIMEOUT  The timeout in ms waiting for a data packet
    -----------------------------------------------------------------------*/
    #define READ_BUFSIZE                    (20)
    #define VERBOSE_MODE                    (0)
    #define BLE_READPACKET_TIMEOUT          (500)
    #define PACKET_ACC_LEN                  (15)
    #define PACKET_GYRO_LEN                 (15)
    #define PACKET_MAG_LEN                  (15)
    #define PACKET_QUAT_LEN                 (19)
    #define PACKET_BUTTON_LEN               (5)
    #define PACKET_COLOR_LEN                (6)
    #define PACKET_LOCATION_LEN             (15)
/*=========================================================================*/

/* Buffer to hold incoming characters */
uint8_t replybuffer[READ_BUFSIZE+1];

/* Constructors */
#if TRANSPORT == TRANSPORT_HWSPI
  Adafruit_BLE_HWSPI ble(BLUEFRUIT_SPI_CS_PIN,
                         BLUEFRUIT_SPI_IRQ_PIN /*, BLUEFRUIT_SPI_RST_PIN */);
#elif TRANSPORT == TRANSPORT_SWSERIAL
  Adafruit_BLE_SWUART ble(BLUEFRUIT_UART_RXD_PIN, 
                          BLUEFRUIT_UART_TXD_PIN,
                          BLUEFRUIT_UART_CTS_PIN, 
                          BLUEFRUIT_UART_RTS_PIN, 
                          BLUEFRUIT_UART_MODE_PIN);
#else
  #error No TRANSPORT defined! Please set this to an appropriate value.
#endif

/**************************************************************************/
/*!
    @brief  Helper MACROS to check command execution. Print 'FAILED!' or 'OK!',
            loop forever if failed
*/
/**************************************************************************/
#define EXECUTE(command)\
  do{\
    if ( !(command) ) { Serial.println( F("FAILED!") ); while(1){} }\
    Serial.println( F("OK!") );\
  }while(0)

/**************************************************************************/
/*! 
    @brief  Prints a hexadecimal value in plain characters
    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void printHex(const uint8_t * data, const uint32_t numBytes)
{
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++) 
  {
    Serial.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
    {
      Serial.print(F("0"));
      Serial.print(data[szPos] & 0xf, HEX);
    }
    else
    {
      Serial.print(data[szPos] & 0xff, HEX);
    }
    // Add a trailing space if appropriate
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      Serial.print(F(" "));
    }
  }
  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Casts the four bytes at the specified address to a float
*/
/**************************************************************************/
float parsefloat(uint8_t *buffer) 
{
  float f = ((float *)buffer)[0];
  return f;
}

/**************************************************************************/
/*!
    @brief  Waits for incoming data and parses it
*/
/**************************************************************************/
uint8_t readPacket(uint16_t timeout = BLE_READPACKET_TIMEOUT) 
{
  uint16_t replyidx = 0;

  while (timeout--) {
    if (replyidx >= 20) break;
    if ((replybuffer[1] == 'A') && (replyidx == PACKET_ACC_LEN))
      break;
    if ((replybuffer[1] == 'G') && (replyidx == PACKET_GYRO_LEN))
      break;
    if ((replybuffer[1] == 'M') && (replyidx == PACKET_MAG_LEN))
      break;
    if ((replybuffer[1] == 'Q') && (replyidx == PACKET_QUAT_LEN))
      break;
    if ((replybuffer[1] == 'B') && (replyidx == PACKET_BUTTON_LEN))
      break;
    if ((replybuffer[1] == 'C') && (replyidx == PACKET_COLOR_LEN))
      break;
    if ((replybuffer[1] == 'L') && (replyidx == PACKET_LOCATION_LEN))
      break;

    while (ble.available()) {
      char c =  ble.read();
      if (c == '!') {
        replyidx = 0;
      }
      replybuffer[replyidx] = c;
      replyidx++;
      timeout = BLE_READPACKET_TIMEOUT;
    }
    
    if (timeout == 0) break;
    delay(1);
  }

  replybuffer[replyidx] = 0;  // null term

  if (!replyidx)  // no data or timeout 
    return 0;
  if (replybuffer[0] != '!')  // doesn't start with '!' packet beginning
    return 0;
  
  // check checksum!
  uint8_t xsum = 0;
  uint8_t checksum = replybuffer[replyidx-1];
  
  for (uint8_t i=0; i<replyidx-1; i++) {
    xsum += replybuffer[i];
  }
  xsum = ~xsum;

  // Throw an error message if the checksum's don't match
  if (xsum != checksum)
  {
    Serial.print("Checksum mismatch in packet : ");
    printHex(replybuffer, replyidx+1);
    return 0;
  }
  
  // checksum passed!
  return replyidx;
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("CONTROLLER EXAMPLE"));
  Serial.println(F("------------------"));

  #if TRANSPORT == TRANSPORT_SWSERIAL
  Serial.println( F("") );
  Serial.println( F("Make sure the mode selection switch on your UART FRIEND") );
  Serial.println( F("is set to CMD mode to enable SW-based mode control!"));
  Serial.println( F("") );
  #endif

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin() )
  {
    Serial.println( F("FAILED! (Check your wiring?)") );
    while(1){}
  }
  Serial.println( F("OK!") );
  
  /* Switch to CMD mode if we're using a UART module */
  #if TRANSPORT == TRANSPORT_SWSERIAL
  Serial.print( F("Switching to CMD mode: ") );
  EXECUTE( ble.setModePin(BLUEFRUIT_MODE_COMMAND) );
  #endif

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.print(F("Performing a factory reset: "));
  EXECUTE( ble.factoryReset() );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Set the device into VERBOSE mode if requested (echo commands) */
  ble.verbose(VERBOSE_MODE);

  /* Switch to DATA/UART mode if we're using a UART module */
  #if TRANSPORT == TRANSPORT_SWSERIAL
  Serial.print( F("Switching to DATA/UART mode: ") );
  EXECUTE( ble.setModePin(BLUEFRUIT_MODE_DATA) );
  #endif

  Serial.println();
  Serial.println("Please start the Bluefruit LE Connect app on Android or");
  Serial.println("iOS and connect to this peripheral, then select the");
  Serial.println("'Controller' utility from the popup menu ...");
  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  memset(replybuffer, 0, READ_BUFSIZE);

  /* Wait for new data to arrive */
  uint8_t len = readPacket();
  if (len == 0) return;

  /* Got a packet! */
  // printHex(replybuffer, len);

  // Color
  if (replybuffer[1] == 'C') {
    uint8_t red = replybuffer[2];
    uint8_t green = replybuffer[3];
    uint8_t blue = replybuffer[4];
    Serial.print ("RGB #"); 
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }
  
  // Buttons
  if (replybuffer[1] == 'B') {
    uint8_t buttnum = replybuffer[2] - '0';
    boolean pressed = replybuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }
  
  // GPS Location
  if (replybuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(replybuffer+2);    
    lon = parsefloat(replybuffer+6);    
    alt = parsefloat(replybuffer+10);
    Serial.print("GPS Location\t"); 
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }
  
  // Accelerometer
  if (replybuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(replybuffer+2);    
    y = parsefloat(replybuffer+6);    
    z = parsefloat(replybuffer+10);
    Serial.print("Accel\t"); 
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }
  
  // Magnetometer
  if (replybuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(replybuffer+2);    
    y = parsefloat(replybuffer+6);    
    z = parsefloat(replybuffer+10);
    Serial.print("Mag\t"); 
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }
  
  // Gyroscope
  if (replybuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(replybuffer+2);    
    y = parsefloat(replybuffer+6);    
    z = parsefloat(replybuffer+10);
    Serial.print("Gyro\t"); 
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }
  
  // Quaternions
  if (replybuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(replybuffer+2);    
    y = parsefloat(replybuffer+6);    
    z = parsefloat(replybuffer+10);
    w = parsefloat(replybuffer+14);
    Serial.print("Quat\t"); 
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
}