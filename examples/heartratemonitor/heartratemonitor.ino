/**************************************************************************/
/*!
    @file     heartratemonitor.ino
    @author   hathach, ktown (Adafruit Industries)

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
    #define BUFSIZE                         (128)
    #define VERBOSE_MODE                    (0)
/*=========================================================================*/


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

uint32_t hrmServiceId;
uint32_t hrmMeasureCharId;
uint32_t hrmLocationCharId;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("BLE HEART RATE MONITOR (HRM) EXAMPLE"));
  Serial.println(F("------------------------------------"));

  randomSeed(micros());
  
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

  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.print(F("Setting device name to 'Bluefruit HRM': "));
  ble.println("AT+GAPDEVNAME=Bluefruit HRM");
  EXECUTE( ble.waitForOK() ); // check if command's response is OK

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.print(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  ble.println("AT+GATTADDSERVICE=UUID=0x180D");
  EXECUTE ( hrmServiceId = ble.readln_parseInt() );
  ble.waitForOK(); // eat OK

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.print(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  ble.println("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40");
  EXECUTE ( hrmMeasureCharId = ble.readln_parseInt() );
  ble.waitForOK(); // eat OK

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  Serial.print(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  ble.println("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3");
  EXECUTE ( hrmLocationCharId = ble.readln_parseInt() );
  ble.waitForOK(); // eat OK

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.println("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18");
  EXECUTE( ble.waitForOK() ); // check if command's response is OK

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  EXECUTE( ble.reset() );

  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  int heart_rate = random(50, 100);

  Serial.print(F("Updating HRM value to "));
  Serial.print(heart_rate);
  Serial.println(F(" BPM"));

  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",00-") );
  ble.println(heart_rate, HEX);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }

  /* Delay before next measurement update */
  delay(1000);
}

