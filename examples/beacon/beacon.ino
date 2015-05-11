/**************************************************************************/
/*!
    @file     beacon.ino
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
    BEACON_MANUFACTURER_ID  Company Identifier assigned by Bluetooth SIG
                            Full list of Manufacturer ID can be found here
                            https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
    BEACON_UUID             16-bytes UUID in hex format AA-BB-...
    BEACON_MAJOR            16-bit major nunber
    BEACON_MINOR            16-bit minor nunber
    BEACON_RSSI_1M
    -----------------------------------------------------------------------*/
    #define BUFSIZE                         (128)
    #define VERBOSE_MODE                    (0)

    #define MANUFACTURER_APPLE         "0x004C"
    #define MANUFACTURER_NORDIC        "0x0059"

    #define BEACON_MANUFACTURER_ID     MANUFACTURER_APPLE
    #define BEACON_UUID                "01-12-23-34-45-56-67-78-89-9A-AB-BC-CD-DE-EF-F0"
    #define BEACON_MAJOR               "0x0000"
    #define BEACON_MINOR               "0x0000"
    #define BEACON_RSSI_1M             "-54"
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

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("BLE BEACON EXAMPLE"));
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

  /* Print Bluefruit information */
  ble.info();

  Serial.print(F("Setting beacon configuration details: "));

  // AT+BLEBEACON=0x004C,01-12-23-34-45-56-67-78-89-9A-AB-BC-CD-DE-EF-F0,0x0000,0x0000,-54
  ble.print("AT+BLEBEACON="        );
  ble.print(BEACON_MANUFACTURER_ID ); ble.print(',');
  ble.print(BEACON_UUID            ); ble.print(',');
  ble.print(BEACON_MAJOR           ); ble.print(',');
  ble.print(BEACON_MINOR           ); ble.print(',');
  ble.print(BEACON_RSSI_1M         );
  ble.println(); // print line causes the command to execute

  // check response status
  EXECUTE ( ble.waitForOK() );
  
  Serial.print(F("Resetting the module for advertising changes to take effect: "));
  ble.println("ATZ");
  // check response status
  EXECUTE ( ble.waitForOK() );

  Serial.println();
  Serial.println(F("Open your beacon app to test"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
}
