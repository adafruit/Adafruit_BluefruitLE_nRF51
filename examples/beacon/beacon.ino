/*************************************************************************
    @file     beacon.ino
    @author   hathach, ktown (Adafruit Industries)


This example shows how to turn your Adafruit Bluefruit LE (nrf51822) into
a Beacon!
**************************************************************************/
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BLE_HWSPI.h"
#include "Adafruit_BluefruitLE_UART.h"

// If you are using Software Serial....
// The following macros declare the pins used for SW serial, you should
// use these pins if you are connecting the UART Friend to an UNO
#define BLUEFRUIT_SWUART_RXD_PIN        9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN        10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN          11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN          -1   // Optional, set to -1 if unused

// If you are using Hardware Serial
// The following macros declare the Serial port you are using. Uncomment this
// line if you are connecting the BLE to Leonardo/Micro or Flora
//#define BLUEFRUIT_HWSERIAL_NAME           Serial1

// Other recommended pins!
#define BLUEFRUIT_UART_MODE_PIN         12   // Optional but recommended, set to -1 if unused

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
    #define BUFSIZE                    128
    #define VERBOSE_MODE               1

    #define MANUFACTURER_APPLE         "0x004C"
    #define MANUFACTURER_NORDIC        "0x0059"

    #define BEACON_MANUFACTURER_ID     MANUFACTURER_APPLE
    #define BEACON_UUID                "01-12-23-34-45-56-67-78-89-9A-AB-BC-CD-DE-EF-F0"
    #define BEACON_MAJOR               "0x0000"
    #define BEACON_MINOR               "0x0000"
    #define BEACON_RSSI_1M             "-54"
/*=========================================================================*/

/* Create the bluefruit object, either software serial... */

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
//Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Beacon Example"));
  Serial.println(F("---------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.print(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }
  
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  
  Serial.println("Requesting Bluefruit info:");
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
  if (! ble.waitForOK() ) {
    error(F("Didn't get the OK"));
  }
  
  Serial.print(F("Resetting the module for advertising changes to take effect: "));
  ble.println("ATZ");
  // check response status
  if (! ble.waitForOK() ) {
    error(F("Didn't get the OK"));
  }

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