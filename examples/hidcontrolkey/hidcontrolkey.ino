/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
  This example shows how to send HID Consumer Control Keys,
  ncluding the following control key definitions:

  System Control (works on most systems: Windows/OS X/Android/iOS)
    - Sound Mute
    - Brightness Increase, decrease
  Media Control (works on most systems)
    - PlayPause
    - MediaNext
  Application Launchers (works mainly on Windows 8/10)
    - EmailReader
    - Calculator
  Browser Specific (Firefox, file explorer: mainly on Windows 8/10)
    - Back
    - Forward
    - Refresh
    - Search
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

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
  while (!Serial);  // Required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Control Key Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Factory reset failed!"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // This demo only works with firmware 0.6.6 and higher!
  // Request the Bluefruit firmware rev and check if it is valid
  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error(F("This sketch requires firmware version " MINIMUM_FIRMWARE_VERSION " or higher!"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Services (including Control Key): "));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDEN=On"  ))) {
    error(F("Failed to enable HID (firmware >=0.6.6?)"));
  }

  /* Adding or removing services requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  Serial.println();
  Serial.println(F("**********************************************************"));
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("Some Control Key works system-wide: mute, brightness ..."));
  Serial.println(F("Some are application specific: Media play/pause"));
  Serial.println(F("**********************************************************"));

  // Print pre-defined control keys
  printDefinedControlKey();

  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Display prompt
  Serial.print(F("Control (? for help) > "));

  // Check for user input and echo it back if anything was found
  char keys[BUFSIZE+1];
  getUserInput(keys, BUFSIZE);

  Serial.println(keys);

  if ( keys[0] == '?')
  {
    printDefinedControlKey();
  }else
  {
    ble.print("AT+BleHidControlKey=");
    ble.println(keys);

    if( ble.waitForOK() )
    {
      Serial.println( F("OK!") );
    }else
    {
      Serial.println( F("FAILED!") );
      // Failed, probably pairing is not complete yet
      Serial.println( F("Please make sure Bluefruit is paired and try again") );
    }
  }
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );
}

/**************************************************************************/
/*!
    @brief  Print pre-defined control keys
*/
/**************************************************************************/
void printDefinedControlKey(void)
{
  Serial.println();
  Serial.println(F("You can send a raw 16-bit (e.g 0x1234) usage key from the USB" "\n"
                    "HID Consumer Control Page or use one of the the following keys:"));

  Serial.println(F("List of pre-defined control keys:"));
  Serial.print(F(
    "- BRIGHTNESS+" "\n"
    "- BRIGHTNESS-" "\n"
    "- PLAYPAUSE" "\n"
    "- MEDIANEXT" "\n"
    "- MEDIAPREVIOUS" "\n"
    "- MEDIASTOP" "\n"
    "- VOLUME" "\n"
    "- MUTE" "\n"
    "- BASS" "\n"
    "- TREBLE" "\n"
    "- BASS_BOOST" "\n"
    "- VOLUME+" "\n"
    "- VOLUME-" "\n"
    "- BASS+" "\n"
    "- BASS-" "\n"
    "- TREBLE+" "\n"
    "- TREBLE-" "\n"
    "- EMAILREADER" "\n"
    "- CALCULATOR" "\n"
    "- FILEBROWSER" "\n"
    "- SEARCH" "\n"
    "- HOME" "\n"
    "- BACK" "\n"
    "- FORWARD" "\n"
    "- STOP" "\n"
    "- REFRESH" "\n"
    "- BOOKMARKS" "\n"
  ));
}
