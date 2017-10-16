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

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
   
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
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE      1
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

// String to send in the throughput test
#define TEST_STRING     "01234567899876543210"

// Number of total data sent ( 1024 times the test string)
#define TOTAL_BYTES     (1024 * strlen(TEST_STRING))

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
  Serial.println(F("Adafruit Bluefruit Throughput Tester"));
  Serial.println(F("------------------------------------"));

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
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Switch to DATA mode to have a better throughput */
  Serial.println("Switch to DATA mode to have a better throughput ...");

  /* Wait for a connection before starting the test */
  Serial.println("Waiting for a BLE connection to continue ...");
  ble.setMode(BLUEFRUIT_MODE_DATA);

  ble.verbose(false);  // debug info is a little annoying after this point!

  // Wait for connection to finish
  while (! ble.isConnected()) {
      delay(5000);
  }

  // Wait for the connection to complete
  delay(1000);

  Serial.println(F("CONNECTED!"));
  Serial.println(F("**********"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  uint32_t start, stop, sent;
  uint32_t remaining = TOTAL_BYTES;
  start = stop = sent = 0;

  if (ble.isConnected())
  {
    // Wait for user input before trying again
    Serial.println("Connected. Send a key and press enter to start test");
    char command[BUFSIZE+1];
    getUserInput(command, BUFSIZE);

    Serial.print("Sending ");
    Serial.print(remaining);
    Serial.println(" bytes ...");

    start = millis();
    while (remaining > 0)
    {
//      ble.print("AT+BLEUARTTX=");
//      ble.println(TEST_STRING);
//      if (! ble.waitForOK() )
//      {
//        Serial.println(F("Failed to send?"));
//      }

      ble.writeBLEUart(TEST_STRING);
      
      sent      += strlen(TEST_STRING);
      remaining -= strlen(TEST_STRING);

      // Only print every 1KB sent
      if ( (sent % 2000) == 0 )
      {
        Serial.print("Sent: "); Serial.print(sent);
        Serial.print(" Remaining: "); Serial.println(remaining);
      }

      /* Optional: Test for lost connection every packet */
      /* Note that this will slow things down a bit! */
      /*
      if (!ble.isConnected())
      {
        Serial.println("Connection lost");
        remaining = 0;
      }
      */
    }
    stop = millis() - start;

    Serial.print("Sent ");
    Serial.print(sent);
    Serial.print(" bytes in ");
    Serial.print(stop/1000.0F, 2);
    Serial.println(" seconds.");
    
    Serial.println("Speed ");
    Serial.print( (sent/1000.0F) / (stop/1000.0F), 2);
    Serial.println(" KB/s.\r\n");
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
