#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define LOAD_TEST_MS 10

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"


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

bool isConnected = false;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// callback
void handleConnected(void)
{
  isConnected = true;
  
  Serial.println(F(" CONNECTED!"));
  
  Serial.print(F("LOAD TESTING MAJOR C CHORD @ "));
  Serial.print(LOAD_TEST_MS);
  Serial.println(F(" MS."));
  Serial.println();
  delay(1000);

  Serial.println(F("HOLD ONTO YOUR BUTTS."));
}

void handleDisconnected(void)
{
  Serial.println("disconnected");
  isConnected = false;
}

void handleMidiReceive(uint8_t data[], uint16_t len)
{
  Serial.println("MIDI received");

  for(uint8_t i=0; i<len; i++)
  {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  
  Serial.println();
}  

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit MIDI Example"));
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
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  
  /* Set callbacks */
  ble.setHandleConnect(handleConnected);
  ble.setHandleDisconnect(handleDisconnected);
  ble.setHandleBleMidiRx(handleMidiReceive);

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to MIDI': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=BLUEFRUIT MIDI" )) ) {
    error(F("Could not set device name?"));
  }
  
  Serial.println(F("Enable MIDI: "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BLEMIDIEN=1" ))) {
      error(F("Could not enable MIDI"));
    }
    
  } else
  {
    error(F("Wrong version"));
  }
  
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }
  
  ble.verbose(false);

  Serial.print(F("Waiting for a connection..."));
  
  /*
  while (ble.isConnected() == 0) {
    Serial.print(F("."));
    delay(500);
  }
  */
}

void loop(void)
{
  // interval for each scanning ~ 500ms (non blocking)
  ble.loop(500);
  
  if ( isConnected )
  {  
    ble.sendCommandCheckOK( F("AT+BLEMIDITX=90-30-64-34-64-37-64") );
    delay(LOAD_TEST_MS);
    ble.sendCommandCheckOK( F("AT+BLEMIDITX=80-30-64-34-64-37-64") );
    delay(LOAD_TEST_MS);
  }
}
