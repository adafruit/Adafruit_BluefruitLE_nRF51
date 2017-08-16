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
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined(ARDUINO_ARCH_SAMD)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

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
    #define FACTORYRESET_ENABLE         1
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

// GPIO corresponding to HID gamepad
#define LEFT_PIN    5
#define RIGHT_PIN   6
#define UP_PIN      10
#define DOWN_PIN    9
#define BUTTON1_PIN 12
#define BUTTON2_PIN 11

int buttonPins[6]                 = {LEFT_PIN, RIGHT_PIN, UP_PIN, DOWN_PIN, BUTTON1_PIN, BUTTON2_PIN};
int buttonState[6]                = {HIGH,     HIGH,      HIGH,   HIGH,     HIGH,        HIGH};

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Gamepad"));
  Serial.println(F("---------------------------------------"));

  // Initialise the module
  //Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    // Perform a factory reset to make sure everything is in a known state
    Serial.println(F("Performing a factory reset: "));
    ble.factoryReset();
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  // Print Bluefruit information
  ble.info();
 
  // Enable HID Service if not enabled
  int32_t hid_en = 0;  
  ble.sendCommandWithIntReply( F("AT+BLEHIDEN?"), &hid_en);
  if ( !hid_en )
  {
    Serial.println(F("Enable HID: "));
    ble.sendCommandCheckOK(F( "AT+BLEHIDEN=1" ));
  }
  
  // Enable HID Service if not enabled
  int32_t gamepad_en = 0;  
  ble.sendCommandWithIntReply( F("AT+BLEHIDGAMEPADEN?"), &gamepad_en);
  if ( !gamepad_en )
  {
    Serial.println(F("Enable HID GamePad: "));
    ble.sendCommandCheckOK(F( "AT+BLEHIDGAMEPADEN=1" ));
  }

  if ( !hid_en || !gamepad_en )
  {
    // Add or remove service requires a reset
    Serial.println(F("Performing a SW reset (service changes require a reset): "));
    !ble.reset();
  }
  
  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts gamepad input"));
  Serial.println();

  // Set up input Pins
  for(int i=0; i< 6; i++)
  {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  
  attachInterrupt(LEFT_PIN, left_ISR, CHANGE);
  attachInterrupt(RIGHT_PIN, right_ISR, CHANGE);
  attachInterrupt(UP_PIN, up_ISR, CHANGE);
  attachInterrupt(DOWN_PIN, down_ISR, CHANGE);
  attachInterrupt(BUTTON1_PIN, button1_ISR, CHANGE);
  attachInterrupt(BUTTON2_PIN, button2_ISR, CHANGE);
}

volatile boolean stateChanged = false;

void set(int i)
{
  buttonState[i] = digitalRead(buttonPins[i]);
  stateChanged = true;
}

void left_ISR() {
  set(0);
}
void right_ISR() {
  set(1);
}
void up_ISR() {
  set(2);
}
void down_ISR() {
  set(3);
}
void button1_ISR() {
  set(4);
}
void button2_ISR() {
  set(5);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop()
{
  /* scan GPIO, since each report can has up to 6 keys
   * we can just assign a slot in the report for each GPIO 
   */
  if ( stateChanged )
  {
    if ( ble.isConnected() )
    {    
      int x = 0;
      if ( buttonState[0] == LOW )
      {
        x += -1;
      }
      else if ( buttonState[1] == LOW )
      {
        x += 1;
      }
      
      int y = 0;
      if ( buttonState[2] == LOW )
      {
        y += -1;
      }
      else if ( buttonState[3] == LOW )
      {
        y += 1;
      }
      
      int buttons = 0;
      if ( buttonState[4] == LOW )
      {
        buttons |= 1;
      }
      if ( buttonState[5] == LOW )
      {
        buttons |= 2;
      }
      
      // Send axises and buttons
      String command = String("AT+BLEHIDGAMEPAD=") + String(x, DEC) + "," + String(y, DEC) + "," + String(buttons, HEX);
      Serial.println(command);
      ble.println(command);
    }
    stateChanged = false;
  }
  
  // scaning period is 50 ms
  delay(50);
}

