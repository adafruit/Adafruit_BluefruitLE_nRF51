/*********************************************************************
 Circuit Playground NeoPixel color picker.
 
 This is an example for our nRF51822 based Bluefruit LE modules connected
 to the Circuit Playground developer edition board.  Load the sketch and
 then use the Controller mode of the Bluefruit LE app to change the color
 (in the color picker) and the left/right controller buttons to change
 which pixels are lit up.

 Pick up a Circuit Playground & Flora Bluefruit LE module today in the 
 Adafruit shop!
 - https://www.adafruit.com/product/3000
 - https://www.adafruit.com/product/2487

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include <Adafruit_CircuitPlayground.h>

// Configuration (you don't need to change these, but can!):
#define FACTORYRESET_ENABLE     1   // Set to 1 to factory reset the Bluefruit LE
                                    // module.  In general this is a smart idea to
                                    // put the module into a known good state.
                                    // A value of 1 means perform a factory reset
                                    // on start, and 0 means no factory reset.

#define NUMPIXELS               10  // Number of NeoPixels on the board.

#define BRIGHTNESS              255 // NeoPixel brightness (0...255, low to max brightness)

/*=========================================================================*/

// Create the bluefruit object, for Circuit Playground this needs to be
// hardware serial.
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

// Offset that controls which pixels are turned on/off.
// Pressing the left/right controller button moves this offset
// up/down and 'slides' the chain of 10 pixels around.
int currentOffset = 0;

// Last selected color.
uint8_t currentRed, currentGreen, currentBlue;

// Light up a chain of 10 pixels starting at offset global variable value.
void lightPixels(uint8_t red, uint8_t green, uint8_t blue, int offset) {
  CircuitPlayground.strip.clear();
  for (int i = offset; i < offset+NUMPIXELS; ++i) {
    if ((i >= 0) && (i < NUMPIXELS)) {
      CircuitPlayground.strip.setPixelColor(i, red, green, blue);
    }
  }
  CircuitPlayground.strip.show();
}

void setup() {
  // Wait for serial port before starting.  Not required but helps with debugging.
  //while (!Serial);
  //delay(500);
  
  // Initialize Circuit Playground library and turn off the Pixels.
  CircuitPlayground.begin(BRIGHTNESS);
  CircuitPlayground.clearPixels();

  // Initialize serial output.
  Serial.begin(115200);
  Serial.println(F("Adafruit Circuit Playground Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("-------------------------------------------------------------------"));

  // Initialise the module
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in DATA mode & check wiring...."));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    // Perform a factory reset to make sure everything is in a known state
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()) {
      Serial.println(F("Couldn't factory reset, making another attempt..."));
      delay(1000);
      if (!ble.factoryReset()) {
        error(F("Couldn't factory reset!"));
      }
    }
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  // Print Bluefruit information
  ble.info();

  // Change advertised name to Circuit_Playground_BLE
  ble.println("AT+GAPDEVNAME=CPlay_BLE");
  delay(100);
  ble.println("ATZ");
  delay(100);

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  // Wait for connection
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));

}

void loop()
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) {
    return;
  }

  /* Got a packet! */
  //printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    // Parse out the red, green, blue values and save them.
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    // Change the color.
    currentRed = red;
    currentGreen = green;
    currentBlue = blue;
  }

  // Left/right adjust the offset that turns on/off pixels.
  if (packetbuffer[1] == 'B') {
    if (memcmp(packetbuffer, "!B705", 5) == 0) {
      // Left button release.
      currentOffset -= 1;
    }
    else if (memcmp(packetbuffer, "!B804", 5) == 0) {
      // Right button release.
      currentOffset += 1;
    }
    currentOffset = constrain(currentOffset, -NUMPIXELS, NUMPIXELS);
  }

  // Update the pixels again since either the color or offset position
  // must have changed above.
  lightPixels(currentRed, currentGreen, currentBlue, currentOffset);
}
