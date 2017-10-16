/*
  Bluefruit Feather Tester

  This sketch provides a simple tester for Bluefruit Feather boards from Adafruit

  created 31 Jan. 2016
  by K. Townsend (KTOWN)
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/**************************************************************************/
/*!

*/
/**************************************************************************/
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup()
{
  Serial.begin(115200);

  // Wait for the Serial Monitor to open
  while (!Serial) { yield(); }

  // Initialise the BLE module
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  // Display the test suite selection menu
  display_menu();
}

/**************************************************************************/
/*!
    Reads and prints the AT response buffer until we reach OK or ERROR

    Returns 'true' if an error occurred, otherwise 'false'
*/
/**************************************************************************/
bool display_response(void)
{
  // Read the response until we get OK or ERROR
  while (ble.readline())
  {
    Serial.print(ble.buffer);
    if ( strcmp(ble.buffer, "OK") == 0 )
    {
      return false;
    }
    if ( strcmp(ble.buffer, "ERROR") == 0 )
    {
      return true;
    }
  }

  return true;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void display_menu()
{
  delay(500);  // Short delay for cosmetic reasons
  Serial.println("");
  Serial.println("Bluefruit Feather Tester");
  Serial.println("-------------------------------------------------------------------------------");
  Serial.println("Select a menu option below:");
  Serial.println("");
  Serial.println("[1]  - System Info");
  Serial.println("[2]  - AT+HELP");
  Serial.println("");
  Serial.println("Enter your selection in the Serial Monitor and press <enter>");
  Serial.println("");
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void loop()
{
  String user_input = "";
  int selection = 0;

  // Wait for user feedback, then parse feedback one byte at a time
  while((Serial.available()) && !selection)
  {
    char incoming = Serial.read();
    if (isdigit(incoming))
    {
      // Append the current digit to the string placeholder
      user_input += (char)incoming;
    }
    // Parse the string on new-line
    if (incoming == '\n')
    {
      selection = user_input.toInt();
    }
    delay(2);
  }

  // Run the appropriate test suite if we have a number
  if (selection)
  {
    bool error = false;
    switch(selection)
    {
      case 1:
        ble.info();
        break;
      case 2:
        ble.println("AT+HELP");
        error = display_response();
        break;
      default:
        Serial.print("Invalid selection: ");
        Serial.println(selection);
        break;
    }

    // Catch any error responses here
    if (error) Serial.println("ERROR!");

    // Display the main menu again
    Serial.println("");
    display_menu();
  }
}
