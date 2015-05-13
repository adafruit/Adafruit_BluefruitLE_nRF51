/**************************************************************************/
/*!
    @file     Adafruit_BLE_SWUART.cpp
    @author   hathach

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
#include "Adafruit_BluefruitLE_UART.h"

/******************************************************************************/
/*!
    @brief Instantiates a new instance of the Adafruit_BluefruitLE_UART class

    @param[in]  csPin
                The location of the CS pin for the SPI interface
    @param[in]  irqPin
                The location of the HW IRQ pin (pin 2 or pin 3 on the Arduino
                Uno). This must be a HW interrupt pin!
    @param[in]  cts_pin
    @param[in]  rts_pin
    @param[in]  mode_pin
*/
/******************************************************************************/
Adafruit_BluefruitLE_UART::Adafruit_BluefruitLE_UART(HardwareSerial &port, int8_t mode_pin, int8_t cts_pin, int8_t rts_pin) :
  _cts_pin(cts_pin), _rts_pin(rts_pin), _mode_pin(mode_pin)
{
  _verbose = false;
  _timeout = BLE_DEFAULT_TIMEOUT;

#if not defined (_VARIANT_ARDUINO_DUE_X_)
  ss = 0;
#endif

  hs = &port;
  mySerial = &port;
}

#if not defined (_VARIANT_ARDUINO_DUE_X_)

Adafruit_BluefruitLE_UART::Adafruit_BluefruitLE_UART(SoftwareSerial &port, int8_t mode_pin, int8_t cts_pin, int8_t rts_pin) :
  _cts_pin(cts_pin), _rts_pin(rts_pin), _mode_pin(mode_pin)
{
  _verbose = false;
  _timeout = BLE_DEFAULT_TIMEOUT;
  hs = 0;
  ss = &port;
  mySerial = &port;
}

#endif


/******************************************************************************/
/*!
    @brief Class's Destructor
*/
/******************************************************************************/
Adafruit_BluefruitLE_UART::~Adafruit_BluefruitLE_UART()
{
  end();
}

/******************************************************************************/
/*!
    @brief Initialize the HW to enable communication with the BLE module

    @return Returns 'true' if everything initialised correctly, otherwise
            'false' if there was a problem during HW initialisation. If
            'irqPin' is not a HW interrupt pin false will be returned.
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_UART::begin(boolean debug)
{
  _verbose = debug;
  _intercharwritedelay = 0;

  // If hardware mode pin is enabled, set it to CMD first
  if ( _mode_pin >= 0)
  {
    pinMode(_mode_pin, OUTPUT);
    digitalWrite(_mode_pin, BLUEFRUIT_MODE_COMMAND);

    // A bit of delay to make sure mode change take effect
    delay(1);
  }

  // Bluefruit baudrate is fixed to 9600
  if (hs) {
    hs->begin(9600);
  } else {
#if not defined (_VARIANT_ARDUINO_DUE_X_)
    ss->begin(9600);
#endif
  }

  if (_cts_pin > 0) {
    pinMode(_cts_pin, OUTPUT);
    digitalWrite(_cts_pin, HIGH);  // turn off txo
  }

  mySerial->setTimeout(_timeout);
  // reset Bluefruit module upon connect
  return reset();
}

/******************************************************************************/
/*!
    @brief  Uninitializes the SPI interface
*/
/******************************************************************************/
void Adafruit_BluefruitLE_UART::end(void)
{
  if (hs) {
    hs->end();
  } else {
#if not defined (_VARIANT_ARDUINO_DUE_X_)
    ss->end();
#endif
  }
}

/******************************************************************************/
/*!
    @brief  Set the hardware Mode Pin if it is enabled

    @param[in]  mode
                The mode to change, either is BLUEFRUIT_MODE_COMMAND or
                BLUEFRUIT_MODE_DATA

    @return false if Mode Pin is not previously enabled, otherwise true.
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_UART::setModePin(uint8_t mode)
{
  if ( _mode_pin < 0 ) return false;
  digitalWrite(_mode_pin, mode);
  return true;
}

/******************************************************************************/
/*!
    @brief Print API, either buffered data internally or send to bus
    if possible. An \r, \n is command terminator will force the packet to be sent

    @param[in]  c
                Character to send
*/
/******************************************************************************/
size_t Adafruit_BluefruitLE_UART::write(uint8_t c)
{
  // flush left-over before a new command
//  if (c == '\r') flush();

  if (_verbose) Serial.print((char) c);

  if (_rts_pin >= 0) {
    while (digitalRead(_rts_pin)) {
      delay(1);
    }
  } else {
    delay(_intercharwritedelay);
  }

  delayMicroseconds(50);
  return mySerial->write(c);
}

/******************************************************************************/
/*!
    @brief Check if the response from the previous command is ready

    @return 'true' if a response is ready, otherwise 'false'
*/
/******************************************************************************/
int Adafruit_BluefruitLE_UART::available(void)
{
  if (!  mySerial->available() & (_cts_pin > 0)) {
    // toggle flow control to get more byteses
    digitalWrite(_cts_pin, LOW);
    delay(1);
    digitalWrite(_cts_pin, HIGH);
  }
  return mySerial->available();
}

/******************************************************************************/
/*!
    @brief Get a byte from response data, perform SPI transaction if needed

    @return -1 if no data is available
*/
/******************************************************************************/
int Adafruit_BluefruitLE_UART::read(void)
{
  int c = mySerial->read();

  if (_verbose && c > 0) Serial.print((char)c);
  return c;
}

/******************************************************************************/
/*!
    @brief Get a byte from response without removing it, perform SPI transaction
           if needed

    @return -1 if no data is available
*/
/******************************************************************************/
int Adafruit_BluefruitLE_UART::peek(void)
{
  return mySerial->peek();
}

/******************************************************************************/
/*!
    @brief Flush current response data in the internal FIFO

    @return -1 if no data is available
*/
/******************************************************************************/
void Adafruit_BluefruitLE_UART::flush(void)
{
  mySerial->flush();
}
