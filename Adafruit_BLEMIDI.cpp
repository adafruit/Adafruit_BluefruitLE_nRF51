/**************************************************************************/
/*!
    @file     Adafruit_BLEMIDI.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Adafruit Industries (adafruit.com)
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

#include "Adafruit_BLEMIDI.h"

#define VERIFY(condition)                if ( !(condition) ) return false;
#define VERIFY_RETURN(condition, error)  if ( !(condition) ) return error;
#define VERIFY_RETVOID(condition)        if ( !(condition) ) return;

#define MIDI_MINIMUM_FIRMWARE_VERSION    "0.7.0"

Adafruit_BLEMIDI::Adafruit_BLEMIDI(Adafruit_BLE& ble) :
  _ble(ble)
{
}

/******************************************************************************/
/*!
    @brief Enable MIDI service if not already enabled
*/
/******************************************************************************/
bool Adafruit_BLEMIDI::begin(void)
{
  VERIFY( _ble.isVersionAtLeast(MIDI_MINIMUM_FIRMWARE_VERSION) );

  int32_t enabled = 0;
  VERIFY( _ble.sendCommandWithIntReply( F("AT+BLEMIDIEN"), &enabled) );

  if ( enabled ) return true;
  VERIFY( _ble.sendCommandCheckOK( F("AT+BLEMIDIEN=1") ) );

  // Perform Bluefruit reset since service changed
  _ble.reset();

  return true;
}

/******************************************************************************/
/*!
    @brief Stop MIDI service if it is enabled
*/
/******************************************************************************/
bool Adafruit_BLEMIDI::stop(void)
{
  int32_t enabled = 0;
  VERIFY( _ble.sendCommandWithIntReply( F("AT+BLEMIDIEN"), &enabled) );
  if ( !enabled ) return true;

  VERIFY( _ble.sendCommandCheckOK( F("AT+BLEMIDIEN=0") ) );

  // Perform Bluefruit reset since service changed
  _ble.reset();

  return true;
}

/******************************************************************************/
/*!
    @brief Send a MIDI event data
    @param bytes MIDI event data
*/
/******************************************************************************/
bool Adafruit_BLEMIDI::send(const uint8_t bytes[3])
{
  char command[] = "AT+BLEMIDITX=00-00-00";
  uint8_t idx = strlen(command) - 8;

  _ble.convert2ByteArrayString(command+idx, bytes, 3);
  return _ble.sendCommandCheckOK(command);
}

/******************************************************************************/
/*!
    @brief Send multiple MIDI event which shared the same status
    @param status MIDI status
    @param bytes MIDI events data
    @param count number of data in bytes (must be multiple of 2)

    @note count + 1 must less than (20-3) --> count <= 16
*/
/******************************************************************************/
bool Adafruit_BLEMIDI::send_n(uint8_t status, const uint8_t bytes[], uint8_t count)
{
  VERIFY(count <= 16);
  char command[64] = "AT+BLEMIDITX=";

  uint8_t idx = strlen(command);

  idx += _ble.convert2ByteArrayString(command+idx, &status, 1);
  command[idx++] = '-';
  _ble.convert2ByteArrayString(command+idx, bytes, count);

  //Serial.println(command);

  return _ble.sendCommandCheckOK(command);
}

