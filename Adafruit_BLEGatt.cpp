/**************************************************************************/
/*!
    @file     Adafruit_BLEGatt.cpp
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

#include "Adafruit_BLEGatt.h"


/******************************************************************************/
/*!
    @brief Constructor
*/
/******************************************************************************/
Adafruit_BLEGatt::Adafruit_BLEGatt(Adafruit_BLE& ble) :
  _ble(ble)
{

}

/******************************************************************************/
/*!
    @brief Clear all GATT data
*/
/******************************************************************************/
bool Adafruit_BLEGatt::clear(void)
{
  return _ble.atcommand( F("AT+GATTCLEAR") );
}

/******************************************************************************/
/*!
    @brief Add a service with 16-bit UUID
    @return Service ID (starting from 1). If failed 0 is returned
*/
/******************************************************************************/
uint8_t Adafruit_BLEGatt::addService(uint16_t uuid16)
{
  int32_t service_id;
  VERIFY_RETURN_( _ble.atcommandIntReply( F("AT+GATTADDSERVICE=UUID"), &service_id, uuid16), 0 );
  return (uint8_t) service_id;
}

/******************************************************************************/
/*!
    @brief Add a service with 128-bit UUID
    @return Service ID (starting from 1). If failed 0 is returned
*/
/******************************************************************************/
uint8_t Adafruit_BLEGatt::addService(uint8_t uuid128[])
{
  int32_t service_id;
  VERIFY_RETURN_( _ble.atcommandIntReply( F("AT+GATTADDSERVICE=UUID128"), &service_id, uuid128, 16), 0 );
  return (uint8_t) service_id;
}

