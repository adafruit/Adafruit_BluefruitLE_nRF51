/**************************************************************************/
/*!
    @file     Adafruit_BluefruitLE_SPI.cpp
    @author   hathach, ktown (Adafruit Industries)

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
#include "Adafruit_BluefruitLE_SPI.h"
#include <Arduino.h>
#include <stdlib.h>

SPISettings bluefruitSPI(2000000, MSBFIRST, SPI_MODE0);


/******************************************************************************/
/*!
    @brief Instantiates a new instance of the Adafruit_BluefruitLE_SPI class

    @param[in]  csPin
                The location of the CS pin for the SPI interface
    @param[in]  irqPin
                The location of the HW IRQ pin (pin 2 or pin 3 on the Arduino
                Uno). This must be a HW interrupt pin!
    @param[in]  rstPin
*/
/******************************************************************************/
Adafruit_BluefruitLE_SPI::Adafruit_BluefruitLE_SPI(int8_t csPin, int8_t irqPin, int8_t rstPin) :
    m_rx_fifo(m_rx_buffer, sizeof(m_rx_buffer), 1, true)
{
  m_cs_pin  = csPin;
  m_irq_pin = irqPin;
  m_rst_pin = rstPin;

  m_tx_count = 0;
}


/******************************************************************************/
/*!
    @brief Initialize the HW to enable communication with the BLE module

    @return Returns 'true' if everything initialised correctly, otherwise
            'false' if there was a problem during HW initialisation. If
            'irqPin' is not a HW interrupt pin false will be returned.
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_SPI::begin(boolean v)
{
  _verbose = v;

  pinMode(m_irq_pin, INPUT);

  // Set CS pin to output and de-assert by default
  digitalWrite(m_cs_pin, HIGH);
  pinMode(m_cs_pin, OUTPUT);

  SPI.begin();

  bool isOK;

  // Always try to send Initialize command to reset
  // Bluefruit since user can define but not wiring RST signal
  isOK = sendInitializePattern();

  // use hardware reset if available
  if (m_rst_pin >= 0)
  {
    // pull the RST to GND for 10 ms
    pinMode(m_rst_pin, OUTPUT);
    digitalWrite(m_rst_pin, HIGH);
    digitalWrite(m_rst_pin, LOW);
    delay(10);
    digitalWrite(m_rst_pin, HIGH);

    isOK= true;
  }

  // Bluefruit takes 1 second to reboot
  delay(1000);

  return isOK;
}

/******************************************************************************/
/*!
    @brief  Uninitializes the SPI interface
*/
/******************************************************************************/
void Adafruit_BluefruitLE_SPI::end(void)
{
  SPI.end();
}

/******************************************************************************/
/*!
    @brief Handle direct "+++" input command from user.
           User should use setMode instead
*/
/******************************************************************************/
void Adafruit_BluefruitLE_SPI::simulateSwitchMode(void)
{
  _mode = 1 - _mode;

  char ch = '0' + _mode;
  m_rx_fifo.write(&ch);
  m_rx_fifo.write_n("\r\nOK\r\n", 6);
}

/******************************************************************************/
/*!
    @brief Simulate "+++" switch mode command
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_SPI::setMode(uint8_t new_mode)
{
  // invalid mode
  if ( !(new_mode == BLUEFRUIT_MODE_COMMAND || new_mode == BLUEFRUIT_MODE_DATA) ) return false;

  // Already in the wanted mode
  if ( _mode == new_mode ) return true;

  // SPI use different SDEP command when in DATA/COMMAND mode.
  // --> does not switch using +++ command
  _mode = new_mode;

  return true;
}

/******************************************************************************/
/*!
    @brief  Read data from Ble module's SPI interface.  
            This function will retry if the BLE module is not ready
            (response = SPI_IGNORED_BYTE), or terminate when 'length' bytes
            have been read, SPI_OVERREAD_BYTE is returned or 'timeout_ms' have
            passed.

    @param[out] buf
                buffer to store read data
    @param[in]  length 
                number of bytes to be read

    @return     The number of bytes successfully read
*/
/******************************************************************************/
uint32_t Adafruit_BluefruitLE_SPI::bus_read(uint8_t *buf, uint32_t length)
{
  if (length == 0) return 0;
  
  for(uint8_t i=0; i<length; i++)
  {
    *buf++ = SPI.transfer(0xff);
  }

  return length;
}

/******************************************************************************/
/*!
    @brief  Write data to BLE module's SPI interface.
            This function will retry if the BLE module is not ready
            (response = SPI_IGNORED_BYTE), or terminate when 'length' bytes
            have been written or 'timeout_ms' have passed.

    @param[out] buf
                Buffer containing the data to write
    @param[in]  length 
                Number of bytes to write

    @return     The number of bytes written
*/
/******************************************************************************/
uint32_t Adafruit_BluefruitLE_SPI::bus_write (const uint8_t *buf, uint32_t length)
{
  if (length == 0) return 0;

  uint32_t count=0;
  TimeoutTimer tt(_timeout);

  SPI.beginTransaction(bluefruitSPI);

  SPI_CS_ENABLE();
  while ( (count < length) && !tt.expired() )
  {
    // Keep re-sending if SPI_IGNORED_BYTE is received
    while( !tt.expired() )
    {
      uint8_t rd = SPI.transfer(*buf);

      if (rd != SPI_IGNORED_BYTE) break;

      // blemodule is busy processing, wait a bit before retry
      SPI_CS_DISABLE();
      delayMicroseconds(SPI_DEFAULT_DELAY_US);
      SPI_CS_ENABLE();
    }

    buf++;
    count++;
  }
  SPI_CS_DISABLE();

  SPI.endTransaction();

  return count;
}

/******************************************************************************/
/*!
    @brief Send initialize pattern to Bluefruit LE to force a reset. This pattern
    follow the SDEP command syntax with command_id = SDEP_CMDTYPE_INITIALIZE.
    The command has NO response, and is expected to complete within 1 second
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_SPI::sendInitializePattern(void)
{
  sdepMsgHeader_t msgHeader;

  msgHeader.msg_type    = SDEP_MSGTYPE_COMMAND;
  msgHeader.cmd_id_high = highByte(SDEP_CMDTYPE_INITIALIZE);
  msgHeader.cmd_id_low  = lowByte(SDEP_CMDTYPE_INITIALIZE);
  msgHeader.length      = 0;
  msgHeader.more_data   = 0;

  return (sizeof(sdepMsgHeader_t) == bus_write((uint8_t*)&msgHeader, sizeof(sdepMsgHeader_t)));
}

/******************************************************************************/
/*!
    @brief  Send out an packet with data in m_tx_buffer

    @param[in]  more_data
                More Data bitfield, 0 indicates this is not end of transfer yet
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_SPI::sendPacket(uint16_t command, const uint8_t* buf, uint8_t count, uint8_t more_data)
{
  // flush old response before sending the new command
  if (more_data == 0) flush();

  sdepMsgCommand_t msgCmd;

  msgCmd.header.msg_type    = SDEP_MSGTYPE_COMMAND;
  msgCmd.header.cmd_id_high = highByte(command);
  msgCmd.header.cmd_id_low  = lowByte(command);
  msgCmd.header.length      = count;
  msgCmd.header.more_data   = (count == SDEP_MAX_PACKETSIZE) ? more_data : 0;

  if ( buf != NULL && count > 0) memcpy(msgCmd.payload, buf, count);

  // Send the SDEP header
  return sizeof(sdepMsgHeader_t) + count == bus_write((uint8_t*)&msgCmd, sizeof(sdepMsgHeader_t) + count);
}

//bool Adafruit_BluefruitLE_SPI::handleSwitchCmdInDataMode(uint8_t ch)
//{
//  static char cmdBuf[4] = { 0 };
//}

/******************************************************************************/
/*!
    @brief Print API, either buffered data internally or send SDEP packet to bus
    if possible. An \r, \n is command terminator will force the packet to be sent

    @param[in]  c
                Character to send
*/
/******************************************************************************/
size_t Adafruit_BluefruitLE_SPI::write(uint8_t c)
{
  if (_mode == BLUEFRUIT_MODE_DATA)
  {
    sendPacket(SDEP_CMDTYPE_BLE_UARTTX, &c, 1, 0);
    getResponse();
    return 1;
  }

  // Following code handle BLUEFRUIT_MODE_COMMAND

  // Final packet due to \r or \n terminator
  if (c == '\r' || c == '\n')
  {
    if (m_tx_count > 0)
    {
      // +++ command to switch mode
      if ( memcmp(m_tx_buffer, "+++", 3) == 0)
      {
        simulateSwitchMode();
      }else
      {
        sendPacket(SDEP_CMDTYPE_AT_WRAPPER, m_tx_buffer, m_tx_count, 0);
      }
      m_tx_count = 0;
    }
  }
  // More than max packet buffered --> send with more_data = 1
  else if (m_tx_count == SDEP_MAX_PACKETSIZE)
  {
    sendPacket(SDEP_CMDTYPE_AT_WRAPPER, m_tx_buffer, m_tx_count, 1);

    m_tx_buffer[0] = c;
    m_tx_count = 1;
  }
  // Not enough data, continue to buffer
  else
  {
    m_tx_buffer[m_tx_count++] = c;
  }

  if (_verbose) Serial.print((char) c);

  return 1;
}

size_t Adafruit_BluefruitLE_SPI::write(const uint8_t *buf, size_t size)
{
  if ( _mode == BLUEFRUIT_MODE_DATA )
  {
    if ((size >= 3) &&
        !memcmp(buf, "+++", 3) &&
        !(size > 3 && buf[3] != '\r' && buf[3] != '\n') )
    {
      simulateSwitchMode();
    }else
    {
      while(size)
      {
        size_t len = min(size, SDEP_MAX_PACKETSIZE);
        size -= len;

        sendPacket(SDEP_CMDTYPE_BLE_UARTTX, buf, (uint8_t) len, size ? 1 : 0);
        getResponse();

        buf += len;
      }
    }

    return size;
  }
  // Command mode
  else
  {
    size_t n = 0;
    while (size--) {
      n += write(*buf++);
    }
    return n;
  }
}

/******************************************************************************/
/*!
    @brief Check if the response from the previous command is ready

    @return 'true' if a response is ready, otherwise 'false'
*/
/******************************************************************************/
int Adafruit_BluefruitLE_SPI::available(void)
{
  if (! m_rx_fifo.empty() ) {
    //Serial.println( m_rx_fifo.count());
    return m_rx_fifo.count();
  }

  if ( _mode == BLUEFRUIT_MODE_DATA )
  {
    // DATA Mode: query for BLE UART data
    sendPacket(SDEP_CMDTYPE_BLE_UARTRX, NULL, 0, 0);

    // Waiting to get response from Bluefruit
    getResponse();

    return m_rx_fifo.count();
  }else
  {
    //Serial.print('.');
    return (digitalRead(m_irq_pin));
  }
}

/******************************************************************************/
/*!
    @brief Get a byte from response data, perform SPI transaction if needed

    @return -1 if no data is available
*/
/******************************************************************************/
int Adafruit_BluefruitLE_SPI::read(void)
{
  uint8_t ch;

  // try to grab from buffer first...
  if (!m_rx_fifo.empty()) {
    m_rx_fifo.read(&ch);
    return (int)ch;
  }

  if ( _mode == BLUEFRUIT_MODE_DATA )
  {
    // DATA Mode: query for BLE UART data
    sendPacket(SDEP_CMDTYPE_BLE_UARTRX, NULL, 0, 0);

    // Waiting to get response from Bluefruit
    getResponse();
  }else
  {
    // COMMAND Mode: Only read data from Bluefruit if IRQ is raised
    if ( digitalRead(m_irq_pin) ) getResponse();
  }

  return m_rx_fifo.read(&ch) ? ((int) ch) : EOF;

}

/******************************************************************************/
/*!
    @brief Get a byte from response without removing it, perform SPI transaction
           if needed

    @return -1 if no data is available
*/
/******************************************************************************/
int Adafruit_BluefruitLE_SPI::peek(void)
{
  uint8_t ch;

  // try to grab from buffer first...
  if ( !m_rx_fifo.peek(&ch) ) {
    return (int)ch;
  }

  if ( _mode == BLUEFRUIT_MODE_DATA )
  {
    // DATA Mode: query for BLE UART data
    sendPacket(SDEP_CMDTYPE_BLE_UARTRX, NULL, 0, 0);

    // Waiting to get response from Bluefruit
    getResponse();
  }else
  {
    // Read data from Bluefruit if possible
    if ( digitalRead(m_irq_pin) ) getResponse();
  }

  return m_rx_fifo.peek(&ch) ? ch : EOF;
}

/******************************************************************************/
/*!
    @brief Flush current response data in the internal FIFO

    @return -1 if no data is available
*/
/******************************************************************************/
void Adafruit_BluefruitLE_SPI::flush(void)
{
  m_rx_fifo.clear();
}

/******************************************************************************/
/*!
    @brief  Try to perform an full AT response transfer from Bluefruit, or execute
            as many SPI transaction as internal FIFO can hold up.

    @note   If verbose is enabled, all the received data will be print to Serial

    @return
      - true  : if succeeded
      - false : if failed
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_SPI::getResponse(void)
{
  // Blocking wait until IRQ is asserted
  while ( !digitalRead(m_irq_pin) ) {}

  // There is data from Bluefruit & enough room in the fifo
  while ( digitalRead(m_irq_pin) &&
          m_rx_fifo.remaining() >= SDEP_MAX_PACKETSIZE )
  {
    // Get a SDEP packet
    sdepMsgResponse_t msg_response;
    memclr(&msg_response, sizeof(sdepMsgResponse_t));

    if ( !getPacket(&msg_response) ) return false;

    // Write to fifo
    if ( msg_response.header.length > 0)
    {
      m_rx_fifo.write_n(msg_response.payload, msg_response.header.length);
    }

    // No more packet data
    if ( !msg_response.header.more_data ) break;

    // It takes a bit since all Data received to IRQ to get LOW
    // May need to delay a bit for it to be stable before the next try
    // delayMicroseconds(SPI_DEFAULT_DELAY_US);
  }

  return true;
}

/******************************************************************************/
/*!
    @brief      Perform a single SPI SDEP transaction and is used by getReponse to
                get a full response composed of multiple packets.

    @param[in]  buf
                Memory location where payload is copied to

    @return number of bytes in SDEP payload
*/
/******************************************************************************/
bool Adafruit_BluefruitLE_SPI::getPacket(sdepMsgResponse_t* p_response)
{
  bool result=true;
  sdepMsgHeader_t* p_header = &p_response->header;

  SPI.beginTransaction(bluefruitSPI);
  SPI_CS_ENABLE();

  // Bluefruit may not be ready
  while ( (p_header->msg_type = SPI.transfer(0xff)) == SPI_IGNORED_BYTE )
  {
    // Disable & Re-enable CS with a bit of delay for Bluefruit to ready itself
    SPI_CS_DISABLE();
    delayMicroseconds(SPI_DEFAULT_DELAY_US);
    SPI_CS_ENABLE();
  }

  // Look for the header
  while ( p_header->msg_type != SDEP_MSGTYPE_RESPONSE && p_header->msg_type != SDEP_MSGTYPE_ERROR )
  {
    p_header->msg_type = SPI.transfer(0xff);
  }
  result = bus_read( ((uint8_t*) p_header)+1, sizeof(sdepMsgHeader_t)-1 );

  if (result)
  {
    // Command is 16-bit at odd address, may have alignment issue with 32-bit chip
    uint16_t cmd_id = word(p_header->cmd_id_high, p_header->cmd_id_low);

    // Error Message Response or Invalid command
    if ( p_header->msg_type == SDEP_MSGTYPE_ERROR ||
        !(cmd_id == SDEP_CMDTYPE_AT_WRAPPER   || cmd_id == SDEP_CMDTYPE_BLE_UARTTX || cmd_id == SDEP_CMDTYPE_BLE_UARTRX) ||
         (p_header->length > SDEP_MAX_PACKETSIZE))
    {
      result = false;
    }else
    {
      int len = bus_read( p_response->payload, (uint32_t) p_header->length);
      if ( len != p_header->length ) result = false;
    }
  }

  SPI_CS_DISABLE();
  SPI.endTransaction();

  return result;
}


