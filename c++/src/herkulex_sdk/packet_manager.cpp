/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Robótica de la Mixteca
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Universidad Tecnológica de la Mixteca nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 
/////////////////////////////////////////////////////////////////////////////////////////
/// @file PacketManager class implementation. This class builds up the HerkuleX protocol
/// packets for communication with the HerkuleX servomotors.
///
/// @author Victor Esteban Sandoval-Luna
/////////////////////////////////////////////////////////////////////////////////////////

/* TODO */
// Check the ACK packet (checksums) to make sure it is not corrupted

#include "../../include/herkulex_sdk/packet_manager.h"

using namespace herkulex;

PacketManager::PacketManager ()
{
  port.openPort();
  verbosity = 0;

  header[0] = 0xFF;
  header[1] = 0xFF;
  pSize = 0;
  CMD = 0;
  pID = 0;
  cs1 = 0;
  cs2 = 0;
  data[0] = 0x00;
}

PacketManager::PacketManager (const int& verb)
{
  port.openPort();
  verbosity = (verb == 0 || verb == 1) ? verb : 0;

  header[0] = 0xFF;
  header[1] = 0xFF;
  pSize = 0;
  CMD = 0;
  pID = 0;
  cs1 = 0;
  cs2 = 0;
  data[0] = 0x00;
}

PacketManager::PacketManager (char* const port_name, const int& baudrate, const int& verb) : port(port_name,baudrate)
{
  port.openPort();
  verbosity = (verb == 0 || verb == 1) ? verb : 0;

  header[0] = 0xFF;
  header[1] = 0xFF;
  pSize = 0;
  CMD = 0;
  pID = 0;
  cs1 = 0;
  cs2 = 0;
  data[0] = 0x00;
}

int PacketManager::sendPacket (std::vector<uint8_t> buf, int ID) {
  int length = buf.size();
  int n;

  if (length < MIN_BUFFER_LENGTH) {return PACKET_MIN_ERROR;}
  if (length > MAX_BUFFER_LENGTH) {return PACKET_MAX_ERROR;}

  pSize = length + 4;
  pID = ID;
  CMD = buf[2];

  checkSum1(buf);
  checkSum2();

  buildUp(buf);
  n = sendTx();
  
  if (n == -1) {return COM_TX_FAIL;}

  return COM_OK;
}

int PacketManager::sendreceivePacket (std::vector<uint8_t> buf, int ack_length, int ID)
{
  int length = buf.size();
  int n_ack;

  if (length < MIN_BUFFER_LENGTH) {return PACKET_MIN_ERROR;}
  if (length > MAX_BUFFER_LENGTH) {return PACKET_MAX_ERROR;}

  std::vector<uint8_t> ack = std::vector<uint8_t> (15); // ACK packet length up to 15 bytes

  pSize = length + 4;
  pID = ID;
  CMD = buf[2];

  checkSum1(buf);
  checkSum2();

  buildUp(buf);
  n_ack = sendTxRx(ack_length);
  
  if (n_ack != ack_length) {
    if (n_ack == -1) {return COM_TX_FAIL;}
    if (n_ack == 0) {return COM_RX_TIMEOUT;}
    return COM_RX_FAIL;
  }

  return COM_OK;
}

int PacketManager::sendTx ()
{
  int ds = data.size();
  port.clearPort();
  int k = port.writePort(data.data(), ds);
  usleep (ds * 10);

  if (k != ds) {
    return -1;
  }

  if (verbosity) {
    for (int j = 0; j < ds; j++) {
      printf("%X ",data[j]);
    }
    printf("\n");
  }

  return k;
}

int PacketManager::sendTxRx (int ack_length)
{
  ack_packet.resize(ack_length);
  int ds = data.size();

  port.clearPort();
  int k = port.writePort(data.data(), ds);
  usleep ((ds + ack_length) * 10);

  int n = port.readPort(ack_packet.data(), ack_packet.size());

  if (k != ds) {
    return -1;
  }

  if (verbosity) {
    for (int j = 0; j < ds; j++) {
      printf("%X ",data[j]);
    }
    printf("\n");
  }

  return n;
}

int PacketManager::buildUp (std::vector<uint8_t> bytes)
{
  int s = bytes.size();
  data.resize(s + 4);         // 7 - 3 = 4

  // Shifting data
  for (int i = 0; i < s-3; i++) {
    data[i+7] = bytes[i+3];    // 7 -4 = 3
  }

  // Hovis HerkuleX packet format
  data[0] = header[0];
  data[1] = header[1];
  data[2] = pSize;
  data[3] = pID;
  data[4] = CMD;
  data[5] = cs1;
  data[6] = cs2;

  return 0;
}

std::vector<uint8_t> PacketManager::getData () const {return data;}

std::vector<uint8_t> PacketManager::getAckPacket () const {return ack_packet;}

uint8_t PacketManager::checkSum1 (std::vector<uint8_t> bytes)
{
  int bs = bytes.size();

  cs1 = 0;
  for (int j = 0; j < bs; j++) {
    cs1 = cs1 ^ bytes[j];
  }
  cs1 = cs1 & 0xFE;

  return cs1;
}

uint8_t PacketManager::checkSum2 ()
{
  cs2 = 0;
  cs2 = ~(cs1);
  cs2 = cs2 & 0xFE;

  return cs2;
}

