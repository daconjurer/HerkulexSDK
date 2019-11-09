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
/// @file Packet manager
/// @author Victor Esteban Sandoval-Luna
///
/////////////////////////////////////////////////////////////////////////////////////////

#include "../../include/herkulex_sdk/packet_manager.h"

using namespace herkulex;

PacketManager::PacketManager () {
  // char* a = port.getPortName();

  port.setBaudRate(115200);
  port.openPort();

  header[0] = 0xFF;
  header[1] = 0xFF;
  pSize = 0;
  CMD = 0;
  pID = 0;
  cs1 = 0;
  cs2 = 0;
  data[0] = 0x00;
}

int PacketManager::sendTx (int length, std::vector<uint8_t> buf, int ID, int& verb) {
  pSize = length;
  pID = ID;
  CMD = buf[2];

  checkSum1(buf,cs1);
  checkSum2(cs1,cs2);

  buildUp(buf);

  if (verb)
    return sendPacket(1);
  return sendPacket(0);
}

int PacketManager::sendTxSync (int length, std::vector<uint8_t> buf, int& verb) {
  std::vector<uint8_t> buffer = std::vector<uint8_t> (length-7);

  pSize = length;
  pID = buf[1];
  CMD = buf[2];

  // Shifting data
  for (int i = 0; i < length-7; i++) {
    buffer[i] = buf[i+3];
  }

  checkSum1(buffer,cs1);
  checkSum2(cs1,cs2);

  buildUp(buf);

  int a = 0;
  std::cout << "sendsync:" << std::endl;

  for (unsigned int i = 0; i < data.size(); i++) {
    a = data[i];
    std::cout << a << std::endl;
  }

  if (verb)
    return sendPacket(1);
  return sendPacket(0);
}

int PacketManager::sendTxRx (int length, std::vector<uint8_t> buf, int ack_length, int ID, int& verb) {
  std::vector<uint8_t> ack = std::vector<uint8_t> (15); // ACK packet length up to 15 bytes

  pSize = length;
  pID = ID;
  CMD = buf[2];

  checkSum1(buf,cs1);
  checkSum2(cs1,cs2);

  buildUp(buf);

  if (verb)
    return sendreceivePacket(1, ack_length);
  return sendreceivePacket(0, ack_length);
}

bool PacketManager::setPortLabel (const char* portlabel) {
  port.setPortName(portlabel);
  port.openPort();

  return true;
}

std::vector<uint8_t> PacketManager::getData () {
  return data;
}

std::vector<uint8_t> PacketManager::getAckPacket () {
  return ack_packet;
}

bool PacketManager::resizeData (int length) {
  data.resize(length);
  return true;
}

int PacketManager::sendPacket (int verbose) {
  int ds = data.size();
  port.clearPort();
  int k = port.writePort(data.data(), ds);
  usleep (ds * 10);

  if (verbose) {
    for (int j = 0; j < ds; j++) {
      printf("%X ",data[j]);
    }
    printf("\n");
  }

  return k;
}

int PacketManager::sendreceivePacket (int verbose, int ack_length) {
  ack_packet.resize(ack_length);
  int ds = data.size();

  port.clearPort();
  int k = port.writePort(data.data(), ds);
  usleep ((ds + ack_length) * 10);

  int n = port.readPort(ack_packet.data(), ack_packet.size());

  if (k != ds) {
    return -1;
  }

  if (verbose) {
    for (int j = 0; j < ds; j++) {
      printf("%X ",data[j]);
    }
    printf("\n");
  }

  return n;
}

char* PacketManager::buildUp (std::vector<uint8_t> bytes) {
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

  return (char*)"Packet ready.";
}

uint8_t PacketManager::checkSum1 (std::vector<uint8_t> bytes, uint8_t& cs1) {
  //if (MIN_PACKET_SIZE < bytes.size() < MAX_PACKET_SIZE) {
  //  return PACKET_ERR_CS;
  //}
  int bs = bytes.size();

  cs1 = 0;

  for (int j = 0; j < bs; j++) {
    cs1 = cs1 ^ bytes[j];
  }
  cs1 = cs1 & 0xFE;

  return cs1;
}

uint8_t PacketManager::checkSum2 (uint8_t cs1, uint8_t& cs2) {
  cs2 = 0;

  cs2 = ~(cs1);
  cs2 = cs2 & 0xFE;

  return cs2;
}
