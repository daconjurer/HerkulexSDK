/*******************************************************************************
* Copyright 2018 Robótica de la Mixteca
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Victor Esteban Sandoval-Luna */

#include <stdexcept>
using std::runtime_error;

#include "../../include/herkulex_sdk/servo_herkulex.h"

#define H0101 1
#define H0201 2
#define H0401 4
#define H0601 6

using namespace herkulex;

ServoHerkulex::ServoHerkulex (int verb)
{
  // default yet never used in the application
  ID[0] = 0;
  model[0] = H0201;
  verbose_ = verb;
}

ServoHerkulex::ServoHerkulex (int sID, const char *smodel, int verb)
{
  ID[0] = sID;
  model[0] = mapModel(smodel);
  verbose_ = verb;
}

ServoHerkulex::ServoHerkulex (std::vector<int> sIDs, std::vector<std::string> smodels, int verb)
{
  int size = sIDs.size();

  ID.resize(size);
  model.resize(size);
  for (int j = 0 ; j < size; j++) {
    ID[j] = sIDs[j];
    model[j] = mapModel(smodels[j]);
  }

  verbose_ = verb;
}

bool ServoHerkulex::setPortLabel (const char* label)
{
  return manager.setPortLabel(label);
}

bool ServoHerkulex::reboot (int tID)
{
  std::vector<uint8_t> buf = {0x07, 0x00, HX_REBOOT};  // Reboot sequence
  buf[1] = (uint8_t)tID;
  sendData(buf);

  return true;
}

bool ServoHerkulex::ping ()
{
  int k = 0, n = 0, id = 0;
  std::vector<uint8_t> packet = std::vector<uint8_t> (12);

  std::cout << "Servos:" << std::endl;
  for (int j = 0; j < 0xFE; j++) {
    k = pingID(j);

    if (k == 12) {
      packet = manager.getAckPacket();
      id = packet[9];
      std::cout << "Herkulex [" << id << "] UP" << std::endl;
      n = n+1;
    }
    else std::cout << "Herkulex [" << j << "] Dowm" << std::endl;

  }

  if (n == 0) {
    std::cout << "None" << std::endl;
    return false;
  }

  return true;
}

bool ServoHerkulex::setID (int tID, int nID)
{
  std::vector<uint8_t> buf = {0x0A, 0x00, 0x03, 0x35, 0x01, 0x01};  // LED Green
  buf[1] = (uint8_t)tID;
  sendData(buf);

  return true;
}

bool ServoHerkulex::clearError (int tID)
{
  std::vector<uint8_t> buf = {0x0B, 0x00, 0x03, 0x30, 0x02, 0x00, 0x00};  // Clear Error
  buf[1] = (uint8_t)tID;
  sendData(buf);

  buf.resize(6);
  buf = {0x0A, 0x00, 0x03, 0x35, 0x01, 0x00};
  buf[1] = (uint8_t)tID;
  sendData(buf);

  return true;
}

std::vector<uint8_t> ServoHerkulex::getStatus (int tID)
{
  std::vector<uint8_t> buf = {0x07, 0x00, 0x07};
  std::vector<uint8_t> stat;
  buf[1] = (uint8_t)tID;
  sendData(buf, 9);
  stat = manager.getAckPacket();

  if (verbose_) {
    for (unsigned int j = 0; j < stat.size(); j++) {
      printf("%X ",stat[j]);
    }
    printf("\n");
  }

  return stat;
}

bool ServoHerkulex::setACK (int ACK)
{
  std::vector<uint8_t> buf = {0x0A, 0xFE, 0x03, 0x34, 0x01, 0x01};
  // 0 = no reply, 1 = reply to READ CMDs only, 2 = always reply
  buf[5] = (uint8_t)ACK;
  sendData(buf);

  return true;
}

bool ServoHerkulex::torqueOn (int tID)
{
  std::vector<uint8_t> buf = {0x0A, 0x00, 0x03, 0x34, 0x01, 0x60};
  buf[1] = (uint8_t)tID;
  sendData(buf);

  return true;
}

bool ServoHerkulex::torqueOff (int tID)
{
  std::vector<uint8_t> buf = {0x0A, 0x00, 0x03, 0x34, 0x01, 0x00};
  buf[1] = (uint8_t)tID;
  sendData(buf);

  return true;
}

bool ServoHerkulex::setLED (int tLED, int tID)
{
  std::vector<uint8_t> buf = {0x0A, 0x00, 0x03, 0x35, 0x01, 0x00};  // Set LED sequence
  buf[1] = (uint8_t)tID;
  buf[5] = (uint8_t)tLED;
  sendData(buf);

  return true;
}

bool ServoHerkulex::stopSpeedO201 (int tID, int tLED)
{
  // The servo will stop if speed is 0 and the stop flag is 0
  // or the speed is not 0 and the stop flag is 1.
  int speed = 0;

  // To make sure the MCU knows the servo is not moving (instead of "moving at speed zero")
  uint8_t uiStop = 1;
  uint8_t uiMode = 1;
  uiMode = uiMode << 1;
  uint8_t uiLED = (uint8_t)(tLED) << 2;
  uint8_t setVal = 0x00;

  uint8_t pt = 0x01;

  std::vector<uint8_t> buf = std::vector<uint8_t> (8);
  buf  = {0x0C, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
  buf[1] = (uint8_t)tID;
  buf[3] = pt;
  buf[7] = buf[1];

  buf[5] = speed >> 8;     // high byte
  buf[4] = speed & 0x00FF; // low byte

  setVal = (uiStop | uiMode | uiLED);
  buf[6] = setVal;

  sendData(buf);

  return true;
}

bool ServoHerkulex::moveAngle0601 (int goal, int tID, int tLED, float playtime)
{
  if (goal > 159 || goal < -159) {
    std::cout << "Goal out of recommended range [-159, 159]." << std::endl;
    return false;
  }

  if (playtime > 2844.0f || playtime < 11.2f) {
    std::cout << "Playtime out of recommended range [11.2ms, 2844ms]." << std::endl;
    return false;
  }

  uint8_t uiStop = 0;
  uint8_t uiMode = 0;
  uiMode = uiMode << 1;
  uint8_t uiLED = (uint8_t)(tLED) << 2;
  uint8_t setVal = 0x00;

  uint8_t pt = 0x00;
  pt = (uint8_t)(playtime/11.2f);

  uint16_t p = (uint16_t)(6.14f*goal) + 1024;

  std::vector<uint8_t> buf = std::vector<uint8_t> (8);
  buf  = {0x0C, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
  buf[1] = (uint8_t)tID;
  buf[3] = pt;
  buf[7] = buf[1];

  buf[5] = p >> 8;     // high byte
  buf[4] = p & 0x00FF; // low byte

  setVal = (uiStop | uiMode | uiLED);
  buf[6] = setVal;

  sendData(buf);

  return true;
}

bool ServoHerkulex::moveAngle0201 (int goal, int tID, int tLED, float playtime)
{
  if (goal > 159 || goal < -159) {
    std::cout << "Goal out of recommended range [-159, 159]." << std::endl;
    return false;
  }

  if (playtime > 2844.0f || playtime < 11.2f) {
    std::cout << "Playtime out of range [11.2ms, 2844ms]." << std::endl;
    return false;
  }

  uint8_t uiStop = 0;
  uint8_t uiMode = 0;
  uiMode = uiMode << 1;
  uint8_t uiLED = (uint8_t)(tLED) << 2;
  uint8_t setVal = 0x00;

  uint8_t pt = 0x00;
  pt = (uint8_t)(playtime/11.2f);

  uint16_t p = (uint16_t)(3.07*goal) + 512;

  std::vector<uint8_t> buf = std::vector<uint8_t> (8);
  buf  = {0x0C, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
  buf[1] = (uint8_t)tID;
  buf[3] = pt;
  buf[7] = buf[1];

  buf[5] = p >> 8;     // high byte
  buf[4] = p & 0x00FF; // low byte

  setVal = (uiStop | uiMode | uiLED);
  buf[6] = setVal;

  sendData(buf);

  return true;
}

bool ServoHerkulex::moveSpeed0201 (int speed, int tID, int tLED, float playtime)
{
  if (playtime > 2844.0f || playtime < 11.2f) {
    std::cout << "Playtime out of range [11.2ms, 2844ms]." << std::endl;
    return false;
  }

  if (speed > 1023 || speed < -1023) {
    std::cout << "Speed out of range [-1023, 1023]." << std::endl;
    return false;
  }

  if (speed < 0) {
    speed = (-1)*speed;
    speed |= 0x4000;
  }

  uint8_t uiStop = 0;
  uint8_t uiMode = 1;
  uiMode = uiMode << 1;
  uint8_t uiLED = (uint8_t)(tLED) << 2;
  uint8_t setVal = 0x00;

  uint8_t pt = 0x00;
  pt = (uint8_t)(playtime/11.2f);

  std::vector<uint8_t> buf = std::vector<uint8_t> (8);
  buf  = {0x0C, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
  buf[1] = (uint8_t)tID;
  buf[3] = pt;
  buf[7] = buf[1];

  buf[5] = speed >> 8;     // high byte
  buf[4] = speed & 0x00FF; // low byte

  setVal = (uiStop | uiMode | uiLED);
  buf[6] = setVal;

  sendData(buf);

  return true;
}

bool ServoHerkulex::moveSyncSpeed (std::vector<int> speed, std::vector<int> tID, std::vector<int> tLED, float playtime)
{
  int size = speed.size();
  int k, id = 0, c = 0, sp = 0;

  if (playtime > 2844.0f || playtime < 11.2f) {
    std::cout << "Playtime out of range [11.2ms, 2844ms]." << std::endl;
    return false;
  }

  for (k = 0; k < size; k++) {
    id = tID[k];
    sp = speed[k];

    if (sp > 1023 || sp < -1023) {
      std::cout << "Speed out of range [-1023, 1023]: servo [" << id << "]." << std::endl;
      return false;
    }
  }

  for (k = 0; k < size; k++) {
    sp = speed[k];

    if (sp < 0) {
      speed[k] = (-1)*speed[k];
      speed[k] |= 0x4000;
    }
  }

  sync_buffer.resize(4*size);

  sp = 0;
  int led = 0;
  uint8_t lsb = 0x00;
  uint8_t msb = 0x00;
  uint8_t setVal = 0x00;
  uint8_t uiStop = 0;
  uint8_t uiLED = 0;
  uint8_t uiMode = 1;
  uiMode = uiMode << 1;

  for (k = 0; k < size; k++) {
    led = tLED[k];
    sp = speed[k];
    id = tID[k];

    uiLED = (uint8_t)(led) << 2;
    msb = sp >> 8;     // high byte
    lsb = sp & 0x00FF; // low byte
    setVal = (uiStop | uiMode | uiLED);

    sync_buffer[c] = lsb;
    sync_buffer[c+1] = msb;
    sync_buffer[c+2] = setVal;
    sync_buffer[c+3] = uint8_t(id);

    c = c+4;
  }

  actionAll(playtime);

  return true;
}

bool ServoHerkulex::moveSync (std::vector<int> goal, std::vector<int> tID, std::vector<int> tLED, float playtime)
{
  int size = goal.size();
  int k, id = 0, c = 0;

  for (k = 0; k < size; k++) {
    id = tID[k];

    if (goal[k] > 159.8 || goal[k] < -159.8) {
      std::cout << "Goal out of recommended range [-159.8, 159.8]: servo [" << id << "]." << std::endl;
      return false;
    }
  }

  if (playtime > 2844.0f || playtime < 11.2f) {
    std::cout << "Playtime out of range [11.2ms, 2844ms] servo [" << id << "]." << std::endl;
    return false;
  }

  sync_buffer.resize(4*size);

  int led = 0, go = 0;

  uint8_t lsb = 0x00;
  uint8_t msb = 0x00;
  uint8_t setVal = 0x00;
  uint8_t uiStop = 0;
  uint8_t uiLED = 0;
  uint16_t p = 0x0000;
  uint16_t temp = 0x0000;
  uint8_t uiMode = 0;
  uiMode = uiMode << 1;

  for (k = 0; k < size; k++) {
    led = tLED[k];
    go = goal[k];
    id = tID[k];

    uiLED = (uint8_t)(led) << 2;
    temp = (uint16_t)(3.07*go);
    p = 512.0 + temp;
    msb = p >> 8;     // high byte
    lsb = p & 0x00FF; // low byte
    setVal = (uiStop | uiMode | uiLED);

    sync_buffer[c] = lsb;
    sync_buffer[c+1] = msb;
    sync_buffer[c+2] = setVal;
    sync_buffer[c+3] = uint8_t(id);

    c = c+4;
  }

  actionAll(playtime);
  return true;
}

bool ServoHerkulex::moveAsync (std::vector<int> goal, std::vector<int> tID, std::vector<int> tLED, std::vector<float> playtime)
{
  int size = goal.size();
  int k, id = 0, c = 0;

  for (k = 0; k < size; k++) {
    id = tID[k];

    if (goal[k] > 159.8 || goal[k] < -159.8) {
      std::cout << "Goal out of recommended range [-159.8, 159.8]: servo [" << id << "]." << std::endl;
      return false;
    }

    if (playtime[k] > 2844.0f || playtime[k] < 11.2f) {
      std::cout << "Playtime out of range [11.2ms, 2844ms] servo [" << id << "]." << std::endl;
      return false;
    }
  }

  sync_buffer.resize(5*size);

  int led = 0, go = 0, play = 0;

  uint8_t lsb = 0x00;
  uint8_t msb = 0x00;
  uint8_t setVal = 0x00;
  uint8_t uiStop = 0;
  uint8_t uiLED = 0;
  uint16_t p = 0x0000;
  uint16_t temp = 0x0000;
  uint8_t pt = 0x00;
  uint8_t uiMode = 0;
  uiMode = uiMode << 1;

  for (k = 0; k < size; k++) {
    led = tLED[k];
    go = goal[k];
    id = tID[k];
    play = playtime[k];

    uiLED = (uint8_t)(led) << 2;
    temp = (uint16_t)(3.07*go);
    p = 512.0 + temp;
    msb = p >> 8;     // high byte
    lsb = p & 0x00FF; // low byte
    setVal = (uiStop | uiMode | uiLED);
    pt = (uint8_t)(play/11.2f);

    sync_buffer[c] = lsb;
    sync_buffer[c+1] = msb;
    sync_buffer[c+2] = setVal;
    sync_buffer[c+3] = uint8_t(id);
    sync_buffer[c+4] = pt;

    c = c+5;
  }

  int a = 0;
  for (unsigned int j = 0; j < sync_buffer.size(); j++) {
    a = sync_buffer[j];
    std::cout << a << " ";
  }

  std::cout << std::endl;

  actionAll();
  return true;

}

std::vector<std::string> ServoHerkulex::getModels ()
{
  int s = model.size();
  std::vector<std::string> models = std::vector<std::string> (s);

  for (int j = 0 ; j < s; j++) {
    models[j] = remapModel(model[j]);
  }

  return models;
}

std::vector<int> ServoHerkulex::getIDs ()
{
  return ID;
}


float ServoHerkulex::getAngle0601 (int tID)
{
  float pos = (float) getPosition0601(tID);

	return (pos-1024.0f)*0.163f;
}

float ServoHerkulex::getAngle0201 (int tID)
{
  float pos = (float) getPosition0201(tID);

	return (pos-512.0f)*0.325f;
}

uint16_t ServoHerkulex::getPosition0201 (int tID)
{
  std::vector<uint8_t> ack  = std::vector<uint8_t> (13);
  uint16_t pos = 0;

  std::vector<uint8_t> buf = {0x09, 0x00, 0x04, 0x3A, 0x02}; // 3A
  buf[1] = (uint8_t)tID;
  sendData(buf, 13);

  ack = manager.getAckPacket();

  pos = ((ack[10]&0x03) << 8) | ack[9];

	return pos;
}

uint16_t ServoHerkulex::getPosition0601 (int tID)
{
  std::vector<uint8_t> ack  = std::vector<uint8_t> (13);
  uint16_t pos = 0;

  std::vector<uint8_t> buf = {0x09, 0x00, 0x04, 0x3A, 0x02}; // 3C
  buf[1] = (uint8_t)tID;
  sendData(buf, 13);

  ack = manager.getAckPacket();

  pos = ((ack[10]&0x07) << 8) | ack[9];

	return pos;
}

int ServoHerkulex::getSpeed (int tID)
{
  std::vector<uint8_t> ack = std::vector<uint8_t> (13);
  int sp  = 0;

  std::vector<uint8_t> buf = {0x09, 0x00, 0x04, 0x40, 0x02};
  buf[1] = (uint8_t)tID;
  sendData(buf, 13);

  ack = manager.getAckPacket();

  sp = ((ack[10]&0xFF) << 8) | ack[9];

	return sp;
}

int ServoHerkulex::mapModel (std::string mmodel)
{
  if (mmodel == "0101") {
    return H0101;
  }
  else if (mmodel == "0201") {
    return H0201;
  }
  else if (mmodel == "0401") {
    return H0401;
  }
  else if (mmodel == "0601") {
    return H0601;
  }
  else {
    std::cout << "Invalid model " << mmodel << ": setting default (O201)." <<  std::endl;
    return H0201;
  }
}

std::string ServoHerkulex::remapModel (int mmodel)
{
  if (mmodel == H0101) {
    return "0101";
  }
  else if (mmodel == H0201) {
    return "0201";
  }
  else if (mmodel == H0401) {
    return "0401";
  }
  else if (mmodel == H0601) {
    return "0601";
  }
  else return '\0';
}

int ServoHerkulex::pingID (int tID)
{
  int k = getID(tID);

  if (k == 12) {
    return k;
  }

  return -1;
}

int ServoHerkulex::getID (int gID)
{
  std::vector<uint8_t> buf = {0x09, 0x00, 0x04, 0x00, 0x01};
  buf[1] = (uint8_t)gID;
  return sendData(buf, 12);
}

int ServoHerkulex::getModel (int gID)
{
  std::vector<uint8_t> buf = {0x09, 0x00, 0x02, 0x00, 0x01};
  buf[1] = (uint8_t)gID;
  return sendData(buf, 12);
}

int ServoHerkulex::sendData (std::vector<uint8_t>& packet)
{
  return manager.sendTx(packet.size()+4, packet, packet[1], verbose_);
}

int ServoHerkulex::sendData (std::vector<uint8_t>& packet, int ack_length)
{
  return manager.sendTxRx(packet.size()+4, packet, ack_length, packet[1], verbose_);
}

void ServoHerkulex::addSync(uint8_t goalLSB, uint8_t goalMSB, uint8_t tSET, uint8_t tID)
{
  //sync_buffer[c++] = goalLSB;
  //sync_buffer[c++] = goalLSB;
  //sync_buffer[c++] = goalLSB;
  //sync_buffer[c++] = goalLSB;
}

bool ServoHerkulex::resizeBuffer (int length)
{
  manager.resizeData(length);

  return true;
}

bool ServoHerkulex::actionAll (float playtime)
{
  uint8_t pt = 0x00;
  pt = (uint8_t)(playtime/11.2f);
  int s = sync_buffer.size();

  std::vector<uint8_t> buf = std::vector<uint8_t> (s+4);
  buf[0] = s+8;
  buf[1] = (s == 4) ? sync_buffer[3] : 0xFE;
  buf[2] = 0x06;
  buf[3] = pt;

  // Shifting data
  for (int i = 0; i < s; i++) {
    buf[i+4] = sync_buffer[i];
  }

  sendData(buf);

  return true;
}

bool ServoHerkulex::actionAll ()
{
  int s = sync_buffer.size();

  std::vector<uint8_t> buf = std::vector<uint8_t> (s+3);
  buf[0] = s+7;
  buf[1] = (s == 5) ? sync_buffer[3] : 0xFE;
  buf[2] = 0x05;

  // Shifting data
  for (int i = 0; i < s; i++) {
    buf[i+3] = sync_buffer[i];
  }

  sendData(buf);

  return true;
}

uint8_t ServoHerkulex::checkSum1 (std::vector<uint8_t> bytes)
{
  //if (MIN_PACKET_SIZE < bytes.size() < MAX_PACKET_SIZE) {
  //  return PACKET_ERR_CS;
  //}
  int bs = bytes.size();

  uint8_t cs1 = 0;

  for (int j = 0; j < bs; j++) {
    cs1 = cs1 ^ bytes[j];
  }
  cs1 = cs1 & 0xFE;

  return cs1;
}

uint8_t ServoHerkulex::checkSum2 (uint8_t cs1)
{
  uint8_t cs2 = 0;

  cs2 = ~(cs1);
  cs2 = cs2 & 0xFE;

  return cs2;
}
