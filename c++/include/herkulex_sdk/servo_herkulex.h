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

////////////////////////////////////////////////////////
/// @file Hovis HerkuleX servo library
/// @author Victor Esteban Sandoval-Luna
////////////////////////////////////////////////////////

#ifndef HERKULEX_SDK_SERVOHERKULEX_H_
#define HERKULEX_SDK_SERVOHERKULEX_H_

#include <iostream>
#include <cerrno>
#include <vector>
#include <string>

#include "packet_manager.h"

// Hovis HerkuleX Request Packet CMDs
#define HX_EEPWRITE         0x01  // Rom write Request Packet
#define HX_EEPREAD          0x02  // Rom read Request Packet
#define HX_RAMWRITE         0x03  // Ram write Request Packet
#define HX_RAMREAD          0x04  // Ram read Request Packet
#define HX_IJOG             0x05  // IJOG Request Packet (Write command to servo with different oepration timing)
#define HX_SJOG             0x06  // SJOG Request Packet (Write command to servos simultaneously)
#define HX_STAT             0x07  // Stat Request Packet (Read error)
#define HX_ROLLBACK         0x08  // Rollback Request Packet (Back to factory value)
#define HX_REBOOT           0x09  // Reboot Request Packet

// Hovis HerkuleX ACK Packet CMDs
#define HX_EEPWRITE_ACK     0x41  // Rom write ACK Packet
#define HX_EEPREAD_ACK      0x42  // Rom read ACK Packet
#define HX_RAMWRITE_ACK     0x43  // Ram write ACK Packet
#define HX_RAMREAD_ACK      0x44  // Ram read ACK Packet
#define HX_IJOG_ACK         0x45  // IJOG ACK Packet
#define HX_SJOG_ACK         0x46  // SJOG ACK Packet
#define HX_STAT_ACK         0x47  // Stat ACK Packet
#define HX_ROLLBACK_ACK     0x48  // Rollback ACK Packet
#define HX_REBOOT_ACK       0x49  // Reboot ACK Packet

// Hovis HerkuleX Status Error
#define HX_STATUS_OK        0x00
#define HX_ERR_VOLT         0x01
#define HX_ERR_POT          0x02
#define HX_ERR_TEMP         0x04
#define HX_ERR_PCKT         0x08
#define HX_ERR_OVLD         0x10
#define HX_ERR_DRIVER       0x20
#define HX_ERR_EEPREG       0x40

// Hovis HerkuleX LEDs
#define LED_GREEN           0x01
#define LED_BLUE            0x02
#define LED_CYAN            0x03
#define LED_RED             0x04
#define LED_LGREEN          0x05
#define LED_PURPLE          0x06
#define LED_WHITE           0x07


namespace herkulex
{

class ServoHerkulex
{
  private:
    // attributes
    std::vector<int> ID = std::vector<int> (1);     // default
    std::vector<int> model = std::vector<int> (1);  // default
    std::vector<uint8_t> sync_buffer = std::vector<uint8_t> (1);
    PacketManager manager;
    int verbose_;

    // methods
    int mapModel (std::string mmodel);
    std::string remapModel (int mmodel);
    int pingID(int tID);
    int getID (int gID);
    int getModel (int gID);
    int sendData (std::vector<uint8_t>& packet);
    int sendData (std::vector<uint8_t>& packet, int ack_length);

    void addSync (uint8_t goalLSB, uint8_t goalMSB, uint8_t tSET, uint8_t tID);
    bool resizeBuffer (int length);
    bool actionAll (float playtime);
    bool actionAll ();

    // uint16_t getPosition (int tID);

    uint8_t checkSum1 (std::vector<uint8_t> bytes);
    uint8_t checkSum2 (uint8_t cs1);

  public:
    // Constructors
    ServoHerkulex (int verb);
    ServoHerkulex (int tID, const char *smodel, int verb);
    ServoHerkulex (std::vector<int> sIDs, std::vector<std::string> smodels, int verb);

    bool setPortLabel (const char* label);

    // Hovis HerkuleX servos register map
    bool reboot (int sID);
    bool ping ();
    bool setID (int cID, int nID);
    bool clearError (int tID);
    std::vector<uint8_t> getStatus (int tID);
    bool setACK (int tID);
    bool torqueOn (int tID);
    bool torqueOff (int tID);
    bool setLED (int tLED, int tID);

    bool stopSpeedO201 (int tID, int tLED);

    bool moveAngle0601 (int goal, int tID, int tLED, float playtime);
    bool moveAngle0201 (int goal, int tID, int tLED, float playtime);

    bool moveSpeed0201 (int speed, int tID, int tLED, float playtime);

    bool moveSyncSpeed (std::vector<int> speed, std::vector<int> tID, std::vector<int> tLED, float playtime);

    bool moveSync (std::vector<int> goal, std::vector<int> tID, std::vector<int> tLED, float playtime);
    bool moveAsync (std::vector<int> goal, std::vector<int> tID, std::vector<int> tLED, std::vector<float> playtime);

    // Hovis HerkuleX servos class
    std::vector<std::string> getModels ();
    std::vector<int> getIDs ();
    float getAngle0601 (int tID);
    float getAngle0201 (int tID);
    int getSpeed (int tID);

    uint16_t getPosition0201 (int tID);
    uint16_t getPosition0601 (int tID);

};

}

#endif
