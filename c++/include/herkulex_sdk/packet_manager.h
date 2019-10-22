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

////////////////////////////////////////////
/// @file Hovis HerkuleX Packets manager
/// @author Victor Esteban Sandoval-Luna
////////////////////////////////////////////

#ifndef HERKULEX_SDK_PACKETMANAGER_H_
#define HERKULEX_SDK_PACKETMANAGER_H_

#include <iostream>
#include <vector>

#include "port_handler.h"

#define BROADCAST_ID        0xFE

#define MIN_PACKET_SIZE     7
#define MAX_PACKET_SIZE     233

// Communication Result
#define PORT_OK             0     // tx or rx packet communication success
#define PORT_BUSY           -11   // Port is busy
#define COM_RX_FAIL         -12   // Failed instruction packet transmition
#define COM_TX_FAIL         -13   // Failed getting status packet
#define COMM_TX_ERROR       -14   // Incorrect instruction packet
#define COMM_RX_RECIEVING   -15   // Recieving status packet
#define COMM_RX_TIMEOUT     -16   // No status packet
#define COMM_RX_CORRUPT     -17   // Incorrect status packet

// Packet Check
#define PACKET_ERR_CS       -21   // Checksum error

namespace herkulex
{

class PacketManager
{
  private:
    // attributes
    PortHandler port;
    std::vector<uint8_t> header = std::vector<uint8_t> (2);
    uint8_t pSize;
    uint8_t CMD;
    uint8_t pID;
    uint8_t cs1;
    uint8_t cs2;
    std::vector<uint8_t> data = std::vector<uint8_t> (1);
    std::vector<uint8_t> ack_packet = std::vector<uint8_t> (15);

    // methods
    int sendPacket (int verbose);
    int sendreceivePacket (int verbose, int ack_length);
    char *buildUp (std::vector<uint8_t> bytes);
    uint8_t checkSum1 (std::vector<uint8_t> bytes, uint8_t& cs1);
    uint8_t checkSum2 (uint8_t cs1, uint8_t& cs2);

  public:

    PacketManager ();
    virtual ~PacketManager () { }

    int sendTx (int length, std::vector<uint8_t> buf, int ID, int& verb);
    int sendTxSync (int length, std::vector<uint8_t> buf, int& verb);
    int sendTxRx (int length, std::vector<uint8_t> buf, int ack_length, int ID, int& verb);
    bool setPortLabel (const char* portlabel);

    std::vector<uint8_t> getData ();
    std::vector<uint8_t> getAckPacket ();

    bool resizeData (int length);
};

}

#endif
