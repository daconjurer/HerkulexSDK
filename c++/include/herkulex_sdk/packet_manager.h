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
    char* buildUp (std::vector<uint8_t> bytes);
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
    std::vector<uint8_t> getACKPacket ();

    bool resizeData (int length);
};

}

#endif
