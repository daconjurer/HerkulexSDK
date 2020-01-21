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

/*!
  \file
  \brief PacketManager class definition.
  
  The PacketManager class builds up the HerkuleX packets based on the function requested by the user through the ServoHerkulex instance. It also has methods for sending and recieving such packets to and from the servomotors through serial port using a PortHandler instance.
*/

/////////////////////////////////////////////////////////////////////////////////////////
// @author Victor Esteban Sandoval-Luna
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef HERKULEX_SDK_PACKETMANAGER_H_
#define HERKULEX_SDK_PACKETMANAGER_H_

#include <iostream>
#include <vector>

#include "port_handler.h"

/** \brief ID for broadcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless their Error flag is UP.
 */
#define BROADCAST_ID        0xFE
/** \brief Minimum packet size. All the packets should be at least 7 bytes in lenght. Please see the 
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define MIN_PACKET_SIZE     7
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define MAX_PACKET_SIZE     233
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#ifdef MIN_PACKET_SIZE
  #define MIN_BUFFER_LENGTH (MIN_PACKET_SIZE - 4)
#endif
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#ifdef MAX_PACKET_SIZE
  #define MAX_BUFFER_LENGTH (MAX_PACKET_SIZE - 4)
#endif
// Communication Result

/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define COM_OK              0     // Tx or Rx packet communication success
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define COM_TX_FAIL         -11   // Failed instruction packet transmission
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define COM_RX_FAIL         -12   // Failed getting status packet
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define COM_RX_TIMEOUT      -13   // No status packet
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define COM_RX_CORRUPT      -14   // Incorrect status packet (ACK)
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define PACKET_MIN_ERROR    -15   // Incorrect instruction packet (MIN)
/** \brief ID for boradcasting i.e. all the servos follow the instruction.
 
 When using the broadcast ID in the packet, all the servos that are connected will execute the command unless the Error flag is UP.
 */
#define PACKET_MAX_ERROR    -16   // Incorrect instruction packet (MAX)

namespace herkulex
{

/**
  \brief It handles the HerkuleX commands.
  
  The PacketManager class builds up the HerkuleX packets based on the function requested by the user through the ServoHerkulex instance. It also has methods for sending and recieving such packets to and from the servomotors through serial port using a PortHandler instance.
*/
class PacketManager
{
  private:
    // Attributes
    
    // PortHandler member for reading/writing to serial port
    PortHandler port;
    // Size of the packet
    uint8_t pSize;
    // Command code (see datasheet)
    uint8_t CMD;
    // Servo ID
    uint8_t pID;
    // Checksum1 result (see datasheet)
    uint8_t cs1;
    // Checksum2 result (see datasheet)
    uint8_t cs2;
    // Packet header (see datasheet)
    std::vector<uint8_t> header = std::vector<uint8_t> (2);
    // Actual data of the packet (see datasheet)
    std::vector<uint8_t> data = std::vector<uint8_t> (1);
    // ACK packet, stores the ACK sent by the servo (see datasheet)
    std::vector<uint8_t> ack_packet = std::vector<uint8_t> (15);
    
    // Verbosity
    int verbosity;

    // Methods
    
    // Writes the data to serial port using PorHandler member
    int sendTx ();
    // Writes to and reads from serial port using PorHandler member
    int sendTxRx (int ack_length);
    // Builds up the packet adding the header and the checksums (see datasheet)
    int buildUp (std::vector<uint8_t> bytes);
    // Checksums methods for error checking (see datasheet)
    uint8_t checkSum1 (std::vector<uint8_t> bytes);
    uint8_t checkSum2 ();

  public:
    // Constructors
    PacketManager ();
    PacketManager (const int& verb);
    PacketManager (char* port_name, const int& baudrate, const int& verb);
    virtual ~PacketManager () { }

    // Sends the packet (this method interacts with the ServoHerkulex object directly)
    int sendPacket (std::vector<uint8_t> buf, int ID);
    // Sends the packet and reads the ACK (this method interacts with the ServoHerkulex object directly)
    int sendreceivePacket (std::vector<uint8_t> buf, int ack_length, int ID);

    // Debug methods (get the instruction packet and ACK packet, respectively)
    std::vector<uint8_t> getData () const;
    std::vector<uint8_t> getAckPacket () const;

    void endPacketManager ();
};

}

#endif
