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

/**
  \file
  \brief PacketManager class definition. Please see the PacketManager
  class for further reference
*/

#ifndef HERKULEX_SDK_PACKETMANAGER_H_
#define HERKULEX_SDK_PACKETMANAGER_H_

#include <iostream>
#include <vector>

#include "port_handler.h"

/**
  \brief Value for broadcasting i.e. all the servos follow the
  instruction.
  
  When using the broadcast ID in the packet, all the servos that are
  connected will execute the command unless their Error flag is UP.
*/
#define BROADCAST_ID        0xFE
/** \brief Minimum HerkuleX packet size.
    
    All the packets should be at least 7 bytes in length (no data).
    Please see the datasheet for further details.
*/
#define MIN_PACKET_SIZE     7
/**
  \brief Maximum HerkuleX packet size.
  
  All the packets should have at most 233 bytes in length. Please see
  the datasheet for further details.
*/
#define MAX_PACKET_SIZE     233
/**
  \brief Minimum buffer actual length.
  
  This buffer represents the packet without the header and the
  checksum values. Therefore the minimum value is 3.
*/
#ifdef MIN_PACKET_SIZE
  #define MIN_BUFFER_LENGTH (MIN_PACKET_SIZE - 4)
#endif
/**
  \brief Maximum buffer actual length.
  
  This buffer represents the packet without the header and the
  checksum values. Therefore the maximum value is 229.
*/
#ifdef MAX_PACKET_SIZE
  #define MAX_BUFFER_LENGTH (MAX_PACKET_SIZE - 4)
#endif
/**
  \brief Communication result.
  
  Tx or Rx communication successful result.
*/
#define COM_OK              0
/**
  \brief Packet transmission fail result.
  
  The packet transmission failed.
*/
#define COM_TX_FAIL         -11
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define COM_RX_FAIL         -12
/**
  \brief Timed out reception result.
  
  No status packet (ACK) was received.
*/
#define COM_RX_TIMEOUT      -13
/**
  \brief Corrupted packet result.
  
  The status packet (ACK) is corrupted.
*/
#define COM_RX_CORRUPT      -14
/**
  \brief Command packet .
  
  Incorrect command packet (MIN).
*/
#define PACKET_MIN_ERROR    -15
/**
  \brief Command packet .
  
  The command packet is longer (MAX).
*/
#define PACKET_MAX_ERROR    -16

namespace herkulex
{

/**
  \brief It handles the HerkuleX commands.
  
  The PacketManager class builds up the HerkuleX packets based on the
  function requested by the user through the ServoHerkulex instance.
  It also has methods for sending and recieving such packets to and from
  the servomotors through serial port using a PortHandler instance.
  
  \author Victor Esteban Sandoval-Luna
*/
class PacketManager
{
  private:
    /* Attributes */
    /** PortHandler member for reading/writing from/to serial port */
    PortHandler port;
    /** Size of the packet */
    uint8_t pSize;
    /** Command code (see datasheet) */
    uint8_t CMD;
    /** Servo ID */
    uint8_t pID;
    /** Checksum1 result (see datasheet) */
    uint8_t cs1;
    /** Checksum2 result (see datasheet) */
    uint8_t cs2;
    /** Packet header (see datasheet) */
    std::vector<uint8_t> header = std::vector<uint8_t> (2);
    /** Actual data of the packet (see datasheet) */
    std::vector<uint8_t> data = std::vector<uint8_t> (1);
    /** ACK packet, stores the ACK sent by the servo (see datasheet) */
    std::vector<uint8_t> ack_packet = std::vector<uint8_t> (15);
    
    /* Verbosity attribute. It allow displaying the data that has been
      sent or has been received. */
    int verbosity;

    /* Methods */
    /** Writes the data to serial port using the PorHandler member */
    int sendTx ();
    /** Writes the data to and reads the data from serial port using
      the PorHandler member */
    int sendTxRx (int ack_length);
    /** Builds up the packet adding the header and the checksums (see
      datasheet) */
    int buildUp (std::vector<uint8_t> bytes);
    /** Checksum1 method for error checking (see datasheet) */
    uint8_t checkSum1 (std::vector<uint8_t> bytes);
    /** Checksum2 method for error checking (see datasheet) */
    uint8_t checkSum2 ();

  public:
    /* Constructors */
    /** Default constructor.
      The default constructor sets 115200 bps as the baudrate, sets the
      verbosity attribute as 1 and initializes the buffer with zeros
      and the header. */
    PacketManager ();
    /** Constructor with verbosity paramater.
      This constructor sets 115200 bps as the baudrate, takes the
      verbosity parameters in and initializes the buffer with zeros and
      the header.
      \param verb Verbosity flag (0 or 1) for printing out the content
      of the packet to console. */
    PacketManager (const int& verb);
    /** Constructor with port name, verbosity and baudrate paramaters.
      Takes the parameters in and initializes the buffer with zeros and
      the header.
      \param port_name The label of the device for communication.
      \param baudrate The baudrate in bps e.g.115200.
      \param verb Verbosity flag (0 or 1) for printing out the content
      of the packet to console. */
    PacketManager (char* port_name, const int& baudrate,
      const int& verb);
    virtual ~PacketManager () { }

    /** Sends the packet (this method interacts with the ServoHerkulex
      object directly)
      \param buf The HerkuleX packet to be sent.
      \param ID THe ID of the servo. */
    int sendPacket (std::vector<uint8_t> buf, int ID);
    /** Sends the packet and reads the ACK (this method interacts with
      the ServoHerkulex object directly)
      \param buf The HerkuleX packet to be sent.
      \param ack_length The length of the ACK packet to be received.
      \param ID THe ID of the servo. */
    int sendreceivePacket (std::vector<uint8_t> buf, int ack_length,
      int ID);

    /** Gets the instruction packet (for debug purposes). */
    std::vector<uint8_t> getData () const;
    /** Gets the ACK packet  (for debug purposes). */
    std::vector<uint8_t> getAckPacket () const;
    /** Disables the device communication */
    void endPacketManager ();
};

}

#endif
