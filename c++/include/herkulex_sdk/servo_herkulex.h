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
  \file servo_herkulex.h
  \brief ServoHerkulex class definition.
*/

#ifndef HERKULEX_SDK_SERVOHERKULEX_H_
#define HERKULEX_SDK_SERVOHERKULEX_H_

#include <iostream>
#include <cerrno>
#include <vector>
#include <string>

#include "packet_manager.h"

/* Hovis HerkuleX Request Packet CMDs */
/** \brief ROM write Request Packet command (see datasheet). */
#define HX_EEPWRITE         0x01
/** \brief ROM read Request Packet command (see datasheet). */
#define HX_EEPREAD          0x02
/** \brief RAM write Request Packet command. */
#define HX_RAMWRITE         0x03
/** \brief RAM read Request Packet command. */
#define HX_RAMREAD          0x04
/** \brief IJOG Request Packet command.
  Write command to servo with different operation timing (see
  datasheet) */
#define HX_IJOG             0x05
/** \brief SJOG Request Packet command.
  Write command to servos simultaneously (see datasheet) */
#define HX_SJOG             0x06
/** \brief Status Request Packet command. It reads the error status
  (see datasheet). */
#define HX_STAT             0x07
/** \brief Rollback Request Packet command (Back to factory value). */
#define HX_ROLLBACK         0x08
/** \brief Reboot Request Packet command. */
#define HX_REBOOT           0x09

/* Hovis HerkuleX ACK Packet CMDs */
/** \brief ROM write ACK Packet command. */
#define HX_EEPWRITE_ACK     0x41
/** \brief ROM read ACK Packet command. */
#define HX_EEPREAD_ACK      0x42
/** \brief RAM write ACK Packet command. */
#define HX_RAMWRITE_ACK     0x43
/** \brief RAM read ACK Packet command. */
#define HX_RAMREAD_ACK      0x44
/** \brief IJOG ACK Packet command. */
#define HX_IJOG_ACK         0x45
/** \brief SJOG ACK Packet command. */
#define HX_SJOG_ACK         0x46
/** \brief Status ACK Packet command. */
#define HX_STAT_ACK         0x47
/** \brief Rollback ACK Packet command. */
#define HX_ROLLBACK_ACK     0x48
/** \brief Reboot ACK Packet command. */
#define HX_REBOOT_ACK       0x49

/* Hovis HerkuleX Status Error */
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_STATUS_OK        0x00
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_ERR_VOLT         0x01
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_ERR_POT          0x02
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_ERR_TEMP         0x04
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_ERR_PCKT         0x08
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_ERR_OVLD         0x10
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_ERR_DRIVER       0x20
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define HX_ERR_EEPREG       0x40

/* Hovis HerkuleX LEDs */
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define LED_GREEN           0x01
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define LED_BLUE            0x02
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define LED_CYAN            0x03
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define LED_RED             0x04
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define LED_LGREEN          0x05
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define LED_PURPLE          0x06
/**
  \brief Packet reception fail result.
  
  The status packet (ACK) reception failed.
*/
#define LED_WHITE           0x07

namespace herkulex
{

/**
  \brief It provides the abstraction layer for the servomotors
  functions.
  
  The ServoHerkulex class offers high level functions for controlling,
  monitoring and defining the behavior of the HerkuleX servomotors using
  a serial port in Linux.
  
  \author Victor Esteban Sandoval-Luna
*/
class ServoHerkulex
{
  private:
    /* Attributes */
    /** PacketManager member for building up the HerkuleX packets */
    PacketManager manager;
    /** Verbosity */
    int verbosity;
    
    /** For connected servos IDs and models traceability whithin
      applications */
    std::vector<int> ID = std::vector<int> (1);
    /** For connected servos IDs and models traceability whithin
      applications */
    std::vector<int> model = std::vector<int> (1);
    
    /** Buffer for sync operations (see datasheet) */
    std::vector<uint8_t> sync_buffer = std::vector<uint8_t> (1);

    /* Methods */
    /** */
    int pingID(int tID);
    /** */
    int getID (int gID);
    /** */
    int getModel (int gID);
    /** */
    int sendData (std::vector<uint8_t>& packet);
    /** */
    int sendData (std::vector<uint8_t>& packet, int ack_length);

    /** */
    bool resizeBuffer (int length);
    /** */
    bool actionAll (float playtime);
    /** */
    bool actionAll ();

    /** */
    uint8_t checkSum1 (std::vector<uint8_t> bytes);
    /** */
    uint8_t checkSum2 (uint8_t cs1);

  public:
    /* Constructors */
    /** Default constructor.
      The default constructor sets 115200 bps as the baudrate and sets
      the verbosity attribute as 1. */
    ServoHerkulex (const int& verb);
    /** Constructor with port name, baudrate and verbosity parameters.
      \param port_name The label of the device used for the
      communication with the servos
      \param baudrate The baudrate in bps e.g. 115200.
      \param verbosity The verbosity flag (0 or 1). */
    ServoHerkulex (char* port_name, const int& baudrate,
      const int& verb);

    /* Hovis HerkuleX servos class */
    /** It sends the reboot packet (see datasheet).
      \param ID The ID of the servo. */
    bool reboot (int sID);
    /** It performs a ping through the line to all the
      servos (from 0 to 255) and prints the IDs of the servos that are
      connected. */
    bool ping ();
    /** It changes the ID of the servo.
      \param cID The current ID of the servo.
      \param nID The new ID of the servo. */
    bool setID (int cID, int nID);
    /** It clears the error flag of the servo (see datasheet).
      \param tID The ID of the servo. */
    bool clearError (int tID);
    /** It gets the error status of the servo (see datasheet).
      \param tID The ID of the servo. */
    std::vector<uint8_t> getStatus (int tID);
    /** It sets the ACK policy (see datasheet).
      \param tID the ID of the servo. */
    bool setACK (int tID);
    /** It enables the torque of the motor.
      \param tID the ID of the servo. */
    bool torqueOn (int tID);
    /** It disables the torque of the motor.
      \param tID the ID of the servo. */
    bool torqueOff (int tID);
    /** It enables and sets the color of the servo's' LED.
      \param tLED The color code.
      \param tID The ID of the servo. */
    bool setLED (int tLED, int tID);
    /** It stops a DRS-0201 servo that is moving at constant speed. */
    bool stopSpeedO201 (int tID, int tLED);
    /** It moves a DRS-0601 servo to the given angle in the given
      playtime. */
    bool moveAngle0601 (int goal, int tID, int tLED, float playtime);
    /** It moves a DRS-0201 servo to the given angle in the given
      playtime. */
    bool moveAngle0201 (int goal, int tID, int tLED, float playtime);
    /** It moves a servo at the given speed in the given playtime. */
    bool moveSpeed0201 (int speed, int tID, int tLED, float playtime);
    /** It moves a group of servos at the given speed in the given
      playtime. */
    bool moveSyncSpeed (std::vector<int> speed, std::vector<int> tID,
      std::vector<int> tLED, float playtime);
    /** It moves a group of servos to the given angles in the given
      playtime. */
    bool moveSync (std::vector<int> goal, std::vector<int> tID,
      std::vector<int> tLED, float playtime);
    /** It moves a group of servos to the given angles, each one in the
      respective playtime. */
    bool moveAsync (std::vector<int> goal, std::vector<int> tID,
      std::vector<int> tLED, std::vector<float> playtime);

    /** It gets the models of the servos that are connected. */
    std::vector<std::string> getModels ();
    /** It gets the IDs of the servos that are connected. */
    std::vector<int> getIDs () const;
    /** It gets the angular position (in degrees) of the DRS-0601 servo.
      \param tID The ID of the servo. */
    float getAngle0601 (int tID);
    /** It gets the angular position (in degrees) of the DRS-0201 servo.
      \param tID The ID of the servo. */
    float getAngle0201 (int tID);
    /** It gets the angular speed (in bits) of the servo (DRS-0201
      and DRS-0601).
      \param tID The ID of the servo. */
    int getSpeed (int tID);

    /** It gets the angular position (in bits) of the DRS-0201 servo.
      \param tID The ID of the servo. */
    uint16_t getPosition0201 (int tID);
    /** It gets the angular position (in bits) of the DRS-0601 servo.
      \param tID The ID of the servo. */
    uint16_t getPosition0601 (int tID);

    /** Disables the device communication through the PacketManager
      attribute. */
    void closeHerkulex ();
};

}

#endif

