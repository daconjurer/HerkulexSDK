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
/// @file Direct port handler through USB-TTL converter
/// @author Victor Esteban Sandoval-Luna
////////////////////////////////////////////////////////

#ifndef HERKULEX_SDK_PORTHANDLER_H_
#define HERKULEX_SDK_PORTHANDLER_H_

#include <iostream>
#include <cerrno>
#include <fcntl.h>
#include <string.h>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace herkulex
{

class PortHandler
{
  private:
    // attributes
    char*   port_name;
    int     socket_fd;
    int     baudrate_;

    double  packet_start_time_;
    double  packet_timeout_;
    double  tx_time_per_byte;

    bool using_;

    // methods
    double getCurrentTime();
    double getTimeSinceStart();

    // Sets the attributes of the serial interface
    int setInterfaceAttribs (int fd, int baudrate, int parity);
    // Sets if the port blocks or not
    void setBlocking (int fd, int block);

    int remapBaudRate(const int baudrate);

  public:
    // Default HerkuleX Baudrate
    const int DEFAULT_BAUDRATE = 115200;

    // Constructors
    PortHandler ();

    PortHandler (const char *portname);

    virtual ~PortHandler() { }

    // Opens the serial port
    bool openPort();

    // Closes the serial port
    void closePort();

    // Clears the port
    void clearPort();

    // Sets the name of the port
    void setPortName(const char* port_name);

    // Gets the name of the port
    char *getPortName ();

    // Sets the baudrate of the port
    int setBaudRate(const int baudrate);

    // Gets the baudrate of the port
    int getBaudRate();

    // Gets the number of bits available for reading from the port buffer
    int getBytesAvailable();

    // Reads port
    int readPort(uint8_t *packet, int length);

    // Writes to port
    int writePort(uint8_t *packet, int length);

};

}

#endif
