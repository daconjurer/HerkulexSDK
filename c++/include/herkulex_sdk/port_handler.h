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
  \file port_handler.h
  \brief PortHandler class definition.
*/

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

/**
  \brief It configures and enables the serial port in Linux.
  
  The PortHandler class enables the Linux serial port for writing and
  reading. It has configuration methods that allow serial port
  communication using USB-to-TTL converters.
  
  \author Victor Esteban Sandoval-Luna
*/
class PortHandler
{
  private:
    /* Attributes */
    /** The port name label. */
    char*   port_name_;
    /** The socket status flag. */
    int     socket_fd_;
    /** The baudrate of the port (see the speed_t definitions in
    termios.h). */
    int     baudrate_;

    /** Default Baudrate. */
    const int DEFAULT_BAUDRATE = 115200;

    /* Methods */

    /** Sets the attributes of the serial interface. */
    int setInterfaceAttribs (const int& fd, const int& baudrate,
      const int& parity);

  public:
    /* Constructors */
    /** Default constructor.
      The default constructor uses the label "/dev/ttyUSB0" as the name
      of the USB-to-TTL device and sets 115200 as the port's
      communication speed. */
    PortHandler ();
    /** Constructor with port name and baudrate parameters.
      This constructor uses the portname parameter as the name of the
      USB-to-TTL device and the baudrate parameter for setting the
      communication speed (between 9600 bps and 460800 bps).
      \param portname The port name label e.g. "/dev/ttACM0".
      \param baudrate The baudrate in bps e.g. 9600. */
    PortHandler (char* portname, const int& baudrate);

    virtual ~PortHandler() { }

    /** Opens the serial port using the port_name_, the baudrate_ and
      the socket_fd_ attributes. */
    bool openPort();

    /** Closes the serial port using the socket_fd_ attribute. */
    void closePort();

    /** Clears the serial port with tcflush. */
    void clearPort();

    /** Sets the baudrate of the port. */
    int setBaudRate(const int& baudrate);

    /** Retrieves the name of the port, for debug purposes. */
    char* getPortName () const;

    /** Gets the number of bytes available for reading from the port
    buffer. */
    int getBytesAvailable ();

    /** Reads from port.
      \param packet The packet of bytes that is to be sent.
      \param length The length of the packet (in bytes). */
    int readPort(uint8_t *packet, const int& length);

    /** Writes to port.
      \param packet The packet of bytes that is to be sent.
      \param length The length of the packet (in bytes). */
    int writePort(uint8_t *packet, const int& length);
};

}

#endif
