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
/// @file PortHandler class implementation. This class allow serial port read/write
/// non-canonical control. It is intended to be used with FTDI USB-to-TTL converters.
///
/// @author Victor Esteban Sandoval-Luna
/////////////////////////////////////////////////////////////////////////////////////////

#include "../../include/herkulex_sdk/port_handler.h"

using namespace herkulex;

// Default yet never used in the application
PortHandler::PortHandler ()
{
  port_name_ = (char*)"/dev/ttyUSB0";
  socket_fd_ = -1;
  setBaudRate(DEFAULT_BAUDRATE);
}

PortHandler::PortHandler (char* const portname, const int& baudrate)
{
  port_name_ = portname;
  socket_fd_ = -1;
  setBaudRate(baudrate);
}

bool PortHandler::openPort ()
{
  socket_fd_ = open (port_name_, O_RDWR | O_NOCTTY | O_SYNC);

  if (socket_fd_ < 0) {
    std::cout << "Error opening " << port_name_ << ": "<< std::strerror(errno) <<  std::endl;
    return false;
  }

  setInterfaceAttribs(socket_fd_, baudrate_, 0);
  tcflush(socket_fd_, TCIFLUSH);

  return true;
}

void PortHandler::closePort ()
{
  if(socket_fd_ != -1) {close(socket_fd_);}
  socket_fd_ = -1;
}

void PortHandler::clearPort ()
{
  tcflush(socket_fd_, TCIFLUSH);
}

void PortHandler::setPortName (char* const portname)
{
  port_name_ = (char*)portname;
}

char* PortHandler::getPortName () const {return port_name_;}

int PortHandler::setBaudRate (const int& baudrate)
{
  // Considering both Hovis HerkuleX servos limits and USB-to-TTL hardware limits (up to 500000 bps)
  int br = -1;

  switch (baudrate) {
    case 9600:
      baudrate_ = B9600;
      br = 0;
    case 19200:
      baudrate_ = B19200;
      br = 0;
    case 38400:
      baudrate_ = B38400;
      br = 0;
    case 57600:
      baudrate_ = B57600;
      br = 0;
    case 115200:
      baudrate_ = B115200;
      br = 0;
    case 230400:
      baudrate_ = B230400;
      br = 0;
    case 460800:
      baudrate_ = B460800;
      br = 0;
    default:
      baudrate_ = B115200;
      br = 0;
  }

  if (br == -1) {
    std::cout << "Error setting baudrate (Invalid baudrate): Default baudrate 115200 bps." << std::endl;
  }

  return br;
}

int PortHandler::getBytesAvailable ()
{
  int bytes_available;
  ioctl(socket_fd_, FIONREAD, &bytes_available);
  return bytes_available;
}

int PortHandler::readPort (uint8_t* const packet, const int& length)
{
  return read(socket_fd_, packet, length);
}

int PortHandler::writePort (uint8_t* const packet, const int& length)
{
  return write(socket_fd_, packet, length);
}

int PortHandler::setInterfaceAttribs (const int& fd, const int& baudrate, const int& parity)
{
  struct termios tty;

  if (tcgetattr(fd, &tty) < 0) {
    std::cout << "Error getting term attributes of " << port_name_ << ": " << std::strerror(errno) <<  std::endl;;
    return -1;
  }

  cfsetospeed(&tty, baudrate);
  cfsetispeed(&tty, baudrate);

  // cflag
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;		// 8-bit chars
  tty.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  // iflag
  tty.c_iflag &= ~IGNBRK;		// disable break processing
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);		// shut off xon/xoff ctrl

  // lflag
  tty.c_lflag = 0;	// no signaling chars, no echo,non-canonical processing

  // oflag
  tty.c_oflag = 0;	// no remapping, no delays

  // cc
  tty.c_cc[VMIN]  = 0;	// read doesn't block
  tty.c_cc[VTIME] = 5;	// 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cout << "Error " << std::strerror(errno) << " from tcsetattr.\n";
    return -1;
  }

  if (!isatty(fd)) {
    std::cout << "Error " << std::strerror(errno) << " from isatty.\n";
    return -1;
  }

  if (tcgetattr(fd, &tty) != 0) {
    std::cout << "Error " << std::strerror(errno) << " from tcgetattr.\n";
    return -1;
  }
  return 0;
}

