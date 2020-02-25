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

/* Author: Victor Esteban Sandoval-Luna */

#include "herkulex_sdk.h"
#include <unistd.h>

using namespace herkulex;

int main()
{
  /* Constructor parameters */
  const char* PORT_LABEL = "/dev/ttyUSB0";
  const int BAUDRATE = 115200;
  const int VERBOSITY = 1;

  bool a;     /* Serial communication error variable */
  int f = 0;  /* Serial communication error tracking flag */

  int id = 1; /* Servo ID */

  /* ServoHerkulex instance */
  ServoHerkulex hklx((char*)PORT_LABEL, (int)BAUDRATE, (int)VERBOSITY);

  while (!(getchar() == 27));

  /* Clear error flag (often needed) */
  std::cout << "Clearing error flag..." << std::endl;
  a = hklx.clearError(id);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  if (!f)
    std::cout << "DONE" << std::endl;

  while (!(getchar() == 27));

  /* Set LED */
  std::cout << "Setting led..." << std::endl;
  a = hklx.setLED(LED_CYAN,id);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  if (!f)
    std::cout << "DONE" << std::endl;

  while (!(getchar() == 27));

  /* Set torque */
  std::cout << "Setting torque..." << std::endl;
  a = hklx.torqueOn(id);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  if (!f)
    std::cout << "DONE" << std::endl;

  while (!(getchar() == 27));

  /* Move servo at speed X */
  std::cout << "Moving at speed..." << std::endl;
  a = hklx.moveSpeed0201(800,id,LED_GREEN,12);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  if (!f)
    std::cout << "DONE" << std::endl;

  while (!(getchar() == 27));

  /* Get and print out the speed */
  std::cout << "Speed: " << hklx.getSpeed(id) << std::endl;

  while (!(getchar() == 27));

  /* Stop the servo */
  std::cout << "Stopping servo..." << std::endl;
  a = hklx.stopSpeedO201(id,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  if (!f)
    std::cout << "DONE" << std::endl;

  while (!(getchar() == 27));
  /* Get and print out the angle */
  std::cout << "Initial angle: " << hklx.getAngle0201(id) << std::endl;

  while (!(getchar() == 27));

  /* Move servo to angle X */
  std::cout << "Moving to angle..." << std::endl;
  a = hklx.moveAngle0201(25,1,LED_PURPLE,2000);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  if (!f)
    std::cout << "DONE" << std::endl;

  /* Get and print out the angle */
  std::cout << "Final angle: " << hklx.getAngle0201(id) << std::endl;

  return 0;
}

