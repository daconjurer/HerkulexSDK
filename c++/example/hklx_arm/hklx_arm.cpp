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

/* Author: Victor Esteban Sandoval-Luna */

#include "herkulex_sdk.h"
#include <unistd.h>

#define DEVICENAME "/dev/ttyUSB0"

using namespace herkulex;

int main()
{
  int n = 4;  // Number of servos
  bool a;     // Serial communication error variable
  int f = 0;  // Serial communication error tracking flag
  std::vector<uint8_t> stat;  // Status packet

  std::vector<int> ids = std::vector<int> (3);
  std::vector<std::string> models = std::vector<std::string> (3);
  models = {"0601","0201","0201","0201"};
  ids = {2,3,5,6};

  ServoHerkulex hklx(ids,models,1);
  hklx.setPortLabel(DEVICENAME);

  std::cout << "Clear error flag" << '\n';
  for (int i = 1; i < n+1; i++) {
    a = hklx.clearError(i);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  std::cout << "Set leds" << '\n';
  for (int i = 1; i < n+1; i++) {
    a = hklx.setLED(LED_CYAN,i);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  std::cout << "Set torques" << '\n';
  for (int i = 1; i < n+1; i++) {
    a = hklx.torqueOn(i);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  int x = getchar();

  /*
  std::cout << "move 5 syncSpeed" << '\n';
  c = hklx.moveSyncSpeed(speeds,sids,sleds,11.2);
  std::cout << c << std::endl;
  */

  std::cout << "Moving" << '\n';

  a = hklx.moveSpeed0201(512,3,LED_GREEN,2800);
  if (!a)
    std::cout << a << std::endl;

  stat = hklx.getStatus(3);

/*
  a = hklx.moveAngle0601(0,2,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;
*/

  return 0;
}
