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

  std::vector<int> ids = std::vector<int> (5);
  std::vector<std::string> models = std::vector<std::string> (5);
  models = {"0201","0601","0201","0201","0201"};
  ids = {1,2,3,4,5};

  ServoHerkulex hklx(ids,models,1);

  std::cout << "set leds" << '\n';
  bool a = hklx.setLED(2,2);
  std::cout << a << std::endl;
  a = hklx.setLED(2,3);
  std::cout << a << std::endl;

  std::cout << "set torques" << '\n';
  bool b = hklx.torqueOn(2);
  std::cout << b << std::endl;
  b = hklx.torqueOn(3);
  std::cout << b << std::endl;

  int x = getchar();
  bool c;

/*
  std::vector<int> sids = std::vector<int> (2);
  std::vector<int> speeds = std::vector<int> (2);
  std::vector<int> sleds = std::vector<int> (2);
  sids = {5};
  speeds = {204};
  sleds = {1};*/

  /*
  std::cout << "move 5 syncSpeed" << '\n';
  c = hklx.moveSyncSpeed(speeds,sids,sleds,11.2);
  std::cout << c << std::endl;
  */

  std::cout << "Moving" << '\n';
  c = hklx.moveSpeed0201(512,3,4,2800);
  std::cout << c << std::endl;
  
  c = hklx.moveAngle0601(90,2,4,12);
  std::cout << c << std::endl;


  /*
  std::cout << "move 5 position" << '\n';
  c = hklx.moveAngle0201(0,5, 1, 11.2);
  std::cout << c << std::endl;
  */

  x = getchar();


  //std::cout << "get 5 speed" << '\n';
  //int q = hklx.getSpeed(5);
  //std::cout << q << std::endl;

  /*
  std::cout << "get 5 angle" << '\n';
  float t = hklx.getAngle(5);
  std::cout << t << std::endl;
  */

  return 0;
}
