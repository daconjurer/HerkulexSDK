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
#include <string>
#include <time.h>

void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Stroing start time
    clock_t start_time = clock();

    // looping till required time is not acheived
    while (clock() < start_time + milli_seconds)
        ;
}

using namespace herkulex;

int main()
{
  // Port label (/dev/ttyUSB*)
  const char* port_label = "/dev/ttyUSB0";

  int n = 4;  // Number of servos
  bool a;     // Serial communication error variable
  int f = 0;  // Serial communication error tracking flag
  std::vector<uint8_t> stat;  // Status packet
  //float p = 0;
  //float e = 0;

  std::vector<int> ids = std::vector<int> (3);
  std::vector<std::string> models = std::vector<std::string> (3);
  models = {"0201","0601","0201","0201"};
  ids = {1,2,4,5};

  if (ids.size() != models.size()) {
    std::cout << "Number of models and IDs mismatched." << std::endl;
    return 1;
  }

  ServoHerkulex hklx(ids,models,1);
  hklx.setPortLabel(port_label);

  // Clear error flags (often needed)
  std::cout << "Clearing error flag..." << '\n';
  for (int i = 0; i < n; i++) {
    a = hklx.clearError(ids[i]);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }

  if (!f)
    std::cout << "DONE" << std::endl;

  // Set LEDs
  std::cout << "Setting leds..." << '\n';
  for (int i = 0; i < n; i++) {
    a = hklx.setLED(LED_CYAN,ids[i]);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  while (!(getchar() == 27));

  // Set torques
  std::cout << "Setting torques..." << '\n';
  for (int i = 0; i < n; i++) {
    a = hklx.torqueOn(ids[i]);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  if (!f)
    std::cout << "DONE" << std::endl;     

  std::cout << "Moving servo 1..." << '\n';
  std::cout << "Initial speed: " << hklx.getSpeed(1) << std::endl;

  getchar();
  //levantamiento brazo principal 
  while (!(getchar() == 27));
  //int angle = -50; 
  int limit = -82;   // goal
  //e = 2;            // random inital e+rror
  a = hklx.moveAngle0601(limit,2,LED_PURPLE,2000);

  //movimiento de levantamiento
  while (!(getchar() == 27));
  int limit2 = 25;   // goal
  //e = 2;            // random inital error
  a = hklx.moveAngle0201(limit2,4,LED_PURPLE,2000);
  while (!(getchar() == 27));

  //cerrar griper 
  a = hklx.moveSpeed0201(800,5,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(5,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  //girar muñeca (ABRIR)
  
  while (!(getchar() == 27));
  //a = hklx.clearError(ids[0]);
  a = hklx.moveSpeed0201(500,1,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(1,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  //abrir griper
  while (!(getchar() == 27));
  a = hklx.moveSpeed0201(-800,5,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(5,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  //cerrar griper
  while (!(getchar() == 27));
  a = hklx.moveSpeed0201(800,5,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(5,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  //girar muñeca (CERRAR)
  while (!(getchar() == 27));
  a = hklx.moveSpeed0201(-500,1,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(1,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  //abrir griper
  while (!(getchar() == 27));
  a = hklx.moveSpeed0201(-800,5,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(5,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }






  std::cout << "Final speed: " << hklx.getSpeed(1) << std::endl;

/*
  std::cout << "Moving servo 1..." << '\n';
  std::cout << "Initial angle: " << hklx.getAngle0201(1) << std::endl;

  getchar();

  int angle = 159;
  e = angle;
  a = hklx.moveAngle0201(angle,1,LED_PURPLE,1000);
  if (!a)
    std::cout << a << std::endl;

  while (e > 1 ||  e < -1) {
    p = hklx.getAngle0201(1);
    std::cout << "angle: " << p << std::endl;
    e = p - angle;
    std::cout << "error: " << e << std::endl;
  }

  std::cout << "final angle: " << hklx.getAngle0201(1) << std::endl;

  */

  //////////////////////////////////////////////////////////////////////////////
  // If you want to move the servos within a for loop, you might
  // find it useful to define both a playtimes vector and a goals
  // vector:

  // Playtimes (2844.0 >= playtime <= 11.2)

  //std::vector<float> playtime = std::vector<float> (5);
  //playtime = {1344,1344,1344,1344,1344};

  // Goal angles (in degrees)

  //std::vector<int> goal = std::vector<int> (5);
  //goal = {90,0,0,0,0};

  // In case a different LED color is needed for each servo:

  //std::vector<int> color = std::vector<int> (5);
  //color = {1,1,1,1,1};

  //////////////////////////////////////////////////////////////////////////////

  /*
  std::cout << "Moving servo 2..." << '\n';
  //std::cout << hklx.getAngle(2) << std::endl;
  a = hklx.moveAngle0601(0,2,LED_PURPLE,12);
  if (!a)
    std::cout << a << std::endl;

  //std::cout << hklx.getAngle(2) << std::endl;

  std::cout << "Moving servo 3..." << '\n';
  // a = hklx.moveAngle0201(90,3,LED_GREEN,12);
  a = hklx.moveSpeed0201(512,3,LED_GREEN,2800);
  if (!a)
    std::cout << a << std::endl;

  std::cout << "Moving servo 4..." << '\n';
    a = hklx.moveAngle0201(0,4,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  // Stop execution
  while (!(getchar() == 27));

  std::cout << "Stopping servo 2..." << '\n';

  */
  /*
  for (int i = 1; i < n; i++) {
    a = hklx.torqueOn(i);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  */

  /*
  a = hklx.stopSpeedO201(3,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  if (!f)
    std::cout << "DONE" << std::endl;

*/

///////////////////////////////////////////////
// Please do not uncomment this section yet.
///////////////////////////////////////////////

/*
  // The two DOF of the gripper will move in wheel mode (moveSpeed0201 method)
  std::cout << "Moving servo 5..." << '\n';
  a = hklx.moveSpeed0201(512,ids[4],LED_GREEN,2800);
  if (!a)
    std::cout << a << std::endl;

  std::cout << "Moving servo 1..." << '\n';
    a = hklx.moveSpeed0201(512,ids[0],LED_GREEN,2800);
  if (!a)
    std::cout << a << std::endl;

  */

  return 0;
}
