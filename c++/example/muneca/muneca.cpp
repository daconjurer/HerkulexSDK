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
/* El siguiente programa tiene como finalidad mover culaquier servo en modo RUEDA, 
   Cambiar el id y el modelo segun crresponda al servo que se planea utilizar  */
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

  int n = 1;  // Number of servos
  bool a;     // Serial communication error variable
  int f = 0;  // Serial communication error tracking flag
  std::vector<uint8_t> stat;  // Status packet
  float p = 0;
  float e = 0;

  
  // Aqui se debe de cmbiar el modelo y el id segun corresponda al servo que se ocupara 
  char* model = "0201";
  int id = 1;

  ServoHerkulex hklx(id,model,1);
  hklx.setPortLabel(port_label);

  // Clear error flags (often needed)
  std::cout << "Clearing error flag..." << '\n';
  for (int i = 0; i < n; i++) {
    a = hklx.clearError(id);
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
    a = hklx.setLED(LED_CYAN,id);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  // Set torques
  std::cout << "Setting torques..." << '\n';
  for (int i = 0; i < n; i++) {
    a = hklx.torqueOn(id);
    if (!a) {
      std::cout << a << std::endl;
      f = 1;
    }
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  std::cout << "Moving servo 1..." << '\n';
  std::cout << "Initial speed: " << hklx.getSpeed(id) << std::endl;
  /* El inicio y paro de los motores se ejecuta mediante ESC + ENTER 
   Para variar la velocidad se tiene que cambir el primer valor del siguiente comando
  a = hklx.moveSpeed0201(500,id,LED_GREEN,12); hasta un maximo de 2844*/   
  
  while (!(getchar() == 27));

  a = hklx.moveSpeed0201(500,id,LED_GREEN,12);

  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(id,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  std::cout << "Final speed: " << hklx.getSpeed(id) << std::endl;



  return 0;
}
