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

/*Este programa tiene como finalidad encontrar el area de movimiento correcta de cada servo 
  por separado, se debe de poner el servo en las posiciones fisicas limites y posteriormente
   corre el programa para conocer los angulos limites y asi trabajar dentro de esos margenes
   */


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
  //////////////////////////
  // SETUP
  //////////////////////////

  // Port label (/dev/ttyUSB*)
  const char* port_label = "/dev/ttyUSB0";

  int n = 1;  // Number of servos
  bool a;     // Serial communication error variable
  int f = 0;  // Serial communication error tracking flag
  std::vector<uint8_t> stat;  // Status packet
  float p = 0;
  float e = 0;

  char* model = "0201"; // cambiar el modelo de 0201 por 0601 segun sea el caso 
                        //del servo a utilizar
  int id = 4; // El id esta visible pintado a un costado de cada servo, usar el id del servo 
              // que se desea ocupar 



  ServoHerkulex hklx(id,model,1);
  hklx.setPortLabel(port_label);

  // Clear error flags (often needed)
  std::cout << "Clearing error flag..." << '\n';
  a = hklx.clearError(id);
  a = hklx.clearError(3);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  // Set LEDs
  std::cout << "Setting leds..." << '\n';
  a = hklx.setLED(LED_CYAN,id);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }
  if (!f)
    std::cout << "DONE" << std::endl;

  //while(!(getchar() == 27));

  // Set torques
  std::cout << "Setting torques..." << '\n';
  a = hklx.torqueOn(id);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }
  if (!f)
    std::cout << "DONE" << std::endl;

//////////////////////////
//////////////////////////
//////////////////////////


/*Las funciones siguientes son para leer el angulo del servo 0601 o del servo 0201
  std::cout << "Initial angle: " << hklx.getAngle0601(id) << std::endl;
  std::cout << "Initial angle: " << hklx.getAngle0201(id) << std::endl; */ 

  std::cout << "Moving servo 1..." << '\n';
  a = hklx.setLED(LED_GREEN,id);
  //std::cout << "Initial angle: " << hklx.getAngle0601(id) << std::endl;
  std::cout << "Initial angle: " << hklx.getAngle0201(id) << std::endl;
  

  

  getchar();
/* Las funciones siguientes sirven para mover los servos con la pulsacion de ESC + ENTER 
  una ves introducido un angulo que se encuentre entre los limites */
   
  int limit = 25;   // goal
  e = 2;            // random inital error
  //a = hklx.moveAngle0601(limit,id,LED_PURPLE,12);
  a = hklx.moveAngle0201(limit,id,LED_PURPLE,1800);
  
  
  

  return 0;
}
