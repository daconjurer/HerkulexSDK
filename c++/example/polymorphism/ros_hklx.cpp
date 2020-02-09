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

using namespace herkulex;

class ros_hklx: public ServoHerkulex
{
  private:

  public:
    ros_hklx (int verb): ServoHerkulex(verb) {
    }

};//End of class ros_hklx

int main()
{
  ros_hklx *servo;
  ros_hklx hklx(1);
  servo = &hklx;

  servo->setLED(2,5);

  int a = getchar();

  std::cout << "set torque" << '\n';
  bool b = servo->torqueOn(5);
  std::cout << b << std::endl;

  std::cout << "move 5 angle" << '\n';
  bool c = servo->moveAngle0201(0,5,1,11.2);
  std::cout << c << std::endl;

  return 0;
}
