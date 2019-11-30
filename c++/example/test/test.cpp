/*
  std::cout << "Moving servo 1..." << '\n';
  std::cout << "Initial speed: " << hklx.getSpeed(1) << std::endl;

  getchar();

  a = hklx.moveSpeed0201(1000,1,LED_GREEN,12);
  if (!a)
    std::cout << a << std::endl;

  while (!(getchar() == 27));

  a = hklx.stopSpeedO201(1,LED_CYAN);
  if (!a) {
    std::cout << a << std::endl;
    f = 1;
  }

  std::cout << "Final speed: " << hklx.getSpeed(1) << std::endl;
  
*/

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
