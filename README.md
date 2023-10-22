# HerkulexSDK

This is a library for controlling the Hovis HerkuleX servo motors (tested on models DRS-0101, DRS-DRS-0201 and DRS-0601) in C++, running on Linux. The library communicates the computer with the servos through serial port and a generic USB-to-TTL converter and currently allows basic angular movement commands as well as angular speed setting commands. It also provides basic functions such as ping, LEDs turning on/off, position/speed reading and ID setting.

### Prerequisites

The library is compiled with ```make``` using ```GCC```. Therefore ```build-essential``` should be installed.

```
sudo apt-get install build-essential
```

## Build and Install

```
mkdir build && cd build && cmake .. && sudo make install
```


Then you can start using the library by including the header *"herkulex_sdk.h"* in your *.cpp* files.

## Authors

* **Victor Sandoval** - [daconjurer](https://github.com/daconjurer)

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

* The code is strongly based on the ROBOTIS Dynamixel SDK (Protocol1.0/2.0) https://github.com/ROBOTIS-GIT/DynamixelSDK part of the ROBOTIS Official GitHub ([ROBOTIS-GIT](https://github.com/ROBOTIS-GIT)).


## Note

* The HerkuleX trademark is property of [HYULIMROBOT](http://www.dstrobot.com/). This SDK intends to be an API for developers and is free software.
