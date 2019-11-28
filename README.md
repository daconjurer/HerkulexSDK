# HerkulexSDK

This is a library for controlling the Hovis HerkuleX servo motors (tested on models DRS-0101, DRS-DRS-0201 and DRS-0601) in C++, running on Linux. The library communicates the computer with the servos through serial port and a generic USB-to-TTL converter and currently allows basic angular movement commands as well as angular speed setting commands. It also provides basic functions such as ping, LEDs turning on/off, position/speed reading and ID setting.

### Prerequisites

The library is compiled with ```make``` using ```GCC```. Therefore ```build-essential``` should be installed.

```
sudo apt-get install build-essential
```

### Installing

Upon downloading the repo, on the *build* directory run the command

```
make
```

to compile the dynamic library. And then

```
sudo make install
```

to install the library on your */usr/local/* directory. Then you can start using the library by including the header *"herkulex_sdk.h"* in your *.cpp* files.

## Running the examples

The *example* directory includes some examples using the library. To create a new example just create a new folder *example_folder* and copy one of the existing *makefiles*, replacing the *TARGET* name. Every example is compiled using the command ```make``` on the corresponding *example_folder* folder.

## Authors

* **Victor Sandoval** - [daconjurer](https://github.com/daconjurer)

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

* The code is strongly based on the ROBOTIS Dynamixel SDK (Protocol1.0/2.0) https://github.com/ROBOTIS-GIT/DynamixelSDK part of the ROBOTIS Official GitHub ([ROBOTIS-GIT](https://github.com/ROBOTIS-GIT)).


