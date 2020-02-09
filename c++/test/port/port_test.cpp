#include "herkulex_sdk.h"

using namespace herkulex;

int main()
{
	const char* port_name = "/dev/ttyUSB0";
	PortHandler port((char*)port_name,115200);
	uint8_t a[10] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	uint8_t b[10] = {0};

	port.openPort();
	port.clearPort();
	port.writePort(a,10);

	int f = port.readPort(b,10);
	
	if (f == 10) {
		for (int i = 0; i < 10; i++) {
			std::cout << b[i] << std::endl;
		}
	}
}
