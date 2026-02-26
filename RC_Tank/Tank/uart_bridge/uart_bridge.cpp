//
#include "UART.hpp"
#include <iostream>
#include <stdint.h>


using namespace std;

#define DEV "/dev/ttyUSB0"
#define BAUDRATE 9600


///////////////////////////////////////////////////////////////////////////////

UART u (
	DEV,
	BAUDRATE
);

int main(int argc, char** argv) {
	uint8_t data_to_send[] = {10};
	uint8_t data_to_receive[1];


	u.write(data_to_send);

	cout << data_to_receive[0] << endl;
	//sleep(1);


	u.read(data_to_receive);

	cout << data_to_receive[0] << endl;

	return 0;
}


