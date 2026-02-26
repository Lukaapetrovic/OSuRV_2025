#include "SerialCPP.h"
#include <unistd.h>

int main()
{
    SerialCPP::SerialCPP uart("/dev/ttyAMA0", SerialCPP::BaudRate::BR_115200);
    if (!uart.open()) return -1;

    uart.writeBytes({0x00}); // STOP
    usleep(100000);
    uart.close();
    return 0;
}
