
#include <stdio.h>
#include <string.h>

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "SteeringSerial.h"

int main()
{
    InitSteeringSerial();

    while (true)
    {
        uint8_t byte = ReadByte();
        if (byte == '\n')
        {
            SendAnswer();
        }
    }
}
