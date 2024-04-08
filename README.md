# linux_serial_port
Library for managing Linux serial ports. It is based on the [CppLinuxSerial](https://github.com/gbmhunter/CppLinuxSerial/tree/master) library.

### Example

```cpp
#include <string>

#include "serial_port/serial_port.hpp"


int main()
{
    serial::SerialPort port("/dev/ttyS2");

    serial::ttyConfig config {
        .baudRate = serial::BaudRate::B_115200,
        .numDataBits = serial::NumDataBits::EIGHT,
        .numStopBits = serial::NumStopBits::ONE,
        .parity = serial::Parity::NONE,
        .hardwareFlowControl = serial::HardwareFlowControl::OFF,
        .softwareFlowControl = serial::SoftwareFlowControl::OFF,
        .vtime = 10,
        .vmin = 0
    };

    port.set_config(config);
    port.open();

    port.write("Hello");

    std::string answer;
    port.read(answer);

    port.close();

    return 0;
}
```

### References

[Linux Serial Ports Using C/C++](https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/)

[Terminal Modes](https://www.gnu.org/software/libc/manual/html_node/Terminal-Modes.html)

