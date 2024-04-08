
#ifndef SERIAL_PORT_HPP_
#define SERIAL_PORT_HPP_

#include <cstdint>
#include <string>
#include <vector>


namespace serial
{

enum class BaudRate : uint32_t
{
    B_9600 = 9600,
    B_19200 = 19200,
    B_38400 = 38400,
    B_57600 = 57600,
    B_115200 = 115200,
    B_230400 = 230400,
    B_460800 = 460800,
    B_CUSTOM
};

enum class NumDataBits : uint8_t
{
    FIVE = 5,
    SIX = 6,
    SEVEN = 7,
    EIGHT = 8
};

enum class NumStopBits : uint8_t
{
    ONE = 1,
    TWO = 2
};

enum class Parity : uint8_t
{
    NONE,
    EVEN,
    ODD
};

enum class HardwareFlowControl : uint8_t
{
    OFF,
    ON
};

enum class SoftwareFlowControl : uint8_t
{
    OFF,
    ON
};

struct ttyConfig
{
    BaudRate baudRate = BaudRate::B_9600;
    NumDataBits numDataBits = NumDataBits::EIGHT;
    NumStopBits numStopBits = NumStopBits::ONE;
    Parity parity = Parity::NONE;
    HardwareFlowControl hardwareFlowControl = HardwareFlowControl::OFF;
    SoftwareFlowControl softwareFlowControl = SoftwareFlowControl::OFF;
    uint16_t vtime = 10;  // Timeout in deciseconds (0.1s)
    uint16_t vmin = 0;    // read() wait for vmin bytes before returning
    uint32_t customBaudRate = 0;
};

enum class State : uint8_t
{
    CLOSED,
    OPEN
};

class SerialPort
{
public:
    SerialPort() = default;
    SerialPort(const std::string &device);
    SerialPort(const std::string &device, const ttyConfig &config);
    ~SerialPort();

    void set_device(const std::string &device);
    void set_config(const ttyConfig &config);

    const std::string &get_device() const;
    const ttyConfig &get_config() const;
    State get_state() const;

    void open();
    void close();

    size_t write(const std::string &data) const;
    size_t write_binary(const std::vector<uint8_t> &data) const;
    size_t read(std::string &data);
    size_t read_binary(std::vector<uint8_t> &data);
private:
    const size_t BUFFER_SIZE = 255;
private:
    std::string device_;
    ttyConfig config_;
    State state_ = State::CLOSED;
    int fileDesc_;
    std::vector<uint8_t> buffer_;
private:
    void configure_termios() const;
};

}  // namespace serial

#endif  // SERIAL_PORT_HPP_
