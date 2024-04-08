
#include "serial_port/serial_port.hpp"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <errno.h>      // Error number definition
#include <fcntl.h>      // File control definition
#include <sys/ioctl.h>  // Used for TCGETS2, which is required for custom baud rate
#include <unistd.h>     // UNIX standard function definition

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

#include "serial_port/serial_port_exception.hpp"


namespace serial
{
    
SerialPort::SerialPort(const std::string &device) : device_{device}
{}

SerialPort::SerialPort(const std::string &device, const ttyConfig &config) : device_{device}
{
    set_config(config);
}

SerialPort::~SerialPort()
{
    if (state_ == State::OPEN) {
        close();
    }
}

void SerialPort::set_device(const std::string &device)
{
    if (state_ == State::OPEN) {
        THROW_EXCEPT("The port is already open. To configure it, please close it!");
    }

    device_ = device;
}

void SerialPort::set_config(const ttyConfig &config)
{
    if (state_ == State::OPEN) {
        THROW_EXCEPT("The port is already open. To configure it, please close it!");
    }

    config_ = config;
}

const std::string &SerialPort::get_device() const
{
    return device_;
}

const ttyConfig &SerialPort::get_config() const
{
    return config_;
}

State SerialPort::get_state() const
{
    return state_;
}

void SerialPort::open()
{
    if (state_ == State::OPEN) {
        THROW_EXCEPT("The port is already open!");
    }

    if (device_.empty()) {
        THROW_EXCEPT("Attempted to open the port, but the device has not yet been assigned!");
    }

    fileDesc_ = ::open(device_.c_str(), O_RDWR);

    if (fileDesc_ == -1) {
        THROW_EXCEPT("Could not open " + device_ + "." + " Check the device and r/w permissions!");
    }

    configure_termios();
    buffer_.reserve(BUFFER_SIZE);
    state_ = State::OPEN;
}

void SerialPort::close()
{
    if (state_ == State::CLOSED) {
        THROW_EXCEPT("The port is already closed!");
    }

    if (fileDesc_ != -1)
    {
        auto retVal = ::close(fileDesc_);

        if (retVal != 0) {
            THROW_EXCEPT("Could not close " + device_ + "!");
        }

        fileDesc_ = -1;
    }

    state_ = State::CLOSED;
}

size_t SerialPort::write(const std::string &data) const
{
    if (state_ == State::CLOSED) {
        THROW_EXCEPT("The port is closed!");
    }

    ssize_t writeResult = ::write(fileDesc_, data.c_str(), data.size());

    if (writeResult == -1) {
        throw std::system_error(EFAULT, std::system_category());
    }

    return static_cast<size_t>(writeResult);
}

size_t SerialPort::write_binary(const std::vector<uint8_t> &data) const
{
    if (state_ == State::CLOSED) {
        THROW_EXCEPT("The port is closed!");
    }

    ssize_t writeResult = ::write(fileDesc_, data.data(), data.size());

    if (writeResult == -1) {
        throw std::system_error(EFAULT, std::system_category());
    }

    return static_cast<size_t>(writeResult);
}

size_t SerialPort::read(std::string &data)
{
    if (state_ == State::CLOSED) {
        THROW_EXCEPT("The port is closed!");
    }

    ssize_t readResult = ::read(fileDesc_, buffer_.data(), BUFFER_SIZE);

    if (readResult == -1) {
        throw std::system_error(EFAULT, std::system_category());
    }

    // Means EOS or device disconnection
    // Try to make termios2 distinguish between these two states
    if (readResult == 0)
    {
        struct termios2 tty;
        auto retVal = ioctl(fileDesc_, TCGETS2, &tty);
        if (retVal != 0) {
            throw std::system_error(EFAULT, std::system_category());
        }
    }

    data += std::string(reinterpret_cast<char*>(buffer_.data()), readResult);

    return static_cast<size_t>(readResult);
}

size_t SerialPort::read_binary(std::vector<uint8_t> &data)
{
    if (state_ == State::CLOSED) {
        THROW_EXCEPT("The port is closed!");
    }

    ssize_t readResult = ::read(fileDesc_, buffer_.data(), BUFFER_SIZE);

    if (readResult == -1) {
        throw std::system_error(EFAULT, std::system_category());
    }

    // Means EOS or device disconnection
    // Try to make termios2 distinguish between these two states
    if (readResult == 0)
    {
        struct termios2 tty;
        auto retVal = ioctl(fileDesc_, TCGETS2, &tty);
        if (retVal != 0) {
            throw std::system_error(EFAULT, std::system_category());
        }
    }

    std::copy(buffer_.begin(), buffer_.begin() + readResult, std::back_inserter(data));

    return static_cast<size_t>(readResult);
}

void SerialPort::configure_termios() const
{
    struct termios2 tty;

    ioctl(fileDesc_, TCGETS2, &tty);

    // More see https://man7.org/linux/man-pages/man3/tcflush.3.html
    tty.c_cflag &= ~CSIZE;
    switch (config_.numDataBits)
    {
        case NumDataBits::FIVE:
            tty.c_cflag |= CS5;
            break;
        case NumDataBits::SIX:
            tty.c_cflag |= CS6;
            break;
        case NumDataBits::SEVEN:
            tty.c_cflag |= CS7;
            break;
        case NumDataBits::EIGHT:
            tty.c_cflag |= CS8;
            break;
        default:
            THROW_EXCEPT("The number of data bits is not supported!");
    }

    switch (config_.numStopBits)
    {
        case NumStopBits::ONE:
            tty.c_cflag &= ~CSTOPB;
            break;
        case NumStopBits::TWO:
            tty.c_cflag |= CSTOPB;
            break;
        default:
            THROW_EXCEPT("The number of stop bits is not supported!");
    }

    switch (config_.parity)
    {
        case Parity::NONE:
            tty.c_cflag &= ~PARENB;
            break;
        case Parity::EVEN:
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD;
            break;
        case Parity::ODD:
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD;
            break;
        default:
            THROW_EXCEPT("Parity value is not supported!");
    }

    switch (config_.hardwareFlowControl)
    {
        case HardwareFlowControl::OFF:
            tty.c_cflag &= ~CRTSCTS;
            break;
        case HardwareFlowControl::ON:
            tty.c_cflag |= CRTSCTS;
            break;
        default:
            THROW_EXCEPT("Hardware flow control value is not supported!");
    }

    switch (config_.softwareFlowControl)
    {
        case SoftwareFlowControl::OFF:
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
        case SoftwareFlowControl::ON:
            tty.c_iflag |= (IXON | IXOFF | IXANY);
            break;
        default:
            THROW_EXCEPT("Software flow control value is not supported!");
    }

    tty.c_cflag |= CREAD | CLOCAL;  // Enable receiver and Ignore modem control lines

    // Baud rate settings
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= CBAUDEX;

    if (config_.baudRate == BaudRate::B_CUSTOM)
    {
        tty.c_ispeed = static_cast<speed_t>(config_.customBaudRate);
        tty.c_ospeed = static_cast<speed_t>(config_.customBaudRate);
    } else {
        tty.c_ispeed = static_cast<speed_t>(config_.baudRate);
        tty.c_ospeed = static_cast<speed_t>(config_.baudRate);
    }

    tty.c_oflag = 0;  // No remapping, no delays
    // tty.c_oflag &= ~OPOST;   // Prevent special interpretation of output bytes (e.g. newline)
    // tty.c_oflag &= ~ONLCR;   // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS;  // Prevent conversion of tabs to spaces (NOT IN LINUX)
    // tty.c_oflag &= ~ONOEOT;  // Prevent removal of EOT characters in output (NOT IN LINUX)

    // Control character settings
    // c_cc[VTIME] sets the inter-character timer, in units of 0.1s
    // c_cc[VMIN] sets the number of characters to block (wait) for when read() is called
    // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
    // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
    // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
    // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
    //                      after first character has elapsed

    tty.c_cc[VTIME] = static_cast<cc_t>(config_.vtime);
    tty.c_cc[VMIN] = static_cast<cc_t>(config_.vmin);

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Local mode settings
    tty.c_lflag &= ~ICANON;  // Turn off canonical input
    tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo

    ioctl(fileDesc_, TCSETS2, &tty);
}

}  // namespace serial
