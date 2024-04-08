
#ifndef SERIAL_PORT_EXCEPTION_HPP_
#define SERIAL_PORT_EXCEPTION_HPP_

#include <stdexcept>
#include <string>


namespace serial
{
  class SerialPortException : public std::runtime_error
  {
  public:
    SerialPortException(const char *file, int line, const std::string &arg)
    : std::runtime_error(arg)
    {
      msg_ = std::string(file) + ":" + std::to_string(line) + ": " + arg;
    }

    const char *what() const noexcept override
    {
      return msg_.c_str();
    }

  private:
    std::string msg_;
  };

#define THROW_EXCEPT(arg) throw serial::SerialPortException(__FILE__, __LINE__, arg);
}  // namespace serial

#endif  // SERIAL_PORT_EXCEPTION_HPP_
