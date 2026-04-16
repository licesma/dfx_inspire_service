#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <functional>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>

class SerialPort
{
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2,
             const std::function<void(const std::string&)>& raise_error = nullptr)
  : raise_error_(raise_error), port_(port)
  {
    set_timeout(timeout_ms);
    Init(port, baudrate);
  }

  ~SerialPort()
  {
    if (fd_ >= 0) close(fd_);
  }

  ssize_t send(const uint8_t* data, size_t len)
  {
    ssize_t ret = ::write(fd_, data, len);
    if (ret < 0 && raise_error_)
      raise_error_("[SerialPort] send() failed on " + port_);
    else if (ret >= 0 && static_cast<size_t>(ret) != len && raise_error_)
      raise_error_("[SerialPort] short write on " + port_);
    return ret;
  }

  ssize_t recv(uint8_t* data, size_t len)
  {
    FD_ZERO(&rSet_);
    FD_SET(fd_, &rSet_);
    ssize_t recv_len = 0;

    switch (select(fd_ + 1, &rSet_, NULL, NULL, &timeout_))
    {
    case -1: // error
      if (raise_error_) raise_error_("[SerialPort] select() failed on " + port_);
      break;
    case 0: // timeout
      if (raise_error_) raise_error_("[SerialPort] recv timeout on " + port_ + " — device not responding");
      break;
    default:
      recv_len = ::read(fd_, data, len);
      break;
    }

    return recv_len;
  }

  void set_timeout(int timeout_ms)
  {
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
  }

private:
  void Init(std::string port, speed_t baudrate)
  {
    int ret;
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
      if (raise_error_) raise_error_("[SerialPort] Failed to open " + port);
      return;
    }

    // Set attributes
    struct termios option;
    memset(&option, 0, sizeof(option));
    ret = tcgetattr(fd_, &option);

    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_iflag = 0;

    cfsetispeed(&option, baudrate);
    cfsetospeed(&option, baudrate);

    option.c_cflag &= ~CSIZE;
    option.c_cflag |= CS8; // 8
    option.c_cflag &= ~PARENB; // no parity
    option.c_iflag &= ~INPCK; // no parity
    option.c_cflag &= ~CSTOPB; // 1 stop bit

    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;
    option.c_lflag |= CBAUDEX;

    ret = tcflush(fd_, TCIFLUSH);
    ret = tcsetattr(fd_, TCSANOW, &option);
  }

  int fd_{-1};
	fd_set rSet_;
  timeval timeout_;
  std::function<void(const std::string&)> raise_error_;
  std::string port_;

  std::queue<uint8_t> recv_queue;
  std::array<uint8_t, 1024> recv_buf;
};

#endif // SERIAL_PORT_H