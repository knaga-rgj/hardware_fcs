// Copyright 2023 K.Naga <knaga.rgj@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// MT-safe reder()/writer()

// [TODO] many things
// [TODO] depends on linux (/sys/bus/usb-serial)

// Linux/c++11 is supported

#ifndef HARDWARE_FCS__USB_SERIAL_H_
#define HARDWARE_FCS__USB_SERIAL_H_

#include <string>
#include <mutex>

#include <cstring>  // strerror()

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


namespace hardware_fcs
{

class UsbSerial
{
 private:
  std::string devpath_;
  int fd_;
  int baudrate_;
  std::recursive_mutex rmtx_;

 public:
  UsbSerial(const std::string &devname, int baudrate=115200) : devpath_(devname), fd_(-1), baudrate_(baudrate) {}
  virtual ~UsbSerial() { close(); }

  inline void lock(void)   { rmtx_.lock();   }
  inline void unlock(void) { rmtx_.unlock(); }
  
  int open(void) {
    int fd = ::open(devpath_.c_str(), O_RDWR);

    if (fd < 0) {
      EMesg::EMesg(EMesg::ERR, 256, "failed to open(%s): %s", devpath_.c_str(), std::strerror(errno));
      return -1;
    }

#ifndef HARDWARE_FCS__SKIP_SERIAL_SETTINGS
    struct termios tconf;
    tcgetattr(fd_, &tconf);
    cfmakeraw(&tconf);
    cfsetspeed(&tconf, baudrate_);
    tconf.c_cflag &= ~PARODD;
    tconf.c_cflag &= ~(CSTOPB | PARENB);  // stop 1, non-parity
    tconf.c_cc[VMIN]  = 0;
    tconf.c_cc[VTIME] = 2;  // timeout 200 ms
    int ret = tcsetattr(fd, TCSANOW, &tconf);
    if (ret < 0) {
      EMesg::EMesg(EMesg::ERR, 256, "failed to tcsetattr(%s): %s", devpath_.c_str(), std::strerror(errno));
      ::close(fd);
      return -1;
    }
#endif
    
    fd_ = fd;
    return 0;
  }
      
  void close(void) { if (fd_!=-1) { ::close(fd_);  fd_ = -1; } }
  ssize_t write(const void *buf, const size_t count) { std::lock_guard<std::recursive_mutex> lg(rmtx_); return ::write(fd_, buf, count); }
  //ssize_t  read(void *buf, const size_t count)       { std::lock_guard<std::recursive_mutex> lg(rmtx_); return  ::read(fd_, buf, count); }
  ssize_t  read(void *buf, const size_t count) {
    std::lock_guard<std::recursive_mutex> lg(rmtx_);
    int ret = ::read(fd_, buf, count);
    if (ret<0) {
      EMesg::EMesg(EMesg::ERR, 256, "failed to read(%s): %s", devpath_.c_str(), std::strerror(errno));
    }
    return ret;
  }
  
};  // class UsbSerial

}  // namespace hardware_fcs

#endif  // HARDWARE_FCS__USB_SERIAL_H_
