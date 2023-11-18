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

// [TODO] many things
// [TODO] depends on linux (/sys/bus/usb-serial)

// Linux/c++11 is supported

#ifndef EMESG_STDERR_H_
#define EMESG_STDERR_H_

#include <cstdarg>
#include <ctime>

#include <mutex>
#include <iostream>

namespace EMesg {

  const char *ERR = "**";
  const char *WRN = "++";
  const char *INF = "--";

  std::mutex mtx;

  void timestamp(char *buf, int size)
  {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    gmtime_r(&(ts.tv_sec), &tm);
    char buf1[64];
    std::strftime(buf1, 63, "%Y%m%dT%H%M%S.%%06dZ 0x%%08X", &tm);
    std::snprintf(buf, size, buf1, ts.tv_nsec/1000, pthread_self());
  }
  
  void EMesg(const char *type, const char *mesg)
  {
    char tsbuf[64];
    timestamp(tsbuf, 64);

    {
      std::lock_guard<std::mutex> lg(mtx);
      std::cerr << type << " [" << tsbuf << "]: " << mesg << std::endl;
    }
    
  }

  void EMesg(const char *type, int size, const char *mesg, ...)
  {
    std::va_list ap;
    va_start(ap, mesg);
    char buf[size];
    std::vsnprintf(buf, size, mesg, ap);
    EMesg(type, buf);
  }

}  // namespace emesg

#endif  // EMESG_STDERR_H_
