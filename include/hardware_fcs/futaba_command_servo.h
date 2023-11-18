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

#ifndef HARDWARE_FCS__FUTABA_COMMAND_SERVER_H_
#define HARDWARE_FCS__FUTABA_COMMAND_SERVER_H_

#include "hardware_fcs/usb_serial.h"

//#include <map>
//#include <string>
#include <vector>
#include <cstdint>
#include <cmath>

#include <unistd.h>  // write()

namespace hardware_fcs
{

const std::uint8_t kGoalPositionIndex = 0x1e;
const std::uint8_t kGoalPositionSize = 2;
const std::uint8_t kTorqueEnableIndex = 0x24;
const std::uint8_t kTorqueEnableSize = 1;
const std::uint8_t kPresentPositionIndex = 0x2a;
const std::uint8_t kPresentPositionToPresentCurrentSize = 8;

const int NUMMAX=64;  // [TODO] fixme
const int BUFSIZE=NUMMAX*4+7;  // [TODO] fixme

union PktBufLong {
  std::uint8_t bytes[BUFSIZE];
  struct  __attribute__ ((__packed__)) {
    std::uint8_t hdr[2];
    std::uint8_t id;
    std::uint8_t flag;
    std::uint8_t address;
    std::uint8_t length;
    std::uint8_t count;
    union {
      //std::uint8_t databuf[BUFSIZE-7];
      struct __attribute__ ((__packed__)) {
        std::uint8_t svidx;
        std::int16_t val;
      } data16[NUMMAX];
      struct __attribute__ ((__packed__)) {
        std::uint8_t svidx;
        std::uint8_t val;
      } data8[NUMMAX];
      struct __attribute__ ((__packed__)) {
        std::uint8_t svidx;
        std::uint8_t val1;
        std::uint8_t val2;
      } data8x2[NUMMAX];
    } payload;
  } pkt;

  PktBufLong(std::uint8_t addr, std::uint8_t size, std::uint8_t cnt) {
    pkt.hdr[0] = 0xfa;
    pkt.hdr[1] = 0xaf;
    pkt.id     = 0x00;  // id: 0 for long packet
    pkt.flag   = 0x00;
    pkt.address= addr;
    pkt.length = size+1;  // size(data) + size(id)
    pkt.count  = cnt;
  }

  int size(void) const {
    return pkt.length * pkt.count + 8;
  }

  void append_checksum(void) {
    std::uint8_t cs = bytes[2];
    int tail = size()-1;
    int idx;
    for(idx=3; idx<tail; idx++) {
      cs ^= bytes[idx];
    }
    bytes[idx] = cs;
  }
};  // union PktBufLong


union PktBufAll {
  std::uint8_t bytes[BUFSIZE];
  struct  __attribute__ ((__packed__)) {
    std::uint8_t hdr[2];
    std::uint8_t id;
    std::uint8_t flag;
    std::uint8_t address;
    std::uint8_t length;
    std::uint8_t count;
    std::uint8_t datum;
  } pkt;

  PktBufAll(std::uint8_t addr) {
    pkt.hdr[0] = 0xfa;
    pkt.hdr[1] = 0xaf;
    pkt.id     = 0xFF;  // id: 0xFF for all
    pkt.flag   = 0x00;
    pkt.address= addr;
    pkt.length = 1;
    pkt.count  = 1;
  }

  int size(void) const { return 9; }

  void append_checksum(void) {
    std::uint8_t cs = bytes[2];
    int tail = 8;  // size()-1;
    int idx;
    for(idx=3; idx<tail; idx++) {
      cs ^= bytes[idx];
    }
    bytes[idx] = cs;
  }

};  // union PktBufAll


union PktBufReq {
  std::uint8_t bytes[BUFSIZE];
  struct  __attribute__ ((__packed__)) {
    std::uint8_t hdr[2];
    std::uint8_t id;
    std::uint8_t flag;
    std::uint8_t address;
    std::uint8_t length;
    std::uint8_t count;
  } pkt;

  PktBufReq(std::uint8_t addr, std::uint8_t length) {
    pkt.hdr[0] = 0xfa;
    pkt.hdr[1] = 0xaf;
    pkt.id     = 0x00;
    pkt.flag   = 0x0F;
    pkt.address= addr;
    pkt.length = length;
    pkt.count  = 0;
  }

  int size(void) const { return 8; }

  void append_checksum(void) {
    std::uint8_t cs = bytes[2];
    int tail = 7;  // size()-1;
    int idx;
    for(idx=3; idx<tail; idx++) {
      cs ^= bytes[idx];
    }
    bytes[idx] = cs;
  }

};  // union PktBufReq


union PktBufRet {
  std::uint8_t bytes[BUFSIZE];
  struct  __attribute__ ((__packed__)) {
    std::uint8_t hdr[2];
    std::uint8_t id;
    std::uint8_t flag;
    std::uint8_t address;
    std::uint8_t length;
    std::uint8_t count;
    union {
      std::int16_t i16[NUMMAX];
      std::int8_t  i8[NUMMAX];
      std::uint8_t u8[NUMMAX];
    } payload;
  } pkt;

  int size(void) const {
    return pkt.length * pkt.count + 8;
  }

  bool is_checksum_ok(void) {
    std::uint8_t cs = bytes[2];
    int tail = size()-1;
    int idx;
    for(idx=3; idx<tail; idx++) {
      cs ^= bytes[idx];
    }
    return (bytes[idx] == cs ? true : false);
  }

  bool read_pkt(hardware_fcs::UsbSerial &serdev) {
    while(1) {
      std::uint8_t buf[1];
      int idx=0;

      if (serdev.read(buf, 1) < 1) break;  // timeout
      if (buf[0]!=0xfd) continue;
      bytes[idx++] = buf[0];

      if (serdev.read(buf, 1) < 1) break;  // timeout
      if (buf[0]!=0xdf) continue;
      bytes[idx++] = buf[0];

      while(idx<7) {  // id, flags, address, length, count
        if (serdev.read(buf, 1) < 1) break;
        bytes[idx++] = buf[0];
      }
      if (idx<7) break;  // timeout

      int num = pkt.length * pkt.count +1;
      for(int i=0; i<num; i++) {
        if (serdev.read(buf, 1) < 1) break;  // timeout
        bytes[idx++] = buf[0];
      }
      
      break;
    }

    return is_checksum_ok();
  }



};  // union PktBufRet


class MotorChain
{
 private:
  UsbSerial serdev_;
  PktBufLong position_pkt_;
  PktBufAll torque_enable_all_pkt_;
  PktBufReq request_status_pkt_;
  PktBufRet return_status_pkt_;
  int jnum_;
  
 public:
  //std::map<std::string, std::uint16_t> mcidx_map_;
  std::vector<std::uint16_t> mcidx_list_;
  
 public:
  MotorChain(const std::string &devname, std::uint8_t jnum, int baudrate=115200)
      : serdev_(devname, baudrate), position_pkt_(kGoalPositionIndex, kGoalPositionSize, jnum),
        torque_enable_all_pkt_(kTorqueEnableIndex),
        request_status_pkt_(kPresentPositionIndex, kPresentPositionToPresentCurrentSize),
        jnum_(jnum)
  {
    //serdev_.open();
  }

  virtual ~MotorChain() {
    close_dev();
  }

  inline int open_dev(void) {
    return serdev_.open();
  }

  inline void close_dev(void) {
    serdev_.close();
  }

  inline void lock_dev(void) {
    serdev_.lock();
  }

  inline void unlock_dev(void) {
    serdev_.unlock();
  }
  
  inline void transmit_position_pkt() {
    position_pkt_.append_checksum();
    serdev_.write(position_pkt_.bytes, position_pkt_.size());
  }

  void set_position(std::uint16_t mcidx, double rad) {
    int buf_idx = (mcidx & 0xFF00) >> 8;
    std::int16_t value = static_cast<int16_t>(1800.0*rad/M_PI);
    position_pkt_.pkt.payload.data16[buf_idx].svidx = (mcidx & 0xFF);
    position_pkt_.pkt.payload.data16[buf_idx].val   = value;
  }

  static const std::uint8_t kTorqueDisable=0x00;
  static const std::uint8_t kTorqueEnable=0x01;

  void transmit_torque_enable_all(const std::uint8_t val) {
    torque_enable_all_pkt_.pkt.datum = val;  // short packet, ID:255
    torque_enable_all_pkt_.append_checksum();
    serdev_.write(torque_enable_all_pkt_.bytes, torque_enable_all_pkt_.size());
  }

  void transmit_request_status_pkt(const std::uint8_t svidx) {
    request_status_pkt_.pkt.id = svidx;
    request_status_pkt_.append_checksum();
    serdev_.write(request_status_pkt_.bytes, request_status_pkt_.size());
  }

  bool receive_return_pkt(void) {
    return return_status_pkt_.read_pkt(serdev_);
  }
  
  void get_status_from_retpkt(double &rad, double &current) {
    rad = return_status_pkt_.pkt.payload.i16[0] * M_PI/1800;
    current = return_status_pkt_.pkt.payload.i16[3] / 1000;  // mA
  }



};  // class MotorChain

}  // namespace hardware_fcs

#endif  // HARDWARE_FCS__FUTABA_COMMAND_SERVER_H_
