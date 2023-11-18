//
//

#include "hardware_fcs/emesg_stderr.h"  // before usb_serial.h

#include "hardware_fcs/usb_serial.h"
#include "hardware_fcs/futaba_command_servo.h"

#include <string>
#include <thread>
#include <cstdio>

int main(int argc, char *argv[]) {

  int svidx=1;

  if (argc>1) {
    svidx = std::stoi(argv[1]);
  }

  hardware_fcs::PktBufReq9 reqstat;
  reqstat.pkt.id = (svidx & 0xFF);
  reqstat.append_checksum();
  
  std::printf("req: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
              reqstat.bytes[0], reqstat.bytes[1], reqstat.bytes[2], reqstat.bytes[3],
              reqstat.bytes[4], reqstat.bytes[5], reqstat.bytes[6], reqstat.bytes[7]);
  
  hardware_fcs::UsbSerial serdev("/dev/ttyUSB0");
  serdev.open();
  //std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  serdev.write(reqstat.bytes, reqstat.size());
  std::printf("-- written %d bytes.\n", reqstat.size());

  //std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for(int num=0; num<26; num++) {
    std::uint8_t buf[1];
    serdev.read(buf, 1);
    std::printf(" 0x%02X", buf[0]);
    std::fflush(stdout);
  }
  
  std::printf("\n");

  serdev.close();

  return 0;
}
