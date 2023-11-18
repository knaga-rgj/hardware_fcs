//
//

#include "hardware_fcs/emesg_stderr.h"  // before usb_serial.h

#include "hardware_fcs/usb_serial.h"
#include "hardware_fcs/futaba_command_servo.h"

#include <string>
#include <thread>
#include <cstdio>

bool read_pkt(hardware_fcs::UsbSerial &serdev, hardware_fcs::PktBufRet &retpkt) {
  
  while(1) {
    std::uint8_t buf[1];
    int idx=0;

    serdev.read(buf, 1);
    if (buf[0]!=0xfd) continue;
    retpkt.bytes[idx++] = buf[0];
    std::printf(" 0x%02X", buf[0]);
    std::fflush(stdout);

    serdev.read(buf, 1);
    if (buf[0]!=0xdf) continue;
    retpkt.bytes[idx++] = buf[0];
    std::printf(" 0x%02X", buf[0]);
    std::fflush(stdout);

    while(idx<7) {  // id, flags, address, length, count
      serdev.read(buf, 1);
      retpkt.bytes[idx++] = buf[0];
      std::printf(" 0x%02X", buf[0]);
      std::fflush(stdout);
    }

    int num = retpkt.pkt.length * retpkt.pkt.count +1;
    for(int i=0; i<num; i++) {
      serdev.read(buf, 1);
      retpkt.bytes[idx++] = buf[0];
      std::printf(" 0x%02X", buf[0]);
      std::fflush(stdout);
    }

    break;
  }

  std::printf("\n");
  return retpkt.is_checksum_ok();
}


int main(void) {

  hardware_fcs::PktBufReq reqstat(hardware_fcs::kPresentPositionIndex,
                                  hardware_fcs::kPresentPositionToPresentCurrentSize);
                                  //2);

  hardware_fcs::UsbSerial serdev("/dev/ttyUSB0");
  serdev.open();

  for(std::uint8_t svidx=1; svidx<5; svidx++) {
  
    reqstat.pkt.id = (svidx & 0xFF);
    reqstat.append_checksum();
  
    std::printf("req: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
                reqstat.bytes[0], reqstat.bytes[1], reqstat.bytes[2], reqstat.bytes[3],
                reqstat.bytes[4], reqstat.bytes[5], reqstat.bytes[6], reqstat.bytes[7]);
  
    serdev.write(reqstat.bytes, reqstat.size());
    std::printf("-- written %d bytes.\n", reqstat.size());

    hardware_fcs::PktBufRet retpkt;
    if(read_pkt(serdev, retpkt)) {
      std::printf("-- succeeded\n");
    } else {
      std::printf("-- failed to read\n");
    }
  }


  serdev.close();

  return 0;
}
