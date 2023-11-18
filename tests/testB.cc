//
//

#include "hardware_fcs/emesg_stderr.h"  // before usb_serial.h

#include "hardware_fcs/usb_serial.h"
#include "hardware_fcs/futaba_command_servo.h"

#include <string>
#include <thread>
#include <cstdio>

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
    if(retpkt.read_pkt(serdev)) {
      std::printf("got: ");
      for(int idx=0; idx<retpkt.size(); idx++) {
        std::printf(" 0x%02X", retpkt.bytes[idx]);
      }
      std::printf("\n");
      std::printf("-- succeeded\n");
    } else {
      std::printf("-- failed to read\n");
    }
  }


  serdev.close();

  return 0;
}
