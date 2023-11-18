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

  hardware_fcs::UsbSerial serdev("/dev/ttyUSB0");
  serdev.open();

  std::int64_t count=0;
  while(1) {
    for(std::uint8_t svidx=1; svidx<5; svidx++) {
  
      reqstat.pkt.id = (svidx & 0xFF);
      reqstat.append_checksum();
      
      serdev.write(reqstat.bytes, reqstat.size());
      
      hardware_fcs::PktBufRet retpkt;
      if(retpkt.read_pkt(serdev)) {
        if(count%100==0) {
          std::printf(".");
          std::fflush(stdout);
        }
        count++;
      } else {
        std::printf("*");
        std::fflush(stdout);
      }
    }
  }    

  serdev.close();

  return 0;
}
