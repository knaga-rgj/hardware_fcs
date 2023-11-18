//
//

#include "hardware_fcs/emesg_stderr.h"  // before usb_serial.h

#define HARDWARE_FCS__SKIP_SERIAL_SETTINGS  // for named pipe ('mkfifo /tmp/testpipe')
#include "hardware_fcs/usb_serial.h"

#include <thread>

int main(int argc, char *argv[])
{
  //hardware_fcs::UsbSerial ser("/dev/ttyUSB0");
  hardware_fcs::UsbSerial ser("/tmp/testpipe");

  if (ser.open()==0) {
    if (argc < 2) {
      while(1) {
        ser.write("U", 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    } else {
      unsigned char buf[2];
      while(1) {
        ser.read(buf, 1);
        EMesg::EMesg(EMesg::INF, 128, "got 0x%02X", (int)buf[0]);
      }
    }
  }
  
  return 0;
}
