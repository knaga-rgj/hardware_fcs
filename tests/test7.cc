//
//

#include "hardware_fcs/emesg_stderr.h"  // before usb_serial.h

//#define USE_TESTPIPE

#ifdef USE_TESTPIPE
#define HARDWARE_FCS__SKIP_SERIAL_SETTINGS  // for named pipe ('mkfifo /tmp/testpipe')
#endif

#include "hardware_fcs/usb_serial.h"
#include "hardware_fcs/futaba_command_servo.h"

int main(int argc, char *argv[])
{
#ifdef USE_TESTPIPE
  hardware_fcs::MotorChain mc("/tmp/testpipe", 4);
#else
  hardware_fcs::MotorChain mc("/dev/ttyUSB0", 4);
#endif
  
  if (mc.open_dev()==0) {

#if 0
    mc.mcidx_list_.clear();
    mc.mcidx_list_.emplace_back(0x0001);
    mc.mcidx_list_.emplace_back(0x0102);
    mc.mcidx_list_.emplace_back(0x0203);
    mc.mcidx_list_.emplace_back(0x0304);
#endif
    
    if (argc<2) {
      mc.transmit_torque_enable_all(hardware_fcs::MotorChain::kTorqueDisable);

    } else {
      mc.transmit_torque_enable_all(hardware_fcs::MotorChain::kTorqueEnable);
    }

  }
  
  return 0;
}
