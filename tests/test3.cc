
#define HARDWARE_FCS__SKIP_SERIAL_SETTINGS  // for named pipe ('mkfifo /tmp/testpipe')

#include "hardware_fcs/emesg_stderr.h"  // befor futaba_command_servo.h
#include "hardware_fcs/futaba_command_servo.h"

int main(void)
{
  std::uint8_t jnum=4;

  hardware_fcs::MotorChain mc0("/tmp/testpipe", jnum);

  mc0.open_dev();
  
  mc0.set_position( 0x0001,  M_PI/6);
  mc0.set_position( 0x0103, -M_PI/4);
  mc0.set_position( 0x0205,  M_PI/3);
  mc0.set_position( 0x0306, -M_PI/2);

  mc0.transmit_position_pkt();
  return 0;
}
