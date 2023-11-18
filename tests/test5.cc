
#include "hardware_fcs/emesg_stderr.h"  // before usb_serial.h

#include<thread>

int main(void)
{

  EMesg::EMesg(EMesg::ERR, 64, "this is test");
  EMesg::EMesg(EMesg::WRN, 128, "int %d, double %f", 10, 0.2);

  const char *daa = "Daa";

  EMesg::EMesg(EMesg::INF, 128, "str: '%s'", daa);

  return 0;

}
