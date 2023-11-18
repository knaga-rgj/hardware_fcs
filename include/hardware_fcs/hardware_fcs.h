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

// Linux/c++11 is supported
// [TODO] mac or else ?

#ifndef HARDWARE_FCS__HARDWARE_FCS_H_
#define HARDWARE_FCS__HARDWARE_FCS_H_


#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <map>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
//#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
//#include "rclcpp_lifecycle/state.hpp"
//#include "ros2_control_demo_example_1/visibility_control.h"

#include "hardware_fcs/futaba_command_servo.h"

#if defined _WIN32 || defined __CYGWIN__
#error "**** Windows not supported ****"
#else
#define HARDWARE_FCS_VISIBLE __attribute__((visibility("default")))
#define HARDWARE_FCS_HIDDEN  __attribute__((visibility("hidden")))
#endif


namespace hardware_fcs
{
class HardwareFCS : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HardwareFCS);

  HARDWARE_FCS_VISIBLE
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  HARDWARE_FCS_VISIBLE
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_FCS_VISIBLE
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_FCS_VISIBLE
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_FCS_VISIBLE
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_FCS_VISIBLE
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_FCS_VISIBLE
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_FCS_VISIBLE
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  HARDWARE_FCS_VISIBLE
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  //
  std::string logname_;  // logging name
  //
  std::string device_name_;
  std::int32_t servo_num_;
  //
  std::vector<MotorChain *> chains_;
  typedef std::pair<MotorChain *, std::uint16_t> ChainAndIndex;
  std::vector<ChainAndIndex> chain_index_list_;
  //typedef std::map<std::string, std::pair<ChainAndIndex > JNameMap;
  //JNameMap jname2index_;

  // Store the command for the simulated robot
  std::vector<double> fcs_command_positions_;
  std::vector<double> fcs_state_positions_;
  std::vector<double> fcs_state_efforts_;

  std::vector<double> fcs_state_positions_inner_;  // lock by mtx_
  std::vector<double> fcs_state_currents_inner_;   // lock by mtx_
  
  std::mutex mtx_;
  std::condition_variable cnd_;
  enum { kReading, kRead, kWriting, kWritten } comm_state_;  // lock by mtx_
  std::thread *reader_thr_;

  static void reader_proc(HardwareFCS *self);

  std::uint64_t count_;  // to reduce logging message
  inline bool check_count_of_logging(void) { return (count_%100==0); }
};

}  // namespace ros2_control_demo_example_1

#endif  // HARDWARE_FCS__HARDWARE_FCS_H_
