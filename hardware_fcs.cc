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

// [TODO] to use parameters
// [TODO] change the name 'mcidx'
// [TODO] realtime scheduling settings
// [TODO] initial pose
// [TODO] proper inactive checking (now string comparison)

#include "hardware_fcs/emesg_stderr.h"
#include "hardware_fcs/hardware_fcs.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace HWIF = hardware_interface;

namespace hardware_fcs
{
HWIF::CallbackReturn HardwareFCS::on_init(
    const HWIF::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logname_ = info_.name + " : HardwareFCS";

  RCLCPP_INFO(
      rclcpp::get_logger(logname_),
      "on_init() called.");

  if (info_.hardware_parameters.count("device")!=0) {
    device_name_ = info_.hardware_parameters["device"];
    RCLCPP_INFO(
        rclcpp::get_logger(logname_),
        "on_init() parameters['device']: %s", device_name_.c_str());
  } else {
    device_name_ = "/dev/ttyUSB0";
  }
  
  if (info_.hardware_parameters.count("device")!=0) {
    servo_num_ = std::stoi(info_.hardware_parameters["num"]);
    RCLCPP_INFO(
        rclcpp::get_logger(logname_),
        "on_init() parameters['num']: %d", servo_num_);
  } else {
    servo_num_ = 0;
  }

  fcs_command_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  fcs_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  fcs_state_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  fcs_state_positions_inner_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  fcs_state_currents_inner_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  comm_state_ = kWritten;

  count_ = 0;

  return HWIF::CallbackReturn::SUCCESS;
}

static inline double rad_from_deg(double deg) { return M_PI*deg/180.0; }

HWIF::CallbackReturn HardwareFCS::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
      rclcpp::get_logger(logname_),
      "on_configure() called. ('%s', %d)", device_name_.c_str(), servo_num_);

  MotorChain *mc = new MotorChain(device_name_, servo_num_);
  chains_.emplace_back(mc);
  mc->mcidx_list_.clear();

  std::uint16_t mcidx;

  // info_[0].jname must be "hj0"  [TODO] fixme
  mcidx = 0x0001;  // bufidx: 0,  svidx: 1
  chain_index_list_.emplace_back(std::make_pair(mc, mcidx));
  mc->mcidx_list_.emplace_back(mcidx);

  // info_[1].jname must be "hj1"  [TODO] fixme
  mcidx = 0x0102;  // bufidx: 1, svidx: 2
  chain_index_list_.emplace_back(std::make_pair(mc, mcidx));
  mc->mcidx_list_.emplace_back(mcidx);
  
  // info_[2].jname must be "hj2"  [TODO] fixme
  mcidx = 0x0203;  // bufidx: 2, svidx: 3
  chain_index_list_.emplace_back(std::make_pair(mc, mcidx));
  mc->mcidx_list_.emplace_back(mcidx);
  
  // info_[3].jname must be "hj3"  [TODO] fixme
  mcidx = 0x0304;  // bufidx: 3, svidx: 4
  chain_index_list_.emplace_back(std::make_pair(mc, mcidx));
  mc->mcidx_list_.emplace_back(mcidx);

  HWIF::CallbackReturn retval = HWIF::CallbackReturn::SUCCESS;
  
  if (mc->open_dev()<0) retval = HWIF::CallbackReturn::ERROR;

  //HWIF::CallbackReturn retval = HWIF::CallbackReturn::SUCCESS;
  
  // [TODO] must be read value from device
  for (std::uint32_t i = 0; i < fcs_command_positions_.size(); i++) {
    fcs_command_positions_[i] = 0;
    fcs_state_positions_[i] = 0;
    fcs_state_efforts_[i] = 0;
  }

  // [TODO] initial pose
  fcs_state_positions_[0] = fcs_command_positions_[0] = rad_from_deg(-30);
  fcs_state_positions_[1] = fcs_command_positions_[1] = rad_from_deg( 90);
  fcs_state_positions_[2] = fcs_command_positions_[2] = rad_from_deg( 30);
  fcs_state_positions_[3] = fcs_command_positions_[3] = rad_from_deg(-90);

  {
    std::lock_guard<std::mutex> lg(mtx_);
    for (std::uint32_t i = 0; i < fcs_command_positions_.size(); i++) {
      fcs_state_positions_inner_[i] = 0;
      fcs_state_currents_inner_[i] = 0;
    }
  }

  reader_thr_ = new std::thread(reader_proc, this);

  return retval;
}

std::vector<HWIF::StateInterface>
HardwareFCS::export_state_interfaces()
{
  std::vector<HWIF::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(HWIF::StateInterface(
      info_.joints[i].name, HWIF::HW_IF_POSITION, &fcs_state_positions_[i]));
    state_interfaces.emplace_back(HWIF::StateInterface(
      info_.joints[i].name, HWIF::HW_IF_EFFORT, &fcs_state_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<HWIF::CommandInterface>
HardwareFCS::export_command_interfaces()
{
  std::vector<HWIF::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(HWIF::CommandInterface(
      info_.joints[i].name, HWIF::HW_IF_POSITION, &fcs_command_positions_[i]));
  }

  return command_interfaces;
}

HWIF::CallbackReturn HardwareFCS::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
      rclcpp::get_logger(logname_),
      "on_activate() called.");

  for (MotorChain *mc : chains_) {
    mc->transmit_torque_enable_all(MotorChain::kTorqueEnable);
  }


  return HWIF::CallbackReturn::SUCCESS;
}


HWIF::CallbackReturn HardwareFCS::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
      rclcpp::get_logger(logname_),
      "on_deactivate() called.");

  for (MotorChain *mc : chains_) {
    mc->transmit_torque_enable_all(MotorChain::kTorqueDisable);
  }

  // [TODO] join reader thread

  return HWIF::CallbackReturn::SUCCESS;
}


HWIF::CallbackReturn HardwareFCS::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
      rclcpp::get_logger(logname_),
      "on_cleanup() called.");

  for (MotorChain *mc : chains_) {
    mc->transmit_torque_enable_all(MotorChain::kTorqueDisable);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // [TODO] fixme
  
  while(chains_.size() != 0) {
    MotorChain *mc = chains_.back();
    mc->close_dev();
    chains_.pop_back();
    delete mc;
  }

  chain_index_list_.clear();

  return HWIF::CallbackReturn::SUCCESS;
}


HWIF::return_type HardwareFCS::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (check_count_of_logging()) {
    RCLCPP_INFO(
        rclcpp::get_logger(logname_),
        "read() called.");
  }

  // need lock ?
  if (comm_state_ == kRead) {
    {
      std::lock_guard<std::mutex> lg(mtx_);
      fcs_state_positions_ = fcs_state_positions_inner_;
      fcs_state_efforts_   = fcs_state_currents_inner_;
    }  // lock guard block

    if (check_count_of_logging()) {
      RCLCPP_INFO(
          rclcpp::get_logger(logname_),
          "read() updated.");
    }
  }

  {
    std::lock_guard<std::mutex> lg(mtx_);
    if (comm_state_ == kRead) {
      comm_state_ = kWriting;
    }
  }
  
  return HWIF::return_type::OK;
}

HWIF::return_type HardwareFCS::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (check_count_of_logging()) {
    RCLCPP_INFO(
        rclcpp::get_logger(logname_),
        "write() called.");
  }


  if (comm_state_ == kWriting) {
  
    for(unsigned int idx=0; idx<chain_index_list_.size(); idx++) {
      MotorChain *mc = chain_index_list_[idx].first;
      std::uint16_t mcidx = chain_index_list_[idx].second;
      mc->set_position(mcidx, fcs_command_positions_[idx]);
    }
    for(MotorChain *mc : chains_) {
      mc->transmit_position_pkt();
    }

    bool prev = check_count_of_logging();
    count_++;
    if (check_count_of_logging() || prev) {
      RCLCPP_INFO(
          rclcpp::get_logger(logname_),
          "write() transmitted.");
    }
  }

  if ((comm_state_ == kWriting) && check_count_of_logging()) {
    RCLCPP_INFO(
        rclcpp::get_logger(logname_),
        "write(): notifying.");
  }
  
  bool notified = false;
  {
    //std::lock_gurad<std::mutex> lg(mtx_);
    std::unique_lock<std::mutex> lg(mtx_);
    if (comm_state_ == kWriting) {
      notified = true;
      comm_state_ = kWritten;
      cnd_.notify_one();
    }
  }

  if (notified && check_count_of_logging()) {
    RCLCPP_INFO(
        rclcpp::get_logger(logname_),
        "write() notified.");
  }

  if (lifecycle_state_.label() == "inactive") {
    // [TODO] ./hardware_interface/types/lifecycle_state_names.hpp
    for(unsigned int idx=0; idx<fcs_command_positions_.size(); idx++) {
      fcs_command_positions_[idx] = fcs_state_positions_[idx];
    }
  }

  return HWIF::return_type::OK;
}


static inline std::uint8_t svidx_of(std::uint16_t mcidx) { return mcidx&0xFF; }
static inline std::uint8_t bufidx_of(std::uint16_t mcidx) { return (mcidx>>8)&0xFF; }

void HardwareFCS::reader_proc(HardwareFCS *self) {

  // reader_proc() が launch する時には chains_ と chains_[i]->mcidx_list_ が準備できている必要がある。
  // 各チェインを並列に処理するため、各チェインが現在何番目のサーボにアクセスしているかを indexes[] で管理する。
  // indexes[i] == chains_[i].mcidex_list_.size() の時が終了条件
  // リクエストパケット送信とリターンパケット受信の間は UsbSerial にとってクリティカルセクションのため、lock/unlcok で保護する。
  
  RCLCPP_INFO(
      rclcpp::get_logger(self->logname_),
      "reader_proc() launched.");

  {
    struct sched_param param = { sched_priority: 51 };  // [TODO] fixme, 51 is prior than ros2_control (50).
    int policy = SCHED_FIFO;
    pthread_setschedparam(pthread_self(), policy, &param);
  }

  {
    int policy;
    struct sched_param param;
    if (pthread_getschedparam(pthread_self(), &policy, &param) == 0) {
      RCLCPP_INFO(
          rclcpp::get_logger(self->logname_),
          "reader_proc(): policy %d/%d/%d, prio: %d",
          policy, SCHED_FIFO, SCHED_OTHER,
          param.sched_priority);

    } else {
      RCLCPP_INFO(
          rclcpp::get_logger(self->logname_),
          "reader_proc(): failed to pthread_getschedparam()");
    }
  }
      
  std::vector<unsigned long> indexes;
  indexes.resize(self->chains_.size());

  while(1) {  // main loop
    {  // lock_guard block
      std::unique_lock<std::mutex> lg(self->mtx_);
      while (self->comm_state_ != kWritten) self->cnd_.wait(lg);
      self->comm_state_ = kReading;
    }  // lock_guard block

    if (self->check_count_of_logging()) {
      RCLCPP_INFO(
          rclcpp::get_logger(self->logname_),
          "reader_proc() waken up.");
    }
    
    for(unsigned int idx=0; idx<indexes.size(); idx++) indexes[idx]=0;

    while(true) {

      // 各チェインのリクエストを送る
      for (unsigned long idx=0; idx<self->chains_.size(); idx++) {
        MotorChain *mc=self->chains_[idx];
        if (indexes[idx] != mc->mcidx_list_.size()) {
          std::uint8_t svidx = svidx_of(mc->mcidx_list_[indexes[idx]]);
          mc->lock_dev();
          mc->transmit_request_status_pkt(svidx);
        }
      }

      // 各チェインの応答を受信する
      std::size_t finished=0;
      for (unsigned long idx=0; idx<self->chains_.size(); idx++) {
        MotorChain *mc=self->chains_[idx];
        if (indexes[idx] != mc->mcidx_list_.size()) {
          if (mc->receive_return_pkt()) {
            double position, current;
            mc->get_status_from_retpkt(position, current);

            std::uint8_t bufidx = bufidx_of(mc->mcidx_list_[indexes[idx]]);
            {  // lock guard block
              std::lock_guard<std::mutex> lg(self->mtx_);
              self->fcs_state_positions_inner_[bufidx] = position;
              self->fcs_state_currents_inner_[bufidx] = current;
            }  // lock guard block
          }
          mc->unlock_dev();
          indexes[idx]++;
        }

        if (indexes[idx] == mc->mcidx_list_.size()) {
          finished+=1;
        }
      }  // for (int idx=0; idx<self->chains_.size(); idx++) {

      if (finished == self->chains_.size()) break;
    }

    {  // lock_guard block
      std::unique_lock<std::mutex> lg(self->mtx_);
      self->comm_state_ = kRead;
    }  // lock_guard block

    if (self->check_count_of_logging()) {
      RCLCPP_INFO(
          rclcpp::get_logger(self->logname_),
          "reader_proc() one cycle done.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }  // while(1) main loop

  
  return;
}

}  // namespace hardware_fcs




#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hardware_fcs::HardwareFCS, HWIF::SystemInterface)
