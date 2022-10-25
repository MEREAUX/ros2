#ifndef NABOT_BASE__NABOT_HARDWARE_HPP_
#define NABOT_BASE__NABOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "roboclaw/roboclaw.hpp"


using namespace std::chrono_literals;

struct MOTORS {   // Declare PERSON struct type
    float proportional_;   // PID coeff
    float integral_;       // PID coeff
    float derivative_;     // PID coeff
    std::pair<int,int> max_speed_;
} motor;  


namespace nabot_base
{

class NabotSystemHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NabotSystemHardware)

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  void resetTravelOffset();
  void writeCommandsToHardware();
  void limitDifferentialSpeed(doublehw_states_position_ &diff_speed_left, double &diff_speed_right);
  void updateJointsFromHardware();

  // PARAMETRES
  //// CREER ICI L'ENSEMBLE DES PARAMS NECESSAIRES

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> , hw_states_position_offset_, hw_states_velocity_;

  uint8_t left_cmd_joint_index_, right_cmd_joint_index_;

};

}  // namespace nabot_base

#endif  // NABOT_BASE__NABOT_HARDWARE_HPP_
