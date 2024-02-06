#include "diffdrive_arduino/diffdrive_arduino.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"


#include <pluginlib/class_list_macros.hpp>


DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}





return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  


  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());
  joint_name_={"arm_base_forearm_joint","forearm_hand_1_joint","forearm_hand_2_joint","forearm_hand_3_joint","forearm_claw_joint","joint_4"};
  position_commands_ = { 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, };
  prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, };
  position_states_ = { 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, };
  RCLCPP_INFO(logger_, "Finished Configuration");




  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel
  RCLCPP_INFO(logger_, "START STATE");
  
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("arm_base_forearm_joint", hardware_interface::HW_IF_POSITION, &position_states_[0]));
  // for (size_t i = 2; i < info_.joints.size(); i++)
  // {RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "get joint: " << info_.joints[i].name);
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //       info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        
  // }
  for (size_t i = 0; i < joint_name_.size(); i++)
  {
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "get joint: " << joint_name_[i]);
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]));
        
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));
  // command_interfaces.emplace_back(hardware_interface::CommandInterface("arm_base_forearm_joint", hardware_interface::HW_IF_POSITION, &position_commands_[0]));
  // for (size_t i = 2; i < info_.joints.size(); i++)
  // {
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  //   RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "get joint: " << info_.joints[i].name);
  //   RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "joint_size: " << info_.joints.size());
  // }



  for (size_t i = 0; i < joint_name_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "get joint: " << joint_name_[i]);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "joint_size: " << joint_name_.size());
  }
  return command_interfaces;
}


return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  // position_commands_ = { 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, };
  // prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, };
  // position_states_ = { 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, };

  arduino_.setPidValues(30, 20, 0, 100);
  arduino_.setServoPosition(0,90);
  arduino_.setServoPosition(1,20);
  arduino_.setServoPosition(2,20);
  arduino_.setServoPosition(3,20);
  arduino_.setServoPosition(4,90);
  arduino_.setServoPosition(5,90);

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  position_states_ = position_commands_;

  return return_type::OK;

  
}

hardware_interface::return_type DiffDriveArduino::write()
{
std::string msg;
  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }
  if (position_commands_ == prev_position_commands_)
  {
    arduino_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

    // Nothing changed, do not send any command
    return return_type::OK;
  }
  

  try
  {
    
    // arduino_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);
    for (size_t i = 0; i < position_commands_.size(); i++){
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "get joint: " << joint_name_[i]);
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "i=: " << i);
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Sending new command " << position_commands_[i]);
      double pos  = static_cast<double>(((position_commands_.at(i) + (M_PI / 2)) * 180) / M_PI);
      arduino_.setServoPosition(i,pos);
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Sending new command " << pos);
    }
    
  }
  catch (...)
  {

    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;



  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)