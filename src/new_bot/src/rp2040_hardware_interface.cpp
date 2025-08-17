#include "new_bot/rp2040_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>
#include <sstream>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace new_bot
{
hardware_interface::CallbackReturn RP2040HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get hardware parameters
  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
  retry_count_ = std::stoi(info_.hardware_parameters["retry_count"]);

  // Initialize joint vectors
  hw_joint_positions_.resize(info_.joints.size(), 0.0);
  hw_joint_velocities_.resize(info_.joints.size(), 0.0);
  hw_joint_position_commands_.resize(info_.joints.size(), 0.0);
  hw_joint_velocity_commands_.resize(info_.joints.size(), 0.0);

  // Store joint names
  joint_names_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  // Initialize sensor data
  hw_imu_orientation_.resize(4, 0.0);      // quaternion x,y,z,w
  hw_imu_angular_velocity_.resize(3, 0.0); // rad/s x,y,z
  hw_imu_linear_accel_.resize(3, 0.0);     // m/s² x,y,z
  hw_range_sensors_.resize(3, 0.0);        // front, left, right distances

  // Set initial IMU quaternion to identity
  hw_imu_orientation_[3] = 1.0; // w = 1

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RP2040HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RP2040HardwareInterface"), "Configuring ...");

  // Initialize serial connection
  if (!connect_to_rp2040())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RP2040HardwareInterface"), 
                 "Failed to connect to RP2040 on port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RP2040HardwareInterface"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RP2040HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint state interfaces
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocities_[i]));
  }

  // IMU state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.x", &hw_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.y", &hw_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.z", &hw_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.w", &hw_imu_orientation_[3]));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.x", &hw_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.y", &hw_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.z", &hw_imu_angular_velocity_[2]));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.x", &hw_imu_linear_accel_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.y", &hw_imu_linear_accel_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.z", &hw_imu_linear_accel_[2]));

  // Range sensor state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "tf_luna_front", "range", &hw_range_sensors_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "tf_luna_left", "range", &hw_range_sensors_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "tf_luna_right", "range", &hw_range_sensors_[2]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RP2040HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // Check if joint has position command interface
    for (const auto & command_interface : info_.joints[i].command_interfaces)
    {
      if (command_interface.name == hardware_interface::HW_IF_POSITION)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_position_commands_[i]));
      }
      else if (command_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocity_commands_[i]));
      }
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RP2040HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RP2040HardwareInterface"), "Activating ...");

  // Send initialization command to RP2040
  if (!send_command("INIT"))
  {
    RCLCPP_ERROR(rclcpp::get_logger("RP2040HardwareInterface"), "Failed to initialize RP2040");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RP2040HardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RP2040HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RP2040HardwareInterface"), "Deactivating ...");

  // Send stop command to RP2040
  send_command("STOP");
  
  disconnect_from_rp2040();

  RCLCPP_INFO(rclcpp::get_logger("RP2040HardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RP2040HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Request sensor data from RP2040
  RCLCPP_DEBUG(rclcpp::get_logger("RP2040HardwareInterface"), "Sending READ command");
  if (!send_command("READ"))
  {
    RCLCPP_WARN(rclcpp::get_logger("RP2040HardwareInterface"), "Failed to send READ command");
    return hardware_interface::return_type::ERROR;
  }

  // Receive and parse response
  std::string response = receive_response();
  RCLCPP_DEBUG(rclcpp::get_logger("RP2040HardwareInterface"), 
               "Received response: '%s'", response.c_str());
               
  if (!parse_sensor_data(response))
  {
    RCLCPP_WARN(rclcpp::get_logger("RP2040HardwareInterface"), "Failed to parse sensor data");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RP2040HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Prepare command string for RP2040
  std::string command = "CMD";
  
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command += ",";
    
    // Check which command interface is active for this joint
    bool has_position_cmd = false;
    bool has_velocity_cmd = false;
    
    for (const auto & cmd_interface : info_.joints[i].command_interfaces)
    {
      if (cmd_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_cmd = true;
      }
      else if (cmd_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_cmd = true;
      }
    }
    
    if (has_position_cmd)
    {
      command += "P:" + std::to_string(hw_joint_position_commands_[i]);
    }
    else if (has_velocity_cmd)
    {
      command += "V:" + std::to_string(hw_joint_velocity_commands_[i]);
    }
  }

  // Send command to RP2040
  if (!send_command(command))
  {
    RCLCPP_WARN(rclcpp::get_logger("RP2040HardwareInterface"), "Failed to send command to RP2040");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool RP2040HardwareInterface::connect_to_rp2040()
{
  try
  {
    serial_ = std::make_unique<LibSerial::SerialPort>(serial_port_);
    
    if (!serial_->IsOpen())
    {
      serial_->Open(serial_port_);
    }
    
    // Configure serial port
    serial_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);  
    serial_->SetParity(LibSerial::Parity::PARITY_NONE);
    
    // Wait for connection to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Clear any existing data in buffers
    serial_->FlushIOBuffers();
    
    return serial_->IsOpen();
  }
  catch (const LibSerial::OpenFailed& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RP2040HardwareInterface"), 
                 "Serial connection failed: %s", e.what());
    return false;
  }
}

void RP2040HardwareInterface::disconnect_from_rp2040()
{
  if (serial_ && serial_->IsOpen())
  {
    serial_->Close();
  }
}

bool RP2040HardwareInterface::send_command(const std::string& command)
{
  if (!serial_ || !serial_->IsOpen())
  {
    RCLCPP_ERROR(rclcpp::get_logger("RP2040HardwareInterface"), 
                 "Serial port not open");
    return false;
  }

  try
  {
    std::string full_command = START_MARKER + command + END_MARKER + "\n";
    RCLCPP_DEBUG(rclcpp::get_logger("RP2040HardwareInterface"), 
                 "Sending command: '%s'", full_command.c_str());
    serial_->Write(full_command);
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RP2040HardwareInterface"), 
                 "Failed to send command: %s", e.what());
    return false;
  }
}

std::string RP2040HardwareInterface::receive_response()
{
  if (!serial_ || !serial_->IsOpen())
  {
    return "";
  }

  try
  {
    std::string response;
    serial_->ReadLine(response, '\n', timeout_ms_);
    
    // Remove newline characters
    response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());
    response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
    return response;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RP2040HardwareInterface"), 
                 "Failed to receive response: %s", e.what());
    return "";
  }
}

bool RP2040HardwareInterface::parse_sensor_data(const std::string& data)
{
  RCLCPP_DEBUG(rclcpp::get_logger("RP2040HardwareInterface"), 
               "Received data: '%s'", data.c_str());

  if (data.empty() || data.front() != START_MARKER || data.back() != END_MARKER)
  {
    RCLCPP_WARN(rclcpp::get_logger("RP2040HardwareInterface"), 
                "Invalid data format. Data: '%s', Length: %ld", data.c_str(), data.length());
    return false;
  }

  // Remove start and end markers
  std::string payload = data.substr(1, data.length() - 2);
  
  // Split by comma
  std::vector<std::string> tokens;
  std::stringstream ss(payload);
  std::string token;
  
  while (std::getline(ss, token, DELIMITER))
  {
    tokens.push_back(token);
  }

  // Expected format: <JOINT_POS_0,JOINT_VEL_0,JOINT_POS_1,JOINT_VEL_1,...,IMU_QX,IMU_QY,IMU_QZ,IMU_QW,IMU_GX,IMU_GY,IMU_GZ,IMU_AX,IMU_AY,IMU_AZ,RANGE_F,RANGE_L,RANGE_R>
  // Firmware provides 4 joints: steering(0), back_left(1), back_right(2), spare(3)
  
  size_t firmware_joints = 4;  // Fixed number from firmware
  size_t expected_tokens = firmware_joints * 2 + 10 + 3; // 4 joints*2 + IMU(10) + ranges(3)
  RCLCPP_DEBUG(rclcpp::get_logger("RP2040HardwareInterface"), 
               "Parsed %ld tokens, expected %ld. Payload: '%s'", 
               tokens.size(), expected_tokens, payload.c_str());
               
  if (tokens.size() != expected_tokens)
  {
    RCLCPP_WARN(rclcpp::get_logger("RP2040HardwareInterface"), 
                "Invalid sensor data format. Expected %ld tokens, got %ld. Payload: '%s'", 
                expected_tokens, tokens.size(), payload.c_str());
    return false;
  }

  try
  {
    size_t idx = 0;
    
    // Parse firmware joint data and map to URDF joints
    // Firmware joint mapping:
    // 0: steering_joint
    // 1: back_left_wheel_joint
    // 2: back_right_wheel_joint
    // 3: spare/unused
    
    double firmware_joint_pos[4];
    double firmware_joint_vel[4];
    
    for (uint i = 0; i < firmware_joints; i++)
    {
      firmware_joint_pos[i] = std::stod(tokens[idx++]);
      firmware_joint_vel[i] = std::stod(tokens[idx++]);
    }
    
    // Map firmware data to URDF joints by name
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      const std::string& joint_name = joint_names_[i];
      
      if (joint_name == "steering_joint")
      {
        hw_joint_positions_[i] = firmware_joint_pos[0];
        hw_joint_velocities_[i] = firmware_joint_vel[0];
      }
      else if (joint_name == "back_left_wheel_joint")
      {
        hw_joint_positions_[i] = firmware_joint_pos[1];
        hw_joint_velocities_[i] = firmware_joint_vel[1];
      }
      else if (joint_name == "back_right_wheel_joint")
      {
        hw_joint_positions_[i] = firmware_joint_pos[2];
        hw_joint_velocities_[i] = firmware_joint_vel[2];
      }
      else if (joint_name == "front_left_wheel_joint")
      {
        // Front left wheel follows steering angle
        hw_joint_positions_[i] = firmware_joint_pos[0]; // Same as steering
        hw_joint_velocities_[i] = 0.0; // No velocity command for steering wheels
      }
      else if (joint_name == "front_right_wheel_joint")
      {
        // Front right wheel follows steering angle  
        hw_joint_positions_[i] = firmware_joint_pos[0]; // Same as steering
        hw_joint_velocities_[i] = 0.0; // No velocity command for steering wheels
      }
      else
      {
        // Unknown joint - set to zero
        hw_joint_positions_[i] = 0.0;
        hw_joint_velocities_[i] = 0.0;
      }
    }
    
    // Parse IMU data (quaternion)
    hw_imu_orientation_[0] = std::stod(tokens[idx++]); // qx
    hw_imu_orientation_[1] = std::stod(tokens[idx++]); // qy
    hw_imu_orientation_[2] = std::stod(tokens[idx++]); // qz
    hw_imu_orientation_[3] = std::stod(tokens[idx++]); // qw
    
    // Parse IMU angular velocity
    hw_imu_angular_velocity_[0] = std::stod(tokens[idx++]); // gx
    hw_imu_angular_velocity_[1] = std::stod(tokens[idx++]); // gy
    hw_imu_angular_velocity_[2] = std::stod(tokens[idx++]); // gz
    
    // Parse IMU linear acceleration
    hw_imu_linear_accel_[0] = std::stod(tokens[idx++]); // ax
    hw_imu_linear_accel_[1] = std::stod(tokens[idx++]); // ay
    hw_imu_linear_accel_[2] = std::stod(tokens[idx++]); // az
    
    // Parse range sensors
    hw_range_sensors_[0] = std::stod(tokens[idx++]); // front
    hw_range_sensors_[1] = std::stod(tokens[idx++]); // left
    hw_range_sensors_[2] = std::stod(tokens[idx++]); // right
    
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RP2040HardwareInterface"), 
                 "Failed to parse sensor data: %s", e.what());
    return false;
  }
}

}  // namespace new_bot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(new_bot::RP2040HardwareInterface, hardware_interface::SystemInterface)
