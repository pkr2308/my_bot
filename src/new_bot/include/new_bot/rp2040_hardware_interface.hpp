#ifndef NEW_BOT__RP2040_HARDWARE_INTERFACE_HPP_
#define NEW_BOT__RP2040_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <libserial/SerialPort.h>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace new_bot
{
class RP2040HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RP2040HardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  std::unique_ptr<LibSerial::SerialPort> serial_;
  std::string serial_port_;
  int baud_rate_;
  int timeout_ms_;
  int retry_count_;

  // Joint states
  std::vector<std::string> joint_names_;
  std::vector<double> hw_joint_positions_;
  std::vector<double> hw_joint_velocities_;
  std::vector<double> hw_joint_position_commands_;
  std::vector<double> hw_joint_velocity_commands_;

  // Sensor states - IMU data
  std::vector<double> hw_imu_orientation_;       // quaternion x,y,z,w
  std::vector<double> hw_imu_angular_velocity_;  // rad/s x,y,z
  std::vector<double> hw_imu_linear_accel_;      // m/s² x,y,z
  
  // Sensor states - Range data
  std::vector<double> hw_range_sensors_;         // front, left, right distances

  struct IMUData {
    double orientation_x, orientation_y, orientation_z, orientation_w;
    double angular_velocity_x, angular_velocity_y, angular_velocity_z;
    double linear_acceleration_x, linear_acceleration_y, linear_acceleration_z;
  } imu_data_;

  struct RangeData {
    double front_range;
    double left_range; 
    double right_range;
  } range_data_;

  // Communication protocol
  bool connect_to_rp2040();
  void disconnect_from_rp2040();
  bool send_command(const std::string& command);
  std::string receive_response();
  bool parse_sensor_data(const std::string& data);
  bool sendCommand(const std::string& command);
  std::string receiveResponse();
  bool parseSerialData(const std::string& data);
  void updateSensorData(const std::string& sensor_data);
  
  // Helper functions
  std::vector<std::string> splitString(const std::string& str, char delimiter);
  bool isValidFloat(const std::string& str);
  
  // Protocol constants
  static constexpr char START_MARKER = '<';
  static constexpr char END_MARKER = '>';
  static constexpr char DELIMITER = ',';
};

}  // namespace new_bot

#endif  // NEW_BOT__RP2040_HARDWARE_INTERFACE_HPP_
