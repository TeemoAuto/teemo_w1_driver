#ifndef TEEMO_VEHICLE_INTERFACE_NODE_HPP_
#define TEEMO_VEHICLE_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>

// === 新增：灯光相关消息头文件 ===
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>

// === 终极修改：新增 Twist 融合速度头文件 ===
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <cmath>
#include <memory>

class TeemoVehicleInterfaceNode : public rclcpp::Node
{
public:
  TeemoVehicleInterfaceNode();

private:
  // Publishers
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frame_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_rpt_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_rpt_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr pub_gear_rpt_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_mode_rpt_;

  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr pub_turn_indicators_rpt_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr pub_hazard_lights_rpt_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr pub_battery_rpt_;

  // === 终极修改：新增 Twist 融合速度发布者 ===
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_with_cov_rpt_;

  // Subscribers
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frame_;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr sub_gear_cmd_;

  // === 新增：灯光 Subscribers ===
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr sub_hazard_lights_cmd_;

  // Callbacks
  void on_can_frame(const can_msgs::msg::Frame::SharedPtr msg);
  void on_control_cmd(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void on_gear_cmd(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg);
  
  // === 新增：灯光 Callbacks ===
  void on_turn_indicators_cmd(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg);
  void on_hazard_lights_cmd(const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg);

  void on_timer();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Stored Commands
  autoware_control_msgs::msg::Control current_control_cmd_;
  autoware_vehicle_msgs::msg::GearCommand current_gear_cmd_;
  // === 新增：灯光状态变量 ===
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand current_turn_indicators_cmd_;
  autoware_vehicle_msgs::msg::HazardLightsCommand current_hazard_lights_cmd_;
  
  bool is_control_cmd_received_ = false;

  // Parameters
  double max_speed_kmh_;

  // CAN ID Definitions
  static constexpr uint32_t ID_VCU_DIAGNOSIS = 0x301;
  static constexpr uint32_t ID_VCU_STATUS_1 = 0x303; 
  static constexpr uint32_t ID_VCU_STATUS_2 = 0x304; 
  static constexpr uint32_t ID_AD_CTRL_FLAG = 0x501;
  static constexpr uint32_t ID_AD_CTRL_STEER = 0x502;
  static constexpr uint32_t ID_AD_CTRL_BRAKE = 0x503;
  static constexpr uint32_t ID_AD_CTRL_ACCEL = 0x504;
  // === 新增：车身控制 ID ===
  static constexpr uint32_t ID_AD_CTRL_BODY  = 0x506;
};

#endif  // TEEMO_VEHICLE_INTERFACE_NODE_HPP_