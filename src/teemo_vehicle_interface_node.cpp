#include "teemo_vehicle_interface/teemo_vehicle_interface_node.hpp"

using std::placeholders::_1;

TeemoVehicleInterfaceNode::TeemoVehicleInterfaceNode()
: Node("teemo_vehicle_interface")
{
  this->declare_parameter<double>("max_speed", 5.0);
  this->get_parameter("max_speed", max_speed_kmh_);

  RCLCPP_INFO(get_logger(), "Teemo Interface Initialized (Universe). Max Speed: %.2f km/h", max_speed_kmh_);

  // === Publishers ===
  pub_can_frame_ = create_publisher<can_msgs::msg::Frame>("/to_can_bus", 100);
  
  // 核心状态
  pub_velocity_rpt_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 1);
  pub_steering_rpt_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 1);
  pub_gear_rpt_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 1);
  pub_mode_rpt_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 1);

  // [新增] 补充状态 (符合 Universe 标准)
  pub_turn_indicators_rpt_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", 1);
  pub_hazard_lights_rpt_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status", 1);
  pub_battery_rpt_ = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", 1);

  // === 终极修改：新增发布 gyro_odometer 真正需要的 Twist 话题 ===
  pub_twist_with_cov_rpt_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/sensing/vehicle_velocity_converter/twist_with_covariance", 10);

  // === Subscribers ===
  sub_can_frame_ = create_subscription<can_msgs::msg::Frame>(
    "/from_can_bus", rclcpp::SensorDataQoS(), std::bind(&TeemoVehicleInterfaceNode::on_can_frame, this, _1));
  
  sub_control_cmd_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 1, std::bind(&TeemoVehicleInterfaceNode::on_control_cmd, this, _1));
  
  sub_gear_cmd_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&TeemoVehicleInterfaceNode::on_gear_cmd, this, _1));

  sub_turn_indicators_cmd_ = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "/control/command/turn_indicators_cmd", 1, std::bind(&TeemoVehicleInterfaceNode::on_turn_indicators_cmd, this, _1));

  sub_hazard_lights_cmd_ = create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "/control/command/hazard_lights_cmd", 1, std::bind(&TeemoVehicleInterfaceNode::on_hazard_lights_cmd, this, _1));

  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&TeemoVehicleInterfaceNode::on_timer, this));
}

rclcpp::Time last_cmd_time_;

void TeemoVehicleInterfaceNode::on_control_cmd(const autoware_control_msgs::msg::Control::SharedPtr msg) {
  current_control_cmd_ = *msg;
  is_control_cmd_received_ = true;
  last_cmd_time_ = this->now();
}

void TeemoVehicleInterfaceNode::on_gear_cmd(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg) {
  current_gear_cmd_ = *msg;
}

void TeemoVehicleInterfaceNode::on_turn_indicators_cmd(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg) {
  current_turn_indicators_cmd_ = *msg;
}

void TeemoVehicleInterfaceNode::on_hazard_lights_cmd(const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg) {
  current_hazard_lights_cmd_ = *msg;
}

void TeemoVehicleInterfaceNode::on_can_frame(const can_msgs::msg::Frame::SharedPtr msg) {
  // 获取统一的工控机当前时间戳，确保同一帧解析出的数据时间绝对一致
  auto current_time = this->now();

  // 1. 速度与转向 (0x304)
  if (msg->id == ID_VCU_STATUS_2) {
    autoware_vehicle_msgs::msg::VelocityReport vel_rpt;
    autoware_vehicle_msgs::msg::SteeringReport steer_rpt;
    
    vel_rpt.header.frame_id = "base_link";
    vel_rpt.header.stamp = current_time;
    uint16_t raw_speed = (msg->data[1] << 8) | msg->data[0];
    double speed_kmh = (raw_speed * 0.1) - 80.0;
    vel_rpt.longitudinal_velocity = speed_kmh / 3.6; 
    
    steer_rpt.stamp = current_time;
    uint16_t raw_steer = msg->data[4] | ((msg->data[5] & 0x03) << 8); 
    double steer_deg = (raw_steer * 0.1) - 35.0;
    steer_rpt.steering_tire_angle = steer_deg * (M_PI / 180.0); 

    pub_velocity_rpt_->publish(vel_rpt);
    pub_steering_rpt_->publish(steer_rpt);

    // ==========================================================
    // === 终极修改：打包真实的底盘 Twist 数据并注入协方差发给大脑 ===
    // ==========================================================
    geometry_msgs::msg::TwistWithCovarianceStamped twist_cov_msg;
    twist_cov_msg.header.stamp = current_time; 
    twist_cov_msg.header.frame_id = "base_link";

    twist_cov_msg.twist.twist.linear.x = vel_rpt.longitudinal_velocity;
    twist_cov_msg.twist.twist.linear.y = 0.0;
    twist_cov_msg.twist.twist.linear.z = 0.0;
    
    twist_cov_msg.twist.twist.angular.x = 0.0;
    twist_cov_msg.twist.twist.angular.y = 0.0;
    twist_cov_msg.twist.twist.angular.z = 0.0; 

    twist_cov_msg.twist.covariance.fill(0.0);
    twist_cov_msg.twist.covariance[0]  = 0.01; 
    twist_cov_msg.twist.covariance[7]  = 0.01; 
    twist_cov_msg.twist.covariance[14] = 0.01; 
    twist_cov_msg.twist.covariance[21] = 0.01; 
    twist_cov_msg.twist.covariance[28] = 0.01; 
    twist_cov_msg.twist.covariance[35] = 0.01; 

    pub_twist_with_cov_rpt_->publish(twist_cov_msg);
  }

  // 2. 档位与电量 (0x303)
  if (msg->id == ID_VCU_STATUS_1) {
    autoware_vehicle_msgs::msg::GearReport gear_rpt;
    gear_rpt.stamp = current_time; 
    uint8_t raw_gear = msg->data[0] & 0x03;
    switch(raw_gear) {
        case 0: gear_rpt.report = autoware_vehicle_msgs::msg::GearReport::PARK; break;
        case 1: gear_rpt.report = autoware_vehicle_msgs::msg::GearReport::DRIVE; break;
        case 2: gear_rpt.report = autoware_vehicle_msgs::msg::GearReport::NEUTRAL; break;
        case 3: gear_rpt.report = autoware_vehicle_msgs::msg::GearReport::REVERSE; break;
        default: gear_rpt.report = autoware_vehicle_msgs::msg::GearReport::NONE; break;
    }
    pub_gear_rpt_->publish(gear_rpt);
    
    autoware_vehicle_msgs::msg::ControlModeReport mode_rpt;
    mode_rpt.stamp = current_time; 
    mode_rpt.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    pub_mode_rpt_->publish(mode_rpt);

    tier4_vehicle_msgs::msg::BatteryStatus batt_rpt;
    batt_rpt.stamp = current_time;
    batt_rpt.energy_level = static_cast<float>(msg->data[6]); 
    pub_battery_rpt_->publish(batt_rpt);
  }

  // 3. 灯光状态反馈 (0x301 VCU_Vehicle_Diagnosis)
  if (msg->id == ID_VCU_DIAGNOSIS) {
    bool left_on = (msg->data[4] & 0x01) != 0;
    bool right_on = (msg->data[6] & 0x01) != 0;

    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_rpt;
    turn_rpt.stamp = current_time;
    if (left_on && right_on) {
        turn_rpt.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT; 
    } else if (left_on) {
        turn_rpt.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
    } else if (right_on) {
        turn_rpt.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
    } else {
        turn_rpt.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
    }
    pub_turn_indicators_rpt_->publish(turn_rpt);

    autoware_vehicle_msgs::msg::HazardLightsReport hazard_rpt;
    hazard_rpt.stamp = current_time;
    if (left_on && right_on) {
        hazard_rpt.report = autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE;
    } else {
        hazard_rpt.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    }
    pub_hazard_lights_rpt_->publish(hazard_rpt);
  }
}

static uint8_t rolling_counter = 0;

void TeemoVehicleInterfaceNode::on_timer() {
  if (!is_control_cmd_received_) return;

  // 超时保护 (Fail-Safe)
  auto dt = (this->now() - last_cmd_time_).seconds();
  if (dt > 0.200) {
      current_control_cmd_.lateral.steering_tire_angle = 0.0;
      current_control_cmd_.longitudinal.velocity = 0.0;
      current_control_cmd_.longitudinal.acceleration = -2.0; 
  }

  rolling_counter++;
  if (rolling_counter > 15) rolling_counter = 0;

  // 0. AD_Control_Flag (0x501)
  can_msgs::msg::Frame flag_frame;
  flag_frame.id = ID_AD_CTRL_FLAG;
  flag_frame.dlc = 8;
  flag_frame.data.fill(0);
  flag_frame.data[0] = 0x01 | (rolling_counter << 4);
  pub_can_frame_->publish(flag_frame);

  // 1. AD_Control_Steering (0x502)
  can_msgs::msg::Frame steer_frame;
  steer_frame.id = ID_AD_CTRL_STEER;
  steer_frame.dlc = 8;
  steer_frame.data.fill(0);
  steer_frame.data[0] = 0x01 | (rolling_counter << 4);

  double target_steer_deg = current_control_cmd_.lateral.steering_tire_angle * (180.0 / M_PI);
  target_steer_deg = std::max(-30.0, std::min(30.0, target_steer_deg));
  uint16_t raw_steer_cmd = static_cast<uint16_t>((target_steer_deg + 30.0) * 10.0);
  steer_frame.data[4] = raw_steer_cmd & 0xFF;
  steer_frame.data[5] = (raw_steer_cmd >> 8) & 0xFF;
  pub_can_frame_->publish(steer_frame);

  // 2. AD_Control_Accelerate (0x504)
  can_msgs::msg::Frame acc_frame;
  acc_frame.id = ID_AD_CTRL_ACCEL;
  acc_frame.dlc = 8;
  acc_frame.data.fill(0);
  acc_frame.data[0] = 0x01 | (rolling_counter << 4);
  acc_frame.data[2] = 0x01; // Speed Control

  uint8_t target_gear = 2; 
  switch(current_gear_cmd_.command) {
      case autoware_vehicle_msgs::msg::GearCommand::PARK: target_gear = 0; break;
      case autoware_vehicle_msgs::msg::GearCommand::DRIVE: target_gear = 1; break;
      case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL: target_gear = 2; break;
      case autoware_vehicle_msgs::msg::GearCommand::REVERSE: target_gear = 3; break;
  }
  acc_frame.data[3] = target_gear;

  // ==========================================
  // 获取大脑下发的速度和加速度指令
  // ==========================================
  double target_acc = current_control_cmd_.longitudinal.acceleration;
  target_acc = std::max(-5.0, std::min(5.0, target_acc));
  double target_vel = current_control_cmd_.longitudinal.velocity;

  uint8_t raw_acc = static_cast<uint8_t>((target_acc + 5.0) * 10.0);
  acc_frame.data[4] = raw_acc;

  double cmd_speed_kmh = std::abs(target_vel * 3.6);
  if (cmd_speed_kmh > max_speed_kmh_) cmd_speed_kmh = max_speed_kmh_;
  uint16_t raw_speed_cmd = static_cast<uint16_t>(cmd_speed_kmh * 10.0);
  acc_frame.data[6] = raw_speed_cmd & 0xFF;
  acc_frame.data[7] = (raw_speed_cmd >> 8) & 0xFF;
  pub_can_frame_->publish(acc_frame);

  // ==========================================
  // 3. AD_Control_Brake (0x503) 
  // [终极防蠕动版本]
  // ==========================================
  can_msgs::msg::Frame brake_frame;
  brake_frame.id = ID_AD_CTRL_BRAKE;
  brake_frame.dlc = 8;
  brake_frame.data.fill(0);
  brake_frame.data[0] = 0x01 | (rolling_counter << 4);

  uint8_t brake_val = 0;

  // 逻辑 A: 动态刹车 (加入消除刹车间隙的基础压力)
  if (target_acc < -0.01) {
      // 【关键标定】：液压刹车是有死区的！
      // 15.0 是起步压力补偿，保证哪怕是 -0.1 的微小减速，也能直接越过死区贴死刹车盘
      double brake_pct = 15.0 + (std::abs(target_acc) / 5.0) * 85.0;
      brake_val = static_cast<uint8_t>(std::min(100.0, brake_pct));
  }

  // 逻辑 B: 彻底停稳的保底防蠕动
  if (std::abs(target_vel) < 0.05) {
      // 【核心修复】：必须使用 std::max！
      // 绝不覆盖逻辑A里可能出现的大刹车力！
      // 40% (约 3.2MPa) 的高压，足够把车死死钉在原地抵抗平路 D 档怠速！
      brake_val = std::max(brake_val, static_cast<uint8_t>(40)); 
  }

  brake_frame.data[1] = brake_val;
  pub_can_frame_->publish(brake_frame);

  // ==============================================================
  // 4. AD_Control_Body (0x506) - 融入智能刹车灯与 DBC 心跳机制
  // ==============================================================
  can_msgs::msg::Frame body_frame;
  body_frame.id = ID_AD_CTRL_BODY;
  body_frame.dlc = 8;
  body_frame.data.fill(0);
  
  bool left_on = false;
  bool right_on = false;
  bool brake_light_on = false;

  // 逻辑 1：智能刹车灯控制（基于加速度减速需求）
  if (target_acc < -0.5) {
      brake_light_on = true;
  }

  // 车辆处于目标静止状态 (速度指令为 0)
  if (std::abs(target_vel) < 0.01) {
      brake_light_on = true;
  }

  // 逻辑 2：MRM 紧急状态强制双闪 + 刹车灯保底
  if (current_hazard_lights_cmd_.command == autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE) {
      // brake_light_on = true;
      left_on = true;
      right_on = true;
  } else {
      // 逻辑 3: Autoware 指令 (变道/转弯)
      if (current_turn_indicators_cmd_.command == autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT) {
          left_on = true;
      } else if (current_turn_indicators_cmd_.command == autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT) {
          right_on = true;
      } else {
          // 逻辑 4: 物理转角自动转向灯 (转角超过 8 度自动打灯)
          const double TURN_SIGNAL_THRESHOLD_RAD = 8.0 * M_PI / 180.0;
          double current_steer_rad = current_control_cmd_.lateral.steering_tire_angle;
          
          if (current_steer_rad > TURN_SIGNAL_THRESHOLD_RAD) {
              left_on = true;
          } else if (current_steer_rad < -TURN_SIGNAL_THRESHOLD_RAD) {
              right_on = true;
          }
      }
  }

  // Byte 0: 组装转向灯并注入 MsgCntr(高4位)
  uint8_t body_byte0 = 0;
  if (left_on)  body_byte0 |= (1 << 0); // Bit 0
  if (right_on) body_byte0 |= (1 << 1); // Bit 1
  body_byte0 |= (rolling_counter << 4); // MsgCntr (Bit 4-7)
  body_frame.data[0] = body_byte0;

  // Byte 1: 组装刹车灯
  uint8_t body_byte1 = 0;
  if (brake_light_on) {
      body_byte1 |= 0x01; // Bit 0 (全局的 Bit 8)
  }
  body_frame.data[1] = body_byte1;

  pub_can_frame_->publish(body_frame);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeemoVehicleInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}