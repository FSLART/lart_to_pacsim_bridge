#include "rclcpp/rclcpp.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"

class LartToPacSimBridge : public rclcpp::Node {
public:
  LartToPacSimBridge() : rclcpp::Node("lart_to_pacsim_bridge") {
    declare_parameter<double>("powered_ground_value", 1.0);

    pub_steer_  = create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10);
    pub_pg_     = create_publisher<pacsim::msg::StampedScalar>("/pacsim/powerground_setpoint", 10);
    pub_torque_ = create_publisher<pacsim::msg::Wheels>("/pacsim/wheelspeed_setpoints", 10);

    sub_cmd_ = create_subscription<lart_msgs::msg::DynamicsCMD>(
      "/pc_origin/dynamics", 10,
      std::bind(&LartToPacSimBridge::onDynamicsCmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LART to PacSim bridge started");
  }

private:
  void onDynamicsCmd(const lart_msgs::msg::DynamicsCMD::SharedPtr msg) {
    // Steering (rad)
    pacsim::msg::StampedScalar steer;
    steer.stamp = now();
    steer.value = msg->steering_angle;
    pub_steer_->publish(steer);

    // Powered ground (0..1)
    pacsim::msg::StampedScalar pg;
    pg.stamp = now();
    pg.value = static_cast<float>(get_parameter("powered_ground_value").as_double());
    pub_pg_->publish(pg);

    // Torque command (Nm) — placeholder mapping from RPM
    pacsim::msg::Wheels tq;
    const float t = rpmToTorque(static_cast<float>(msg->rpm));
    tq.fl = t; 
    tq.fr = t; 
    tq.rl = t; 
    tq.rr = t;
    pub_torque_->publish(tq);
  }

  float rpmToTorque(float rpm) { return rpm / 4.0f; } 

  rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr sub_cmd_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr pub_steer_, pub_pg_;
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr pub_torque_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LartToPacSimBridge>());
  rclcpp::shutdown();
  return 0;
}
