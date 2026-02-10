#include <rclcpp/rclcpp.hpp>
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/cone.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "pacsim/msg/landmark.hpp"
#include "pacsim/msg/perception_detections.hpp"

class LartToPacSimBridge : public rclcpp::Node {
public:
  LartToPacSimBridge() : rclcpp::Node("lart_to_pacsim_bridge") {
    declare_parameter<double>("powered_ground_value", 1.0);

    pub_steer_  = create_publisher<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 10);
    pub_pg_     = create_publisher<pacsim::msg::StampedScalar>("/pacsim/powerground_setpoint", 10);
    pub_torque_ = create_publisher<pacsim::msg::Wheels>("/pacsim/wheelspeed_setpoints", 10);

    pub_cones_ = create_publisher<lart_msgs::msg::ConeArray>("/mapping/cones", 10);

    sub_cmd_ = create_subscription<lart_msgs::msg::DynamicsCMD>(
      "/pc_origin/dynamics", 10,
      std::bind(&LartToPacSimBridge::onDynamicsCmd, this, std::placeholders::_1));

    sub_landmark_ = create_subscription<pacsim::msg::PerceptionDetections>(
      "/pacsim/detections", 10,
      std::bind(&LartToPacSimBridge::landmarksCallback, this, std::placeholders::_1));

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
    tq.fl = msg->acc_cmd; 
    tq.fr = msg->acc_cmd; 
    tq.rl = msg->acc_cmd; 
    tq.rr = msg->acc_cmd; 
    pub_torque_->publish(tq);
  }

  void landmarksCallback(const pacsim::msg::PerceptionDetections::SharedPtr msg) {
    lart_msgs::msg::ConeArray cone_array;
    for (const auto& det : msg->detections) {
      lart_msgs::msg::Cone cone;
      cone.position.x = det.pose.pose.position.x;
      cone.position.y = det.pose.pose.position.y;
      cone.position.z = det.pose.pose.position.z;
      cone.class_type.data = pacsimConeToLartCone(getConeClass(det.class_probabilities));
      cone_array.cones.push_back(cone);
    }
    pub_cones_->publish(cone_array);
  }

  float rpmToTorque(float rpm) { return rpm / 4.0f; } 

  int getConeClass(std::array<double, 7> class_probs) {
    int max_index = 0;
    double max_prob = class_probs[0];
    for (size_t i = 1; i < class_probs.size(); ++i) {
      if (class_probs[i] > max_prob) {
        max_prob = class_probs[i];
        max_index = i;
      }
    }
    return pacsimConeToLartCone(max_index);
  }

  int pacsimConeToLartCone(int pacsim_class) {
    switch (pacsim_class) {
      case 0: return lart_msgs::msg::Cone::UNKNOWN; // Unknown
      case 1: return lart_msgs::msg::Cone::UNKNOWN; // Invisible
      case 2: return lart_msgs::msg::Cone::BLUE; // Blue
      case 3: return lart_msgs::msg::Cone::YELLOW; // Yellow
      case 4: return lart_msgs::msg::Cone::ORANGE_SMALL; // Orange small
      case 5: return lart_msgs::msg::Cone::ORANGE_BIG; // Orange big
      case 6: return lart_msgs::msg::Cone::UNKNOWN; //timekeeping
      default: return lart_msgs::msg::Cone::UNKNOWN; // Unknown
    }
  }

  rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr sub_cmd_;
  rclcpp::Subscription<pacsim::msg::PerceptionDetections>::SharedPtr sub_landmark_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr pub_steer_, pub_pg_;
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr pub_torque_;
  rclcpp::Publisher<lart_msgs::msg::ConeArray>::SharedPtr pub_cones_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LartToPacSimBridge>());
  rclcpp::shutdown();
  return 0;
}
