#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase_node") {

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create turtle2 velocity publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);

    // Call on_timer function every second
    timer_ =
        this->create_wall_timer(1s, std::bind(&RobotChase::on_timer, this));
  }

private:
  void on_timer() {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = "morty/base_link"; // target frame
    std::string toFrameRel = "rick/base_link";

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2
    // frames and send velocity commands for turtle2 to reach target_frame
    try {
      t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    // Relative position
    double dx = t.transform.translation.x;
    double dy = t.transform.translation.y;

    // Errors
    double error_distance = std::sqrt(dx * dx + dy * dy);
    double error_yaw = std::atan2(dy, dx);

    // Set the gains
    kp_distance = 0.8;
    kp_yaw = 2.0;

    double min_distance = 0.35; // start slowing down when closer than this

    // Compute velocity
    linear_velocity = kp_distance * error_distance;
    angular_velocity = kp_yaw * error_yaw;

    // Slow down if too close
    if (error_distance < min_distance) {
      linear_velocity = 0.0;
    }

    // Publish Twist
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = linear_velocity;
    cmd_msg.angular.z = angular_velocity;

    publisher_->publish(cmd_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Gains of Rick'sdistance and yaw controllers
  double kp_distance;
  double kp_yaw;

  // Rick's velocities
  double linear_velocity;
  double angular_velocity;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();

  return 0;
}