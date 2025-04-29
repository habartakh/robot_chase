#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
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

    // Create rick's velocity publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);

    // Call on_timer function every second
    timer_ =
        this->create_wall_timer(500ms, std::bind(&RobotChase::on_timer, this));

    // Set the gains
    kp_distance = 1.0;
    kp_yaw = 1.0;

    // indicates if morty's initial position is saved or not yet
    initialized = false;

    RCLCPP_INFO(this->get_logger(), "Robot chase Node Ready !");
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
    }

    catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    // Relative position of rick and morty
    double dx = t.transform.translation.x;
    double dy = t.transform.translation.y;

    if (!initialized) {
      // Save Morty's initial position
      initial_morty_x = dx;
      initial_morty_y = dy;
      initialized = true;
      RCLCPP_INFO(this->get_logger(), "Initial morty's position obtained.");
    }

    double delta_x = dx - initial_morty_x;
    double delta_y = dy - initial_morty_y;
    double moved_distance = std::hypot(delta_x, delta_y);

    if (!morty_started_moving_ && moved_distance > 0.05) {
      morty_started_moving_ = true;
      // RCLCPP_INFO(this->get_logger(), "Morty has started moving. Chase
      // him!");
    }

    // If Morty hasn't started moving, Rick stays still
    if (!morty_started_moving_) {
      RCLCPP_INFO(this->get_logger(),
                  "Morty didn't start moving yet. Remain still.");
      linear_velocity = 0.0;
      angular_velocity = 0.0;
    }

    else {

      // Errors
      double error_distance = std::hypot(dx, dy);
      std::cout << "error_distance :" << error_distance << std::endl;
      //   double error_yaw = std::atan2(dy, dx);
      // Compute the yaw between the robots from tf rotation
      tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y,
                        t.transform.rotation.z, t.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      double error_yaw = yaw; // Yaw difference between Rick and Morty

      std::cout << "error_yaw : " << error_yaw << std::endl;

      double min_distance = 0.2; // start slowing down when closer than this

      // Compute velocity and get the sign + or - of velocity
      linear_velocity = kp_distance * error_distance * dx / std::abs(dx);
      angular_velocity = kp_yaw * error_yaw;

      // Slow down if too close
      if (error_distance < min_distance) {
        RCLCPP_INFO(this->get_logger(), "Very close to Morty, stop!");
        linear_velocity = 0.0;
        angular_velocity = 0.0;
        initialized = false; // restart the position initialization to know when
                             // morty will move again

        return;
      }
    }

    // Publish Twist

    cmd_msg.linear.x = linear_velocity;
    cmd_msg.angular.z = angular_velocity;

    publisher_->publish(cmd_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Gains of Rick's distance and yaw controllers
  double kp_distance;
  double kp_yaw;

  // Rick's velocities
  double linear_velocity;
  double angular_velocity;
  geometry_msgs::msg::Twist cmd_msg;

  bool morty_started_moving_;
  bool initialized;
  double initial_morty_x;
  double initial_morty_y;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();

  return 0;
}