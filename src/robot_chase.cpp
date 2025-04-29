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
        this->create_wall_timer(100ms, std::bind(&RobotChase::on_timer, this));

    // Set the gains and max velocities
    kp_distance = 0.5;
    kp_yaw = 0.5;
    max_linear_velocity_ = 0.5;
    max_angular_velocity_ = 0.8;
    min_distance = 0.4; // start slowing down when closer than this

    // indicates if morty's initial position is saved or not yet
    is_pose_initialized = false;
    morty_started_moving_ = false;

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

    // std::cout << "dx = " << dx << std::endl;
    // std::cout << "dy = " << dy << std::endl;

    if (!is_pose_initialized) {
      // Save Morty's initial position
      initial_morty_x = dx;
      initial_morty_y = dy;
      is_pose_initialized = true;
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
      double error_distance = std::sqrt(dx * dx + dy * dy);
      std::cout << "error_distance :" << error_distance << std::endl;

      double error_yaw = std::atan2(dy, dx);
      // std::cout << "error_yaw : " << error_yaw << std::endl;

      // Compute velocity and get the direction of movement
      linear_velocity = kp_distance * error_distance;
      angular_velocity = kp_yaw * error_yaw;

      // Slow down if too close
      if (error_distance < min_distance) {
        RCLCPP_INFO(this->get_logger(), "Very close to Morty, stop!");
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        publisher_->publish(cmd_msg);

        is_pose_initialized = false; // restart the position initialization to
                                     //   know when morty will move again

        return;
      }
    }

    // Publish Twist
    cmd_msg.linear.x = std::min(linear_velocity, max_linear_velocity_);
    cmd_msg.angular.z = std::min(angular_velocity, max_angular_velocity_);
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
  double max_linear_velocity_;
  double max_angular_velocity_;

  // Morty's initial state
  bool morty_started_moving_;
  bool is_pose_initialized;
  double initial_morty_x;
  double initial_morty_y;
  double min_distance;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();

  return 0;
}