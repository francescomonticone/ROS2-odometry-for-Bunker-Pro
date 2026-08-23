#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp" //base library ROS2

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h" //for tf buffer

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp" //to compute travelled_distance from linear velocity 

#include "first_project/msg/tf_error_msg.hpp" //custom message used


using std::placeholders::_1;

class TfErrorNode : public rclcpp::Node
{
public:
  TfErrorNode()
  : Node("tf_error"),
    start_time_(this->now()), //use_sim_time=true uses simulated time from the bag
    travelled_distance_(0.0)
  {
  
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_link_gt_frame_ = this->declare_parameter<std::string>("base_link_gt", "base_link");
    base_link_our_frame_ = this->declare_parameter<std::string>("base_link_our", "base_link2");

    // custom message publisher
    error_pub_ = this->create_publisher<first_project::msg::TfErrorMsg>("/tf_error_msg", 10);

    // computed odometry subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/project_odom", 10, std::bind(&TfErrorNode::odomCallback, this, _1));

    // Tf buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer for error computation
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), //each 50ms 
      std::bind(&TfErrorNode::computeAndPublishError, this));

    RCLCPP_INFO(this->get_logger(), "tf_error node started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Compute travelled_distance from odometry
    rclcpp::Time current_time(msg->header.stamp);
    static rclcpp::Time last_time = current_time; //static variable
    double dt = (current_time - last_time).seconds();
    if (dt > 0.0 && dt < 1.0) { //if bag is not suspended
      double v = msg->twist.twist.linear.x; 
      travelled_distance_ += std::fabs(v) * dt; //|v| * dt 
    }
    last_time = current_time;
  }

  void computeAndPublishError()
  {
    // tf odom - base_link (gt)
    geometry_msgs::msg::TransformStamped tf_gt;
    try {
      tf_gt = tf_buffer_->lookupTransform(odom_frame_, base_link_gt_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Could not get transform %s -> %s: %s",
                           odom_frame_.c_str(), base_link_gt_frame_.c_str(), ex.what());
      return;
    }

    // tf odom - base_link2 (our odometry)
    geometry_msgs::msg::TransformStamped tf_our;
    try {
      tf_our = tf_buffer_->lookupTransform(odom_frame_, base_link_our_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Could not get transform %s -> %s: %s",
                           odom_frame_.c_str(), base_link_our_frame_.c_str(), ex.what());
      return;
    }

    //Compute error as 2D distance
    double dx = tf_gt.transform.translation.x - tf_our.transform.translation.x;
    double dy = tf_gt.transform.translation.y - tf_our.transform.translation.y;
    double error = std::sqrt(dx*dx + dy*dy);

    //Compute time from start (start time is when the Node is constructed)
    rclcpp::Time now = this->now();
    if (!start_time_initialized_ && now.nanoseconds() > 0) {
    start_time_ = now;
    start_time_initialized_ = true;
    }   
    if (!start_time_initialized_) return; //if the time in not already initialized

int32_t time_from_start = static_cast<int32_t>((now - start_time_).seconds());

    //Publish custom msg
    auto msg = first_project::msg::TfErrorMsg();
    msg.header.stamp = now;
    msg.tf_error = static_cast<float>(error);
    msg.time_from_start = time_from_start;
    msg.travelled_distance = static_cast<float>(travelled_distance_);

    error_pub_->publish(msg);

    // Log for debugging every 2.0 seconds
    static rclcpp::Time last_log = now;
    if ((now - last_log).seconds() > 2.0) {
      RCLCPP_INFO(this->get_logger(),
                  "error=%.3f m, travelled=%.2f m, time=%d s",
                  error, travelled_distance_, time_from_start);
      last_log = now;
    }
  }

  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr error_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_frame_;
  std::string base_link_gt_frame_;
  std::string base_link_our_frame_;

  rclcpp::Time start_time_;
  double travelled_distance_;
  bool start_time_initialized_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfErrorNode>());
  rclcpp::shutdown();
  return 0;
}