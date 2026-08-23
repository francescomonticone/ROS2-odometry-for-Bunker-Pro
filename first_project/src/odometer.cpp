#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp" //base library ROS2

#include "nav_msgs/msg/odometry.hpp" //msg for odometry
#include "geometry_msgs/msg/transform_stamped.hpp" //msg for tf

#include "std_srvs/srv/empty.hpp" //service without arguments for reset

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h" 
#include "tf2_ros/transform_broadcaster.h"

#include "bunker_msgs/msg/bunker_status.hpp" //custom msg of the robot

using std::placeholders::_1;
using std::placeholders::_2;

class Odometer : public rclcpp::Node
{
public:
  Odometer()
  : Node("odometer"),
    x_(0.0),
    y_(0.0),
    theta_(0.0),
    initialized_(false)
  {
    k_ = this->declare_parameter<double>("k", 0.00117); //our estimated value of k

    wheel_base_ = this->declare_parameter<double>("wheel_base", 0.72); //our estimate value of apparent baseline L

    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link2");

    //creation of pub, sub and service 
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);

    status_sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10, std::bind(&Odometer::statusCallback, this, _1));
    
    //used to initialize the position according to the first value of the bag file (if the bag doesn't start from zero position)
    gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Odometer::gpsCallback, this, _1)); 

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset", std::bind(&Odometer::resetCallback, this, _1, _2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "odometer started (k=%.6f, wheel_base=%.3f)", k_, wheel_base_);
  }

private:
  void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!initialized_) { //initilize the position according to the bag file starting point
      x_ = msg->pose.pose.position.x;
      y_ = msg->pose.pose.position.y;

      tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
      
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      theta_ = yaw;

      last_time_ = rclcpp::Time(msg->header.stamp); //initial timestamp for the first integration
      initialized_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Initialized from /odom_gps: x=%.3f y=%.3f theta=%.3f rad",
                  x_, y_, theta_);
    }
  }

  bool extractWheelSpeeds(
    const bunker_msgs::msg::BunkerStatus & msg,
    double & v_left,
    double & v_right)
  {
    if (msg.actuator_states.size() < 2)
      return false;
    v_right  = k_ * msg.actuator_states[0].rpm; //right wheel RPM converted to Vr
    v_left = k_ * msg.actuator_states[1].rpm; //left wheel RPM converted to Vl
    return true;
  }

  void statusCallback(const bunker_msgs::msg::BunkerStatus & msg)
  {
    if (!initialized_) {
      //wait for the first GPS message to initialize the position
      return;
    }

    double v_left, v_right;
    if (!extractWheelSpeeds(msg, v_left, v_right)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Cannot extract wheel speeds");
      return;
    }

    rclcpp::Time now = this->get_clock()->now(); //use_sim_time:=true in the launch file will use simulated time from /clock topic
    
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0) return;

    //differential drive kinematic approximation

    double v = 0.5 * (v_right + v_left);
    double omega = (v_right - v_left) / wheel_base_;

    double dtheta = omega * dt;
    double theta_mid = theta_ + 0.5 * dtheta; //2 order Runge-Kutta approximation

    x_ += v * std::cos(theta_mid) * dt;
    y_ += v * std::sin(theta_mid) * dt;
    theta_ += dtheta;

    // theta normalization btw -pi and +pi
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    publishOdometry(now, v, omega);
    publishTf(now);
  }

  void publishOdometry(const rclcpp::Time & stamp, double v, double omega)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;

    odom_pub_->publish(odom); //publish odometry computation
  }

  void publishTf(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t); //publish tf odom - base_link2
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    initialized_ = false;  // waits for another /odom_gps starting position reset
    RCLCPP_INFO(this->get_logger(), "Odometry reset - waiting for /odom_gps");
  }

  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double x_, y_, theta_;
  double k_, wheel_base_;
  bool initialized_;
  rclcpp::Time last_time_;
  std::string odom_frame_, base_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}