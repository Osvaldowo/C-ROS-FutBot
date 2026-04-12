#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("odom_publisher"), x_(0.0), y_(0.0), th_(0.0), v_left_(0.0), v_right_(0.0)
  {
    sub_vel_left_ = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_vel_left", 10, std::bind(&OdomPublisher::vel_left_cb, this, std::placeholders::_1));
    sub_vel_right_ = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_vel_right", 10, std::bind(&OdomPublisher::vel_right_cb, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = this->now();
    last_vel_time_ = this->now(); // Inicializar el tiempo del watchdog

    timer_ = this->create_wall_timer(
      20ms, std::bind(&OdomPublisher::update_odometry, this));
      
    RCLCPP_INFO(this->get_logger(), "Nodo de Odometria Mejorada Iniciado.");
  }

private:
  void vel_left_cb(const std_msgs::msg::Float32::SharedPtr msg) { 
      v_left_ = msg->data; 
      last_vel_time_ = this->now(); 
  }
  void vel_right_cb(const std_msgs::msg::Float32::SharedPtr msg) { 
      v_right_ = msg->data; 
      last_vel_time_ = this->now(); 
  }

  void update_odometry()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // WATCHDOG: Si no hay mensajes en 0.5s, asume que el robot se detuvo
    if ((current_time - last_vel_time_).seconds() > 0.5) {
        v_left_ = 0.0;
        v_right_ = 0.0;
    }

    double v_L = v_left_ / 1000.0;
    double v_R = v_right_ / 1000.0;
    double L = 0.166; 

    double v_center = (v_R + v_L) / 2.0; 
    double omega = (v_R - v_L) / L;      

    // INTEGRACIÓN RUNGE-KUTTA (Punto Medio) para mayor precisión
    double delta_th = omega * dt;
    double th_mid = th_ + (delta_th / 2.0); // Ángulo a la mitad del movimiento
    
    double delta_x = v_center * cos(th_mid) * dt;
    double delta_y = v_center * sin(th_mid) * dt;
    
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
    
    // Normalizar theta
    if (th_ > M_PI) th_ -= 2.0 * M_PI;
    if (th_ < -M_PI) th_ += 2.0 * M_PI;

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);

    // Publicar TF
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(odom_tf);

    // Publicar Odometría
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.twist.twist.linear.x = v_center;
    odom_msg.twist.twist.angular.z = omega;
    odom_pub_->publish(odom_msg);

    last_time_ = current_time;
  }

  double x_, y_, th_;
  double v_left_, v_right_;
  rclcpp::Time last_time_;
  rclcpp::Time last_vel_time_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vel_left_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vel_right_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}