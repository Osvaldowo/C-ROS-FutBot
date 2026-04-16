#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PIStraightSequencer : public rclcpp::Node
{
public:
  PIStraightSequencer()
  : Node("pi_straight_sequencer"), state_(0), integral_theta_error_(0.0), initialized_(false)
  {
    // Parámetros para el Lazo de Distancia (Lineal)
    this->declare_parameter("kp_linear", 1.0);
    this->declare_parameter("max_linear_speed", 0.88);
    this->declare_parameter("target_distance", 1.0); // Metros a avanzar
    this->declare_parameter("linear_tolerance", 0.02); // 2 cm de tolerancia

    // Parámetros para el Lazo de Rumbo (Angular)
    this->declare_parameter("kp_angular", 2.0);
    this->declare_parameter("ki_angular", 0.05);
    this->declare_parameter("max_angular_speed", 1.0);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PIStraightSequencer::odom_callback, this, std::placeholders::_1));

    last_time_ = this->now();
    timer_ = this->create_wall_timer(20ms, std::bind(&PIStraightSequencer::control_loop, this));
      
    RCLCPP_INFO(this->get_logger(), "Secuenciador de Línea Recta con Heading Lock Iniciado...");
  }

private:
  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    current_theta_ = std::atan2(siny_cosp, cosy_cosp);

    if (!initialized_) {
      // Guardar el punto de partida como ancla
      start_x_ = current_x_;
      start_y_ = current_y_;
      start_theta_ = current_theta_; // Rumbo a mantener
      
      integral_theta_error_ = 0.0;
      last_time_ = this->now();
      initialized_ = true;
    }
  }

  void control_loop()
  {
    if (!initialized_) return;

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double kp_linear = this->get_parameter("kp_linear").as_double();
    double max_lin_speed = this->get_parameter("max_linear_speed").as_double();
    double target_dist = this->get_parameter("target_distance").as_double();
    double lin_tol = this->get_parameter("linear_tolerance").as_double();

    double kp_angular = this->get_parameter("kp_angular").as_double();
    double ki_angular = this->get_parameter("ki_angular").as_double();
    double max_ang_speed = this->get_parameter("max_angular_speed").as_double();

    auto msg = geometry_msgs::msg::Twist();

    if (state_ == 0) { // ESTADO: AVANZANDO
      
      // 1. Lazo Lineal (Distancia)
      double dist_traveled = std::hypot(current_x_ - start_x_, current_y_ - start_y_);
      double error_dist = target_dist - dist_traveled;

      if (std::abs(error_dist) < lin_tol) {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        state_ = 1;
        stop_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Meta alcanzada. Distancia: %.3f m.", dist_traveled);
      } 
      else {
        // Velocidad hacia adelante
        double cmd_v = kp_linear * error_dist;
        cmd_v = std::clamp(cmd_v, -max_lin_speed, max_lin_speed);
        
        // 2. Lazo Angular (Heading Lock)
        // Intentamos mantener current_theta_ IGUAL a start_theta_
        double error_theta = normalize_angle(start_theta_ - current_theta_);
        integral_theta_error_ += error_theta * dt;
        integral_theta_error_ = std::clamp(integral_theta_error_, -0.5, 0.5); // Anti-windup
        
        double cmd_w = (kp_angular * error_theta) + (ki_angular * integral_theta_error_);
        cmd_w = std::clamp(cmd_w, -max_ang_speed, max_ang_speed);

        msg.linear.x = cmd_v;
        msg.angular.z = cmd_w; // Inyectamos la corrección de rumbo mientras avanza
      }
      
    } else if (state_ == 1) { // ESTADO: ESPERA
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      
      if ((this->now() - stop_time_).seconds() >= 4.0) {
        // Renovar el punto de partida para avanzar otro metro
        start_x_ = current_x_;
        start_y_ = current_y_;
        start_theta_ = current_theta_;
        integral_theta_error_ = 0.0; 

        state_ = 0;
        RCLCPP_INFO(this->get_logger(), "Iniciando siguiente tramo recto...");
      }
    }
    publisher_->publish(msg);
  }

  int state_; 
  double current_x_, current_y_, current_theta_;
  double start_x_, start_y_, start_theta_;
  double integral_theta_error_;
  bool initialized_;
  rclcpp::Time last_time_, stop_time_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIStraightSequencer>());
  rclcpp::shutdown();
  return 0;
}