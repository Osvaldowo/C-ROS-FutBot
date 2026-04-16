#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm> // para std::clamp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PITurnSequencer : public rclcpp::Node
{
public:
  PITurnSequencer()
  : Node("pi_turn_sequencer"), state_(0), current_theta_(0.0), target_theta_(0.0), prev_error_(0.0), integral_error_(0.0), initialized_(false)
  {
    // --- PARÁMETROS DEL CONTROLADOR PI ---
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("ki", 0.2); // Ganancia Integral
    this->declare_parameter("max_integral", 1.5); // Límite Anti-Windup
    this->declare_parameter("max_speed", 1.5); // Velocidad angular máxima
    this->declare_parameter("tolerance", 0.02); // Tolerancia más estricta (aprox 1 grado)

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Suscribirse a la odometría que publica Gazebo
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PITurnSequencer::odom_callback, this, std::placeholders::_1));

    last_time_ = this->now();

    timer_ = this->create_wall_timer(
      20ms, std::bind(&PITurnSequencer::control_loop, this));
      
    RCLCPP_INFO(this->get_logger(), "Secuenciador PI en Gazebo Iniciado...");
  }

private:
  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extraer Yaw del Cuaternión que envía Gazebo
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    current_theta_ = std::atan2(siny_cosp, cosy_cosp);

    if (!initialized_) {
      target_theta_ = normalize_angle(current_theta_ + M_PI / 2.0); 
      prev_error_ = normalize_angle(target_theta_ - current_theta_);
      integral_error_ = 0.0;
      last_time_ = this->now();
      initialized_ = true;
    }
  }

  void control_loop()
  {
    if (!initialized_) return;

    // Calcular dt real
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Obtener parámetros en tiempo real de ROS 2
    double kp = this->get_parameter("kp").as_double();
    double ki = this->get_parameter("ki").as_double();
    double max_integral = this->get_parameter("max_integral").as_double();
    double max_speed = this->get_parameter("max_speed").as_double();
    double tolerance = this->get_parameter("tolerance").as_double();

    auto msg = geometry_msgs::msg::Twist();
    double error = normalize_angle(target_theta_ - current_theta_);

    if (state_ == 0) { // ESTADO: GIRANDO
      
      if (std::abs(error) < tolerance) {
        msg.angular.z = 0.0;
        state_ = 1;
        stop_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Objetivo alcanzado. Error final: %.4f rad.", error);
      } 
      else {
        // 1. Acumular error integral
        integral_error_ += error * dt;
        
        // 2. Aplicar Anti-Windup (Saturar la integral)
        integral_error_ = std::clamp(integral_error_, -max_integral, max_integral);

        // 3. Ecuación del Controlador PI
        double cmd_w = (kp * error) + (ki * integral_error_);
        
        // 4. Saturar la velocidad enviada a las ruedas
        cmd_w = std::clamp(cmd_w, -max_speed, max_speed);
        
        msg.angular.z = cmd_w;
      }
      prev_error_ = error;
      
    } else if (state_ == 1) { // ESTADO: ESPERA
      msg.angular.z = 0.0;
      if ((this->now() - stop_time_).seconds() >= 4.0) {
        // Siguiente objetivo
        target_theta_ = normalize_angle(target_theta_ + M_PI / 2.0);
        prev_error_ = normalize_angle(target_theta_ - current_theta_); 
        
        // ¡VITAL! Resetear la memoria integral para el nuevo giro
        integral_error_ = 0.0; 

        state_ = 0;
        RCLCPP_INFO(this->get_logger(), "Iniciando siguiente giro...");
      }
    }
    publisher_->publish(msg);
  }

  int state_; 
  double current_theta_;
  double target_theta_;
  double prev_error_;
  double integral_error_; // Término Acumulativo (I)
  bool initialized_;
  rclcpp::Time last_time_;
  rclcpp::Time stop_time_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PITurnSequencer>());
  rclcpp::shutdown();
  return 0;
}