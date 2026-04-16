#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class VisualTrackerNode : public rclcpp::Node
{
public:
  VisualTrackerNode()
  : Node("visual_tracker_node"), ball_visible_(false), first_frame_(true), prev_error_x_(0.0)
  {
    // Parámetros Base
    this->declare_parameter("image_center_x", 320.0); 
    this->declare_parameter("target_distance_cm", 15.0); 

    // --- NUEVO: Parámetros del Controlador PD ---
    this->declare_parameter("kp_angular", 0.005); 
    this->declare_parameter("kd_angular", 0.002); // El "freno" derivativo
    this->declare_parameter("angular_deadband", 20.0); // Tolerancia en píxeles (Zona Muerta)
    
    this->declare_parameter("kp_linear", 0.02);   

    // Límites físicos
    this->declare_parameter("max_linear_speed", 0.88); 
    this->declare_parameter("max_angular_speed", 3.0); 

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    sub_vision_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/vision/ball_position", 10, 
      std::bind(&VisualTrackerNode::vision_callback, this, std::placeholders::_1));

    watchdog_timer_ = this->create_wall_timer(
      100ms, std::bind(&VisualTrackerNode::watchdog_loop, this));
      
    RCLCPP_INFO(this->get_logger(), "Servocontrol Visual PD Iniciado. Modo Caza Activado.");
  }

private:
  void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();
    
    if (!ball_visible_) {
      // Si acabamos de ver la pelota, reiniciamos el tiempo para no tener un dt gigante
      last_calc_time_ = current_time;
      first_frame_ = true;
    }
    
    ball_visible_ = true;
    last_seen_time_ = current_time;

    // Calcular diferencial de tiempo (dt)
    double dt = (current_time - last_calc_time_).seconds();
    last_calc_time_ = current_time;

    // Obtener parámetros
    double center_x = this->get_parameter("image_center_x").as_double();
    double target_z = this->get_parameter("target_distance_cm").as_double();
    double kp_ang = this->get_parameter("kp_angular").as_double();
    double kd_ang = this->get_parameter("kd_angular").as_double();
    double deadband = this->get_parameter("angular_deadband").as_double();
    double kp_lin = this->get_parameter("kp_linear").as_double();
    double max_v = this->get_parameter("max_linear_speed").as_double();
    double max_w = this->get_parameter("max_angular_speed").as_double();

    auto twist_msg = geometry_msgs::msg::Twist();

    // --- 1. LAZO ANGULAR PD ---
    double error_x = center_x - msg->x;
    double cmd_w = 0.0;

    // Solo calculamos giro si está fuera de la zona muerta (ej. fuera del rango 300-340)
    if (std::abs(error_x) > deadband) {
        double derivative_x = 0.0;
        
        if (!first_frame_ && dt > 0.0) {
            derivative_x = (error_x - prev_error_x_) / dt;
        }
        
        // La magia del PD
        cmd_w = (kp_ang * error_x) + (kd_ang * derivative_x);
    }

    // --- 2. LAZO LINEAL ---
    double error_z = msg->z - target_z;
    double cmd_v = 0.0;
    
    if (std::abs(error_z) > 2.0) { 
        cmd_v = kp_lin * error_z;
    }

    // --- 3. SATURACIÓN Y ACOPLAMIENTO ---
    twist_msg.angular.z = std::clamp(cmd_w, -max_w, max_w);
    
    double speed_reduction_factor = 1.0 - std::min(1.0, std::abs(error_x) / center_x);
    cmd_v = cmd_v * speed_reduction_factor;
    twist_msg.linear.x = std::clamp(cmd_v, -max_v, max_v);

    publisher_->publish(twist_msg);

    // Guardar estado para el siguiente fotograma
    prev_error_x_ = error_x;
    first_frame_ = false;
  }

  void watchdog_loop()
  {
    if (ball_visible_) {
      auto now = this->now();
      if ((now - last_seen_time_).seconds() > 0.3) {
        ball_visible_ = false;
        first_frame_ = true; // Resetear el estado del derivador
        
        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        publisher_->publish(stop_msg);
        
        RCLCPP_WARN(this->get_logger(), "Pelota perdida. Frenando...");
      }
    }
  }

  bool ball_visible_;
  bool first_frame_;
  double prev_error_x_;
  rclcpp::Time last_calc_time_;
  rclcpp::Time last_seen_time_;
  
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_vision_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualTrackerNode>());
  rclcpp::shutdown();
  return 0;
}