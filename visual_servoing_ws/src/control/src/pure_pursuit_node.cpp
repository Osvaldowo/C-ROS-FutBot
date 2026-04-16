#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode() : Node("pure_pursuit_node"), has_path_(false)
  {
    // --- PARÁMETROS DEL PURE PURSUIT ---
    this->declare_parameter("lookahead_distance", 0.35); // Distancia del punto virtual (metros)
    this->declare_parameter("linear_velocity", 0.5);     // Velocidad base de persecución
    this->declare_parameter("max_angular_speed", 3.0);   // Límite de giro

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10, std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PurePursuitNode::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Controlador Pure Pursuit Iniciado.");
  }

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      has_path_ = false;
      return;
    }
    current_path_ = *msg;
    has_path_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_path_) return;

    double robot_x = msg->pose.pose.position.x;
    double robot_y = msg->pose.pose.position.y;
    
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    double robot_theta = std::atan2(siny_cosp, cosy_cosp);

    double Ld = this->get_parameter("lookahead_distance").as_double();
    double v = this->get_parameter("linear_velocity").as_double();
    double max_w = this->get_parameter("max_angular_speed").as_double();

    // 1. Buscar el punto "Lookahead" en la trayectoria
    double target_x = robot_x;
    double target_y = robot_y;
    bool point_found = false;

    // Recorremos la curva trazada buscando el punto que esté a distancia Ld
    for (const auto& pose : current_path_.poses) {
      double px = pose.pose.position.x;
      double py = pose.pose.position.y;
      double dist = std::hypot(px - robot_x, py - robot_y);
      
      if (dist >= Ld) {
        target_x = px;
        target_y = py;
        point_found = true;
        break;
      }
    }

    // Si la curva es más corta que Ld (la pelota está muy cerca), apuntamos al último punto
    if (!point_found && !current_path_.poses.empty()) {
      target_x = current_path_.poses.back().pose.position.x;
      target_y = current_path_.poses.back().pose.position.y;
      
      // Detenerse si ya llegamos a la pelota (ej. menos de 10 cm)
      if (std::hypot(target_x - robot_x, target_y - robot_y) < 0.1) {
        auto stop_msg = geometry_msgs::msg::Twist();
        publisher_->publish(stop_msg);
        return;
      }
    }

    // 2. Matemáticas de Pure Pursuit
    // Ángulo absoluto hacia el punto objetivo
    double angle_to_target = std::atan2(target_y - robot_y, target_x - robot_x);
    
    // Ángulo relativo (Alpha) respetando la orientación del robot
    double alpha = angle_to_target - robot_theta;
    
    // Normalizar Alpha entre -PI y PI
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    // Calcular velocidad angular (Curvatura)
    double cmd_w = (2.0 * v * std::sin(alpha)) / Ld;

    // 3. Publicar velocidades
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = std::clamp(cmd_w, -max_w, max_w);
    publisher_->publish(cmd_msg);
  }

  bool has_path_;
  nav_msgs::msg::Path current_path_;
  
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}