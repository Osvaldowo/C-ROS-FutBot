#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class BallPlannerNode : public rclcpp::Node
{
public:
  BallPlannerNode() : Node("ball_planner_node"), has_odom_(false)
  {
    // Parámetros de la cámara
    this->declare_parameter("camera_hfov", 1.1944);
    this->declare_parameter("image_width", 640.0);

    // Publicador de la trayectoria (Path)
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    // Publicador para visualizar el vector de repulsión en RViz (opcional)
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("repulsion_vector", 10);

    // Suscriptores (Se ajusta la QoS de la odometría a SensorData para compatibilidad con Gazebo)
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&BallPlannerNode::odom_callback, this, std::placeholders::_1));

    sub_vision_ = this->create_subscription<geometry_msgs::msg::Point>(
      "vision/ball_position", 10, std::bind(&BallPlannerNode::vision_callback, this, std::placeholders::_1));

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&BallPlannerNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Planificador iniciado. Esperando flujos de datos...");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Aviso en terminal que solo se dispara la primera vez que recibe el dato
    RCLCPP_INFO_ONCE(this->get_logger(), "¡Odometría detectada y acoplada!");

    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    robot_theta_ = std::atan2(siny_cosp, cosy_cosp);

    has_odom_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!has_odom_) return;

    double force_x = 0.0;
    double force_y = 0.0;
    double k_rep = 0.005; 
    double safe_dist = 0.45; 

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      
      if (std::isfinite(r) && r > msg->range_min && r < safe_dist) {
        double angle_local = msg->angle_min + i * msg->angle_increment;
        double angle_global = robot_theta_ + angle_local;
        double magnitude = k_rep / (r * r);

        // 1. Fuerza de repulsión clásica (hacia afuera)
        double f_rep_x = -magnitude * std::cos(angle_global);
        double f_rep_y = -magnitude * std::sin(angle_global);

        // 2. Fuerza Tangencial (Rotación de 90 grados)
        // Determinar de qué lado está el obstáculo respecto al frente del robot
        double sign = (angle_local > 0.0) ? -1.0 : 1.0; 
        
        double f_tan_x = -sign * magnitude * std::sin(angle_global);
        double f_tan_y = sign * magnitude * std::cos(angle_global);

        // 3. Mezcla aerodinámica (20% alejar, 80% rodear)
        force_x += (0.2 * f_rep_x) + (0.8 * f_tan_x);
        force_y += (0.2 * f_rep_y) + (0.8 * f_tan_y);
      }
    }

    // CLAMPING de seguridad
    double max_force = 0.6; 
    double total_force = std::hypot(force_x, force_y);
    
    if (total_force > max_force) {
        force_x = (force_x / total_force) * max_force;
        force_y = (force_y / total_force) * max_force;
    }

    rep_x_ = force_x;
    rep_y_ = force_y;

    // ==========================================
    // DIBUJAR EL VECTOR DE REPULSIÓN EN RVIZ
    // ==========================================
    visualization_msgs::msg::Marker marker;
    std::string ns = this->get_namespace();
    marker.header.frame_id = (ns == "/" || ns == "") ? "odom" : ns.substr(1) + "/odom";
    marker.header.stamp = this->now();
    marker.ns = "repulsion";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Punto de origen de la flecha (El centro del robot, un poco elevado para que se vea)
    geometry_msgs::msg::Point start_point;
    start_point.x = robot_x_;
    start_point.y = robot_y_;
    start_point.z = 0.25; 

    // Punto final de la flecha
    geometry_msgs::msg::Point end_point;
    end_point.x = robot_x_ + (rep_x_); 
    end_point.y = robot_y_ + (rep_y_);
    end_point.z = 0.25;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // Dimensiones de la flecha (Grosor del cuerpo, grosor de la cabeza, longitud de la cabeza)
    marker.scale.x = 0.005; 
    marker.scale.y = 0.01;  
    marker.scale.z = 0.01;  

    // Color Rojo brillante para indicar "Peligro / Repulsión"
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Transparencia (1.0 = sólido)

    pub_marker_->publish(marker);
  }

  void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (!has_odom_) return;

    // Aviso de que el lazo de control maestro está operando
    RCLCPP_INFO_ONCE(this->get_logger(), "¡Visión recibida! Calculando Curvas de Bezier...");

    double hfov = this->get_parameter("camera_hfov").as_double();
    double width = this->get_parameter("image_width").as_double();

    // 1. Calcular coordenadas globales
    double center_x = width / 2.0;
    double pixel_error = center_x - msg->x;
    double angle_to_ball_local = (pixel_error / width) * hfov;
    double distance_m = msg->z / 100.0;

    double global_ball_x = robot_x_ + distance_m * std::cos(robot_theta_ + angle_to_ball_local);
    double global_ball_y = robot_y_ + distance_m * std::sin(robot_theta_ + angle_to_ball_local);

    // 2. Construir la trayectoria con el Frame ID correcto según el Namespace
    nav_msgs::msg::Path path_msg;
    
    std::string ns = this->get_namespace();
    // Limpiar la barra inclinada inicial si existe para que RViz lo lea bien (ej: "/futbot_1" -> "futbot_1/odom")
    std::string target_frame = (ns == "/" || ns == "") ? "odom" : ns.substr(1) + "/odom";
    
    path_msg.header.frame_id = target_frame;
    path_msg.header.stamp = this->now();

    // 3. Puntos de Control de Bezier
    double P0_x = robot_x_;
    double P0_y = robot_y_;
    
    double k = distance_m * 0.5; 
    double P1_x = robot_x_ + k * std::cos(robot_theta_) + rep_x_; 
    double P1_y = robot_y_ + k * std::sin(robot_theta_) + rep_y_; 

    double P2_x = global_ball_x;
    double P2_y = global_ball_y;
    double P3_x = global_ball_x;
    double P3_y = global_ball_y;

    // 4. Generar curva
    int num_waypoints = 20; 
    for (int i = 0; i <= num_waypoints; ++i) {
      double t = (double)i / num_waypoints;
      double u = 1.0 - t;
      
      double x = (u*u*u)*P0_x + 3*(u*u)*t*P1_x + 3*u*(t*t)*P2_x + (t*t*t)*P3_x;
      double y = (u*u*u)*P0_y + 3*(u*u)*t*P1_y + 3*u*(t*t)*P2_y + (t*t*t)*P3_y;

      geometry_msgs::msg::PoseStamped pose_inter;
      pose_inter.pose.position.x = x;
      pose_inter.pose.position.y = y;
      path_msg.poses.push_back(pose_inter);
    }

    pub_path_->publish(path_msg);
  }

  bool has_odom_;
  double robot_x_, robot_y_, robot_theta_;
  double rep_x_ = 0.0;
  double rep_y_ = 0.0;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_vision_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallPlannerNode>());
  rclcpp::shutdown();
  return 0;
}