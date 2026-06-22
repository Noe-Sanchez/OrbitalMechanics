#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

struct KeplerianElements {
    double a;      // Semi-major axis (m)
    double e;      // Eccentricity
    double i;      // Inclination (rad)
    double raan;   // Right Ascension of the Ascending Node (rad)
    double arg_p;  // Argument of Periapsis (rad)
    double nu;     // True Anomaly (rad)
};

class OrbitalSimulator : public rclcpp::Node{
  public: OrbitalSimulator() : Node("orbital_simulator"){
    this->declare_parameter<bool>("control_in_eci", true);
  
    // 1. Publishers and Subscribers
    odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>("spacecraft/state", 10);
    path_pub_        = this->create_publisher<nav_msgs::msg::Path>("spacecraft/path", 10);
    orbit_pub_       = this->create_publisher<nav_msgs::msg::Path>("spacecraft/orbit", 10);
    std_orbit_pub_   = this->create_publisher<nav_msgs::msg::Path>("earth/std_orbit", 10);
    viz_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("spacecraft/markers", 10); 
    elements_pub_    = this->create_publisher<std_msgs::msg::Float64MultiArray>("spacecraft/orbital_elements", 10);
    
    // Control interface: expects an acceleration vector (m/s^2)
    /*ctrl_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "spacecraft/control_accel", 10,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
            a_ctrl_ << msg->x, msg->y, msg->z;
        });
    */
    ctrl_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("spacecraft/control_accel", 10, std::bind(&OrbitalSimulator::rtn_accel_callback, this, std::placeholders::_1));
   
    // 2. Initial Conditions (Low Earth Orbit approx 400km altitude)
    // Earth standard gravitational parameter (m^3/s^2)
    mu_ = 3.986004418e14; 
    
    double r_initial = 6371000.0 + 400000.0; // Earth radius + 400km
    double v_initial = std::sqrt(mu_ / r_initial);
    
    r_ << r_initial, 0.0, 0.0;
    v_ << 0.0, v_initial, 0.0;
    a_ctrl_ << 0.0, 0.0, 0.0;
   
    // Path message setup
    path_msg_.header.frame_id = "earth_center";
   
    // 3. Simulation Loop (Timer)
    //dt_ = 0.1; // 100ms simulation step
    dt_ = 1; // FTR factor of 100 for faster simulation
    timer_ = this->create_wall_timer(20ms, std::bind(&OrbitalSimulator::simulation_step, this));

    tf_broadcaster      = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Mesh Earth marker for visualization
    visualization_msgs::msg::Marker earth_marker;
    earth_marker.header.frame_id = "earth_center";
    earth_marker.ns = "earth";
    earth_marker.id = 0;
    earth_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    earth_marker.action = visualization_msgs::msg::Marker::ADD;
    earth_marker.mesh_resource = "package://orbital_mechanics/meshes/earth.dae";
    earth_marker.mesh_use_embedded_materials = true;
    earth_marker.frame_locked = true;
    earth_marker.pose.orientation.w = 0.7071; // Rotate 90 degrees around X-axis to align with RViz's Z-up convention
    earth_marker.pose.orientation.x = 0.7071;
    earth_marker.scale.x = 10.0;
    earth_marker.scale.y = 10.0; 
    earth_marker.scale.z = 10.0;

    viz_markers_pub_->publish(earth_marker);

    spacecraft_marker.header.frame_id = "spacecraft";
    spacecraft_marker.ns = "spacecraft";
    spacecraft_marker.id = 1;
    spacecraft_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    spacecraft_marker.action = visualization_msgs::msg::Marker::ADD;
    spacecraft_marker.mesh_resource = "package://orbital_mechanics/meshes/spacecraft.dae";
    spacecraft_marker.mesh_use_embedded_materials = true;
    spacecraft_marker.frame_locked = true;
    spacecraft_marker.pose.orientation.w = 1.0;
    spacecraft_marker.scale.x = 1e-2;
    spacecraft_marker.scale.y = 1e-2;
    spacecraft_marker.scale.z = 1e-2;

    viz_markers_pub_->publish(spacecraft_marker);

    // Publish standard Earth orbit for reference (circular at 70km)
    // This is just a visual reference and does not affect the simulation
    KeplerianElements std_orbit;
    std_orbit.a = 6371000.0 + 70000.0;
    std_orbit.e = 0.0;
    std_orbit.i = 0.0;
    std_orbit.raan = 0.0;
    std_orbit.arg_p = 0.0;
    std_orbit.nu = 0.0;
    nav_msgs::msg::Path std_orbit_msg = generateAnalyticalOrbit(std_orbit, this->get_clock()->now());
    std_orbit_pub_->publish(std_orbit_msg);
        
    RCLCPP_INFO(this->get_logger(), "Orbital Simulation started.");
  }

  private:
    void rtn_accel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
      // Convert RTN acceleration command to ECI frame
      //Eigen::Vector3d a_rtn(msg->x, msg->y, msg->z);
      //Eigen::Matrix3d R_ECI_RTN = calculateRtnToEci(r_, v_);
      //a_ctrl_ = R_ECI_RTN * a_rtn;
      //a_ctrl_ << msg->x, msg->y, msg->z;
      if (this->get_parameter("control_in_eci").as_bool()) {
        a_ctrl_ << msg->x, msg->y, msg->z;
      } else {
	Eigen::Vector3d a_rtn(msg->x, msg->y, msg->z);
	Eigen::Matrix3d R_ECI_RTN = calculateRtnToEci(r_, v_);
	a_ctrl_ = R_ECI_RTN * a_rtn;
      }
    }

    Eigen::Matrix3d calculateRtnToEci(const Eigen::Vector3d& r, const Eigen::Vector3d& v){
      // Radial unit vector (R)
      Eigen::Vector3d u_R = r.normalized();
      
      // Normal unit vector (N) - perpendicular to orbital plane
      Eigen::Vector3d h = r.cross(v); // Angular momentum vector
      Eigen::Vector3d u_N = h.normalized();
      
      // Transverse/Tangential unit vector (T) - completes right-handed system
      Eigen::Vector3d u_T = u_N.cross(u_R);

      // Rotation matrix: Columns are the RTN basis vectors expressed in ECI
      Eigen::Matrix3d R_ECI_RTN;
      R_ECI_RTN.col(0) = u_R;
      R_ECI_RTN.col(1) = u_T;
      R_ECI_RTN.col(2) = u_N;

      return R_ECI_RTN;
    }

    KeplerianElements cartesianToKeplerian(const Eigen::Vector3d& r, const Eigen::Vector3d& v){
      KeplerianElements elem;
      
      double r_mag = r.norm();
      double v_mag = v.norm();
      
      // Specific angular momentum
      Eigen::Vector3d h = r.cross(v);
      double h_mag = h.norm();
      
      // Node vector
      Eigen::Vector3d k(0, 0, 1);
      Eigen::Vector3d n = k.cross(h);
      double n_mag = n.norm();
      
      // Eccentricity vector
      Eigen::Vector3d e_vec = ((v_mag * v_mag - mu_ / r_mag) * r - (r.dot(v)) * v) / mu_;
      elem.e = e_vec.norm();
      
      // Specific mechanical energy
      double energy = (v_mag * v_mag) / 2.0 - (mu_ / r_mag);
      
      // Semi-major axis
      if (std::abs(energy) > 1e-8) {
          elem.a = -mu_ / (2.0 * energy);
      } else {
          elem.a = 0.0; // Parabolic trajectory edge case
      }
      
      // Inclination
      elem.i = std::acos(std::clamp(h.z() / h_mag, -1.0, 1.0));
      
      // RAAN
      if (n_mag > 1e-8) {
          elem.raan = std::acos(std::clamp(n.x() / n_mag, -1.0, 1.0));
          if (n.y() < 0) elem.raan = 2.0 * M_PI - elem.raan;
      } else {
          elem.raan = 0.0; // Equatorial orbit
      }
      
      // Argument of Periapsis
      if (n_mag > 1e-8 && elem.e > 1e-8) {
          elem.arg_p = std::acos(std::clamp(n.dot(e_vec) / (n_mag * elem.e), -1.0, 1.0));
          if (e_vec.z() < 0) elem.arg_p = 2.0 * M_PI - elem.arg_p;
      } else {
          elem.arg_p = 0.0; // Circular or equatorial orbit
      }
      
      // True Anomaly
      if (elem.e > 1e-8) {
          elem.nu = std::acos(std::clamp(e_vec.dot(r) / (elem.e * r_mag), -1.0, 1.0));
          if (r.dot(v) < 0) elem.nu = 2.0 * M_PI - elem.nu;
      } else {
          // Circular orbit, true anomaly is angle from node vector or X axis
          if (n_mag > 1e-8) {
              elem.nu = std::acos(std::clamp(n.dot(r) / (n_mag * r_mag), -1.0, 1.0));
              if (r.z() < 0) elem.nu = 2.0 * M_PI - elem.nu;
          } else {
              elem.nu = std::atan2(r.y(), r.x());
          }
      }
      
      return elem;
    }
    
    nav_msgs::msg::Path generateAnalyticalOrbit(const KeplerianElements& elem, rclcpp::Time stamp){
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = stamp;
      path_msg.header.frame_id = "earth_center";
      
      // We only draw closed orbits (ellipses) for this visualization
      if (elem.e >= 1.0) return path_msg;

      // Create rotation matrix from Perifocal (PQW) frame to Earth-Centered Inertial (ECI) frame
      // R_ECI_PQW = Rz(-RAAN) * Rx(-i) * Rz(-arg_p)
      Eigen::Matrix3d R_ECI_PQW;
      R_ECI_PQW = Eigen::AngleAxisd(elem.raan, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(elem.i, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(elem.arg_p, Eigen::Vector3d::UnitZ());

      // Sweep true anomaly from 0 to 2*PI to draw the ellipse
      const int num_points = 100;
      for (int step = 0; step <= num_points; ++step) {
        double theta = (double)step / num_points * 2.0 * M_PI;
        
	// Radius at this true anomaly
        double r_mag = (elem.a * (1.0 - elem.e * elem.e)) / (1.0 + elem.e * std::cos(theta));
        
        // Position in perifocal frame (Z is 0)
        Eigen::Vector3d r_pqw(r_mag * std::cos(theta), r_mag * std::sin(theta), 0.0);
        
        // Rotate to global frame
        Eigen::Vector3d r_eci = R_ECI_PQW * r_pqw;
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = r_eci.x() / 1e5; // Scale down for visualization
        pose.pose.position.y = r_eci.y() / 1e5;
        pose.pose.position.z = r_eci.z() / 1e5;
        
        path_msg.poses.push_back(pose);
      }
      
      return path_msg;
    }

    void simulation_step(){
      // --- PHYSICS UPDATE (Semi-Implicit Euler) ---
      
      // 1. Calculate gravitational acceleration
      double r_norm = r_.norm();
      Eigen::Vector3d a_grav = -mu_ * r_ / std::pow(r_norm, 3);
      
      // 2. Total acceleration
      Eigen::Vector3d a_total = a_grav + a_ctrl_;
      
      // 3. Integrate velocity and position
      v_ += a_total * dt_;
      r_ += v_ * dt_;

      // --- ROS 2 PUBLISHING ---
      auto current_time = this->get_clock()->now();

      KeplerianElements elements = cartesianToKeplerian(r_, v_);

      // Publish Odometry (Current State)
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = current_time;
      odom_msg.header.frame_id = "earth_center";
      odom_msg.child_frame_id = "spacecraft";
      
      odom_msg.pose.pose.position.x = r_.x();
      odom_msg.pose.pose.position.y = r_.y();
      odom_msg.pose.pose.position.z = r_.z();
      
      odom_msg.twist.twist.linear.x = v_.x();
      odom_msg.twist.twist.linear.y = v_.y();
      odom_msg.twist.twist.linear.z = v_.z();
      
      odom_pub_->publish(odom_msg);

      // Publish Path (Trajectory Indication)
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = current_time;
      pose.header.frame_id = "earth_center";
      // Position scaled down for visualization purposes
      pose.pose.position.x = r_.x() / 1e5;
      pose.pose.position.y = r_.y() / 1e5;
      pose.pose.position.z = r_.z() / 1e5;
      
      path_msg_.poses.push_back(pose);

      // Populate and publish tf
      viz_tf_.header.stamp = current_time;
      viz_tf_.header.frame_id = "earth_center";
      viz_tf_.child_frame_id = "spacecraft";
      viz_tf_.transform.translation.x = r_.x() / 1e5;
      viz_tf_.transform.translation.y = r_.y() / 1e5;
      viz_tf_.transform.translation.z = r_.z() / 1e5;

      // Send
      tf_broadcaster->sendTransform(viz_tf_);
      
      // Prevent path from eating all memory over long simulations
      if (path_msg_.poses.size() > 5000) {
          path_msg_.poses.erase(path_msg_.poses.begin());
      }
      
      path_pub_->publish(path_msg_);

      std_msgs::msg::Float64MultiArray elem_msg;
      elem_msg.data = {elements.a, elements.e, elements.i, elements.raan, elements.arg_p, elements.nu};
      elements_pub_->publish(elem_msg);

      nav_msgs::msg::Path path_msg = generateAnalyticalOrbit(elements, current_time);
      orbit_pub_->publish(path_msg);
    }

    // ROS Nodes
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr orbit_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr std_orbit_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr elements_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ctrl_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Simulation Variables
    Eigen::Vector3d r_;       // Position vector (m)
    Eigen::Vector3d v_;       // Velocity vector (m/s)
    Eigen::Vector3d a_ctrl_;  // Commanded acceleration vector (m/s^2)
    double mu_;               // Gravitational parameter
    double dt_;               // Time step (s)
    
    nav_msgs::msg::Path path_msg_;
    geometry_msgs::msg::TransformStamped viz_tf_;
    visualization_msgs::msg::Marker spacecraft_marker;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbitalSimulator>());
  rclcpp::shutdown();
  return 0;
}
