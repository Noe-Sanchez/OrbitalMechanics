#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <cmath>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace Eigen;
using namespace std;

// Constants
const double PI = 3.14159265358979323846;

// ========================================================================
// HELPER FUNCTIONS
// ========================================================================


class MechanicsNode : public rclcpp::Node{
  public:
    MechanicsNode(): Node("mechanics_node"){
      // Asume we get conf from file, hardcode for now

      this->declare_parameter<double>("time_step", 0.01);
      dt = this->get_parameter("time_step").as_double(); 

      // Subscribers
      thrust_subscriber     = this->create_subscription<geometry_msgs::msg::Twist>("sat_thrust", 10, std::bind(&MechanicsNode::sat_thrust_callback, this, std::placeholders::_1));
      burning_subscriber    = this->create_subscription<std_msgs::msg::Bool>("sat_burning", 10, std::bind(&MechanicsNode::sat_burning_callback, this, std::placeholders::_1));

      // Publishers
      state_publisher    = this->create_publisher<geometry_msgs::msg::PoseStamped>("sat_state", 10);
      ellipse_publisher  = this->create_publisher<visualization_msgs::msg::Marker>("orbit_ellipse", 10);
      orbital_points_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("orbital_points", 10);

      // Make 0.1s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&MechanicsNode::control_callback, this));
      viz_timer     = this->create_wall_timer(200ms, std::bind(&MechanicsNode::viz_callback, this));
  
      r << 7000000.0, 0.0, 0.0;
      v << 0.0, 7546.0, 0.0;

      step = 0;
      coe_initial = computeCOE(r, v, mu);
    }

    void sat_thrust_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      thrust_msg = *msg;
    }
    void sat_burning_callback(const std_msgs::msg::Bool::SharedPtr msg){
      burning = msg->data;
    }


    struct OrbitalElements {
      double a;       // Semi-major axis
      double e;       // Eccentricity
      double i;       // Inclination (radians)
      double Omega;   // RAAN (radians)
      double omega;   // Argument of periapsis (radians)
      double nu;      // True anomaly (radians)
    };
    
    // Visualization current orbit with proper 3D rotation
    void viz_callback(){
      // Compute current orbital elements
      OrbitalElements coe_current = computeCOE(r, v, mu);

      // Create ellipse marker
      visualization_msgs::msg::Marker ellipse_marker;
      ellipse_marker.header.frame_id = "world";
      ellipse_marker.header.stamp = this->now();
      ellipse_marker.ns = "orbit";
      ellipse_marker.id = 0;
      ellipse_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      ellipse_marker.action = visualization_msgs::msg::Marker::ADD;
      ellipse_marker.scale.x = 0.05;  // Line width
      ellipse_marker.color.a = 1.0;
      ellipse_marker.color.r = 0.0;
      ellipse_marker.color.g = 1.0;
      ellipse_marker.color.b = 0.0;

      // Generate points along the orbit
      int num_points = 100;
      for (int i = 0; i <= num_points; i++) {
        double theta = 2.0 * PI * i / num_points;
        double r_mag = (coe_current.a * (1 - coe_current.e * coe_current.e)) /
                         (1 + coe_current.e * cos(theta));

        // Position in orbital plane (perifocal frame)
        double x_peri = r_mag * cos(theta);
        double y_peri = r_mag * sin(theta);
        double z_peri = 0.0;

        // Apply 3D rotation from perifocal to inertial frame
        Vector3d pos_peri(x_peri, y_peri, z_peri);
        Vector3d pos_inertial = perifocalToInertial(pos_peri, coe_current.i, 
                                                      coe_current.Omega, coe_current.omega);

        geometry_msgs::msg::Point p;
        p.x = pos_inertial(0) / 1000000.0;  // Scale down for visualization
        p.y = pos_inertial(1) / 1000000.0;
        p.z = pos_inertial(2) / 1000000.0;

        ellipse_marker.points.push_back(p);
      }

      ellipse_publisher->publish(ellipse_marker);

      // Publish orbital points (periapsis, apoapsis, nodes)
      publishOrbitalPoints(coe_current);
    }

    void publishOrbitalPoints(const OrbitalElements& coe) {
      visualization_msgs::msg::MarkerArray marker_array;
      
      // Periapsis marker (theta = 0)
      double r_peri = coe.a * (1 - coe.e);
      Vector3d peri_peri(r_peri, 0, 0);
      Vector3d peri_inertial = perifocalToInertial(peri_peri, coe.i, coe.Omega, coe.omega);
      
      visualization_msgs::msg::Marker peri_marker;
      peri_marker.header.frame_id = "world";
      peri_marker.header.stamp = this->now();
      peri_marker.ns = "orbital_points";
      peri_marker.id = 0;
      peri_marker.type = visualization_msgs::msg::Marker::SPHERE;
      peri_marker.action = visualization_msgs::msg::Marker::ADD;
      peri_marker.pose.position.x = peri_inertial(0) / 1000000.0;
      peri_marker.pose.position.y = peri_inertial(1) / 1000000.0;
      peri_marker.pose.position.z = peri_inertial(2) / 1000000.0;
      peri_marker.scale.x = 0.3;
      peri_marker.scale.y = 0.3;
      peri_marker.scale.z = 0.3;
      peri_marker.color.r = 1.0;
      peri_marker.color.g = 0.0;
      peri_marker.color.b = 0.0;
      peri_marker.color.a = 1.0;
      marker_array.markers.push_back(peri_marker);

      // Apoapsis marker (theta = PI)
      double r_apo = coe.a * (1 + coe.e);
      Vector3d apo_peri(-r_apo, 0, 0);
      Vector3d apo_inertial = perifocalToInertial(apo_peri, coe.i, coe.Omega, coe.omega);
      
      visualization_msgs::msg::Marker apo_marker;
      apo_marker.header.frame_id = "world";
      apo_marker.header.stamp = this->now();
      apo_marker.ns = "orbital_points";
      apo_marker.id = 1;
      apo_marker.type = visualization_msgs::msg::Marker::SPHERE;
      apo_marker.action = visualization_msgs::msg::Marker::ADD;
      apo_marker.pose.position.x = apo_inertial(0) / 1000000.0;
      apo_marker.pose.position.y = apo_inertial(1) / 1000000.0;
      apo_marker.pose.position.z = apo_inertial(2) / 1000000.0;
      apo_marker.scale.x = 0.3;
      apo_marker.scale.y = 0.3;
      apo_marker.scale.z = 0.3;
      apo_marker.color.r = 0.0;
      apo_marker.color.g = 0.0;
      apo_marker.color.b = 1.0;
      apo_marker.color.a = 1.0;
      marker_array.markers.push_back(apo_marker);

      // Ascending node (where orbit crosses xy-plane going up)
      // Only show if orbit is inclined
      if (coe.i > 1e-3 && coe.i < PI - 1e-3) {
        // Ascending node is at the intersection of orbital plane with xy-plane
        // In perifocal frame, we need to find where z=0 in inertial frame
        // The ascending node direction is along the node vector n = k x h
        Vector3d h = r.cross(v);
        Vector3d k(0, 0, 1);
        Vector3d n = k.cross(h);
        double n_mag = n.norm();
        
        if (n_mag > 1e-10) {
          Vector3d ascend_node = n / n_mag;
          
          // Find radius at ascending node
          // Ascending node is at angle = -omega from periapsis in perifocal frame
          double theta_an = -coe.omega;
          double r_an = (coe.a * (1 - coe.e * coe.e)) / (1 + coe.e * cos(theta_an));
          
          Vector3d an_pos = r_an * ascend_node;
          
          visualization_msgs::msg::Marker an_marker;
          an_marker.header.frame_id = "world";
          an_marker.header.stamp = this->now();
          an_marker.ns = "orbital_points";
          an_marker.id = 2;
          an_marker.type = visualization_msgs::msg::Marker::SPHERE;
          an_marker.action = visualization_msgs::msg::Marker::ADD;
          an_marker.pose.position.x = an_pos(0) / 1000000.0;
          an_marker.pose.position.y = an_pos(1) / 1000000.0;
          an_marker.pose.position.z = an_pos(2) / 1000000.0;
          an_marker.scale.x = 0.25;
          an_marker.scale.y = 0.25;
          an_marker.scale.z = 0.25;
          an_marker.color.r = 1.0;
          an_marker.color.g = 1.0;
          an_marker.color.b = 0.0;
          an_marker.color.a = 1.0;
          marker_array.markers.push_back(an_marker);

          // Descending node (opposite direction)
          Vector3d descend_node = -ascend_node;
          double theta_dn = PI - coe.omega;
          double r_dn = (coe.a * (1 - coe.e * coe.e)) / (1 + coe.e * cos(theta_dn));
          
          Vector3d dn_pos = r_dn * descend_node;
          
          visualization_msgs::msg::Marker dn_marker;
          dn_marker.header.frame_id = "world";
          dn_marker.header.stamp = this->now();
          dn_marker.ns = "orbital_points";
          dn_marker.id = 3;
          dn_marker.type = visualization_msgs::msg::Marker::SPHERE;
          dn_marker.action = visualization_msgs::msg::Marker::ADD;
          dn_marker.pose.position.x = dn_pos(0) / 1000000.0;
          dn_marker.pose.position.y = dn_pos(1) / 1000000.0;
          dn_marker.pose.position.z = dn_pos(2) / 1000000.0;
          dn_marker.scale.x = 0.25;
          dn_marker.scale.y = 0.25;
          dn_marker.scale.z = 0.25;
          dn_marker.color.r = 1.0;
          dn_marker.color.g = 0.5;
          dn_marker.color.b = 0.0;
          dn_marker.color.a = 1.0;
          marker_array.markers.push_back(dn_marker);
        }
      }

      orbital_points_publisher->publish(marker_array);
    }

    void control_callback(){
      dt = this->get_parameter("time_step").as_double();
      rk4Step(r, v, dt, mu, a_thrust_mag);

      current_state_msg.header.stamp = this->now();
      current_state_msg.header.frame_id = "world";
      current_state_msg.pose.position.x = r(0) / 1000000.0;  // Scale down for visualization
      current_state_msg.pose.position.y = r(1) / 1000000.0;
      current_state_msg.pose.position.z = r(2) / 1000000.0;

      state_publisher->publish(current_state_msg);

      step++;
    }


    // Get thrust direction 
    Vector3d getThrustDirection(const Vector3d& r, const Vector3d& v) {
      double v_mag = v.norm();
      if (v_mag > 1e-10) {
        // Prograde (along velocity)
        if ( thrust_msg.linear.x > 0.1 ) { 
          return v / v_mag;
        } 
        // Retrograde (against velocity)
        else if ( thrust_msg.linear.x < -0.1 ) {
          return -v / v_mag;
        }

        // Radial (along position)
        if ( thrust_msg.linear.y > 0.1 ) {
          return r / r.norm();
        } 
        // Anti-radial
        else if ( thrust_msg.linear.y < -0.1 ) {
          return -r / r.norm();
        }

        // Normal (perpendicular to orbital plane, along angular momentum)
        // This will rotate the orbit plane!
        if ( thrust_msg.linear.z > 0.1 ) {
          Vector3d h = r.cross(v);
          return h / h.norm();
        } 
        // Anti-normal (opposite to angular momentum)
        else if ( thrust_msg.linear.z < -0.1 ) { 
          Vector3d h = r.cross(v);
          return -h / h.norm();
        }
      }
      return Vector3d::Zero();
    }

    struct StateDerivative {
      Vector3d dr_dt;
      Vector3d dv_dt;
    };

    StateDerivative equationsOfMotion(
      const Vector3d& r, 
      const Vector3d& v, 
      double mu,
      double a_thrust_mag
    ) {
      StateDerivative deriv;
    
      // 1. Gravitational acceleration: a_grav = -mu * r / |r|^3
      double r_mag = r.norm();
    
      if (r_mag < 1.0) {  // Safety check (1 meter from center)
        deriv.dr_dt = Vector3d::Zero();
        deriv.dv_dt = Vector3d::Zero();
        return deriv;
      }
    
      Vector3d a_gravity = -mu / (r_mag * r_mag * r_mag) * r;
    
      // 2. Thrust acceleration
      Vector3d a_thrust = Vector3d::Zero();
    
      if (burning) {
        a_thrust = a_thrust_mag * getThrustDirection(r, v);
      } else {
        a_thrust = Vector3d::Zero();
      }
    
      // 3. Total acceleration
      Vector3d a_total = a_gravity + a_thrust;
    
      deriv.dr_dt = v;
      deriv.dv_dt = a_total;
    
      return deriv;
    }

    OrbitalElements computeCOE(const Vector3d& r, const Vector3d& v, double mu) {
      OrbitalElements coe;
    
      // 1. Angular momentum vector
      Vector3d h = r.cross(v);
      double h_mag = h.norm();
    
      // 2. Eccentricity vector and magnitude
      double r_mag = r.norm();
      double v_mag = v.norm();
    
      // e_vec = [(v^2 - mu/r)*r - (r·v)*v] / mu
      Vector3d e_vec = ((v_mag * v_mag - mu / r_mag) * r - r.dot(v) * v) / mu;
      coe.e = e_vec.norm();
    
      // 3. Specific mechanical energy and semi-major axis
      double E = 0.5 * v_mag * v_mag - mu / r_mag;
    
      if (abs(coe.e - 1.0) > 1e-6) {  // Elliptic or hyperbolic
          coe.a = -mu / (2.0 * E);
      } else {  // Parabolic
          coe.a = INFINITY;
      }
    
      // 4. Inclination
      coe.i = acos(std::clamp(h(2) / h_mag, -1.0, 1.0));  // h.z / |h|
    
      // 5. Node vector and RAAN (Right Ascension of Ascending Node)
      Vector3d k(0, 0, 1);
      Vector3d n = k.cross(h);
      double n_mag = n.norm();
      
      if (n_mag > 1e-10) {  // Non-equatorial orbit
          coe.Omega = acos(std::clamp(n(0) / n_mag, -1.0, 1.0));  // n.x / |n|
          if (n(1) < 0) {  // Quadrant check
              coe.Omega = 2.0 * PI - coe.Omega;
          }
      } else {  // Equatorial orbit
          coe.Omega = 0.0;
      }
      
      // 6. Argument of periapsis
      if (coe.e > 1e-8 && n_mag > 1e-10) {
          coe.omega = acos(std::clamp(n.dot(e_vec) / (n_mag * coe.e), -1.0, 1.0));
          if (e_vec(2) < 0) {  // Quadrant check
              coe.omega = 2.0 * PI - coe.omega;
          }
      } else if (coe.e > 1e-8) {
          // Inclined orbit but near-zero inclination: use x-axis as reference
          coe.omega = atan2(e_vec(1), e_vec(0));
          if (coe.omega < 0) coe.omega += 2.0 * PI;
      } else {
          coe.omega = 0.0;
      }
      
      // 7. True anomaly
      if (coe.e > 1e-8) {
          // Non-circular orbit
          coe.nu = acos(std::clamp(e_vec.dot(r) / (coe.e * r_mag), -1.0, 1.0));
          if (r.dot(v) < 0) {  // Quadrant check
              coe.nu = 2.0 * PI - coe.nu;
          }
      } else {
          // Circular orbit: measure angle from ascending node
          if (n_mag > 1e-10) {
              coe.nu = acos(std::clamp(n.dot(r) / (n_mag * r_mag), -1.0, 1.0));
              if (r(2) < 0) {  // Quadrant check
                  coe.nu = 2.0 * PI - coe.nu;
              }
          } else {
              // Circular equatorial: measure from x-axis
              coe.nu = atan2(r(1), r(0));
              if (coe.nu < 0) coe.nu += 2.0 * PI;
          }
      }
      
      return coe;
    }

    // Transform from perifocal (orbital plane) to inertial frame
    Vector3d perifocalToInertial(const Vector3d& pos_peri, double i, double Omega, double omega) {
      // For equatorial orbits (i ≈ 0), use simplified rotation
      if (i < 1e-6) {
        // Just rotate by omega around z-axis
        Matrix3d R_z;
        R_z << cos(omega), -sin(omega), 0,
               sin(omega),  cos(omega), 0,
               0,           0,           1;
        return R_z * pos_peri;
      }
      
      // Rotation matrix from perifocal to inertial frame
      // R = R_z(Omega) * R_x(i) * R_z(omega)
      
      // R_z(omega)
      Matrix3d R_z_omega;
      R_z_omega << cos(omega), -sin(omega), 0,
                   sin(omega),  cos(omega), 0,
                   0,           0,          1;
      
      // R_x(i)
      Matrix3d R_x_i;
      R_x_i << 1,  0,       0,
               0,  cos(i), -sin(i),
               0,  sin(i),  cos(i);
      
      // R_z(Omega)
      Matrix3d R_z_Omega;
      R_z_Omega << cos(Omega), -sin(Omega), 0,
                   sin(Omega),  cos(Omega), 0,
                   0,           0,           1;
      
      // Complete transformation
      Matrix3d R_total = R_z_Omega * R_x_i * R_z_omega;
      
      return R_total * pos_peri;
    }

    // ========================================================================
    // RK4 INTEGRATION STEP
    // ========================================================================

    void rk4Step(
        Vector3d& r, 
        Vector3d& v, 
        double dt,
        double mu,
        double a_thrust_mag
    ) {
        // K1
        StateDerivative k1 = equationsOfMotion(r, v, mu, a_thrust_mag);
        
        // K2
        Vector3d r2 = r + k1.dr_dt * (dt / 2.0);
        Vector3d v2 = v + k1.dv_dt * (dt / 2.0);
        StateDerivative k2 = equationsOfMotion(r2, v2, mu, a_thrust_mag);
        
        // K3
        Vector3d r3 = r + k2.dr_dt * (dt / 2.0);
        Vector3d v3 = v + k2.dv_dt * (dt / 2.0);
        StateDerivative k3 = equationsOfMotion(r3, v3, mu, a_thrust_mag);
        
        // K4
        Vector3d r4 = r + k3.dr_dt * dt;
        Vector3d v4 = v + k3.dv_dt * dt;
        StateDerivative k4 = equationsOfMotion(r4, v4, mu, a_thrust_mag);
        
        // Weighted average
        Vector3d dr_avg = (k1.dr_dt + 2.0*k2.dr_dt + 2.0*k3.dr_dt + k4.dr_dt) / 6.0;
        Vector3d dv_avg = (k1.dv_dt + 2.0*k2.dv_dt + 2.0*k3.dv_dt + k4.dv_dt) / 6.0;
        
        // Update state
        r += dr_avg * dt;
        v += dv_avg * dt;
    }

  private:

    geometry_msgs::msg::PoseStamped current_state_msg;
    geometry_msgs::msg::Twist thrust_msg;
    int step;
    const double mu = 3.986004418e14;  // Earth GM (m^3/s^2)
    
    // Simulation parameters
    double dt = 0.01;      // Time step (seconds)
    
    OrbitalElements coe_initial; 
    
    // Initial state (LEO example: ~400 km altitude)
    Vector3d r;
    Vector3d v;
    double t = 0.0;
    bool burning = false;
    
    // Thrust parameters (60-second burn at t=1800s)
    const double thrust_magnitude = 500.0;   // Newtons
    const double spacecraft_mass = 1000.0;   // kg
    const double a_thrust_mag = thrust_magnitude / spacecraft_mass;
    const double thrust_start_time = 1800.0;
    const double thrust_end_time = 1860.0;

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::TimerBase::SharedPtr viz_timer;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ellipse_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr orbital_points_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr thrust_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr burning_subscriber;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MechanicsNode>());
  rclcpp::shutdown();
  return 0;
}
