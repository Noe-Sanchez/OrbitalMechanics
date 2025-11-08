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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
      thrust_subscriber     = this->create_subscription<geometry_msgs::msg::TwistStamped>("sat_thrust", 10, std::bind(&MechanicsNode::sat_thrust_callback, this, std::placeholders::_1));
      burning_subscriber    = this->create_subscription<std_msgs::msg::Bool>("sat_burning", 10, std::bind(&MechanicsNode::sat_burning_callback, this, std::placeholders::_1));

      // Publishers
      state_publisher    = this->create_publisher<geometry_msgs::msg::PoseStamped>("sat_state", 10);
      ellipse_publisher  = this->create_publisher<visualization_msgs::msg::Marker>("orbit_ellipse", 10);

      // Make 0.1s timer
      control_timer = this->create_wall_timer(10ms, std::bind(&MechanicsNode::control_callback, this));
      viz_timer     = this->create_wall_timer(200ms, std::bind(&MechanicsNode::viz_callback, this));
  
      r << 7000000.0, 0.0, 0.0;
      v << 0.0, 7546.0, 0.0;

      step = 0;
      coe_initial = computeCOE(r, v, mu);
    }

    void sat_thrust_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
      thrust_msg = *msg;
    }
    void sat_burning_callback(const std_msgs::msg::Bool::SharedPtr msg){
      burning = msg->data;
    }
    
    // Visualization current orbit
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

				// Position in orbital plane
				double x_orb = r_mag * cos(theta);
				double y_orb = r_mag * sin(theta);
				double z_orb = 0.0;

				// Rotate to inertial frame
				// (Skipping rotation for simplicity; assuming circular equatorial orbit)
				geometry_msgs::msg::Point p;
				p.x = x_orb / 1000000.0;  // Scale down for visualization
				p.y = y_orb / 1000000.0;
				p.z = z_orb / 1000000.0;

				ellipse_marker.points.push_back(p);
			}

			ellipse_publisher->publish(ellipse_marker);
		}

    void control_callback(){
      dt = this->get_parameter("time_step").as_double();
      //rk4Step(r, v, t, dt, mu, a_thrust_mag, thrust_start_time, thrust_end_time);
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
    Vector3d getThrustDirection(const Vector3d& v) {
      double v_mag = v.norm();
      if (v_mag > 1e-10) {
	if ( thrust_msg.twist.linear.x > 0.1 ) { 
	  return v / v_mag;  // Prograde (along velocity)
	} else if ( thrust_msg.twist.linear.x < -0.1 ) {
	  return -v / v_mag; // Retrograde (against velocity)
	}

	if ( thrust_msg.twist.linear.y > 0.1 ) {
	  return r / r.norm();  // Radial (along position)
	} else if ( thrust_msg.twist.linear.y < -0.1 ) {
	  return -r / r.norm(); // Anti-radial
	}

	// Normal (along angular momentum)
	if ( thrust_msg.twist.linear.z > 0.1 ) {
	  Vector3d h = r.cross(v);
	  return h / h.norm();
	} else if ( thrust_msg.twist.linear.z < -0.1 ) { 
	  Vector3d h = r.cross(v);
	  return -h / h.norm();
	}

	// Retrograde (against velocity)
	//return -v / v_mag;
	
	// Radial (along position)
	//return r / r.norm();

        //return v / v_mag;  // Prograde (along velocity)
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
      //double t,
      double mu,
      double a_thrust_mag
     // double thrust_start_time,
      //double thrust_end_time
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
    
      //if (t >= thrust_start_time && t < thrust_end_time) {
      //  Vector3d thrust_dir = getThrustDirection(v);
      //  a_thrust = a_thrust_mag * thrust_dir;
      //}
      if (burning) {
        a_thrust = a_thrust_mag * getThrustDirection(v);
      } else {
	a_thrust = Vector3d::Zero();
      }
    
      // 3. Total acceleration
      Vector3d a_total = a_gravity + a_thrust;
    
      deriv.dr_dt = v;
      deriv.dv_dt = a_total;
    
      return deriv;
    }


    struct OrbitalElements {
      double a;       // Semi-major axis
      double e;       // Eccentricity
      double i;       // Inclination (radians)
      double Omega;   // RAAN (radians)
      double omega;   // Argument of periapsis (radians)
      double nu;      // True anomaly (radians)
    };

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
      coe.i = acos(h(2) / h_mag);  // h.z / |h|
    
    // 5. Node vector and RAAN (Right Ascension of Ascending Node)
    Vector3d k(0, 0, 1);
    Vector3d n = k.cross(h);
    double n_mag = n.norm();
    
    if (n_mag > 1e-10) {  // Non-equatorial orbit
        coe.Omega = acos(n(0) / n_mag);  // n.x / |n|
        if (n(1) < 0) {  // Quadrant check
            coe.Omega = 2.0 * PI - coe.Omega;
        }
    } else {  // Equatorial orbit
        coe.Omega = 0.0;
    }
    
    // 6. Argument of periapsis
    if (coe.e > 1e-8 && n_mag > 1e-10) {
        coe.omega = acos(n.dot(e_vec) / (n_mag * coe.e));
        if (e_vec(2) < 0) {  // Quadrant check
            coe.omega = 2.0 * PI - coe.omega;
        }
    } else {
        coe.omega = 0.0;
    }
    
    // 7. True anomaly
    if (coe.e > 1e-8) {
        // Non-circular orbit
        coe.nu = acos(e_vec.dot(r) / (coe.e * r_mag));
        if (r.dot(v) < 0) {  // Quadrant check
            coe.nu = 2.0 * PI - coe.nu;
        }
    } else {
        // Circular orbit: measure angle from ascending node
        if (n_mag > 1e-10) {
            coe.nu = acos(n.dot(r) / (n_mag * r_mag));
            if (r(2) < 0) {  // Quadrant check
                coe.nu = 2.0 * PI - coe.nu;
            }
        } else {
            // Circular equatorial: measure from x-axis
            coe.nu = acos(r(0) / r_mag);
            if (r(1) < 0) {
                coe.nu = 2.0 * PI - coe.nu;
            }
        }
    }
    
    return coe;
}

// ========================================================================
// RK4 INTEGRATION STEP
// ========================================================================

void rk4Step(
    Vector3d& r, 
    Vector3d& v, 
    //double& t,
    double dt,
    double mu,
    double a_thrust_mag
    //double thrust_start_time,
    //double thrust_end_time
) {
    // K1
    //StateDerivative k1 = equationsOfMotion(r, v, t, mu, a_thrust_mag, 
    //                                       thrust_start_time, thrust_end_time);
    StateDerivative k1 = equationsOfMotion(r, v, mu, a_thrust_mag);
    
    // K2
    Vector3d r2 = r + k1.dr_dt * (dt / 2.0);
    Vector3d v2 = v + k1.dv_dt * (dt / 2.0);
    //StateDerivative k2 = equationsOfMotion(r2, v2, t + dt/2.0, mu, 
    //                                       a_thrust_mag, thrust_start_time, thrust_end_time);
    StateDerivative k2 = equationsOfMotion(r2, v2, mu, a_thrust_mag);
    
    // K3
    Vector3d r3 = r + k2.dr_dt * (dt / 2.0);
    Vector3d v3 = v + k2.dv_dt * (dt / 2.0);
    //StateDerivative k3 = equationsOfMotion(r3, v3, t + dt/2.0, mu, 
    //                                       a_thrust_mag, thrust_start_time, thrust_end_time);
    StateDerivative k3 = equationsOfMotion(r3, v3, mu, a_thrust_mag);
    
    // K4
    Vector3d r4 = r + k3.dr_dt * dt;
    Vector3d v4 = v + k3.dv_dt * dt;
    //StateDerivative k4 = equationsOfMotion(r4, v4, t + dt, mu, 
    //                                       a_thrust_mag, thrust_start_time, thrust_end_time);
    StateDerivative k4 = equationsOfMotion(r4, v4, mu, a_thrust_mag);
    
    // Weighted average
    Vector3d dr_avg = (k1.dr_dt + 2.0*k2.dr_dt + 2.0*k3.dr_dt + k4.dr_dt) / 6.0;
    Vector3d dv_avg = (k1.dv_dt + 2.0*k2.dv_dt + 2.0*k3.dv_dt + k4.dv_dt) / 6.0;
    
    // Update state
    r += dr_avg * dt;
    v += dv_avg * dt;
    //t += dt;
}

  private:

    geometry_msgs::msg::PoseStamped current_state_msg;
    geometry_msgs::msg::TwistStamped thrust_msg;
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
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr thrust_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr burning_subscriber;

    
    //rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_velocity_subscriber;

    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr uaux_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MechanicsNode>());
  rclcpp::shutdown();
  return 0;
}
