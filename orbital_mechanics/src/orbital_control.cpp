#include <chrono>
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Dense>

using namespace std::chrono_literals;

struct KeplerianElements {
    double a;      // Semi-major axis (m)
    double e;      // Eccentricity
    double i;      // Inclination (rad)
    double raan;   // Right Ascension of the Ascending Node (rad)
    double arg_p;  // Argument of Periapsis (rad)
    double nu;     // True Anomaly (rad)
};

class OrbitalController : public rclcpp::Node{
  public:
    OrbitalController() : Node("orbital_controller"){
      // 1. Initial State Setup
      current_elements_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      
      // Let's set a default target: Slightly higher orbit, circular, equatorial
      target_elements_ = {7000000.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      has_state_ = false;

      // 2. Publishers
      accel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("spacecraft/control_accel", 10);
      desired_orbit_pub_ = this->create_publisher<nav_msgs::msg::Path>("spacecraft/desired_orbit", 10);

      // 3. Subscribers
      state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>( "spacecraft/state", 10, std::bind(&OrbitalController::state_callback, this, std::placeholders::_1));
      elem_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("spacecraft/orbital_elements", 10, std::bind(&OrbitalController::elements_callback, this, std::placeholders::_1));
      target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("spacecraft/target_elements", 10, std::bind(&OrbitalController::target_callback, this, std::placeholders::_1));

      // 4. Control Loop (Running at 10Hz)
      timer_ = this->create_wall_timer(50ms, std::bind(&OrbitalController::control_loop, this));

      RCLCPP_INFO(this->get_logger(), "Orbital Controller started. Awaiting telemetry...");
    }

  private:
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

    // --- HELPER FUNCTION: Angle Wrapping ---
    // Wraps an angle to the [-pi, pi] range to calculate shortest-path rotational errors
    double wrapAngle(double angle){
      return std::atan2(std::sin(angle), std::cos(angle));
    }

    // --- HELPER FUNCTION: RTN to ECI Direction Cosine Matrix ---
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

    // --- CALLBACKS ---
    void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      r_eci_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
      v_eci_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
      has_state_ = true;
    }

    void elements_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      if (msg->data.size() >= 6) {
        current_elements_ = msg->data;
      }
    }

    void target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      if (msg->data.size() >= 6) {
        target_elements_ = msg->data;

        // Publish the analytical orbit path for visualization
        KeplerianElements elem;
        elem.a = target_elements_[0];
        elem.e = target_elements_[1];
        elem.i = target_elements_[2];
        elem.raan = target_elements_[3];
        elem.arg_p = target_elements_[4];
        elem.nu = target_elements_[5];
        nav_msgs::msg::Path orbit_path = generateAnalyticalOrbit(elem, this->get_clock()->now());

        desired_orbit_pub_->publish(orbit_path);
        RCLCPP_INFO(this->get_logger(), "New target orbital elements received.");
      }
    }

    // --- MAIN CONTROL LOOP ---
    void control_loop(){
      if (!has_state_) return;

      // 1. Calculate Error Vector (Target - Current)
      // Array mapping: [0]: a, [1]: e, [2]: i, [3]: RAAN, [4]: arg_p, [5]: nu
      std::vector<double> error(6, 0.0);
      
      // Linear/Magnitude errors
      error[0] = target_elements_[0] - current_elements_[0]; // Delta a (m)
      error[1] = target_elements_[1] - current_elements_[1]; // Delta e
      
      // Angular errors (wrapped to [-pi, pi])
      error[2] = wrapAngle(target_elements_[2] - current_elements_[2]); // Delta i
      error[3] = wrapAngle(target_elements_[3] - current_elements_[3]); // Delta RAAN
      error[4] = wrapAngle(target_elements_[4] - current_elements_[4]); // Delta arg_p
      error[5] = wrapAngle(target_elements_[5] - current_elements_[5]); // Delta nu
      
      std::cout << "Error Vector: [a: " << error[0] << " m, e: " << error[1] << ", i: " << error[2] << " rad, RAAN: " << error[3] << " rad, arg_p: " << error[4] << " rad, nu: " << error[5] << " rad]" << std::endl;

      // 2. COMPUTE CONTROL LAW (RTN FRAME)
      Eigen::Vector3d a_rtn(0.0, 0.0, 0.0); // Acceleration in [Radial, Transverse, Normal]
      
      // ==========================================
      // TODO: Implement Control Mathematics Here
      // Map 'error' array to 'a_rtn' components
      // ==========================================
      
      // 2. COMPUTE CONTROL LAW (RTN FRAME) USING GVE
        
      // Extract current state for readability
      double a = current_elements_[0];
      double e = current_elements_[1];
      double i = current_elements_[2];
      double arg_p = current_elements_[4];
      double nu = current_elements_[5];

      double mu_ = 3.986004418e14; 

      // Derived parameters
      double p = a * (1.0 - e * e);
      double h = std::sqrt(mu_ * p);
      double r_mag = p / (1.0 + e * std::cos(nu));
      double theta = arg_p + nu;

      // Singularity protection for circular (e=0) and equatorial (i=0) orbits
      //double e_safe = std::max(e, 1e-8);
      //double sin_i_safe = std::max(std::sin(i), 1e-8);

      auto safe_clamp = [](double val, double min_mag) {
          if (std::abs(val) < min_mag) {
              return std::signbit(val) ? -min_mag : min_mag;
          }
          return val;
      };

      double e_safe = safe_clamp(e, 1e-8);
      double sin_i_safe = safe_clamp(std::sin(i), 1e-8);

      // Construct the 6x3 GVE Control Matrix B(x)
      Eigen::Matrix<double, 6, 3> B;
      B.setZero();

      // da/dt
      B(0, 0) = (2.0 * a * a / h) * e * std::sin(nu);
      B(0, 1) = (2.0 * a * a / h) * (p / r_mag);
      
      // de/dt
      B(1, 0) = (p / h) * std::sin(nu);
      B(1, 1) = (1.0 / h) * ((p + r_mag) * std::cos(nu) + r_mag * e);
      
      // di/dt
      B(2, 2) = (r_mag * std::cos(theta)) / h;
      
      // dRAAN/dt
      B(3, 2) = (r_mag * std::sin(theta)) / (h * sin_i_safe);
      
      // darg_p/dt
      B(4, 0) = -(p / (h * e_safe)) * std::cos(nu);
      B(4, 1) = ((p + r_mag) / (h * e_safe)) * std::sin(nu);
      B(4, 2) = -(r_mag * std::sin(theta) * std::cos(i)) / (h * sin_i_safe);
      
      // dnu/dt (Control perturbation part only)
      B(5, 0) = (p / (h * e_safe)) * std::cos(nu);
      B(5, 1) = -((p + r_mag) / (h * e_safe)) * std::sin(nu);

      // Map error vector to Eigen format
      Eigen::Matrix<double, 6, 1> delta_x;
      for(int j = 0; j < 6; ++j) delta_x(j) = error[j];

      // Define Diagonal Gain Matrix K
      Eigen::Matrix<double, 6, 6> K;
      K.setIdentity();
      
      // Tuning weights: SMA error is in meters (large), angles are in radians (small).
      // You will need to heavily tune these relative to each other based on control authority.
      K(0, 0) = 5e-4; // a (Semi-major axis)
      K(1, 1) = 0.1;  // e (Eccentricity)
      K(2, 2) = 0.1;  // i (Inclination)
      //K(3, 3) = 0.1;  // RAAN
      //K(4, 4) = 0.1;  // arg_p
      // If targeting an equatorial orbit (i ~ 0), RAAN is undefined. Turn off RAAN control.
      K(3, 3) = (std::abs(target_elements_[2]) < 1e-3) ? 0.0 : 0.1; 
      // If targeting a circular orbit (e ~ 0), arg_p is undefined. Turn off arg_p control.
      K(4, 4) = (std::abs(target_elements_[1]) < 1e-3) ? 0.0 : 0.5;
      /*bool shape_converged = (std::abs(error[0]) < 100000.0) && (std::abs(error[1]) < 0.01);

      if (std::abs(target_elements_[1]) < 1e-3) {
        // Target is circular, arg_p is undefined
        K(4, 4) = 0.0; 
      } else if (shape_converged) {
        // Shape is matched. Spike the gain to force the least-squares solver to fix the rotation.
        K(4, 4) = 5.0; 
      } else {
        // Focus on fixing SMA and Eccentricity first.
        K(4, 4) = 0.0; 
      }
      */
      K(5, 5) = 0.0;  // nu (Often left 0 to allow the spacecraft to just coast into phase)

      // Desired element rates
      Eigen::Matrix<double, 6, 1> x_dot_desired = K * delta_x;

      // Print B
      std::cout << "Control Matrix B:\n" << B << std::endl;

      // Solve for a_rtn: Least squares solution to B * u = x_dot_desired
      a_rtn = B.completeOrthogonalDecomposition().solve(x_dot_desired);

      // Clamp a_rtn 
      a_rtn.x() = std::clamp(a_rtn.x(), -10.0, 10.0);
      a_rtn.y() = std::clamp(a_rtn.y(), -10.0, 10.0);
      a_rtn.z() = std::clamp(a_rtn.z(), -10.0, 10.0);

      // 3. Transform RTN thrust command to ECI frame
      Eigen::Matrix3d R_ECI_RTN = calculateRtnToEci(r_eci_, v_eci_);
      Eigen::Vector3d a_eci = R_ECI_RTN * a_rtn;

      // 4. Publish ECI control effort back to the simulation plant
      geometry_msgs::msg::Vector3 accel_msg;
      accel_msg.x = a_eci.x();
      accel_msg.y = a_eci.y();
      accel_msg.z = a_eci.z();
      accel_pub_->publish(accel_msg);
    }

    // ROS 2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr elem_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_orbit_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State Variables
    Eigen::Vector3d r_eci_;
    Eigen::Vector3d v_eci_;
    bool has_state_;

    std::vector<double> current_elements_;
    std::vector<double> target_elements_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbitalController>());
  rclcpp::shutdown();
  return 0;
}
