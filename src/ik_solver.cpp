#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <cmath>
#include <Eigen/Dense>

// Robot geometry (all in meters)
const double L1 = 0.4;                  // upper arm length
const double L2 = 0.45;                 // forearm length
const double CLAW_CENTER_DIST = 0.12;   // wrist → claw center
const double BASE_OFFSET_X = 0.05;      // base offset in world X (shoulder position)

// Motion parameters
const double JOINT_SPEED = 3.0;
const double DT = 0.05;

// Reachability & capture (3D distance)
const double MAX_RADIAL_REACH = L1 + L2 + CLAW_CENTER_DIST;
const double MIN_RADIAL_REACH = 0.10;
const double CAPTURE_DIST = 0.01;       // 1 cm
const double TOL_POS = 1e-4;

// URDF orientation offset (if the claw visual is rotated)
const double CLAW_URDF_OFFSET = M_PI/4; // adjust as needed

class IKSolver : public rclcpp::Node
{
public:
  IKSolver() : Node("ik_solver_3d"),
               q0_(0.0), q1_(0.0), q2_(0.0), q3_(0.0),
               tq0_(0.0), tq1_(0.0), tq2_(0.0), tq3_(0.0),
               filtered_tx_(0.5), filtered_ty_(0.0), filtered_tz_(0.0),
               alpha_filter_(0.3),
               print_counter_(0),
               captured_(false),
               last_capture_state_(false)
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);

    marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "ik_target", this);
    setup_interactive_marker();

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_position", 10,
      std::bind(&IKSolver::target_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&IKSolver::update, this));

    RCLCPP_INFO(this->get_logger(), "3D IK Solver ready (base yaw + 2-link arm).");
  }

private:
  // Forward kinematics: compute claw center (x,y,z) and world claw angle
  void fk_center(double q0, double q1, double q2, double q3,
                 double& cx, double& cy, double& cz, double& world_angle)
  {
    // q0 = base yaw (around Z)
    // q1 = shoulder pitch (around Y)
    // q2 = elbow pitch (around Y)
    // q3 = wrist pitch (around Y)
    double a1 = q1;
    double a2 = q1 + q2;
    double a3 = q1 + q2 + q3;

    // Position in the arm's local XY plane (X forward, Z up)
    double wrist_x_local = L1 * std::cos(a1) + L2 * std::cos(a2);
    double wrist_z_local = L1 * std::sin(a1) + L2 * std::sin(a2);
    double claw_x_local = wrist_x_local + CLAW_CENTER_DIST * std::cos(a3);
    double claw_z_local = wrist_z_local + CLAW_CENTER_DIST * std::sin(a3);

    // Rotate by base yaw (q0) to get world coordinates
    cx = (claw_x_local) * std::cos(q0) + BASE_OFFSET_X;
    cy = (claw_x_local) * std::sin(q0);
    cz = claw_z_local;

    world_angle = a3;   // claw pitch angle (in the arm's plane)
  }

  // 3D position Jacobian (3x4) using damped pseudo-inverse
  bool solve_position(double tx, double ty, double tz,
                      double& q0, double& q1, double& q2, double& q3)
  {
    const int    MAX_ITER = 500;
    const double ALPHA    = 0.3;
    const double LAMBDA   = 0.01;

    for (int i = 0; i < MAX_ITER; ++i) {
      double cx, cy, cz, a3;
      fk_center(q0, q1, q2, q3, cx, cy, cz, a3);

      double err_x = tx - cx;
      double err_y = ty - cy;
      double err_z = tz - cz;
      if (std::hypot(err_x, err_y, err_z) < TOL_POS)
        return true;

      double a1 = q1;
      double a2 = q1 + q2;
      double a3_val = q1 + q2 + q3;

      double x_local = L1*std::cos(a1) + L2*std::cos(a2) + CLAW_CENTER_DIST*std::cos(a3_val);

      double dx_dq1 = -L1*std::sin(a1) - L2*std::sin(a2) - CLAW_CENTER_DIST*std::sin(a3_val);
      double dz_dq1 =  L1*std::cos(a1) + L2*std::cos(a2) + CLAW_CENTER_DIST*std::cos(a3_val);
      double dx_dq2 = -L2*std::sin(a2) - CLAW_CENTER_DIST*std::sin(a3_val);
      double dz_dq2 =  L2*std::cos(a2) + CLAW_CENTER_DIST*std::cos(a3_val);
      double dx_dq3 = -CLAW_CENTER_DIST*std::sin(a3_val);
      double dz_dq3 =  CLAW_CENTER_DIST*std::cos(a3_val);

      double cos_q0 = std::cos(q0);
      double sin_q0 = std::sin(q0);

      Eigen::Matrix<double, 3, 4> J;
      J(0,0) = -x_local * sin_q0;
      J(1,0) =  x_local * cos_q0;
      J(2,0) =  0.0;

      J(0,1) = cos_q0 * dx_dq1;
      J(1,1) = sin_q0 * dx_dq1;
      J(2,1) = dz_dq1;

      J(0,2) = cos_q0 * dx_dq2;
      J(1,2) = sin_q0 * dx_dq2;
      J(2,2) = dz_dq2;

      J(0,3) = cos_q0 * dx_dq3;
      J(1,3) = sin_q0 * dx_dq3;
      J(2,3) = dz_dq3;

      Eigen::Vector3d err(err_x, err_y, err_z);
      Eigen::Matrix<double, 4, 3> Jt = J.transpose();
      Eigen::Matrix3d JJt = J * Jt;
      JJt.diagonal().array() += LAMBDA;

      Eigen::Vector4d dq = Jt * JJt.ldlt().solve(err);
      q0 += ALPHA * dq(0);
      q1 += ALPHA * dq(1);
      q2 += ALPHA * dq(2);
      q3 += ALPHA * dq(3);
    }
    return false;
  }

  void solve_ik(double tx, double ty, double tz)
  {
    double px = tx - BASE_OFFSET_X;
    double py = ty;
    double pz = tz;

    double radial_dist = std::hypot(px, py);
    double total_dist = std::hypot(radial_dist, pz);
    double max_reach = L1 + L2 + CLAW_CENTER_DIST;

    if (total_dist > max_reach + 1e-3) {
      static int warn_far = 0;
      if (warn_far++ % 30 == 0)
        RCLCPP_WARN(this->get_logger(), "Target too far (dist=%.3f > %.3f). Arm frozen.", total_dist, max_reach);
      return;
    }
    if (radial_dist < MIN_RADIAL_REACH) {
      static int warn_near = 0;
      if (warn_near++ % 30 == 0)
        RCLCPP_WARN(this->get_logger(), "Target too close (radial dist=%.3f < %.3f). Arm frozen.", radial_dist, MIN_RADIAL_REACH);
      return;
    }

    double best_q0 = q0_, best_q1 = q1_, best_q2 = q2_, best_q3 = q3_;
    bool ok = solve_position(tx, ty, tz, best_q0, best_q1, best_q2, best_q3);
    if (!ok) {
      std::vector<std::array<double,4>> seeds = {
        { 0.0,   0.0,  0.0,  0.0},
        { std::atan2(py, px),  0.0,  0.0,  0.0},
        { std::atan2(py, px),  0.5, -0.5,  0.0},
        { std::atan2(py, px), -0.5,  0.5,  0.0},
        { std::atan2(py, px),  0.0,  0.8,  0.0},
      };
      for (auto& s : seeds) {
        double q0 = s[0], q1 = s[1], q2 = s[2], q3 = s[3];
        if (solve_position(tx, ty, tz, q0, q1, q2, q3)) {
          best_q0 = q0; best_q1 = q1; best_q2 = q2; best_q3 = q3;
          ok = true;
          break;
        }
      }
    }
    if (!ok) return;

    double wx = L1*std::cos(best_q1) + L2*std::cos(best_q1+best_q2);
    double wz = L1*std::sin(best_q1) + L2*std::sin(best_q1+best_q2);
    double cos0 = std::cos(best_q0);
    double sin0 = std::sin(best_q0);
    double wrist_world_x = wx * cos0 + BASE_OFFSET_X;
    double wrist_world_y = wx * sin0;
    double wrist_world_z = wz;
    double dx = tx - wrist_world_x;
    double dy = ty - wrist_world_y;
    double dz = tz - wrist_world_z;

    double desired_pitch_angle = std::atan2(dz, std::hypot(dx, dy));
    double forearm_angle = best_q1 + best_q2;
    double raw_q3 = desired_pitch_angle - forearm_angle + CLAW_URDF_OFFSET;
    best_q3 = raw_q3; 

    tq0_ = best_q0;
    tq1_ = best_q1;
    tq2_ = best_q2;
    tq3_ = best_q3;
  }

  void setup_interactive_marker()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.name = "target";
    int_marker.description = "Drag to move arm";
    int_marker.pose.position.x = 0.5;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    int_marker.scale = 0.15;

    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = 0.08;
    sphere.scale.y = 0.08;
    sphere.scale.z = 0.08;
    sphere.color.r = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 0.0;
    sphere.color.a = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl vis_control;
    vis_control.always_visible = true;
    vis_control.markers.push_back(sphere);
    int_marker.controls.push_back(vis_control);

    visualization_msgs::msg::InteractiveMarkerControl move_xy;
    move_xy.name = "move_xy";
    move_xy.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    move_xy.orientation.w = 1.0;
    move_xy.orientation.x = 0.0;
    move_xy.orientation.y = 0.0;
    move_xy.orientation.z = 0.0;
    int_marker.controls.push_back(move_xy);

    visualization_msgs::msg::InteractiveMarkerControl move_xz;
    move_xz.name = "move_xz";
    move_xz.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    move_xz.orientation.w = std::cos(M_PI/4.0);
    move_xz.orientation.x = 0.0;
    move_xz.orientation.y = std::cos(M_PI/4.0);
    move_xz.orientation.z = 0.0;
    int_marker.controls.push_back(move_xz);

    marker_server_->insert(int_marker,
      std::bind(&IKSolver::marker_feedback, this, std::placeholders::_1));
    marker_server_->applyChanges();

    q0_ = 0.0; q1_ = 0.0; q2_ = 0.5; q3_ = 0.0;
    solve_ik(0.5, 0.0, 0.0);
  }

  void marker_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    double x = feedback->pose.position.x;
    double y = feedback->pose.position.y;
    double z = feedback->pose.position.z;
    captured_ = false;
    filtered_tx_ = alpha_filter_ * x + (1.0 - alpha_filter_) * filtered_tx_;
    filtered_ty_ = alpha_filter_ * y + (1.0 - alpha_filter_) * filtered_ty_;
    filtered_tz_ = alpha_filter_ * z + (1.0 - alpha_filter_) * filtered_tz_;
    solve_ik(filtered_tx_, filtered_ty_, filtered_tz_);
    publish_target_sphere(x, y, z);
  }

  void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    captured_ = false;
    filtered_tx_ = alpha_filter_ * msg->x + (1.0 - alpha_filter_) * filtered_tx_;
    filtered_ty_ = alpha_filter_ * msg->y + (1.0 - alpha_filter_) * filtered_ty_;
    filtered_tz_ = alpha_filter_ * msg->z + (1.0 - alpha_filter_) * filtered_tz_;
    solve_ik(filtered_tx_, filtered_ty_, filtered_tz_);
    publish_target_sphere(msg->x, msg->y, msg->z);
  }

  void publish_target_sphere(double x, double y, double z)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = this->now();
    m.ns = "target";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.06;
    m.color.r = captured_ ? 0.0 : 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.9;
    marker_pub_->publish(m);
  }

  void update()
  {
    if (captured_) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->now();
      msg.name = {"base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch",
                  "claw_left_joint", "claw_right_joint"};
      msg.position = {q0_, q1_, q2_, q3_, 0.0, 0.0};
      joint_pub_->publish(msg);
      if (++print_counter_ >= 20) {
        print_counter_ = 0;
        double cx, cy, cz, ang;
        fk_center(q0_, q1_, q2_, q3_, cx, cy, cz, ang);
        RCLCPP_INFO(this->get_logger(),
          "[CAPTURED] center=(%.3f, %.3f, %.3f)  target=(%.3f, %.3f, %.3f)  err=%.4f",
          cx, cy, cz, filtered_tx_, filtered_ty_, filtered_tz_,
          std::hypot(cx - filtered_tx_, cy - filtered_ty_, cz - filtered_tz_));
      }
      return;
    }

    double max_step = JOINT_SPEED * DT;
    q0_ += std::clamp(tq0_ - q0_, -max_step, max_step);
    q1_ += std::clamp(tq1_ - q1_, -max_step, max_step);
    q2_ += std::clamp(tq2_ - q2_, -max_step, max_step);
    q3_ += std::clamp(tq3_ - q3_, -max_step, max_step);

    if (std::isnan(q0_) || std::isnan(q1_) || std::isnan(q2_) || std::isnan(q3_)) {
      RCLCPP_ERROR(this->get_logger(), "NaN in current angles! Resetting to zero.");
      q0_ = 0.0; q1_ = 0.0; q2_ = 0.5; q3_ = 0.0;
      tq0_ = q0_; tq1_ = q1_; tq2_ = q2_; tq3_ = q3_;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = {"base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch",
                "claw_left_joint", "claw_right_joint"};
    msg.position = {q0_, q1_, q2_, q3_, 0.0, 0.0};
    joint_pub_->publish(msg);

    double cx, cy, cz, ang;
    fk_center(q0_, q1_, q2_, q3_, cx, cy, cz, ang);
    double err = std::hypot(cx - filtered_tx_, cy - filtered_ty_, cz - filtered_tz_);
    bool new_capture = (err < CAPTURE_DIST);
    if (new_capture != last_capture_state_) {
      last_capture_state_ = new_capture;
      captured_ = new_capture;
      if (captured_)
        RCLCPP_INFO(this->get_logger(), "*** TARGET CAPTURED (center) *** err=%.4f", err);
      else
        RCLCPP_INFO(this->get_logger(), "*** TARGET LOST *** err=%.4f", err);
    }

    if (++print_counter_ >= 20) {
      print_counter_ = 0;
      RCLCPP_INFO(this->get_logger(),
        "[DEBUG] q0=%.3f  q1=%.3f  q2=%.3f  q3=%.3f  (deg: %.1f %.1f %.1f %.1f)\n"
        "        center=(%.3f, %.3f, %.3f)\n"
        "        target=(%.3f, %.3f, %.3f)  center_err=%.4f",
        q0_, q1_, q2_, q3_,
        q0_*180.0/M_PI, q1_*180.0/M_PI, q2_*180.0/M_PI, q3_*180.0/M_PI,
        cx, cy, cz,
        filtered_tx_, filtered_ty_, filtered_tz_, err);
    }
  }

  double q0_, q1_, q2_, q3_;      // current joint angles
  double tq0_, tq1_, tq2_, tq3_;  // target joint angles
  double filtered_tx_, filtered_ty_, filtered_tz_;
  const double alpha_filter_;
  int    print_counter_;
  bool   captured_;
  bool   last_capture_state_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    joint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr    target_sub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKSolver>());
  rclcpp::shutdown();
  return 0;
}