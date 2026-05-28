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

const double L1 = 0.4;
const double L2 = 0.45;
const double L3 = 0.05;               
const double CLAW_CENTER_DIST = 0.12;  
const double BASE_OFFSET_X = 0.05;

const double JOINT_SPEED = 3.0;
const double DT = 0.05;

const double MAX_REACH = L1 + L2 + CLAW_CENTER_DIST;
const double MIN_REACH = 0.10;
const double CAPTURE_DIST = 0.01;      
const double TOL_POS = 1e-4;
const double TOL_ANG = 0.005;          

const double CLAW_URDF_OFFSET = M_PI/4;  

class IKSolver : public rclcpp::Node
{
public:
  IKSolver() : Node("ik_solver"),
               q1_(0.0), q2_(0.0), q3_(0.0),
               tq1_(0.0), tq2_(0.0), tq3_(0.0),
               filtered_tx_(0.5), filtered_ty_(0.0),
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

    RCLCPP_INFO(this->get_logger(), "IK Solver ready (2x3 Jacobian + orientation decoupled).");
  }

private:
  void fk_center(double q1, double q2, double q3, double& cx, double& cy, double& angle)
  {
    double a12 = q1 + q2;
    double a123 = q1 + q2 + q3;
    double wrist_x = L1 * std::cos(q1) + L2 * std::cos(a12);
    double wrist_y = L1 * std::sin(q1) + L2 * std::sin(a12);
    cx = wrist_x + CLAW_CENTER_DIST * std::cos(a123);
    cy = wrist_y + CLAW_CENTER_DIST * std::sin(a123);
    angle = a123;
  }

  bool solve_position(double px, double py, double& q1, double& q2, double& q3)
  {
    const int    MAX_ITER = 500;
    const double ALPHA    = 0.3;
    const double LAMBDA   = 0.01;     

    for (int i = 0; i < MAX_ITER; ++i) {
      double cx, cy, a123;
      fk_center(q1, q2, q3, cx, cy, a123);

      double err_x = px - cx;
      double err_y = py - cy;
      if (std::hypot(err_x, err_y) < TOL_POS)
        return true;

      double a12 = q1 + q2;
      Eigen::Matrix<double, 2, 3> J;
      J(0,0) = -L1*std::sin(q1) -L2*std::sin(a12) -CLAW_CENTER_DIST*std::sin(a123);
      J(0,1) = -L2*std::sin(a12) - CLAW_CENTER_DIST*std::sin(a123);
      J(0,2) = -CLAW_CENTER_DIST*std::sin(a123);
      J(1,0) =  L1*std::cos(q1) + L2*std::cos(a12) + CLAW_CENTER_DIST*std::cos(a123);
      J(1,1) =  L2*std::cos(a12) + CLAW_CENTER_DIST*std::cos(a123);
      J(1,2) =  CLAW_CENTER_DIST*std::cos(a123);

      // Pseudo‑inverse amortie : J^T (J J^T + λI)^{-1}
      Eigen::Matrix2d JJT = J * J.transpose();
      JJT.diagonal().array() += LAMBDA;
      Eigen::Vector2d err(err_x, err_y);
      Eigen::Vector3d dq = J.transpose() * JJT.ldlt().solve(err);

      q1 += ALPHA * dq(0);
      q2 += ALPHA * dq(1);
      q3 += ALPHA * dq(2);
    }
    return false;
  }

  void solve_ik(double tx, double ty)
  {
    double px = tx - BASE_OFFSET_X;
    double py = ty;

    double tip_dist = std::hypot(px, py);
    if (tip_dist > MAX_REACH + 1e-3) {
      static int warn_far = 0;
      if (warn_far++ % 30 == 0)
        RCLCPP_WARN(this->get_logger(), "Target too far (dist=%.3f > %.3f). Arm frozen.", tip_dist, MAX_REACH);
      return;
    }
    if (tip_dist < MIN_REACH) {
      static int warn_near = 0;
      if (warn_near++ % 30 == 0)
        RCLCPP_WARN(this->get_logger(), "Target too close (dist=%.3f < %.3f). Arm frozen.", tip_dist, MIN_REACH);
      return;
    }

    double best_q1 = q1_, best_q2 = q2_, best_q3 = q3_;
    bool ok = solve_position(px, py, best_q1, best_q2, best_q3);
    if (!ok) {
      std::vector<std::array<double,3>> seeds = {
        { 0.0,   0.0,  0.0},
        { 0.5,  -0.5,  0.0},
        {-0.5,   0.5,  0.0},
        { 0.0,   0.8,  0.0},
        { std::atan2(py, px) * 0.6, std::atan2(py, px) * 0.3, 0.0},
        { 0.0,   0.0,  0.5},
        {-0.3,   0.3,  0.0}
      };
      for (auto& s : seeds) {
        double q1 = s[0], q2 = s[1], q3 = s[2];
        if (solve_position(px, py, q1, q2, q3)) {
          best_q1 = q1; best_q2 = q2; best_q3 = q3;
          ok = true;
          break;
        }
      }
    }

    if (!ok) return;  
    double final_cx, final_cy, final_a123;
    fk_center(best_q1, best_q2, best_q3, final_cx, final_cy, final_a123);

    double desired_world_angle = std::atan2(py, px) + CLAW_URDF_OFFSET;
    double forearm_angle = best_q1 + best_q2;
    double raw_q3 = desired_world_angle - forearm_angle;
    best_q3 = raw_q3;  

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

    visualization_msgs::msg::InteractiveMarkerControl move_control;
    move_control.name = "move_xy";
    move_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    move_control.orientation.w = std::cos(M_PI/4.0);
    move_control.orientation.x = 0.0;
    move_control.orientation.y = std::cos(M_PI/4.0);
    move_control.orientation.z = 0.0;
    int_marker.controls.push_back(move_control);

    marker_server_->insert(int_marker,
      std::bind(&IKSolver::marker_feedback, this, std::placeholders::_1));
    marker_server_->applyChanges();

    q1_ = 0.0; q2_ = 0.5; q3_ = 0.0;
    solve_ik(0.5, 0.0);
  }

  void marker_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    double x = feedback->pose.position.x;
    double y = feedback->pose.position.y;
    captured_ = false;
    filtered_tx_ = alpha_filter_ * x + (1.0 - alpha_filter_) * filtered_tx_;
    filtered_ty_ = alpha_filter_ * y + (1.0 - alpha_filter_) * filtered_ty_;
    solve_ik(filtered_tx_, filtered_ty_);
    publish_target_sphere(x, y);
  }

  void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    captured_ = false;
    filtered_tx_ = alpha_filter_ * msg->x + (1.0 - alpha_filter_) * filtered_tx_;
    filtered_ty_ = alpha_filter_ * msg->y + (1.0 - alpha_filter_) * filtered_ty_;
    solve_ik(filtered_tx_, filtered_ty_);
    publish_target_sphere(msg->x, msg->y);
  }

  void publish_target_sphere(double x, double y)
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
    m.pose.position.z = 0.0;
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
      msg.name     = {"elbow_joint", "wrist_joint", "claw_base_joint",
                      "claw_left_joint", "claw_right_joint"};
      msg.position = {q1_, q2_, q3_, 0.0, 0.0};
      joint_pub_->publish(msg);
      if (++print_counter_ >= 20) {
        print_counter_ = 0;
        double cx, cy, angle;
        fk_center(q1_, q2_, q3_, cx, cy, angle);
        RCLCPP_INFO(this->get_logger(),
          "[CAPTURED] center=(%.3f, %.3f)  target=(%.3f, %.3f)  err=%.4f",
          cx + BASE_OFFSET_X, cy, filtered_tx_, filtered_ty_,
          std::hypot(cx + BASE_OFFSET_X - filtered_tx_, cy - filtered_ty_));
      }
      return;
    }

    double max_step = JOINT_SPEED * DT;
    q1_ += std::clamp(tq1_ - q1_, -max_step, max_step);
    q2_ += std::clamp(tq2_ - q2_, -max_step, max_step);
    q3_ += std::clamp(tq3_ - q3_, -max_step, max_step);

    if (std::isnan(q1_) || std::isnan(q2_) || std::isnan(q3_)) {
      RCLCPP_ERROR(this->get_logger(), "NaN in current angles! Resetting to zero.");
      q1_ = 0.0; q2_ = 0.5; q3_ = 0.0;
      tq1_ = q1_; tq2_ = q2_; tq3_ = q3_;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name     = {"elbow_joint", "wrist_joint", "claw_base_joint",
                    "claw_left_joint", "claw_right_joint"};
    msg.position = {q1_, q2_, q3_, 0.0, 0.0};
    joint_pub_->publish(msg);

    double cx, cy, angle;
    fk_center(q1_, q2_, q3_, cx, cy, angle);
    double err = std::hypot(cx + BASE_OFFSET_X - filtered_tx_, cy - filtered_ty_);
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
      double a12 = q1_ + q2_;
      double a123 = q1_ + q2_ + q3_;
      double wrist_x = BASE_OFFSET_X + L1*std::cos(q1_) + L2*std::cos(a12);
      double wrist_y = L1*std::sin(q1_) + L2*std::sin(a12);
      double claw_world_deg = a123 * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(),
        "[DEBUG] q1=%.3f  q2=%.3f  q3=%.3f  (deg: %.1f  %.1f  %.1f)\n"
        "        wrist=(%.3f, %.3f)  center=(%.3f, %.3f)\n"
        "        target=(%.3f, %.3f)  claw_dir=%.1f°\n"
        "        center_err=%.4f",
        q1_, q2_, q3_,
        q1_*180.0/M_PI, q2_*180.0/M_PI, q3_*180.0/M_PI,
        wrist_x, wrist_y,
        cx + BASE_OFFSET_X, cy,
        filtered_tx_, filtered_ty_,
        claw_world_deg,
        err);
    }
  }

  double q1_, q2_, q3_;
  double tq1_, tq2_, tq3_;
  double filtered_tx_, filtered_ty_;
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