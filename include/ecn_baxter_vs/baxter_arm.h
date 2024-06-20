#ifndef BAXTERARM_H
#define BAXTERARM_H

#include <rclcpp/node.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <image_transport/image_transport.hpp>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <cv_bridge/cv_bridge.h>
#include <ecn_baxter_vs/color_detector.h>
#include <log2plot/log_plotter.h>
#include <ctime>
#include <memory>

class BaxterArm
{

  using JointCommand = baxter_core_msgs::msg::JointCommand;
  using JointState = sensor_msgs::msg::JointState;
  using Image = sensor_msgs::msg::Image;

public:

  BaxterArm(std::string side = "right", bool sim = true);

  inline auto node()
  {
    return node_;
  }

  inline void setControlLoop(const std::function<void()> callback, std::chrono::milliseconds dt)
  {
    static auto timer = node_->create_wall_timer(dt, callback);
  }

  // joint space I/O
  vpColVector jointPosition() {return q_;}
  void setJointPosition(const vpColVector& _q);
  void setJointVelocity(vpColVector _qdot);

  // default arm position
  vpColVector home();

  void plot(vpColVector err);

  void setCameraVelocity(vpColVector _velocity);
  vpHomogeneousMatrix cameraPose();   // aka camera -> base bMc

  // Jacobian in camera frame
  vpMatrix cameraJacobian(const vpColVector &_q) const ;
  inline vpMatrix cameraJacobian() const
  {
    return cameraJacobian(q_);
  }

  // Inverse Kinematics in camera frame
  // returns True if solution found
  // with a given starting estimate
  bool inverseKinematics(const vpColVector &_q0, const vpHomogeneousMatrix &_M_des, vpColVector &_q);
  // or without, then picks current position
  bool inverseKinematics(const vpHomogeneousMatrix &_M_des, vpColVector &_q)
  {
    return inverseKinematics(q_, _M_des, _q);
  }

  inline vpColVector jointMin(){return q_min_;}
  inline vpColVector jointMax() {return q_max_;}
  inline vpColVector velocityMax() {return v_max_;}
  inline double lambda() const {return 0.1*lambda_;}
  inline double rho() const {return 0.01*rho_;}

  // camera part
  void detect(int r, int g ,int b, bool show_segment = false);
  double x() {return cd_.x();}
  double y() {return cd_.y();}
  double area()  {return cd_.area();}
  double area_d() const {return area_d_;}

  inline bool ready() const
  {
    return im_ok && js_ok;
  } 

protected:
  // ROS  
  rclcpp::Node::SharedPtr node_;

  // joints
  rclcpp::Subscription<JointState>::SharedPtr joint_sub;
  std::vector<std::string> names_;

  // cmd
  rclcpp::Publisher<JointCommand>::SharedPtr cmd_pub;
  JointCommand cmd;

  // gain tuning
  int lambda_;
  int rho_;

  // online feedback
  std::array<double, 9> q_plot;
  std::array<double, 3> vs_plot;
  log2plot::LogPlotter logger;

  vpColVector q_;
  vpHomogeneousMatrix wMc_, bMf_;
  vpVelocityTwistMatrix cWw_, fRRb_;

  // some checks
  bool lefty_, is_init_ = false, im_ok = false, js_ok = false;
  double area_d_ =  0.05;    // simulation value

  // image
  image_transport::ImageTransport im_tr;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  ecn::ColorDetector cd_;
  int lost_count = 0;

  // joint limits
  vpColVector q_min_, q_max_, v_max_;

  // internal modeling
  // Direct Kinematic Model   // aka wrist -> fixed fMw
  int fMw(const vpColVector &_q, vpHomogeneousMatrix &_M) const;
  // Classical Jacobian of wrist frame
  int fJw(const vpColVector &_q, vpMatrix &_J) const;


};



#endif // BAXTERARM_H
