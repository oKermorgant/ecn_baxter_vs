#include <ecn_baxter_vs/baxter_arm.h>
#include <urdf/model.h>
#include <opencv2/highgui.hpp>
#include <thread>
#include <rclcpp/parameter_client.hpp>

using namespace std;

BaxterArm::BaxterArm(std::string _side, bool sim) :
  logger("/tmp/baxter_"),
  node_{std::make_shared<rclcpp::Node>("control")},
  im_tr{node_}
{  
  node_->set_parameter(rclcpp::Parameter("use_sim_time", sim));

  // in case of misspell
  if (_side != "left")
    _side = "right";

  // which arm
  lefty_ = (_side == "left");

  // we detect green by default (sim)
  if(sim)
    detect(0, 255, 0);
  else if(lefty_)
    detect(255,0,0, true);
  else
    detect(0,255,0, true);

  std::cout << "BaxterArm initialized for " << _side << " arm ";
  if(sim)
    std::cout << "and in simulation\n";
  else
    std::cout << "on the real robot\n";

  // joint space dimension: 7
  q_.resize(7);

  // init joint URDF names
  cmd.names.resize(7);
  cmd.names[0] = _side + "_s0";
  cmd.names[1] = _side + "_s1";
  cmd.names[2] = _side + "_e0";
  cmd.names[3] = _side + "_e1";
  cmd.names[4] = _side + "_w0";
  cmd.names[5] = _side + "_w1";
  cmd.names[6] = _side + "_w2";
  cmd.command.resize(7);

  // load Baxter description
  const auto rsp_node(std::make_shared<rclcpp::Node>("baxter_rsp"));
  const auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                             (rsp_node, "/robot/robot_state_publisher");
  rsp_param_srv->wait_for_service();
  if(!rsp_param_srv->has_parameter("robot_description"))
  {
    // cannot get the model anyway
    RCLCPP_WARN(node_->get_logger(), "cannot get Baxter model");
    return;
  }
  // init joint limits
  // parse URDF to get robot data (name, DOF, joint limits, etc.)
  urdf::Model model;
  model.initString(rsp_param_srv->get_parameter<string>("robot_description"));

  q_min_.resize(7);
  q_max_.resize(7);
  v_max_.resize(7);
  for(auto& joint: model.joints_)
  {
    for(unsigned int i=0;i<7;++i)
    {
      if(joint.second->name == cmd.names[i])
      {
        v_max_[i] = joint.second->limits->velocity;
        if(!sim)
          v_max_[i] *= .5;
        q_min_[i] = joint.second->limits->lower;
        q_max_[i] = joint.second->limits->upper;
      }
    }
  }

  // init fixed matrices
  // between wrist Fw and camera Fc
  wMc_[0][0] = 0;
  wMc_[0][1] = 1;
  wMc_[0][2] = 0;
  wMc_[0][3] = 0.03825;
  wMc_[1][0] = -1;
  wMc_[1][1] = 0;
  wMc_[1][2] = 0;
  wMc_[1][3] = 0.012;
  wMc_[2][0] = 0;
  wMc_[2][1] = 0;
  wMc_[2][2] = 1;
  wMc_[2][3] = 0.128905;

  cWw_.buildFrom(wMc_.inverse());

  // between Baxter base Fb and root frame of the arm Ff
  if(lefty_)
  {
    bMf_[0][0] = 0.707106781186548;
    bMf_[0][1] = -0.707106781186548;
    bMf_[0][2] = 0;
    bMf_[0][3] = 0.024645;
    bMf_[1][0] = 0.707106781186548;
    bMf_[1][1] = 0.707106781186548;
    bMf_[1][2] = 0;
    bMf_[1][3] = 0.219645;
    bMf_[2][0] = 0;
    bMf_[2][1] = 0;
    bMf_[2][2] = 1;
    bMf_[2][3] = 0.118588;
  }
  else
  {
    bMf_[0][0] = 0.707106781186548;
    bMf_[0][1] = 0.707106781186548;
    bMf_[0][2] = 0;
    bMf_[0][3] = 0.024645;
    bMf_[1][0] = -0.707106781186548;
    bMf_[1][1] = 0.707106781186548;
    bMf_[1][2] = 0;
    bMf_[1][3] = -0.219645;
    bMf_[2][0] = 0;
    bMf_[2][1] = 0;
    bMf_[2][2] = 1;
    bMf_[2][3] = 0.118588;
  }

  vpRotationMatrix bRf;
  bMf_.extract(bRf);
  fRRb_.buildFrom(vpTranslationVector(), bRf.inverse());    // this is just the frame change matrix [[R 0][0 R]]

  if(sim)
  {
    area_d_ = 0.05;
    // simulated camera parameters
    cd_.setCamera(640, 480, ecn::Deg(90));
  }
  else
  {
    // desired area in this case
    area_d_ = 0.03;

    // camera parameters
    if(lefty_)
      cd_.setCamera(403.33,403.33,336.04,208.45);
    else
      cd_.setCamera(404.38,404.38,323.58,196.39);

    // publisher to Baxter image
    image_pub = im_tr.advertise("/robot/xdisplay", 100);
  }

  // publisher to joint command
  cmd_pub = node_->create_publisher<JointCommand>("/robot/limb/"+_side+"/joint_command", 100);

  // subscriber to joint states
  joint_sub = node_->create_subscription<JointState>("/robot/joint_states", 1000, [&](const JointState::SharedPtr msg)
  {
    size_t idx{};
    for(auto &name: msg->name)
    {
      for(unsigned int j=0;j<7;++j)
      {
        if(name == cmd.names[j])
        {
          q_[j] = msg->position[idx];
          continue;
        }
      }
      idx++;
    }
  });

  // set image to None, subscriber instantiated in the image setter
  image_sub = im_tr.subscribe("/cameras/"+_side+"_hand_camera/image", 1, [&](const Image::ConstSharedPtr msg)
  {
    if(!is_init_)
      return;

    // process with color detector
    cv::Mat im_out;
    auto im{cv_bridge::toCvCopy(msg)};

    const auto detected = cd_.process(im->image, im_out);

    im_ok = detected || im_ok;

    if(detected && cd_.area() > 0.002)
      lost_count = 0;
    else
      lost_count++;

    // add setpoint
    cv::circle(im_out, cv::Point(cd_.cam.u0, cd_.cam.v0),
               int(sqrt(area_d_*cd_.cam.px*cd_.cam.py/M_PI)),
               cv::Scalar(0,255,0), 2);

    // display.
    cv::imshow("Baxter",im_out);
    cv::waitKey(1);
  });




  // visualization
  q_plot[7] = -1;
  q_plot[8] = 1;
  logger.save(q_plot, "q", "[q_1,q_2,q_3,q_4,q_5,q_6,q_7,q^-,q^+]", "Normalized joints",false);
  logger.setLineType("[C0,C1,C2,C3,C4,C5,C6,k,k]");
  logger.save(vs_plot, "vs", "[x-x^*, y-y^*, a-a^*]", "VS error", false);

  // display image
  cv::namedWindow("Baxter");
  cv::createTrackbar( "10.lambda", "Baxter", &lambda_, 50);
  cv::setTrackbarPos("10.lambda", "Baxter", 1);

  cv::createTrackbar( "rho percent", "Baxter", &rho_, 50);
  cv::setTrackbarPos("rho percent", "Baxter", 10);
}

void BaxterArm::detect(int r, int g, int b, bool show_segment)
{
  cd_.detectColor(r, g, b);
  cd_.fitCircle();
  if(show_segment)
    cd_.showSegmentation();
  cd_.setSaturationValue(100, 60);
}

vpColVector BaxterArm::home()
{
  std::cout << "Going to init position... ";
  vpColVector q(7);
  q[0] = .24;
  q[1] = -1.15;
  q[3] = 1.64;
  q[5] = -.55;

  setJointPosition(q);
  std::cout << " done. \n";


  is_init_ = true;
  return q;
}


void BaxterArm::setJointPosition(const vpColVector &_q)
{
  for(unsigned int i=0;i<7;++i)
    cmd.command[i] = _q[i];
  cmd.mode = cmd.POSITION_MODE;
  cmd_pub->publish(cmd);

  std::this_thread::sleep_for(1s);
}

void BaxterArm::setJointVelocity(vpColVector _qdot)
{
  if(is_init_ && lost_count > 10)
  {
    std::cout << "Object lost, not moving" << std::endl;
    _qdot = 0;
  }
  else
  {
    // saturate
    for(int i = 0; i < 7; ++i)
      _qdot[i] = min(v_max_[i], max(-v_max_[i], _qdot[i]));
  }

  for(unsigned int i=0;i<7;++i)
    cmd.command[i] = _qdot[i];
  cmd.mode = cmd.VELOCITY_MODE;
  cmd_pub->publish(cmd);
}

vpHomogeneousMatrix BaxterArm::cameraPose()
{
  vpHomogeneousMatrix M;
  fMw(q_, M);          // fMw
  return bMf_ * M * wMc_;  //  bMc
}

void BaxterArm::setCameraVelocity(vpColVector _velocity)
{
  auto cJc = cameraJacobian(q_);                       // Jacobian expressed in camera frame
  setJointVelocity(cJc.pseudoInverse() * _velocity);   // to joint velocity
}


/**
 * @brief get Jacobian expressed in camera frame
 */
vpMatrix BaxterArm::cameraJacobian(const vpColVector &_q) const
{
  vpMatrix J;
  fJw(_q, J);             // in root frame fJw

  // build wRRf, vector transform between Ff and Fw
  vpHomogeneousMatrix M;
  fMw(q_, M);
  vpRotationMatrix wRf;
  M.inverse().extract(wRf);
  vpVelocityTwistMatrix wRRf(vpTranslationVector(), wRf);
  return cWw_ * wRRf * J; // cJc = cWw * wRRf * fJw
}

/**
 * @brief calcul modele geometrique direct
 */
int BaxterArm::fMw(const vpColVector &_q, vpHomogeneousMatrix &_M) const
{
  if(lefty_)
  {
    const double c1 = cos(_q[0]);
    const double c2 = cos(_q[1]);
    const double c3 = cos(_q[2]);
    const double c4 = cos(_q[3]);
    const double c5 = cos(_q[4]);
    const double c6 = cos(_q[5]);
    const double c7 = cos(_q[6]);
    const double s1 = sin(_q[0]);
    const double s2 = sin(_q[1]);
    const double s3 = sin(_q[2]);
    const double s4 = sin(_q[3]);
    const double s5 = sin(_q[4]);
    const double s6 = sin(_q[5]);
    const double s7 = sin(_q[6]);

    _M[0][0] = -((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*c7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*s7;
    _M[0][1] = ((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*s7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*c7;
    _M[0][2] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    _M[0][3] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1 + 0.055695;
    _M[1][0] = -((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*c7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*s7;
    _M[1][1] = ((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*s7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*c7;
    _M[1][2] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    _M[1][3] = -0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 - 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 + 0.01*(s1*s2*s3 + c1*c3)*s5 - 0.37429*(s1*s2*c3 - s3*c1)*s4 - 0.069*s1*s2*c3 + 0.37429*s1*c2*c4 + 0.36442*s1*c2 + 0.069*s1 + 0.069*s3*c1;
    _M[2][0] = (((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*c7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*s7;
    _M[2][1] = -(((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*s7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*c7;
    _M[2][2] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
    _M[2][3] = 0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 + 0.01*(s2*s4 - c2*c3*c4)*c5 - 0.115975*(s2*c4 + s4*c2*c3)*c6 - 0.37429*s2*c4 - 0.36442*s2 + 0.01*s3*s5*c2 - 0.37429*s4*c2*c3 - 0.069*c2*c3 + 0.281388;
    _M[3][0] = 0;
    _M[3][1] = 0;
    _M[3][2] = 0;
    _M[3][3] = 1;
  }
  else
  {

    const double c1 = cos(_q[0]);
    const double c2 = cos(_q[1]);
    const double c3 = cos(_q[2]);
    const double c4 = cos(_q[3]);
    const double c5 = cos(_q[4]);
    const double c6 = cos(_q[5]);
    const double c7 = cos(_q[6]);
    const double s1 = sin(_q[0]);
    const double s2 = sin(_q[1]);
    const double s3 = sin(_q[2]);
    const double s4 = sin(_q[3]);
    const double s5 = sin(_q[4]);
    const double s6 = sin(_q[5]);
    const double s7 = sin(_q[6]);
    _M[0][0] = -((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*c7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*s7;
    _M[0][1] = ((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*s7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*c7;
    _M[0][2] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    _M[0][3] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1 + 0.055695;
    _M[1][0] = -((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*c7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*s7;
    _M[1][1] = ((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*s7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*c7;
    _M[1][2] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    _M[1][3] = -0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 - 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 + 0.01*(s1*s2*s3 + c1*c3)*s5 - 0.37429*(s1*s2*c3 - s3*c1)*s4 - 0.069*s1*s2*c3 + 0.37429*s1*c2*c4 + 0.36442*s1*c2 + 0.069*s1 + 0.069*s3*c1;
    _M[2][0] = (((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*c7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*s7;
    _M[2][1] = -(((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*s7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*c7;
    _M[2][2] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
    _M[2][3] = 0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 + 0.01*(s2*s4 - c2*c3*c4)*c5 - 0.115975*(s2*c4 + s4*c2*c3)*c6 - 0.37429*s2*c4 - 0.36442*s2 + 0.01*s3*s5*c2 - 0.37429*s4*c2*c3 - 0.069*c2*c3 + 0.281388;
    //_M[3][0] = 0;
    //_M[3][1] = 0;
    //_M[3][2] = 0;
    _M[3][3] = 1.;
  }
  return 0;
}


bool BaxterArm::inverseKinematics(const vpColVector &_q0, const vpHomogeneousMatrix &_M_des, vpColVector &_q)
{
  const double eMin = 0.1;
  const unsigned int max_iter = 10000;
  const double lambda = 0.01;
  double e = 2*eMin;

  unsigned int iter = 0;
  vpPoseVector pose_err;
  vpColVector dq(7);
  vpHomogeneousMatrix M;
  vpMatrix J(6,7), J_reduce(6,6);
  _q = _q0;
  std::cout << "Current pose: " << _q0 << std::endl;
  iter = 0;
  while(e > eMin && iter < max_iter)
  {
    fMw(_q, M);
    fJw(_q, J);
    pose_err.buildFrom(M*_M_des.inverse());

    dq = -lambda * J.t() * (vpColVector) pose_err;
    for(unsigned int i=0;i<6;++i)
      if(_q[i] < q_max_[i] && _q[i] > q_min_[i])      // check joint limits valid
        _q[i] += dq[i];

    e = ((vpColVector) pose_err).frobeniusNorm();
    iter++;
  }
  if(e < eMin)
  {
    std::cout << "IK Solution found:: " << _q.t() << std::endl;
    std::cout << "\tIK, remaining error = " << e << std::endl;
    return true;   // Valid solution found
  }
  // else keep trying
  std::cout << "\tIK, remaining error = " << e << std::endl;
  return false;       // no valid solution found
}

/**
 * @brief compute Jacobian (base to wrist left_hand_camera)
 */
int BaxterArm::fJw(const vpColVector &_q, vpMatrix &_J) const
{
  _J.resize(6,7);
  if(lefty_)
  {
    const double c1 = cos(_q[0]);
    const double c2 = cos(_q[1]);
    const double c3 = cos(_q[2]);
    const double c4 = cos(_q[3]);
    const double c5 = cos(_q[4]);
    const double c6 = cos(_q[5]);
    const double s1 = sin(_q[0]);
    const double s2 = sin(_q[1]);
    const double s3 = sin(_q[2]);
    const double s4 = sin(_q[3]);
    const double s5 = sin(_q[4]);
    const double s6 = sin(_q[5]);

    _J[0][0] = 0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 + 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 + 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - 0.01*(s1*s2*s3 + c1*c3)*s5 + 0.37429*(s1*s2*c3 - s3*c1)*s4 + 0.069*s1*s2*c3 - 0.37429*s1*c2*c4 - 0.36442*s1*c2 - 0.069*s1 - 0.069*s3*c1;
    _J[0][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*c1;
    _J[0][2] = 0.115975*s1*s3*s5*s6 + 0.01*s1*s3*s5 - 0.115975*s1*s4*c3*c6 - 0.37429*s1*s4*c3 - 0.115975*s1*s6*c3*c4*c5 - 0.01*s1*c3*c4*c5 - 0.069*s1*c3 + 0.115975*s2*s3*s4*c1*c6 + 0.37429*s2*s3*s4*c1 + 0.115975*s2*s3*s6*c1*c4*c5 + 0.01*s2*s3*c1*c4*c5 + 0.069*s2*s3*c1 + 0.115975*s2*s5*s6*c1*c3 + 0.01*s2*s5*c1*c3;
    _J[0][3] = 0.115975*s1*s3*s4*s6*c5 + 0.01*s1*s3*s4*c5 - 0.115975*s1*s3*c4*c6 - 0.37429*s1*s3*c4 + 0.115975*s2*s4*s6*c1*c3*c5 + 0.01*s2*s4*c1*c3*c5 - 0.115975*s2*c1*c3*c4*c6 - 0.37429*s2*c1*c3*c4 - 0.115975*s4*c1*c2*c6 - 0.37429*s4*c1*c2 - 0.115975*s6*c1*c2*c4*c5 - 0.01*c1*c2*c4*c5;
    _J[0][4] = 0;
    _J[0][5] = 0.115975*s1*s3*s4*s6 - 0.115975*s1*s3*c4*c5*c6 - 0.115975*s1*s5*c3*c6 + 0.115975*s2*s3*s5*c1*c6 + 0.115975*s2*s4*s6*c1*c3 - 0.115975*s2*c1*c3*c4*c5*c6 - 0.115975*s4*c1*c2*c5*c6 - 0.115975*s6*c1*c2*c4;
    _J[0][6] = 0;
    _J[1][0] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1;
    _J[1][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*s1;
    _J[1][2] = 0.115975*s1*s2*s3*s4*c6 + 0.37429*s1*s2*s3*s4 + 0.115975*s1*s2*s3*s6*c4*c5 + 0.01*s1*s2*s3*c4*c5 + 0.069*s1*s2*s3 + 0.115975*s1*s2*s5*s6*c3 + 0.01*s1*s2*s5*c3 - 0.115975*s3*s5*s6*c1 - 0.01*s3*s5*c1 + 0.115975*s4*c1*c3*c6 + 0.37429*s4*c1*c3 + 0.115975*s6*c1*c3*c4*c5 + 0.01*c1*c3*c4*c5 + 0.069*c1*c3;
    _J[1][3] = 0.115975*s1*s2*s4*s6*c3*c5 + 0.01*s1*s2*s4*c3*c5 - 0.115975*s1*s2*c3*c4*c6 - 0.37429*s1*s2*c3*c4 - 0.115975*s1*s4*c2*c6 - 0.37429*s1*s4*c2 - 0.115975*s1*s6*c2*c4*c5 - 0.01*s1*c2*c4*c5 - 0.115975*s3*s4*s6*c1*c5 - 0.01*s3*s4*c1*c5 + 0.115975*s3*c1*c4*c6 + 0.37429*s3*c1*c4;
    _J[1][4] = 0;
    _J[1][5] = 0.115975*s1*s2*s3*s5*c6 + 0.115975*s1*s2*s4*s6*c3 - 0.115975*s1*s2*c3*c4*c5*c6 - 0.115975*s1*s4*c2*c5*c6 - 0.115975*s1*s6*c2*c4 - 0.115975*s3*s4*s6*c1 + 0.115975*s3*c1*c4*c5*c6 + 0.115975*s5*c1*c3*c6;
    _J[1][6] = 0;
    _J[2][0] = 0;
    _J[2][1] = 0;
    _J[2][2] = 0;
    _J[2][3] = 0;
    _J[2][4] = 0;
    _J[2][5] = 0;
    _J[2][6] = 0;
    _J[3][0] = 0;
    _J[3][1] = -s1;
    _J[3][2] = c1*c2;
    _J[3][3] = -s1*c3 + s2*s3*c1;
    _J[3][4] = -(s1*s3 + s2*c1*c3)*s4 + c1*c2*c4;
    _J[3][5] = ((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5;
    _J[3][6] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    _J[4][0] = 0;
    _J[4][1] = c1;
    _J[4][2] = s1*c2;
    _J[4][3] = s1*s2*s3 + c1*c3;
    _J[4][4] = -(s1*s2*c3 - s3*c1)*s4 + s1*c2*c4;
    _J[4][5] = ((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5;
    _J[4][6] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    _J[5][0] = 1;
    _J[5][1] = 0;
    _J[5][2] = -s2;
    _J[5][3] = s3*c2;
    _J[5][4] = -s2*c4 - s4*c2*c3;
    _J[5][5] = -(s2*s4 - c2*c3*c4)*s5 + s3*c2*c5;
    _J[5][6] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
  }
  else
  {
    const double c1 = cos(_q[0]);
    const double c2 = cos(_q[1]);
    const double c3 = cos(_q[2]);
    const double c4 = cos(_q[3]);
    const double c5 = cos(_q[4]);
    const double c6 = cos(_q[5]);
    const double s1 = sin(_q[0]);
    const double s2 = sin(_q[1]);
    const double s3 = sin(_q[2]);
    const double s4 = sin(_q[3]);
    const double s5 = sin(_q[4]);
    const double s6 = sin(_q[5]);
    _J[0][0] = 0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 + 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 + 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - 0.01*(s1*s2*s3 + c1*c3)*s5 + 0.37429*(s1*s2*c3 - s3*c1)*s4 + 0.069*s1*s2*c3 - 0.37429*s1*c2*c4 - 0.36442*s1*c2 - 0.069*s1 - 0.069*s3*c1;
    _J[0][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*c1;
    _J[0][2] = 0.115975*s1*s3*s5*s6 + 0.01*s1*s3*s5 - 0.115975*s1*s4*c3*c6 - 0.37429*s1*s4*c3 - 0.115975*s1*s6*c3*c4*c5 - 0.01*s1*c3*c4*c5 - 0.069*s1*c3 + 0.115975*s2*s3*s4*c1*c6 + 0.37429*s2*s3*s4*c1 + 0.115975*s2*s3*s6*c1*c4*c5 + 0.01*s2*s3*c1*c4*c5 + 0.069*s2*s3*c1 + 0.115975*s2*s5*s6*c1*c3 + 0.01*s2*s5*c1*c3;
    _J[0][3] = 0.115975*s1*s3*s4*s6*c5 + 0.01*s1*s3*s4*c5 - 0.115975*s1*s3*c4*c6 - 0.37429*s1*s3*c4 + 0.115975*s2*s4*s6*c1*c3*c5 + 0.01*s2*s4*c1*c3*c5 - 0.115975*s2*c1*c3*c4*c6 - 0.37429*s2*c1*c3*c4 - 0.115975*s4*c1*c2*c6 - 0.37429*s4*c1*c2 - 0.115975*s6*c1*c2*c4*c5 - 0.01*c1*c2*c4*c5;
    //_J[0][4] = 0;
    _J[0][5] = 0.115975*s1*s3*s4*s6 - 0.115975*s1*s3*c4*c5*c6 - 0.115975*s1*s5*c3*c6 + 0.115975*s2*s3*s5*c1*c6 + 0.115975*s2*s4*s6*c1*c3 - 0.115975*s2*c1*c3*c4*c5*c6 - 0.115975*s4*c1*c2*c5*c6 - 0.115975*s6*c1*c2*c4;
    //_J[0][6] = 0;
    _J[1][0] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1;
    _J[1][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*s1;
    _J[1][2] = 0.115975*s1*s2*s3*s4*c6 + 0.37429*s1*s2*s3*s4 + 0.115975*s1*s2*s3*s6*c4*c5 + 0.01*s1*s2*s3*c4*c5 + 0.069*s1*s2*s3 + 0.115975*s1*s2*s5*s6*c3 + 0.01*s1*s2*s5*c3 - 0.115975*s3*s5*s6*c1 - 0.01*s3*s5*c1 + 0.115975*s4*c1*c3*c6 + 0.37429*s4*c1*c3 + 0.115975*s6*c1*c3*c4*c5 + 0.01*c1*c3*c4*c5 + 0.069*c1*c3;
    _J[1][3] = 0.115975*s1*s2*s4*s6*c3*c5 + 0.01*s1*s2*s4*c3*c5 - 0.115975*s1*s2*c3*c4*c6 - 0.37429*s1*s2*c3*c4 - 0.115975*s1*s4*c2*c6 - 0.37429*s1*s4*c2 - 0.115975*s1*s6*c2*c4*c5 - 0.01*s1*c2*c4*c5 - 0.115975*s3*s4*s6*c1*c5 - 0.01*s3*s4*c1*c5 + 0.115975*s3*c1*c4*c6 + 0.37429*s3*c1*c4;
    //_J[1][4] = 0;
    _J[1][5] = 0.115975*s1*s2*s3*s5*c6 + 0.115975*s1*s2*s4*s6*c3 - 0.115975*s1*s2*c3*c4*c5*c6 - 0.115975*s1*s4*c2*c5*c6 - 0.115975*s1*s6*c2*c4 - 0.115975*s3*s4*s6*c1 + 0.115975*s3*c1*c4*c5*c6 + 0.115975*s5*c1*c3*c6;
    //_J[1][6] = 0;
    //_J[2][0] = 0;
    //_J[2][1] = 0;
    //_J[2][2] = 0;
    //_J[2][3] = 0;
    //_J[2][4] = 0;
    //_J[2][5] = 0;
    //_J[2][6] = 0;
    //_J[3][0] = 0;
    _J[3][1] = -s1;
    _J[3][2] = c1*c2;
    _J[3][3] = -s1*c3 + s2*s3*c1;
    _J[3][4] = -(s1*s3 + s2*c1*c3)*s4 + c1*c2*c4;
    _J[3][5] = ((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5;
    _J[3][6] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    //_J[4][0] = 0;
    _J[4][1] = c1;
    _J[4][2] = s1*c2;
    _J[4][3] = s1*s2*s3 + c1*c3;
    _J[4][4] = -(s1*s2*c3 - s3*c1)*s4 + s1*c2*c4;
    _J[4][5] = ((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5;
    _J[4][6] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    _J[5][0] = 1.;
    //_J[5][1] = 0;
    _J[5][2] = -s2;
    _J[5][3] = s3*c2;
    _J[5][4] = -s2*c4 - s4*c2*c3;
    _J[5][5] = -(s2*s4 - c2*c3*c4)*s5 + s3*c2*c5;
    _J[5][6] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
  }
  return 0;
}

void BaxterArm::plot(vpColVector err)
{
  for(int i = 0; i < 3; ++i)
    vs_plot[i] = err[i];

  for(int i = 0; i< 7; ++i)
    q_plot[i] = -1 + 2*(q_[i] - q_min_[i])/(q_max_[i] - q_min_[i]);

  logger.update();
}

