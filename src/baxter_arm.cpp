#include <ecn_baxter_vs/baxter_arm.h>
#include <urdf/model.h>
#include <visp/vpPixelMeterConversion.h>
#include <std_msgs/Float32MultiArray.h>

using std::vector;

BaxterArm::BaxterArm(ros::NodeHandle &_nh, std::string _side, bool _sim) : it_(_nh), sim_(_sim)
{
    // in case of misspell
    if (_side != "left")
        _side = "right";

    // we detect green by default (sim)
    detect(0, 255, 0);

    // which arm
    lefty_ = (_side == "left");

    std::cout << "BaxterArm initialized for " << _side << " arm ";
    if(sim_)
        std::cout << "and in simulation\n";
    else
        std::cout << "on the real robot\n";

    // joint space dimension: 7
    q_.resize(7);

    // init joint URDF names
    cmd_msg_real.names.resize(7);
    cmd_msg_real.names[0] = _side + "_s0";
    cmd_msg_real.names[1] = _side + "_s1";
    cmd_msg_real.names[2] = _side + "_e0";
    cmd_msg_real.names[3] = _side + "_e1";
    cmd_msg_real.names[4] = _side + "_w0";
    cmd_msg_real.names[5] = _side + "_w1";
    cmd_msg_real.names[6] = _side + "_w2";
    cmd_msg_real.command.resize(7);
    cmd_msg_sim.name = cmd_msg_real.names;
    cmd_msg_sim.position.resize(7);

    // init joint limits
    // parse URDF to get robot data (name, DOF, joint limits, etc.)
    urdf::Model model;

    model.initParam("/robot_description");
    q_min_.resize(7);
    q_max_.resize(7);
    v_max_.resize(7);
    for(auto& joint: model.joints_)
    {
        for(unsigned int i=0;i<7;++i)
        {
            if(joint.second->name == cmd_msg_real.names[i])
            {
                v_max_[i] = joint.second->limits->velocity;
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

    if(sim_)
    {
        // publisher to joint command
        cmd_pub_ = _nh.advertise<sensor_msgs::JointState>("/vrep_ros_interface/joint_command", 100);

        // subscriber to joint states
        joint_subscriber_ = _nh.subscribe("/vrep_ros_interface/joint_states", 1000, &BaxterArm::readJointStates, this);

        // set image to None, subscriber instantiated in the image setter
        image_subscriber_ = it_.subscribe("/vrep_ros_interface/camera/"+_side, 1, &BaxterArm::readImage, this);

        // camera parameters
        cam_.initFromFov(640, 480, M_PI/2, M_PI/2);
    }
    else
    {
        // publisher to joint command
        cmd_pub_ = _nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/"+_side+"/joint_command", 100);

        // subscriber to joint states
        joint_subscriber_ = _nh.subscribe("/robot/joint_states", 1000, &BaxterArm::readJointStates, this);

        // set image to None, subscriber instantiated in the image setter
        image_subscriber_ = it_.subscribe("/cameras/"+_side+"_hand_camera/image", 1, &BaxterArm::readImage, this);

        // camera parameters
        if(lefty_)
            cam_.initPersProjWithoutDistortion(403.33,403.33,336.04,208.45);
        else
            cam_.initPersProjWithoutDistortion(404.38,404.38,323.58,196.39);

        // publisher to Baxter image
        image_publisher_ = it_.advertise("/robot/xdisplay", 100);




    }

    // visualization
    joint_pub_ = _nh.advertise<std_msgs::Float32MultiArray>("/display/" + _side + "/joints", 100);
    vs_pub_ = _nh.advertise<std_msgs::Float32MultiArray>("/display/" + _side + "/vs", 100);

    ros::spinOnce();
}

void BaxterArm::detect(int r, int g, int b, bool show_segment)
{
    show_segment_ = show_segment;
    // convert color to HSV
    const float cmax = std::max(r, std::max(g,b));
    const float cmin = std::min(r, std::min(g,b));
    const float d = cmax - cmin;

    int h = 0;
    if(d)
    {
        if(cmax == r)
            h = 30*(fmod((g-b)/d,6));
        else if(cmax == g)
            h = 30*((b-r)/d + 2);
        else
            h = 30*((r-g)/d + 4);
    }

    // build inRange bounds for hue
    int hthr = 10;
    hue_ = {std::max(h-hthr,0),std::min(h+hthr,179)};

    // other segmentation for h
    if(h < hthr)
    {
        hue_.push_back(179+h-hthr);
        hue_.push_back(179);
    }
    else if(h+hthr > 179)
    {
        hue_.push_back(0);
        hue_.push_back(h+hthr-179);
    }

    // init display
    cv::namedWindow("Baxter");
    cv::createTrackbar( "Saturation", "Baxter", &sat_, 255);
    cv::createTrackbar( "Value", "Baxter", &val_, 255);
    cv::setTrackbarPos("Saturation", "Baxter", 130);
    cv::setTrackbarPos("Value", "Baxter", 95);
}


void BaxterArm::init()
{
    std::cout << "Going to init position... ";
    vpColVector q(7);
    q[0] = .24;
    q[1] = -1.15;
    q[3] = 1.64;
    q[5] = -.55;

    setJointPosition(q);
    std::cout << " done. \n";
}


void BaxterArm::setJointPosition(vpColVector _q)
{
    if(sim_)
    {
        // simulation only allows velocity control
        // -> apply velocity till desired position is reached
        ros::Rate loop(10);
        int it = 0;
        double lambda = 2;
        while((_q - q_).euclideanNorm() > 1e-3 && it < 1000)
        {
            if ((_q - q_).euclideanNorm() < 1e-2)
                lambda = 5;
            it++;
            setJointVelocity(-lambda * (q_ - _q));
            ros::spinOnce();
            loop.sleep();
        }
    }
    else
    {
        for(unsigned int i=0;i<7;++i)
            cmd_msg_real.command[i] = _q[i];
        cmd_msg_real.mode = 1;
        ros::Rate loop(10);
        int it = 0;
        while((_q - q_).euclideanNorm() > 1e-3 && it < 1000)
        {
            it++;
            cmd_pub_.publish(cmd_msg_real);
            std::cout << "Reaching desired position, error=" << (_q - q_).euclideanNorm() << '\n';
            ros::spinOnce();
            loop.sleep();
        }
    }
}

void BaxterArm::setJointVelocity(vpColVector _qdot)
{
    if(sim_)    {
        // simulation uses classical JointState message
        for(int i=0;i<7;++i)
            cmd_msg_sim.position[i] = _qdot[i];
        cmd_pub_.publish(cmd_msg_sim);
    }
    else
    {
        // real Baxter uses JointCommand message
        for(unsigned int i=0;i<7;++i)
            cmd_msg_real.command[i] = _qdot[i];
        cmd_msg_real.mode = 2;
        cmd_pub_.publish(cmd_msg_real);
    }
}


void BaxterArm::setCameraPose(vpHomogeneousMatrix _M)
{
    // bMc
    vpColVector q(7);
    // change frames to get IK
    _M = bMf_.inverse() * _M * wMc_.inverse();    // oMw

    // go to the desired position
    if(inverseKinematics(q_, _M, q))
        setJointPosition(q);
}


vpHomogeneousMatrix BaxterArm::cameraPose()
{
    vpHomogeneousMatrix M;
    fMw(q_, M);          // fMw
    return bMf_ * M * wMc_;  //  bMc
}

void BaxterArm::setCameraVelocity(vpColVector _velocity)
{
    vpMatrix cJc(6,7);
    cameraJacobian(q_, cJc);                             // Jacobian expressed in camera frame
    setJointVelocity(cJc.pseudoInverse() * _velocity);   // to joint velocity
}


/**
 * @brief get Jacobian expressed in camera frame
 */
int BaxterArm::cameraJacobian(const vpColVector &_q, vpMatrix &_cJc)
{
    fJw(_q, _cJc);             // in root frame fJw

    // build wRRf, vector transform between Ff and Fw
    vpHomogeneousMatrix M;
    fMw(q_, M);
    vpRotationMatrix wRf;
    M.inverse().extract(wRf);
    vpVelocityTwistMatrix wRRf(vpTranslationVector(), wRf);
    _cJc = cWw_ * wRRf * _cJc; // cJc = cWw * wRRf * fJw
    return 0;
}

/**
 * @brief calcul modele geometrique direct
 */
int BaxterArm::fMw(const vpColVector &_q, vpHomogeneousMatrix &_M)
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

/**
 * @brief calcul modele geometrique inverse analytique ou iteratif
 * @details WRONG TODO
 */
bool BaxterArm::inverseKinematics(const vpColVector &_q0, const vpHomogeneousMatrix &_M_des, vpColVector &_q)
{
    const double eMin = 0.1;
    const unsigned int max_try = 10, max_iter = 10000;
    const double lambda = 0.01;
    double e = 2*eMin;

    unsigned int nb_try = 0, iter = 0;
    vpPoseVector pose_err;
    vpColVector dq(7);
    vpHomogeneousMatrix M;
    vpMatrix J(6,7), J_reduce(6,6);
    _q = _q0;
    std::cout << "Current pose: " << _q0 << std::endl;
    //_q[6] = q_min_[6];
    // while( nb_try < max_try)
    {
        //   for (int i = 0; i < 6; ++i)     // random initial solution
        //   {
        //       _q[i] = q_min_[i] + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(q_max_[i]-q_min_[i])));
        //   }
        //    std::cout << "Try number " << nb_try << " initial rand sol:: " << _q.t() << std::endl;
        iter = 0; e = 2*eMin;
        while(e > eMin && iter < max_iter)
        {
            fMw(_q, M);
            fJw(_q, J);
            pose_err.buildFrom(M*_M_des.inverse());
            /*   J_reduce[0][0] = J[0][0]; J_reduce[0][1] = J[0][1]; J_reduce[0][2] = J[0][2]; J_reduce[0][3] = J[0][3]; J_reduce[0][4] = J[0][4]; J_reduce[0][5] = J[0][5];
            J_reduce[1][0] = J[1][0]; J_reduce[1][1] = J[1][1]; J_reduce[1][2] = J[1][2]; J_reduce[1][3] = J[1][3]; J_reduce[1][4] = J[1][4]; J_reduce[1][5] = J[1][5];
            J_reduce[2][0] = J[2][0]; J_reduce[2][1] = J[2][1]; J_reduce[2][2] = J[2][2]; J_reduce[2][3] = J[2][3]; J_reduce[2][4] = J[2][4]; J_reduce[2][5] = J[2][5];
            J_reduce[3][0] = J[3][0]; J_reduce[3][1] = J[3][1]; J_reduce[3][2] = J[3][2]; J_reduce[3][3] = J[3][3]; J_reduce[3][4] = J[3][4]; J_reduce[3][5] = J[3][5];
            J_reduce[4][0] = J[4][0]; J_reduce[4][1] = J[4][1]; J_reduce[4][2] = J[4][2]; J_reduce[4][3] = J[4][3]; J_reduce[4][4] = J[4][4]; J_reduce[4][5] = J[4][5];
            J_reduce[5][0] = J[5][0]; J_reduce[5][1] = J[5][1]; J_reduce[5][2] = J[5][2]; J_reduce[5][3] = J[5][3]; J_reduce[5][4] = J[5][4]; J_reduce[5][5] = J[5][5];
     */

            dq = -lambda * J.t() * (vpColVector) pose_err;
            for(unsigned int i=0;i<6;++i)
                if(_q[i] < q_max_[i] && _q[i] > q_min_[i])      // check joint limits valid
                    _q[i] += dq[i];

            e = ((vpColVector) pose_err).euclideanNorm();
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
        nb_try++;
    }
    return false;       // no valid solution found
}

/**
 * @brief compute Jacobian (base to wrist left_hand_camera)
 */
int BaxterArm::fJw(const vpColVector &_q, vpMatrix &_J)
{
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


void BaxterArm::readJointStates(const sensor_msgs::JointState::ConstPtr& _msg)
{
    const bool init = (q_.euclideanNorm() == 0);
    std_msgs::Float32MultiArray msg;
    msg.data.resize(7);
    for(unsigned int i=0;i<_msg->name.size();++i)
    {
        for(unsigned int j=0;j<7;++j)
        {
            if(_msg->name[i] == cmd_msg_real.names[j])
            {
                if(init)
                    q_[j] = _msg->position[i];
                else
                    q_[j] = 0.5*(q_[j] + _msg->position[i]);
                // also publish normalized position (0 <= q <= 1)
                msg.data[j] = (q_[j] - q_min_[j])/(q_max_[j] - q_min_[j]);
            }
        }
    }

    joint_pub_.publish(msg);
}


void BaxterArm::readImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image,
                 img_,
                 cv::COLOR_BGR2HSV);
    cv::GaussianBlur(img_, img_, cv::Size(9,9), 2);

    if(hue_.size())
    {
        // segment for detection of given RGB (from Hue)
        cv::inRange(img_, cv::Scalar(hue_[0], sat_, val_),
                cv::Scalar(hue_[1], 255, 255), seg1_);
        // add for 2nd detection if near red
        if(hue_.size() == 4)
        {
            cv::inRange(img_, cv::Scalar(hue_[2], sat_, val_),
                    cv::Scalar(hue_[3], 255, 255), seg2_);
            seg1_ += seg2_;
        }

        if(show_segment_)
            cv::imshow("inrange", seg1_);

        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours( seg1_, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

        // pop all children
        bool found = true;
        while(found)
        {
            found = false;
            for(unsigned int i=0;i<hierarchy.size();++i)
            {
                if(hierarchy[i][3] > -1)
                {
                    found = true;
                    hierarchy.erase(hierarchy.begin()+i,hierarchy.begin()+i+1);
                    contours.erase(contours.begin()+i, contours.begin()+i+1);
                    break;
                }
            }
        }

        if(contours.size())
        {
            // get largest contour
            auto largest = std::max_element(
                        contours.begin(), contours.end(),
                        [](const vector<cv::Point> &c1, const vector<cv::Point> &c2)
            {return cv::contourArea(c1) < cv::contourArea(c2);});
            int idx = std::distance(contours.begin(), largest);

            cv::Point2f pt;float radius;
            cv::minEnclosingCircle(contours[idx], pt, radius);
            cv::circle(cv_bridge::toCvShare(msg, "bgr8")->image, pt, radius, cv::Scalar(255,255,255), 2);

            // filter position
            x_ = .5*(x_ + pt.x);
            y_ = .5*(y_ + pt.y);
            rad_ = .5*(rad_ + radius);
            std::cout << "Sphere detected at (" << pt.x << ", " << pt.y << ")\n";
            std::cout << "       Filtered at (" << x_ << ", " << y_ << ")\n";

            // publish VS features
            std_msgs::Float32MultiArray msg;
            msg.data = {x(), y(), area()};
            vs_pub_.publish(msg);
        }
    }

    else
    {
        std::cout << "RGB to detect was not defined\n";
    }
    cv::imshow("Baxter",cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
}