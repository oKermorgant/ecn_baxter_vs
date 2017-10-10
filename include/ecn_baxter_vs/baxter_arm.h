#ifndef BAXTERARM_H
#define BAXTERARM_H

#include <ros/ros.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <cv_bridge/cv_bridge.h>
#include <ecn_common/color_detector.h>
#include <ctime>

class BaxterArm
{
public:

    BaxterArm(bool _sim = true, std::string _side = "right");

    // joint space I/O
    vpColVector jointPosition() {return q_;}
    void setJointPosition(vpColVector _q);
    void setJointVelocity(vpColVector _qdot);

    // default arm position
    void init();

    // operational (camera) space I/O

    void setCameraPose(vpHomogeneousMatrix _M);
    inline void setCameraPose(const vpPoseVector &_pose)
    {
        setCameraPose(vpHomogeneousMatrix(_pose));
    }

    void setCameraVelocity(vpColVector _velocity);
    vpHomogeneousMatrix cameraPose();   // aka camera -> base bMc

    // Jacobian in camera frame
    int cameraJacobian(const vpColVector &_q, vpMatrix &_cJc);
    vpMatrix cameraJacobian()
    {
        vpMatrix J(6,7);
        cameraJacobian(q_, J);
        return J;
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

    inline vpColVector jointMin() {return q_min_;}
    inline vpColVector jointMax() {return q_max_;}
    inline vpColVector velocityMax() {return v_max_;}
    inline int lambda() {return lambda_;}

    // camera part
    void detect(int r, int g ,int b, bool show_segment = false);
    float x() {return cd_.x();}
    float y() {return cd_.y();}
    float area() {return cd_.area();}

    bool ok()
    {
        ros::spinOnce();
        loop_.sleep();
        return ros::ok() && q_.euclideanNorm() != 0;
    }


 protected:
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Rate loop_;
    ros::Subscriber joint_subscriber_;
    std::vector<std::string> names_;
    sensor_msgs::JointState cmd_msg_sim;
    baxter_core_msgs::JointCommand cmd_msg_real;

    // gain tuning
    int lambda_;

    // online feedback
    ros::Publisher joint_pub_, vs_pub_;
    
    vpColVector q_;
    vpHomogeneousMatrix wMc_, bMf_;
    vpVelocityTwistMatrix cWw_, fRRb_;

    bool sim_, lefty_;

    // image
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
    ecn::ColorDetector cd_;

    // joint limits
    vpColVector q_min_, q_max_, v_max_;

    // internal modeling
    // Direct Kinematic Model   // aka wrist -> fixed fMw
    int fMw(const vpColVector &_q, vpHomogeneousMatrix &_M);
    // Classical Jacobian of wrist frame
    int fJw(const vpColVector &_q, vpMatrix &_J);

    // ROS functions
    void readJointStates(const sensor_msgs::JointState::ConstPtr& msg);

    void readImage(const sensor_msgs::ImageConstPtr& msg);
};



#endif // BAXTERARM_H
