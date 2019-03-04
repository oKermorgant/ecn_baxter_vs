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
#include <memory>

class BaxterArm
{
public:

    BaxterArm(int argc, char** argv, std::string group = "display", bool _sim = true, std::string _side = "right");

    // joint space I/O
    vpColVector jointPosition() {return q_;}
    void setJointPosition(vpColVector _q);
    void setJointVelocity(vpColVector _qdot);

    // default arm position
    vpColVector init();

    // operational (camera) space I/O
    void setCameraPose(vpHomogeneousMatrix _M);
    inline void setCameraPose(const vpPoseVector &_pose)
    {
        setCameraPose(vpHomogeneousMatrix(_pose));
    }

    void plot(vpColVector err);

    void setCameraVelocity(vpColVector _velocity);
    vpHomogeneousMatrix cameraPose();   // aka camera -> base bMc

    // Jacobian in camera frame
    int cameraJacobian(const vpColVector &_q, vpMatrix &_cJc) const ;
    vpMatrix cameraJacobian(std::vector<int> joints = {}) const
    {
        vpMatrix J(6,7);
        cameraJacobian(q_, J);
        if(joints.size() != 0)
        {
            for(int j = 0; j < 7; ++j)
            {
                if(std::find(joints.begin(), joints.end(), j) == joints.end())
                {
                    for(int i = 0; i < 6; ++i)
                        J[i][j] = 0;
                }
            }
        }
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

    inline vpColVector jointMin() const {return q_min_;}
    inline vpColVector jointMax() const {return q_max_;}
    inline vpColVector velocityMax() const {return v_max_;}
    inline double lambda() const {return 0.1*lambda_;}

    // camera part
    void detect(int r, int g ,int b, bool show_segment = false);
    double x() {return cd_.x();}
    double y() {return cd_.y();}
    double area()  {return cd_.area();}
    double area_d() const {return area_d_;}
    bool ok()
    {
        ros::spinOnce();
        loop_->sleep();
        return q_.euclideanNorm() != 0 && im_ok;
    }

 protected:
    // ROS
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Publisher cmd_pub_;
    std::unique_ptr<ros::Rate> loop_;
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

    // some checks
    bool sim_, lefty_, is_init_ = false, im_ok = false;
    double area_d_ =  0.05;    // simulation value

    // image
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
    ecn::ColorDetector cd_;

    // joint limits
    vpColVector q_min_, q_max_, v_max_;

    // internal modeling
    // Direct Kinematic Model   // aka wrist -> fixed fMw
    int fMw(const vpColVector &_q, vpHomogeneousMatrix &_M) const;
    // Classical Jacobian of wrist frame
    int fJw(const vpColVector &_q, vpMatrix &_J) const;

    // ROS functions
    void readJointStates(const sensor_msgs::JointState::ConstPtr& msg);
    void readImage(const sensor_msgs::ImageConstPtr& msg);


};



#endif // BAXTERARM_H
