#include <rclcpp/rclcpp.hpp>
#include <visp/vpFeaturePoint.h>
#include <ecn_baxter_vs/baxter_arm.h>
#include <ecn_baxter_vs/visp_utils.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>


using namespace std;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  BaxterArm arm("right");    // defaults to right arm

  vpColVector q = arm.home();

  vpColVector qmin = arm.jointMin(), qmax = arm.jointMax();

  // define a simple 2D point feature and its desired value
  vpFeaturePoint p,pd;
  pd.set_xyZ(0,0,1);

  // the error
  vpColVector e(3);
  double x, y, area;
  // desired area
  const double area_d = arm.area_d();

  // loop variables
  vpColVector qdot(7);
  vpMatrix L(3, 6), Js(3,7);

  auto control_loop = [&]()
  {
    cout << "-------------" << endl;

    // get point features
    x = arm.x();
    y = arm.y();
    area = arm.area();
    p.set_xyZ(x,y, 1);
    std::cout << "x: " << x << ", y: " << y << ", area: " << area << '\n';

    // update error vector e

    // update interaction matrix L

    // build H matrix (2nd section) using arm.rho()
    q = arm.jointPosition();

    // compute feature Jacobian from L and cameraJacobian
    const auto J = arm.cameraJacobian(q);

    // send this command to the robot
    arm.setJointVelocity(qdot);

    // display current joint positions and VS error
    arm.plot(e);
  };


  arm.setControlLoop(control_loop, 10ms);
  rclcpp::spin(arm.node());
  rclcpp::shutdown();
}
