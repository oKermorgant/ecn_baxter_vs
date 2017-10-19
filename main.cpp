#include <visp/vpFeaturePoint.h>
#include <ecn_baxter_vs/baxter_arm.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <ecn_common/optim.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");

    // update your group name to work on the real Baxter
    std::string group_name = "students";

    BaxterArm arm(group_name);    // defaults to simulation with right arm
    //BaxterArm arm(group_name, false, "left");   // real robot with left arm

    arm.init(); //


    vpColVector q(7),
                qmin = arm.jointMin(),
                qmax = arm.jointMax(),
                vmax = arm.velocityMax();

    // define a simple 2D point feature and its desired value
    vpFeaturePoint p,pd;
    pd.set_xyZ(0,0,1);

    // the error
    vpColVector e(3);
    double x, y, area;
    // desired area
    double area_d = 0.003;

    // loop variables
    vpColVector qdot;
    vpMatrix L(3, 6), Js;


    while(arm.ok())
        {
            cout << "-------------" << endl;

            // get point features
            x = arm.x();
            y = arm.y();
            area = arm.area();
            p.set_xyZ(x,y, 1);
            std::cout << "x: " << x << ", y: " << y << ", area: " << area << '\n';

            e[0] = p.get_x() - pd.get_x();
            e[1] = p.get_y() - pd.get_y();
            e[2] = area - area_d;

            // interaction matrix of (x,y)
            ecn::putAt(L, p.interaction(), 0, 0);
            // simplified interaction matrix of area
            L[2][2] = 30*area;

            // feature Jacobian
            Js = L * arm.cameraJacobian();

            // to velocity setpoint
            qdot = -arm.lambda() * Js.pseudoInverse() * e;

            // task
            arm.setJointVelocity(qdot);

            // display current joint positions and VS error
            arm.plot(e);
    }

}
