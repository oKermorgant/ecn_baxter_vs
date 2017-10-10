#include <visp/vpFeaturePoint.h>
#include <ecn_baxter_vs/baxter_arm.h>

using namespace std;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "control_node");

    BaxterArm arm;    // defaults to simulation with right arm
    //BaxterArm arm(false, "left");   // real robot with left arm

    arm.init();

    int it = 0;

    vpColVector q(7), qmin = arm.jointMin(), qmax = arm.jointMax();

    vpFeaturePoint p,pd;
    pd.set_xyZ(0,0,1);

    vpMatrix L, J;
    double x, y, a;
    vpColVector v(6);v[0] = .01;

    while(arm.ok())
        {
            it++;
            cout << "-------------" << endl;

            // get point pose
            x = arm.x();
            y = arm.y();
            a = arm.area();
            p.set_xyZ(x,y, 1);
            std::cout << "x: " << x << ", y: " << y << ", a: " << a << '\n';

            //arm.setCameraVelocity(v);

            // interaction matrix
            L = p.interaction();

            // robot Jacobian
       //     J = arm.cameraJacobian();

            // task
       //     v = -(L*J).pseudoInverse() * (p.error(pd));
            //arm.setJointVelocity(v);
            arm.setCameraVelocity(-arm.lambda()*L.pseudoInverse() * p.error(pd));

    }

}
