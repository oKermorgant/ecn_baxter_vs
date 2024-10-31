from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.gz_launch(f"-r {sl.find('ecn_baxter_vs', 'baxter_world.sdf')}")

    sl.include('baxter_gz', 'upload_launch.py')

    sl.gz_world_tf()

    with sl.group(ns = 'ball'):
        bridges = [GazeboBridge('/model/ball/pose', '/tf', 'tf2_msgs/TFMessage', GazeboBridge.gz2ros),
                   GazeboBridge('/model/ball/cmd_vel', 'cmd_vel', 'geometry_msgs/Twist', GazeboBridge.ros2gz)]
        sl.create_gz_bridge(bridges)

        sl.node('slider_publisher', 'slider_publisher',
                arguments = [sl.find('ecn_baxter_vs', 'ball_setpoint.yaml')])

        sl.node('ecn_baxter_vs', 'ball_motion.py')

    return sl.launch_description()
