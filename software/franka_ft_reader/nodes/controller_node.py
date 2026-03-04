#!/usr/bin/env python
import rospy
from franka_example_controllers.msg import PoseWrenchCommand
from std_srvs.srv import SetBool


def get_force_command(force_x, force_y, force_z, quat_x, quat_y, quat_z, quat_w):
    cmd = PoseWrenchCommand()
    cmd.pose.orientation.x=quat_x
    cmd.pose.orientation.y=quat_y
    cmd.pose.orientation.z=quat_z
    cmd.pose.orientation.w=quat_w
    cmd.wrench.force.x=force_x
    cmd.wrench.force.y=force_y
    cmd.wrench.force.z =force_z
    cmd.force_selection = [True, True, True]
    return cmd

def get_tracking_command(pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w):
    cmd = PoseWrenchCommand()
    cmd.pose.orientation.x=quat_x
    cmd.pose.orientation.y=quat_y
    cmd.pose.orientation.z=quat_z
    cmd.pose.orientation.w=quat_w
    cmd.pose.position.x=pos_x
    cmd.pose.position.y=pos_y
    cmd.pose.position.z=pos_z
    cmd.force_selection = [False, False, False]
    return cmd

def get_hybrid_command(pos_x, pos_y, force_z, quat_x, quat_y, quat_z, quat_w):
    cmd = PoseWrenchCommand()
    cmd.pose.orientation.x=quat_x
    cmd.pose.orientation.y=quat_y
    cmd.pose.orientation.z=quat_z
    cmd.pose.orientation.w=quat_w
    cmd.pose.position.x=pos_x
    cmd.pose.position.y=pos_y
    cmd.wrench.force.z = force_z
    cmd.force_selection = [False, False, True]
    return cmd

class RosControllerNode:
    """Simple controller node for the dynaarm."""

    def __init__(self, dt) -> None:
        self.timesteps = dt
        self.elapsed_time = 0
        self.cmd_topic = "/cartesian_impedance_example_controller/pose_wrench_command"
        self.pub = rospy.Publisher(self.cmd_topic, PoseWrenchCommand)
        # wait for service
        rospy.wait_for_service("/measurement_logger/start_logging")
        self.logger_srv = rospy.ServiceProxy("/measurement_logger/start_logging", SetBool)

        x_pos = 0.5
        y_pos = 0.0
        default_orientation = [1,0,0,0] 
        z_force = -2
        
        self.actions = {
            0: get_hybrid_command(x_pos, y_pos, z_force, *default_orientation),
            2: get_hybrid_command(x_pos, y_pos, z_force - 0.5, *default_orientation),
            2.5: get_hybrid_command(x_pos, y_pos, z_force - 1, *default_orientation),
            3: get_hybrid_command(x_pos, y_pos, z_force - 1.5, *default_orientation),
            3.5: get_hybrid_command(x_pos, y_pos, z_force - 2, *default_orientation),
            4: get_hybrid_command(x_pos, y_pos, z_force - 2.5, *default_orientation),
            4.5: get_hybrid_command(x_pos, y_pos, z_force - 3.0, *default_orientation),
            5: get_tracking_command(x_pos, y_pos, 0.1, *default_orientation),
        }
        self.switch_times = list(self.actions.keys())
        self.actions_msgs = list(self.actions.values())
        self.current_action_idx = 0
        self.logger_srv(True)


    def process(self) -> None:
        self.elapsed_time += self.timesteps
        if self.elapsed_time >= self.switch_times[self.current_action_idx]:
            self.current_action_idx += 1

            if self.current_action_idx >= len(self.switch_times):
                self.logger_srv(False)
                print("Done!")
                exit()

            print("Switching to action #", self.current_action_idx)
        msg = self.actions_msgs[self.current_action_idx]
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("controller_node", anonymous=True)
    frequency = rospy.get_param("~frequency", 30)
    node = RosControllerNode(dt=1/frequency)
    rospy.loginfo(f"Controller node up and running. Running inference at {frequency} Hz.")
    rospy.Timer(rospy.Duration(1.0 / frequency), lambda _: node.process())
    # Currently this only runs one thread. Make sure to not have a bottleneck here.
    # If there is an issue, we should consider using c++ for state processing and python for controller inference.
    rospy.spin()
