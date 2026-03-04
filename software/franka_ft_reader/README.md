# README #

1. Start the Franka Controller
```bash
roslaunch panda_moveit_config franka_and_rs.launch
```
2. Enable the cartesian impedance controller
```bash
rosrun rqt_controller_manager rqt_controller_manager 
```
3. Run your Experiment
```bash
roslaunch franka_ft_reader run_experiment.launch
```



1. Go to Google Chrome and open bookmark: https://172.16.0.2

    - Settings --> End-Effector --> Other --> Mass (of force-torque-sensor & gripper)
    
    - to talk to robot via terminal --> Activate FCI

2. Open Terminator

Start the Franka Controller
```bash
roslaunch panda_moveit_config franka_and_rs.launch
```

Enable the cartesian impedance controller
```bash
rosrun rqt_controller_manager rqt_controller_manager 
```
--> when executed [stop and unload] --> "effort_joint_trajectory_controller"
--> and [Load and Start] --> "cartesian_impedance_example_controller"


Run your Experiment
```bash
roslaunch franka_ft_reader run_experiment.launch
```

3. Open Jupiter Lab (open new terminator tab)

    - cd /home/zrene/catkin_ws/src/franka_ft_reader/eval/

    - jupyter lab

4. Open Visual Studio Code

    - in folder franka_ft_reader --> open controller_node.py

    - here you can define and write commands with self.actions = {...}
    

Comments:

- Plot also refrence commands? x,y,z and Fx,Fy,Fz!

- What are Time [steps]? How long is a step?

- Can we do something about the inaccuracies?



------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------

To log the reference positions and forces along with the real values, you need to modify the `Logger` class in the `measurement_logger` file to subscribe to the topics where these reference values are published. Then, you can include these reference values in the logged data. 

Here's how you can modify the `Logger` class to achieve this:

```python
class Logger:
    def __init__(self, output_file, msg_frequency=50):
        # Existing initialization code...

        # Subscribe to the topic where reference positions and forces are published
        self.ref_cmd_topic = "/cartesian_impedance_example_controller/pose_wrench_command"
        self.ref_cmd_sub = rospy.Subscriber(self.ref_cmd_topic, PoseWrenchCommand, self.ref_cmd_cb)

        # Initialize variables to store reference positions and forces
        self.ref_positions = np.zeros(3)
        self.ref_forces = np.zeros(3)

    def ref_cmd_cb(self, msg):
        # Store the reference positions and forces from the received message
        self.ref_positions[0] = msg.pose.position.x
        self.ref_positions[1] = msg.pose.position.y
        self.ref_positions[2] = msg.pose.position.z
        self.ref_forces[0] = msg.wrench.force.x
        self.ref_forces[1] = msg.wrench.force.y
        self.ref_forces[2] = msg.wrench.force.z

    def combined_sub(self, msg: FrankaState, wrench: WrenchStamped):
        # Existing code...

        # Append reference positions and forces to the logged data
        self._data.append([
            force[0],  # fx
            force[1],  # fy
            force[2],  # fz
            pos[0, -1],  # x pos
            pos[1, -1],  # y pos
            pos[2, -1],  # z pos
            self.ref_forces[0],  # ref_fx
            self.ref_forces[1],  # ref_fy
            self.ref_forces[2],  # ref_fz
            self.ref_positions[0],  # ref_x
            self.ref_positions[1],  # ref_y
            self.ref_positions[2],  # ref_z
        ])

        # Logging code...
```

In this modified code:

1. We subscribe to the topic where the reference positions and forces are published (`ref_cmd_topic`).
2. We define a callback function (`ref_cmd_cb`) to extract reference positions and forces from the received messages and store them.
3. In the `combined_sub` method, we append the reference positions and forces along with the real values to the logged data (`_data`).

Make sure to adjust the message types and topics according to your actual implementation if they are different.

--------------------------------------------------------------------------------------------------------------------------------------

