#!/usr/bin/env python
""" Dummy Node that subscribes and logs messages to .csv """
import numpy as np
import rospy
from franka_msgs.msg import FrankaState

import pandas as pd
import time
from time import strftime
from std_srvs.srv import SetBool

class Logger:
    def __init__(self, output_file, msg_frequency = 50):
        self.state_topic = "/franka_state_controller/franka_states"
        self.msg_freq = msg_frequency
        self.output_file = output_file
        print("Logger subscribed to", self.state_topic )
        print("Logging to", self.output_file)
        self._state_sub = rospy.Subscriber(self.state_topic, FrankaState, self.state_cb)

        self.last_call = time.time()
        self._data = []
        with open(output_file, "w") as f:
            f.write("Fx,Fy,Fz,x,y,z\n")
        self.running = False
        self._srv = rospy.Service("/measurement_logger/start_logging", SetBool, self.enable_service_call)

    def enable_service_call(self, data):
        self.running = data.data
        self.last_call = time.time()
        if self.running:
            print("Logger is now running")
        else:
            print("Logger is now stopped. Logged to ", self.output_file)
        return True, "Logger is now running" if self.running else "Logger is now stopped"
    
    def state_cb(self, msg: FrankaState):
        if not self.running:
            rospy.logwarn_throttle(5, "Logger is not running yet. Waiting for service call.")
            return

        diff = time.time() - self.last_call
        if diff < 1 / self.msg_freq:
            return
        
        force = np.array(msg.O_F_ext_hat_K)
        pos = np.array(msg.O_T_EE, order="C").reshape(4,4).T
        
        self._data.append([
            force[0], # fx
            force[1], # fy
            force[2], # fz
            pos[0, -1], # x pos
            pos[1, -1], # y pos
            pos[2, -1], # z pos
        ])

        with open(self.output_file, "a") as f:
            f.write(",".join([str(s) for s in self._data[-1]]) + "\n")
        

    def run(self):
        print("Logger is up and running")
        rospy.spin()

def main():
    rospy.init_node("measurement_logger")
    ts_str = strftime("%Y%m%d%H%M")
    file_name = f"log_{ts_str}.csv"
    logger = Logger(output_file = f"/home/zrene/catkin_ws/src/franka_ft_reader/output/{file_name}" )
    logger.run()



if __name__ == "__main__":
    main()
strftime("%Y%m%d%H%M")