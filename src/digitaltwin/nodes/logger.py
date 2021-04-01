#!/usr/bin/env python3
import rclpy
import json
import numpy as np
from rclpy.node import Node
from datetime import datetime
from digitaltwin.msg import Sensor, SensorList, ControlA, State, StateList, Reward

class Logger(Node):
    def __init__(self):
        super().__init__('uav_logger')
        self.log_fpath = './src/digitaltwin/outputfiles/'
        self.timestep = 0
        self.data = {}
        self.data["sensor_data"] = []
        self.data["sensor_ref"] = []
        self.data["state_truth"] = [[],[]]
        self.data["state_estimate"] = [[],[]]
        self.data["state_estimate_joints"] = []
        self.data["state_estimate_types"] = []
        self.data["reward_estimate"] = {}
        self.data["reward_estimate"]["total"] = []
        self.data["reward_estimate"]["state"] = []
        self.data["reward_estimate"]["control"] = []
        self.data["reward_estimate"]["policy"] = []
        self.data["reward_estimate"]["outputerror"] = []
        self.data["control_data"] = []

        # Listeners
        self.sensor_listener = self.create_subscription(Sensor,'sensor_data', self.sensor_callback, 10)
        self.ref_sensor_listener = self.create_subscription(SensorList,'sensor_ref', self.ref_sensor_callback,10)
        self.state_truth_listener = self.create_subscription(StateList,'state_truth', self.state_truth_callback,10)
        self.state_listener = self.create_subscription(StateList,'state_estimate', self.state_callback,10)
        self.reward_listener = self.create_subscription(Reward, 'reward_estimate', self.reward_callback, 10)
        self.control_listener = self.create_subscription(ControlA, 'control_data', self.control_callback, 10)

    def sensor_callback(self, msg):
        self.data["sensor_data"].append(list(msg.data))
        self.timestep += 1
        # self.get_logger().info('UAV published sensor data for timestep {} : {}'.format(self.timestep, msg.data))

    def ref_sensor_callback(self, msg):
        self.data["sensor_ref"] = [list(m.data) for m in msg.datas]

    def state_truth_callback(self, msg):
        self.data["state_truth"][0] = [m.state1[0] for m in msg.states]
        self.data["state_truth"][1] = [m.state2[0] for m in msg.states]

    def state_callback(self, msg):
        self.data["state_estimate"][0].append([list(m.state1) for m in msg.states])
        self.data["state_estimate"][1].append([list(m.state2) for m in msg.states])
        self.data["state_estimate_types"].append([m.type for m in msg.states])
        self.data["state_estimate_joints"].append([list(m.joint) for m in msg.states])

    def reward_callback(self, msg):
        self.data["reward_estimate"]["total"].append(list(msg.total))
        self.data["reward_estimate"]["state"].append(list(msg.state))
        self.data["reward_estimate"]["control"].append(list(msg.control))
        self.data["reward_estimate"]["policy"].append(list(msg.policy))
        self.data["reward_estimate"]["outputerror"].append(list(msg.outputerror))

    def control_callback(self, msg):
        self.data["control_data"].append([int(a) for a in msg.data][0])
        # self.get_logger().info('UAV recieved control input: {}'.format(msg.data))

    def write_data(self):
        self.log_fname = datetime.now().strftime(self.log_fpath + '%m%d_T%H%M%S.json')
        self.data["T"] = int(self.timestep)
        with open(self.log_fname, 'w') as fp:
            json.dump(self.data, fp,indent=4, sort_keys=True)


def main(args=None):
    rclpy.init(args=args)
    uav_logger = Logger()
    try:
        rclpy.spin(uav_logger)
    except KeyboardInterrupt:
        uav_logger.get_logger().info('Shutdown detected...Logger saving data.')
        uav_logger.write_data()
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        uav_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
