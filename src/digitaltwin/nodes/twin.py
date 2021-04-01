#!/usr/bin/env python3
import rclpy
import numpy as np
import json
import time

from matplotlib import pyplot as plt
from rclpy.node import Node
from rclpy.time import Time
import pygraphviz

from std_msgs.msg import String
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker

from geometry_msgs.msg import PoseStamped, Point
from digitaltwin.msg import Sensor, SensorList, ControlA, ControlP, ControlPList, State, StateList, Reward
from digitaltwin.graphicalmodel import GraphicalModel
from digitaltwin.planner import Planner

class Twin(Node):
    def __init__(self):
        super().__init__('uav_twin')

        ## Instantiate the graphical model
        self.gm = GraphicalModel("DT")

        ## Instantiate a planner, passing in the graphical model information
        self.planner = Planner(self.gm)
        self.gm.policy = self.planner.getPolicy()
        # self.get_logger().info('computed a Policy!')

        ## Publishers
        self.controlA_publisher = self.create_publisher(ControlA, 'control_data', 10)
        self.controlP_publisher = self.create_publisher(ControlPList, 'control_estimate', 10)
        self.state_publisher = self.create_publisher(StateList, 'state_estimate', 10)

        ## Ref. Observation publisher
        self.sensor_ref_publisher = self.create_publisher(SensorList, 'sensor_ref', 10)

        ## Reward publisher
        self.reward_publisher = self.create_publisher(Reward, 'reward_estimate', 10)

        self.gm_publisher = self.create_publisher(String, 'graph_string', 10)

        # timer_period = 2.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timestep = -1
        self.prediction_timestep = -1
        self.prediction_length = 10
        self.controlA = [0]

        ## Sensor Data Subscriber
        self.sensor_listener = self.create_subscription(Sensor,'sensor_data', self.sensor_callback,10)
        self.sensor_listener  # prevent unused variable warning

        time.sleep(2)
        self.process_new_state()
        self.gm.compute_marginals()
        self.process_and_publish_control()


    def choose_action(self):
        # choose an action to take based on max of updated marginal
        self.controlA = [int(np.argmax(a)) for a in self.gm.marginals["ControlP {}".format(self.timestep)]]
    """
    Callback Functions
    """

    def process_new_state(self):
        self.gm.process_new_state()
        self.timestep += 1


    def process_and_publish_control(self):
        # choose an action based on our estimated control policy, add to the graph as a controlA node
        self.choose_action()
        self.gm.process_new_control(self.controlA)
        self.prediction_timestep = self.timestep+self.prediction_length
        self.gm.prepare_prediction(self.prediction_timestep)
        # publish control and data for viz
        self.publish_controlA()


    def sensor_callback(self, msg):
        self.gm.process_new_observation(msg.data)
        #start next timestep
        self.process_new_state()
        self.gm.compute_marginals()
        self.publish_state_estimate()
        self.publish_sensor_ref()
        self.publish_reward_estimate()
        self.publish_control_estimate()
        self.publish_graph()

        # publish next control
        self.process_and_publish_control()

    def publish_graph(self):
        gm_string = String()
        gm_string.data = self.gm.to_string()
        self.gm_publisher.publish(gm_string)

    def publish_state_estimate(self):
        state_list_msg = StateList()
        state_list_msg.states = []

        for t in range(self.prediction_timestep+1):
            # print("Joint at time {}".format(t))
            # print(self.gm.state_joints["Damage {}".format(t)])
            # print("Marginals at time {}".format(t))
            # print(self.gm.marginals["Damage {}".format(t)][0])
            # print(self.gm.marginals["Damage {}".format(t)][1])
            state_msg = State()
            state_msg.state1 = self.gm.marginals["Damage {}".format(t)][0]
            state_msg.state2 = self.gm.marginals["Damage {}".format(t)][1]
            state_msg.joint = self.gm.state_joints["Damage {}".format(t)].flatten().tolist()

            if t < self.timestep:
                # this is an estimate of a past timestep
                state_msg.type = 1
            else:
                #this is a prediction
                state_msg.type = 2
            state_list_msg.states.append(state_msg)

        self.state_publisher.publish(state_list_msg)

    def publish_control_estimate(self):
        control_list_msg = ControlPList()
        control_list_msg.controls = []

        for t in range(self.prediction_timestep+1):
            control_msg = ControlP()
            control_msg.control = list(self.gm.marginals["ControlP {}".format(t)][0])
            if t < self.timestep:
                # this is an estimate of a past timestep
                control_msg.type = 1
            else:
                #this is a prediction
                control_msg.type = 2
            control_list_msg.controls.append(control_msg)
        self.controlP_publisher.publish(control_list_msg)

    def publish_sensor_ref(self):
        sensor_ref_msg_list = SensorList()
        sensor_ref_msg_list.datas = []
        for t in range(self.prediction_timestep+1):
            sensor_ref_msg = Sensor()
            if t < self.timestep:
                # this is an estimate of a past timestep
                sensor_ref_msg.type = 1
            else:
                #this is a prediction
                sensor_ref_msg.type = 2
            ref_obs = None
            ref_obs_var = None
            for key,val in self.gm.marginals["Ref. Observation {}".format(t)].items():
                if ref_obs is None:
                    ref_obs = val*np.array(json.loads(key))
                else:
                    ref_obs += val*np.array(json.loads(key))

                if ref_obs_var is None:
                    ref_obs_var = val*np.power(np.array(json.loads(key)),2)
                else:
                    ref_obs_var += val*np.power(np.array(json.loads(key)),2)
            ref_obs_var -= np.power(ref_obs,2)


            # maxval = 0
            # for key,val in self.gm.marginals["Ref. Observation {}".format(t)].items():
            #     if val > maxval:
            #         ref_obs = np.array(json.loads(key))
            sensor_ref_msg.data = ref_obs.astype(float)
            sensor_ref_msg.vars = ref_obs_var.astype(float)
            sensor_ref_msg_list.datas.append(sensor_ref_msg)

        self.sensor_ref_publisher.publish(sensor_ref_msg_list)

    def publish_reward_estimate(self):
        msg = Reward()
        totalreward = [[],[]]
        statereward = [[],[]]
        controlreward = [[],[]]
        policyreward = [[],[]]
        outputerrorreward = [[],[]]
        type = []

        for t in range(self.prediction_timestep+1):
            # r, s, c, p, o = self.gm.evaluate_reward(t)
            R, R_var, R_health, R_health_var, R_control,R_control_var, R_error,R_error_var = self.gm.evaluate_reward(t)
            totalreward[0].append(float(R))
            totalreward[1].append(float(R_var))

            statereward[0].append(float(R_health))
            statereward[1].append(float(R_health_var))

            controlreward[0].append(float(R_control))
            controlreward[1].append(float(R_control_var))

            policyreward[0].append(0.0)
            policyreward[1].append(0.0)

            outputerrorreward[0].append(float(R_error))
            outputerrorreward[1].append(float(R_error_var))

            if t < self.timestep:
                # this is an estimate of a past timestep
                type.append(1)
            else:
                #this is a prediction
                type.append(2)

        msg.total = totalreward[0]
        msg.total_var = totalreward[1]

        msg.state = statereward[0]
        msg.state_var = statereward[1]

        msg.control = controlreward[0]
        msg.control_var = controlreward[1]

        msg.policy = policyreward[0]
        msg.policy_var = policyreward[1]

        msg.outputerror = outputerrorreward[0]
        msg.outputerror_var = outputerrorreward[1]

        msg.type = type

        self.reward_publisher.publish(msg)

    def publish_controlA(self):
        msg = ControlA()
        msg.data = self.controlA
        self.controlA_publisher.publish(msg)
        self.get_logger().info('Twin published control data for timestep {} : {}'.format(self.timestep, msg.data))


def main(args=None):
    rclpy.init(args=args)
    uav_twin = Twin()
    try:
        rclpy.spin(uav_twin)
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        uav_twin.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # uav_twin.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
