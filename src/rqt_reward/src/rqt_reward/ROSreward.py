#!/usr/bin/env python

import os
import rclpy
import numpy as np
import threading
from rclpy.node import Node
# import rospkg

from digitaltwin.msg import Reward
# reused from ros smach
from ament_index_python.resources import get_resource
# common imports that work for both versions PyQt4 and PyQt5
from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtWidgets import QWidget, QFileDialog,QVBoxLayout
from rqt_gui_py.plugin import Plugin

import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvas
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
plt.rcParams.update({'font.size': 22})
plt.rcParams['svg.fonttype'] = 'none'

import pygraphviz
import tempfile

class ROSReward(Plugin):

    def __init__(self, context):
        super(ROSReward, self).__init__(context)
        self.setObjectName('ROSReward')


        self._context = context
        self._node = context.node
        self._widget = RewardWidget(self._node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

class RewardWidget(QWidget):
    def __init__(self, node):
        super(RewardWidget, self).__init__()
        layout = QVBoxLayout(self)
        self.static_canvas = FigureCanvas(Figure(figsize=(5, 3)))
        layout.addWidget(self.static_canvas)

        self.ax = self.static_canvas.figure.subplots()
        self.ax.set_xlim(0,50)
        # self.ax.set_ylim(0,5)
        self.ax.set_title('Reward Functions')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Reward')
        self.ax.legend()
        self.ax.grid()

        self.reward_total = []
        self.reward_total_var = []

        self.reward_state = []
        self.reward_state_var = []

        self.reward_control = []
        self.reward_control_var = []

        self.reward_policy = []
        self.reward_policy_var = []

        self.reward_outputerror = []
        self.reward_outputerror_var = []

        self.reward_types = []

        self.topic_name = 'reward_estimate'
        self._node = node
        self._node.create_subscription(Reward, self.topic_name, self.ReceivedCallback, 10)

        # rclpy.spin_once(self._sub)
        #
        # # inform user that no graph has been received by drawing a single node in the rqt
        # self.gen_single_node('no dot received')


    def ReceivedCallback(self, msg):
        # '''
        # updating figure
        # '''
        # # save graph in member variable in case user clicks save button later
        # clear the axis
        self.reward_total.append(msg.total)
        self.reward_total_var.append(msg.total_var)

        self.reward_state.append(msg.state)
        self.reward_state_var.append(msg.state_var)

        self.reward_control.append(msg.control)
        self.reward_control_var.append(msg.control_var)

        self.reward_policy.append(msg.policy)
        self.reward_policy_var.append(msg.policy_var)

        self.reward_outputerror.append(msg.outputerror)
        self.reward_outputerror_var.append(msg.outputerror_var)

        self.reward_types.append(msg.type)
        n_estimates = sum(1 if t==1. else 0 for t in self.reward_types[-1])

        xx = range(0,len(msg.total))
        self.ax.clear()
        # self.ax.plot(xx[:n_estimates],self.reward_total[-1][:n_estimates], 'k-', linewidth=3, label='Total')
        # self.ax.fill_between(xxref[:n_estimates], np.array(mean_estimate)-ci_estimate, np.array(mean_estimate)+ci_estimate, color='b', alpha=.1)

        self.ax.plot(xx[:n_estimates], self.reward_state[-1][:n_estimates],'b-', linewidth=2, label='State')
        ci = 2.0*np.sqrt(self.reward_state_var[-1][:n_estimates])
        self.ax.fill_between(xx[:n_estimates], np.array(self.reward_state[-1][:n_estimates])-ci, np.array(self.reward_state[-1][:n_estimates])+ci, color='b', alpha=.1)

        self.ax.plot(xx[:n_estimates], self.reward_control[-1][:n_estimates],'g-', linewidth=2, label='Control')
        ci = 2.0*np.sqrt(self.reward_control_var[-1][:n_estimates])
        self.ax.fill_between(xx[:n_estimates], np.array(self.reward_control[-1][:n_estimates])-ci, np.array(self.reward_control[-1][:n_estimates])+ci, color='g', alpha=.1)

        # self.ax.plot(xx[:n_estimates], self.reward_policy[-1]:n_estimates],'m-', linewidth=2, label='Policy')

        self.ax.plot(xx[:n_estimates], self.reward_outputerror[-1][:n_estimates],'r-', linewidth=2, label='Error')
        ci = 2.0*np.sqrt(self.reward_outputerror_var[-1][:n_estimates])
        self.ax.fill_between(xx[:n_estimates], np.array(self.reward_outputerror[-1][:n_estimates])-ci, np.array(self.reward_outputerror[-1][:n_estimates])+ci, color='r', alpha=.1)

        # self.ax.plot(xx[n_estimates-1:],self.reward_total[-1][n_estimates-1:], 'k--', linewidth=3, label='Total')
        self.ax.plot(xx[n_estimates-1:], self.reward_state[-1][n_estimates-1:],'b--', linewidth=2, label='State')
        ci = 2.0*np.sqrt(self.reward_state_var[-1][n_estimates-1:])
        self.ax.fill_between(xx[n_estimates-1:], np.array(self.reward_state[-1][n_estimates-1:])-ci, np.array(self.reward_state[-1][n_estimates-1:])+ci, color='b', alpha=.1)

        self.ax.plot(xx[n_estimates-1:], self.reward_control[-1][n_estimates-1:],'g--', linewidth=2, label='Control')
        ci = 2.0*np.sqrt(self.reward_control_var[-1][n_estimates-1:])
        self.ax.fill_between(xx[n_estimates-1:], np.array(self.reward_control[-1][n_estimates-1:])-ci, np.array(self.reward_control[-1][n_estimates-1:])+ci, color='g', alpha=.1)

        # self.ax.plot(xx[:n_estimates], self.reward_policy[-1]:n_estimates], 'm--', linewidth=2, label='Policy')
        # self.ax.plot(xx[n_estimates-1:], self.reward_outputerror[-1][n_estimates-1:],'r--', linewidth=2, label='Error')

        self.ax.set_xlim(0,50)
        # self.ax.set_ylim(0,5)
        self.ax.set_title('Reward Functions')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Reward')
        self.ax.legend()
        self.ax.grid()

        self.static_canvas.figure.savefig("/mnt/hgfs/michaelkapteyn/digitaltwin_ws/src/digitaltwin/outputfiles/figures/reward_plot_{}.svg".format(n_estimates), format='svg',transparent=True)
        self.static_canvas.draw_idle()

    def _handle_refresh_clicked(self, checked):
        '''
        called when the refresh button is clicked
        '''
        # self._sub = FigSub(self,self.topicText.text())
        pass

    # def save_graph(self, full_path):
    #     '''
    #     check if last graph msg received is valid (non empty), then save in file.dot
    #     '''
    #     if self.graph:
    #         dot_file = open(full_path,'w')
    #         dot_file.write(self.graph)
    #         dot_file.close()
    #         # rospy.loginfo('graph saved succesfully in %s', full_path)
    #     else:
    #         pass
    #         # if self.graph is None it will fall in this case
    #         # rospy.logerr('Could not save Graph: is empty, currently subscribing to: %s, try' +\
    #                      # ' clicking "Update subscriber" button and make sure graph is published at least one time'\
    #                      # , self.topicText.text())
    #
    # def _handle_save_button_clicked(self, checked):
    #     '''
    #     called when the save button is clicked
    #     '''
    #     # rospy.loginfo('Saving graph to dot file')
    #     fileName = QFileDialog.getSaveFileName(self, 'Save graph to dot file','','Graph xdot Files (*.dot)')
    #     if fileName[0] == '':
    #         pass
    #         # rospy.loginfo("User has cancelled saving process")
    #     else:
    #         # add .dot at the end of the filename
    #         full_dot_path = fileName[0]
    #         if not '.dot' in full_dot_path:
    #             full_dot_path += '.dot'
    #         # rospy.loginfo("path to save dot file: %s", full_dot_path)
    #         self.save_graph(full_dot_path)
    #
    # Qt methods
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
