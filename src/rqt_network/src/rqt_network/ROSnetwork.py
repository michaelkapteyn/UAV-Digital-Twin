#!/usr/bin/env python

import os
import rclpy
import numpy as np
import threading
from rclpy.node import Node
# import rospkg
from std_msgs.msg import String
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

import pygraphviz
import tempfile

class ROSNetwork(Plugin):

    def __init__(self, context):
        super(ROSNetwork, self).__init__(context)
        self.setObjectName('ROSNetwork')


        self._context = context
        self._node = context.node
        self._widget = Network(self._node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

class Network(QWidget):
    def __init__(self, node):
        super(Network, self).__init__()
        layout = QVBoxLayout(self)
        self.static_canvas = FigureCanvas(Figure(figsize=(5, 3)))
        layout.addWidget(self.static_canvas)

        self.ax = self.static_canvas.figure.subplots()
        self.ax.axis('off')
        # t = np.linspace(0, 10, 501)
        # self.ax.plot(t, np.sin(t), ".")

        # # Create QWidget
        # # ui_file = os.path.join(rospkg.RosPack().get_path('rqt_dot'), 'resource', 'rqt_dot.ui')
        # _, package_path = get_resource('packages', 'rqt_dot')
        # ui_file = os.path.join(package_path, 'share', 'rqt_dot', 'resource', 'rqt_dot.ui')
        # loadUi(ui_file, self, {'DotWidget':DotWidget})
        # self.setObjectName('ROSDot')
        #
        # self.refreshButton.clicked[bool].connect(self._handle_refresh_clicked)
        # self.saveButton.clicked[bool].connect(self._handle_save_button_clicked)
        #
        # # flag used to zoom out to fit graph the first time it's received
        # self.first_time_graph_received = True
        # # to store the graph msg received in callback, later on is used to save the graph if needed
        # self.graph = None
        self.topic_name = 'graph_string'
        self._node = node
        self._node.create_subscription(String, self.topic_name, self.ReceivedCallback, 10)

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
        # self.ax.clear()
        G = pygraphviz.AGraph(msg.data)
        G.layout(prog='dot') # [‘neato’|’dot’|’twopi’|’circo’|’fdp’|’nop’]
        with tempfile.NamedTemporaryFile() as tf:
            G.draw(tf.name, format='png')
            self.graph = G
            img = matplotlib.image.imread(tf.name)
            self.ax.imshow(img)
            # self.ax.axis('off')

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

    # Qt methods
    def shutdown_plugin(self):
        self.graph.draw('src/digitaltwin/outputfiles/graph.png', format='png')
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
