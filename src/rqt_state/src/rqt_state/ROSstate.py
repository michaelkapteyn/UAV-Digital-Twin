#!/usr/bin/env python

import os
import rclpy
import numpy as np
import threading
from rclpy.node import Node
# import rospkg
from std_msgs.msg import String
from digitaltwin.msg import State, StateList
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

from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

import pygraphviz
import tempfile

class ROSState(Plugin):

    def __init__(self, context):
        super(ROSState, self).__init__(context)
        self.setObjectName('ROSState')


        self._context = context
        self._node = context.node
        self._widget = StateWidget(self._node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

class StateWidget(QWidget):
    def __init__(self, node):
        super(StateWidget, self).__init__()
        layout = QVBoxLayout(self)
        self.static_canvas = FigureCanvas(Figure(figsize=(5, 3)))
        layout.addWidget(self.static_canvas)
        self.state_truth = [[],[]]
        self.state_estimate = [[],[]]
        self.state_estimate_means = [[],[]]
        self.state_estimate_stddevs = [[],[]]
        self.state_prediction = [[],[]]
        self.state_prediction_stddevs = [[],[]]
        self.state_prediction_means = [[],[]]
        self.estimate_types = []

        self.ax = self.static_canvas.figure.subplots(2,1)

        for ax in self.ax:
            ax.set_xlim(0,50)
            ax.set_ylim(-0.1,4.1)
            ax.legend()
            ax.set_yticks([0,2.5,5])
            ax.set_yticklabels(['0%','50%','100%'])
            ax.set_xlabel('Time')

            ax.grid()
        self.ax[0].set_ylabel('Component 1 State')
        self.ax[1].set_ylabel('Component 2 State')

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
        self._node = node
        self._node.create_subscription(StateList, 'state_estimate', self.ReceivedEstimateCallback, 10)
        self._node.create_subscription(StateList, 'state_truth', self.ReceivedTruthCallback, 10)

        # rclpy.spin_once(self._sub)
        #
        # # inform user that no graph has been received by drawing a single node in the rqt
        # self.gen_single_node('no dot received')

    def plot(self):
        for i in range(2):
            self.ax[i].clear()
            self.ax[i].set_xlim(0,50)
            self.ax[i].set_ylim(-0.1,4.1)
            self.ax[i].set_yticks([0,2.5,5])
            self.ax[i].set_yticklabels(['0%','50%','100%'])
            self.ax[i].set_xlabel('Time')
            self.ax[i].grid()
            self.ax[i].set_ylabel('Component {} State'.format(i+1))
            n_estimates = len(self.state_estimate_means[i])
            n_predictions = len(self.state_prediction_means[i])
            self.ax[i].plot(range(n_estimates-1,n_estimates+n_predictions-1),self.state_prediction_means[i], 'r--', linewidth=2, label='Predicted')
            self.ax[i].plot(range(n_estimates),self.state_estimate_means[i], 'b-', linewidth=2, label='Estimated')
            self.ax[i].plot(range(len(self.state_truth[i])),self.state_truth[i],'k', linewidth=2, label='Ground Truth')

            ci_estimate = 2*np.array(self.state_estimate_stddevs[i])
            ci_prediction = 2*np.array(self.state_prediction_stddevs[i])

            self.ax[i].fill_between(range(n_estimates), np.array(self.state_estimate_means[i])-ci_estimate, np.array(self.state_estimate_means[i])+ci_estimate, color='b', alpha=.1)
            self.ax[i].fill_between(range(n_estimates-1,n_estimates+n_predictions-1), np.array(self.state_prediction_means[i])-ci_prediction, np.array(self.state_prediction_means[i])+ci_prediction, color='r', alpha=.1)
            self.ax[i].legend()



        self.static_canvas.figure.savefig("/mnt/hgfs/michaelkapteyn/digitaltwin_ws/src/digitaltwin/outputfiles/figures/state_plot_{}.svg".format(n_estimates), format='svg',transparent=True)
        self.static_canvas.draw()
        self.static_canvas.draw_idle()


    def ReceivedEstimateCallback(self, msg):
        # '''
        # updating figure
        # '''
        # # save graph in member variable in case user clicks save button later
        # clear the axis
        self.types = [s.type for s in msg.states]
        n_estimates = sum(1 if t==1. else 0 for t in self.types)
        self.state_estimate[0]= [m.state1 for m in msg.states[:n_estimates]]
        self.state_estimate[1] = [m.state2 for m in msg.states[:n_estimates]]
        self.state_prediction[0] = [m.state1 for m in msg.states[n_estimates-1:]]
        self.state_prediction[1] = [m.state2 for m in msg.states[n_estimates-1:]]
        self.joints = [np.array(m.joint).reshape(5,5) for m in msg.states]

        self.state_estimate_means[0] = [np.dot(range(5), m) for m in self.state_estimate[0]]
        self.state_estimate_means[1] = [np.dot(range(5), m) for m in self.state_estimate[1]]

        self.state_estimate_stddevs[0] =[np.power(np.dot(np.power(range(5),2), m) - np.power(np.dot(range(5), m),2),0.5) for m in self.state_estimate[0]]
        self.state_estimate_stddevs[1] =[np.power(np.dot(np.power(range(5),2), m) - np.power(np.dot(range(5), m),2),0.5) for m in self.state_estimate[1]]

        self.state_prediction_means[0] = [np.dot(range(5), m) for m in self.state_prediction[0]]
        self.state_prediction_means[1] = [np.dot(range(5), m) for m in self.state_prediction[1]]

        self.state_prediction_stddevs[0] =[np.power(np.dot(np.power(range(5),2), m) - np.power(np.dot(range(5), m),2),0.5) for m in self.state_prediction[0]]
        self.state_prediction_stddevs[1] =[np.power(np.dot(np.power(range(5),2), m) - np.power(np.dot(range(5), m),2),0.5) for m in self.state_prediction[1]]

        # self.state_estimate_means[0] = [np.argmax(m) for m in self.state_estimate[0]]
        # self.state_estimate_means[1] = [np.argmax(m) for m in self.state_estimate[1]]
        #
        # self.state_prediction_means[0] = [np.argmax(m) for m in self.state_prediction[0]]
        # self.state_prediction_means[1] = [np.argmax(m) for m in self.state_prediction[1]]
        self.plot()



    def ReceivedTruthCallback(self, msg):
        # save the data
        self.state_truth[0] = [s.state1[0] for s in msg.states]
        self.state_truth[1] = [s.state2[0] for s in msg.states]


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

    def plot_ellipses(self):
        for j,t in zip(self.joints[-1:0:-1], self.types[-1:0:-1]):
            if t == 1:
                self.confidence_ellipse(j, n_std = 1.0, edgecolor='b')
            else:
                self.confidence_ellipse(j, n_std = 1.0, edgecolor='r', linestyle='--')

    def confidence_ellipse(self, j, n_std=1.0, edgecolor='k', linestyle='-', **kwargs):
        # """
        # Create a plot of the covariance confidence ellipse of *x* and *y*.
        #
        # Parameters
        # ----------
        # j : joint  probability  table
        #     Input data.
        #
        # ax : matplotlib.axes.Axes
        #     The axes object to draw the ellipse into.
        #
        # n_std : float
        #     The number of standard deviations to determine the ellipse's radiuses.
        #
        # **kwargs
        #     Forwarded to `~matplotlib.patches.Ellipse`
        #
        # Returns
        # -------
        # matplotlib.patches.Ellipse
        # """

        EX = np.dot(np.sum(j,1),np.arange(5));
        EX2 = np.dot(np.sum(j,1),[0,1,4,9,16]);
        EY = np.dot(np.sum(j,0),np.arange(5));
        EY2 = np.dot(np.sum(j,0),[0,1,4,9,16]);
        EXY = sum([j[x,y]*x*y for x in range(5) for y in range(5)])

        cov = np.zeros((2,2))
        cov[0,0] = EX2 - pow(EX,2)
        cov[1,1] = EY2 - pow(EY,2)
        cov[0,1] = EXY - EX*EY
        cov[1,0]  = cov[0,1]
        # if x.size != y.size:
        #     raise ValueError("x and y must be the same size")
        #
        # cov = np.cov(x, y)
        if cov[0,0] < 1e-3 or cov[1, 1] < 1e-3:
            return
        pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
        # Using a special case to obtain the eigenvalues of this
        # two-dimensionl dataset.
        ell_radius_x = np.sqrt(1 + pearson)
        ell_radius_y = np.sqrt(1 - pearson)
        ellipse = Ellipse((0,0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                          facecolor='none', edgecolor=edgecolor,linestyle=linestyle, **kwargs)

        # Calculating the stdandard deviation of x from
        # the squareroot of the variance and multiplying
        # with the given number of standard deviations.
        scale_x = np.sqrt(cov[0, 0]) * n_std
        mean_x = EX

        # calculating the stdandard deviation of y ...
        scale_y = np.sqrt(cov[1, 1]) * n_std
        mean_y = EY

        transf = transforms.Affine2D() \
            .rotate_deg(45) \
            .scale(scale_x, scale_y) \
            .translate(mean_x, mean_y)

        ellipse.set_transform(transf + self.ax.transData)
        self.ax.add_patch(ellipse)
        return
