# -*- coding: utf-8 -*-
import os
import rospkg
import threading
import roslib
import json

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QFileDialog

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import rospy

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

MOTOR_TOPIC_NAME = "/debug"
ENCORDER_TOPIC_NAME = "/joint_states"
FOLLOWjOINT_NAME = "/follow_joint"
JOINT_DATA = ['cmd' ,  'state']

LEG_NAME =['L-B' , 'L-F' , 'R-B' ,'R-F']
JOINT_NAME = ['hip' , 'knee' ,'yaw']
class RosPlotException(Exception):
    pass

class PlotWidget(QWidget):
    _redraw_interval = 100
    def __init__(self):
        super(PlotWidget , self).__init__()
        self.setObjectName('PlotWidget')
        
        #ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('qr_panel') , 'resource' , 'plot.ui')
        loadUi(ui_file , self)
            
        #timer
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        
        #variable
        self._joint_data = JOINT_DATA
        self.curve = {}
        for i in range(len(self._joint_data)):
            self.curve[self._joint_data[i]] = {'topic_name':self._joint_data[i] , 'buff_x':[] , 'buff_y':[] , 'buff_x_temp':[] , 'buff_y_temp' : [] ,'enable' : False}
        self.lock = threading.Lock()
        self.start_time = rospy.get_time()
        
        self.data_plot = None
        self.error = None
        self.data_list = []	
	#flag to current state
        self._if_pause = False
        self._if_load = False
        self._if_preview = False
        self._if_start = False
	self._if_pub = False
	
        #rostopic
        self.pub_topic_name = MOTOR_TOPIC_NAME
        self.sub_topic_name = ENCORDER_TOPIC_NAME
	self.follow_joint_pub_name = FOLLOWjOINT_NAME
        try:
            self.publish_command = rospy.Publisher(self.pub_topic_name, Float64MultiArray, queue_size = 10)
	except ROSException, e:
	    rospy.logerr('qr_panel: Error creating publisher for topic %s (%s)'%(self.pub_topic_name, e))
	try:
	    self.subscriber = rospy.Subscriber(self.sub_topic_name, JointState, self.ros_cb)
	except ROSException, e:
	    rospy.logerr('qr_panel: Error creating subscriber for topic %s (%s)'%(self.sub_topic_name, e))
	try:
	    self.FollowJoint_publish = rospy.Publisher(self.follow_joint_pub_name, JointTrajectoryPoint , queue_size = 10)
	except ROSException, e:
	    rospy.logerr('qr_panel: Error creating subscriber for topic %s (%s)'%(self.sub_topic_name, e))
        
        #comboBox widget
        self.leg_name = LEG_NAME
        self.joint_name = JOINT_NAME
        for key in self.leg_name:
            self.comboBox_leg.addItem(key)
        for key in self.joint_name:
            self.comboBox_joint.addItem(key)
	
	self.current_leg = self.comboBox_leg.currentText()
	self.current_joint = self.comboBox_joint.currentText()
	self.comboBox_leg.currentIndexChanged.connect(self.comboBox_changed)
	self.comboBox_joint.currentIndexChanged.connect(self.comboBox_changed)	
        
        self.pushButton_open.clicked.connect(self.pB_open)
        self.pushButton_load.clicked.connect(self.pB_load)
        self.pushButton_start.setStyleSheet("background-color: rgb(128,255,0)")
        self.label_on.setStyleSheet("background-color:rgb(255,0,0)")
        self.pushButton_start.clicked.connect(self.pB_start)
        self.pushButton_pause.clicked.connect(self.pB_pause)
        self.pushButton_preview.clicked.connect(self.pB_preview)	
     
    def switch_data_plot_widget(self , data_plot):
        self.enable_timer(enabled = False)
        
        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()
        
        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(enabled = True)    
        self.data_plot.redraw()
    
    def ros_cb(self, msg):
	try:
	    self.lock.acquire()
	    try:	
		index = self.leg_name.index(self.current_leg) * 3
		offset = self.joint_name.index(self.current_joint)
		num = index+offset
		self.curve['state']['buff_y'].append(msg.position[num])
		self.curve['state']['buff_x'].append(rospy.get_time() - self.start_time)
	    except AttributeError as e:
		self.error = RosPlotException("invalid topic data")
	finally:
	    self.lock.release()
	        
    def comboBox_changed(self):
	self.current_leg = self.comboBox_leg.currentText()
	self.current_joint = self.comboBox_joint.currentText()
	
    def pB_open(self):
        filename,filetype = QFileDialog.getOpenFileName(self, "pick document", "/home",)
        self.lineEdit_filename.setText(str(filename))
	
    def pB_load(self):
        filename = self.lineEdit_filename.text()
        final_file_name = filename.split('/')[-1]
        if ".json" in final_file_name:
            self._if_load = True
            self.enable_timer(enabled= True)
            self.data_plot.add_curve(self.curve['cmd']['topic_name'], self.curve['cmd']['topic_name'] ,
	                             self.curve['cmd']['buff_x_temp'], self.curve['cmd']['buff_y_temp'])
	    try:
		with open(filename,'r') as f:
		    self.curve_data = json.load(f)
	    except Exception, e:
		print Exception, ' : ', e
		
	    self.json2JointTrajectory(self.curve_data, self.current_leg, self.current_joint)

            try:
                self.data_list = self.curve_data[self.current_leg][self.current_joint]['value']
            except Exception,e:
		print "data_list is wrong!"
                print Exception, ":",e
                 
    def pB_start(self):
        if 'Start' == self.pushButton_start.text():
	    self.curve['state']['enable'] = True
	    self.data_plot.add_curve(self.curve['state']['topic_name'], self.curve['state']['topic_name'] ,
	                             self.curve['state']['buff_x_temp'], self.curve['state']['buff_y_temp'])
            if self._if_load:
                self.curve['cmd']['enable'] = True           
            self.pushButton_start.setText('Stop')
            self.pushButton_start.setStyleSheet("background-color: rgb(255,0,0)")
            self.label_on.setText('ON')
            self.label_on.setStyleSheet("background-color: rgb(128,255,0)")           
	    self._if_pub = True
        elif 'Stop' == self.pushButton_start.text():
            self._if_load = False
	    self._if_pub = False
            self.curve['cmd']['enable'] = False
	    self.curve['state']['enable'] = False
            self.pushButton_start.setText('Start')
            self.pushButton_start.setStyleSheet("background-color: rgb(128,255,0)")
            self.label_on.setText('OFF')
            self.label_on.setStyleSheet("background-color: rgb(255,0,0)")            
            try:
                self.data_plot.remove_curve('cmd')
                self.data_plot.remove_curve('state')
                self.update_plot()
                self.enable_timer(enabled= False)
            except Exception,e:
		print Exception, " : ", e
                print "curve dosen't exist"
                
    def pB_pause(self):
        self._if_pause = not self._if_pause
        if "PAUSE" == self.pushButton_pause.text():
            self.pushButton_pause.setText('RE-PLOT')
        else:
            self.pushButton_pause.setText('PAUSE')
    
    def pB_preview(self):
        self._if_preview = True
	if self._if_load:
	    self.curve['cmd']['enable'] = True

    def next(self):
        #return next data of topic like [xdata] [ydata]
        if self.error:
            raise self.error
        try:
            self.lock.acquire()
            for key in self.curve:
                self.curve[key]['buff_x_temp'] = self.curve[key]['buff_x']
                self.curve[key]['buff_y_temp'] = self.curve[key]['buff_y']
                self.curve[key]['buff_x'] = []
                self.curve[key]['buff_y'] = []
        finally:
            self.lock.release()
        return self.curve
    
    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            try:
		self.next()
                if not self._if_pause:
                    for each_data in self.data_list:
                        self.curve['cmd']['buff_x_temp'].append(rospy.get_time() - self.start_time)
                        self.curve['cmd']['buff_y_temp'].append(each_data)
                    for key in self.curve:
                        if self.curve[key]['enable']:
                            self.data_plot.update_values(self.curve[key]['topic_name'] , self.curve[key]['buff_x_temp'] , self.curve[key]['buff_y_temp'])
                    needs_redraw = True
                    self.curve['cmd']['buff_x_temp'] = []
                    self.curve['cmd']['buff_y_temp'] = []
            except RosPlotException as e:
                qWarning('PlotWidget : update_plot(): error in rosplot %s '%e)
	    
	    self.pub2debug()
        if needs_redraw:
            self.data_plot.redraw()
    
    def pub2debug(self):
	#publish and register the topic from robot
	if self._if_pub:
	    joint_data = Float64MultiArray()
	    joint_data.data = [0 for n in range(12)]
	    index = self.leg_name.index(self.current_leg) * 3
	    offset = self.joint_name.index(self.current_joint)
	    order = index + offset
	    if self._if_load:
		for data in self.data_list:
		    joint_data.data[order] = data
		    self.publish_command.publish(joint_data)
		    
    def json2JointTrajectory(self,curve_data, current_leg, current_joint):
	trajectory_data = JointTrajectoryPoint()
	trajectory_data.positions = [0]
	try:
	    for i in range(len(curve_data['time'])):
		trajectory_data.positions[0] = float(curve_data[current_leg][current_joint]['value'][i])
		trajectory_data.time_from_start.secs= curve_data['time'][i]
		self.FollowJoint_publish.publish(trajectory_data)
	except Exception,e:
	    print "trajectory is wrong!"
	    print Exception, ":", e
               
    def enable_timer(self , enabled = True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
            
    def clean_up_subscribers(self):
        for key in self.curve:
            self.data_plot.remove_curve(self.curve[key]['topic_name'])
	self.publish_command.unregister()
	self.subscriber.unregister()
	self.FollowJoint_publish.unregister()
            
        
            