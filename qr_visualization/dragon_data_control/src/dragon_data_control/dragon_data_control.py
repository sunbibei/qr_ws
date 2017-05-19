# -*- coding:utf-8 -*-
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QBasicTimer
import time

import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET

MAX_VALUE = 2
MIN_VALUE = -2
CMD_TOPIC_NAME = "/debug"
STATE_TOPIC_NAME = "/joint_states"

LEG_NAME = ['L-B' , 'L-F' , 'R-B' , 'R-F']
DATA_TPYE = ['Position' , 'Velocity' , 'Effort']
JOINT_NAME = ['hip', 'knee', 'yaw']

class DragonDataControl(Plugin):

    def __init__(self, context):
        super(DragonDataControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DragonDataControl')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('dragon_data_control'), 'resource', 'data_control.ui')
        self.doc_path = rp.get_path('dragon_data_control') + '/src/dragon_data_control/data.txt'
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DragonDataControl')

        #init variable
        self.dragon_pointer = {}
        self.joint_name = JOINT_NAME
        for key in self.joint_name:
            self.dragon_pointer[key] = {'name': key , 'value': 0.0}
        self.factor = (MAX_VALUE-MIN_VALUE)/100.0
        
	
	#determine the source of the change
        self._if_edit = True
        self._if_slider = True
        self._if_load = False
	
	#initial the data from the ros topic
        self.hip_data = []
        self.knee_data = []
        self.sub_msg_pos = []
        self.sub_msg_vel = []

	
	#threading
	self.lock = threading.Lock()
	
        #ros pub and sub the topic
        self.cmd_topic_name = CMD_TOPIC_NAME
        self.state_topic_name = STATE_TOPIC_NAME
	try:
	    self._publisher_command = rospy.Publisher(self.cmd_topic_name, Float64MultiArray , queue_size=10)
	except ROSException, e:
	    rospy.logerr('Dragon_data_control: Error creating publisher for topic %s (%s)'%(self._pub_topic, e))
        try:
            self.sub_command = rospy.Subscriber(self.state_topic_name, JointState, self.sub_cb)
        except ROSException, e:
            rospy.logerr('Dragon_data_control: Error connecting topic (%s)'%e)
                    
        #widget for comboBox
        self.leg_name = LEG_NAME
        self.data_tpye = DATA_TPYE
        for key in self.leg_name:
            self._widget.comboBox_leg.addItem(key)
        for key in self.data_tpye:
            self._widget.comboBox_type.addItem(key)

	#get slider limit
	self.urdf_path = rp.get_path('qr_description') + '/urdf/dragon.urdf'
	self.urdf_pointer = self.getLimit(self.urdf_path)
	self.current_name = self.getCurrentName()
	self.limits = {}
	for i in range(len(self.current_name)):
	    if "yaw" not in self.current_name[i]:
		lower = self.urdf_pointer[self.current_name[i]]['lower']
		upper = self.urdf_pointer[self.current_name[i]]['upper']
		limit_factor = (upper - lower) / 100.0
		self.limits[self.joint_name[i]] = {"limit" : limit_factor}
	
	#widget slider, lineEdit and pushButton connect
	self.slider_widget = [self._widget.horizontalSlider_hip, self._widget.horizontalSlider_knee,
	                 self._widget.horizontalSlider_yaw]
	self.lineEdit_widget = [self._widget.lineEdit_hip, self._widget.lineEdit_knee,
	                        self._widget.lineEdit_yaw]	
	for slider in self.slider_widget:
	    slider.valueChanged.connect(self.slider_changed)
	for lineEdit in self.lineEdit_widget:
	    lineEdit.textChanged.connect(self.lineEdit_changed)
        
        self._widget.pushButton_reset.clicked.connect(self.pushButton_reset)
        self._widget.pushButton_go.clicked.connect(self.pushButton_go)
        self._widget.pushButton_load.clicked.connect(self.pushButton_load)
        self._widget.pushButton_stop.clicked.connect(self.pushButton_stop)
        self._widget.pushButton_get.clicked.connect(self.pushButton_get)
        
	self.update_lineEdit()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
    def slider_changed(self):
        if self._if_slider:
	    for index in range(len(self.dragon_pointer.keys())):
		self.dragon_pointer[self.joint_name[index]]['value'] = self.slider_widget[index].value() * self.factor
            self.update_lineEdit()
        self._if_slider = True
    
    def lineEdit_changed(self):
        if self._if_edit:
	    for index in range(len(self.dragon_pointer.keys())):
		self.dragon_pointer[self.joint_name[index]]['value'] = float(self.lineEdit_widget[index].text())
            self.update_slider()
	self._if_edit = True
    
    def update_lineEdit(self):
	for index in range(len(self.lineEdit_widget)):
	    self._if_edit = False
	    self.lineEdit_widget[index].setText(str(round(self.dragon_pointer[self.joint_name[index]]['value'], 3)))
                
    def update_slider(self):
	for index in range(len(self.slider_widget)):
	    self._if_slider = False
	    self.slider_widget[index].setValue(self.dragon_pointer[self.joint_name[index]]['value']/self.factor)
        
    def pushButton_reset(self):
        self._if_edit = False
        self._if_slider = False
	for lineEdit in self.lineEdit_widget:
	    lineEdit.setText(str(0.0))
       
    def pushButton_go(self):

	current_leg = self._widget.comboBox_leg.currentText()
	current_type = self._widget.comboBox_type.currentText()
	joint_data = Float64MultiArray()
	joint_data.data = [1000,1000, 1000, 1000, 1000 , 1000, 1000, 1000,1000,1000,1000,1000]
	
	index = self.leg_name.index(current_leg) * 3
	for key in self.joint_name:
	    joint_data.data[index] = self.dragon_pointer[key]['value']
	    index += 1	
	self._publisher_command.publish(joint_data)

    def sub_cb(self,msg):
        self.sub_msg_pos = msg.position
        self.sub_msg_vel = msg.velocity

    def pushButton_get(self):
        current_leg = self._widget.comboBox_leg.currentText()
        current_type = self._widget.comboBox_type.currentText()

	index = self.leg_name.index(current_leg) * 3
	for key in self.joint_name:
	    if "Position" == current_type:
		self.dragon_pointer[key]['value'] = self.sub_msg_pos[index]
	    elif "Velocity" == current_type:
		self.dragon_pointer[key]['value'] = self.sub_msg_vel[index]
	    index += 1
	self.update_lineEdit()
	    
    def data_pub(self):
        while(self._if_load):
            msg = Float64MultiArray()
            msg.data = [0,0,0, 0, 0, 0, 0, 0,0,0,0,0]
            for i in range(len(self.hip_data)):
                if self._if_load:
                    msg.data[0] = self.hip_data[i]
                    #msg.data[1] = self.knee_data[i]
                    self._publisher_command.publish(msg)
                    time.sleep(0.03)

    def pushButton_load(self):
        try:
            with open(self.doc_path,'r') as f:
                data = f.readlines()
                hip_data_str = data[0].split()
                self.hip_data = [float(i) for i in hip_data_str]
                #knee_data_str = data[1].split(',')
                #self.knee_data = [float(i) for i in knee_data_str]
                self._if_load = True
        except Exception,e:
	    print Exception,":",e
            print 'load txt wrong! '
        try:
            self.lock.acquire()
            self.data_thread = threading.Thread(target=self.data_pub)
            self.data_thread.setDaemon(True)
            self.data_thread.start()
        except:
            print 'thread is wrong!'
        finally:
            self.lock.release()

    def pushButton_stop(self):
        self._if_load = False
	
    def getLimit(self, urdf_path):		
    	root = ET.parse(urdf_path)
    	joints = root.findall('joint')
    	joint_pointer = {}
    	for joint_list in joints:
    	    joint_children = joint_list.getchildren()
    	    for node in joint_children:
    		if node.tag == "limit":
    		    joint_pointer[joint_list.attrib['name']] = {"name" : joint_list.attrib['name'], 
    			                                        "upper" : float(node.attrib['upper']), "lower": float(node.attrib['lower']),
    			                                        "effort": float(node.attrib['effort']), "velocity": float(node.attrib['velocity'])}
    	return joint_pointer
    
    def getCurrentName(self):
    	current_leg = self._widget.comboBox_leg.currentText()
    	current_type = self._widget.comboBox_type.currentText()
    	names = []
    	if "L-F" == current_leg:
    	    name = "left_front_"
    	elif "L-B" == current_leg:
    	    name = "left_back_"
    	elif "R-F" == current_leg:
    	    name = "right_front_"
    	elif "R-B" == current_leg:
    	    name = "right_back_"
    	
    	for joint_name in self.joint_name:
    	    names.append(name + joint_name)
    	return names

