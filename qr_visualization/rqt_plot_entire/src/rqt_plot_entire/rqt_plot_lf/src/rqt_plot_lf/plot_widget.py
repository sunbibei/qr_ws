import os
import rospkg
import threading
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import rospy

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

MOTOR_TOPIC_NAME = "/dragon/joint_commands"
#MOTOR_TOPIC_NAME = "/JointCommand"
ENCORDER_TOPIC_NAME = "/joint_states"
JOINT_DATA = ['hip_cmd' , 'knee_cmd' , 'yaw_cmd' , 'hip_state' , 'knee_state'  , 'yaw_state']

class RosPlotException(Exception):
    pass

class PlotWidget(QWidget):
    _redraw_interval = 100
    def __init__(self):
        super(PlotWidget , self).__init__()
        self.setObjectName('PlotWidget')
        
        #ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_plot_entire')+'/src/rqt_plot_entire/rqt_plot_lf', 'resource' , 'plot.ui')
        #ui_file = os.path.join(rp.get_path('rqt_plot_lf') , 'resource' , 'plot.ui')
        loadUi(ui_file , self)
        
        #subscribe
        self._motor_topic_name = MOTOR_TOPIC_NAME
        self._encorder_topic_name = ENCORDER_TOPIC_NAME
        try:
            self._motor_subscriber = rospy.Subscriber(self._motor_topic_name , Float64MultiArray , self._motor_cb)
            #self._motor_subscriber = rospy.Subscriber(self._motor_topic_name , JointState , self._motor_cb)
        except ValueError,e:
            rospy.logerr('rqt_plot: error connect topic (%s)'%e)
        try:
            self._encorder_subscriber = rospy.Subscriber(self._encorder_topic_name , JointState , self._encorder_cb)
        except ValueError,e:
            rospy.logerr('rqt_plot: error connect topic (%s)'%e)
            
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
        
        self._if_click = {}
        checkBox = ['hip','knee','yaw']
        for key in checkBox:
            self._if_click[key] = False
        
    def switch_data_plot_widget(self , data_plot):
        self.enable_timer(enabled = False)
        
        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()
        
        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(enabled = True)
        
        #data_x , data_y = self.next()
        self.next()
        for key in self.curve:
            if self.curve[key]['enable']:
            #self.data_plot.add_curve(self._topic_name , self._topic_name , data_x, data_y)
                self.data_plot.add_curve(self.curve[key]['topic_name'] , self.curve[key]['topic_name'] , self.curve[key]['buff_x_temp'] , self.curve[key]['buff_y_temp'])
        
        self.data_plot.redraw()
        
    def _motor_cb(self , msg):
        try:
            self.lock.acquire()
            try:
                temp_joint_cmd = ['hip_cmd', 'knee_cmd', 'yaw_cmd']
                i = 3
                for index in temp_joint_cmd:
                    #self.curve[index]['buff_y'].append(msg.position[i])
                    self.curve[index]['buff_y'].append(msg.data[i])
                    i += 1
                    self.curve[index]['buff_x'].append(rospy.get_time() - self.start_time)
            except AttributeError as e:
                self.error = RosPlotException("invalid topic data")
        
        finally:
            self.lock.release()
    
    def _encorder_cb(self , msg):
        try:
            self.lock.acquire()
            try:
                temp_joint_state = ['hip_state','knee_state', 'yaw_state']
                j = 3
                if msg.position is not []:
                    for index in temp_joint_state:
                        self.curve[index]['buff_y'].append(msg.position[j])
                        j += 1
                        self.curve[index]['buff_x'].append(rospy.get_time() - self.start_time)
            except AttributeError as e:
                self.error = RosPlotException('invalid topic data in encorder')
        finally:
            self.lock.release()
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
                for key in self.curve:
                    if self.curve[key]['enable']:
                        self.data_plot.update_values(self.curve[key]['topic_name'] , 
                                                     self.curve[key]['buff_x_temp'] , self.curve[key]['buff_y_temp'])                
                needs_redraw = True
            except RosPlotException as e:
                qWarning('PlotWidget : update_plot(): error in rosplot %s '%e)
        if needs_redraw:
            self.data_plot.redraw()
    
    @Slot(bool)
    def on_checkBox_hip_clicked(self , value):
        temp_click_name = ['hip_cmd', 'hip_state']
        if value:
            self._if_click['hip'] = True
            self.enable_timer(enabled= True)
            for index in temp_click_name:
                self.curve[index]['enable'] = value
                self.next()
                self.data_plot.add_curve(self.curve[index]['topic_name'], self.curve[index]['topic_name'], 
                                         self.curve[index]['buff_x_temp'], self.curve[index]['buff_y_temp'])
        else:
            self._if_click['hip'] = False
            for index in temp_click_name:
                self.curve[index]['enable'] = value
                self.data_plot.remove_curve(self.curve[index]['topic_name']) 
            self.update_plot()
            if True not in self._if_click.values():
                self.enable_timer(enabled= False)  
    @Slot(bool)
    def on_checkBox_knee_clicked(self , value):
        temp_click_name = ['knee_cmd', 'knee_state']
        if value:
            self._if_click['knee'] = True
            self.enable_timer(enabled= True)
            for index in temp_click_name:
                self.curve[index]['enable'] = value
                self.next()
                self.data_plot.add_curve(self.curve[index]['topic_name'], self.curve[index]['topic_name'], 
                                         self.curve[index]['buff_x_temp'], self.curve[index]['buff_y_temp'])
        else:
            self._if_click['knee'] = False
            for index in temp_click_name:
                self.curve[index]['enable'] = value
                self.data_plot.remove_curve(self.curve[index]['topic_name']) 
            self.update_plot()
            if True not in self._if_click.values():
                self.enable_timer(enabled= False)  
    
    @Slot(bool)
    def on_checkBox_yaw_clicked(self , value):
        temp_click_name = ['yaw_cmd', 'yaw_state']
        if value:
            self._if_click['yaw'] = True
            self.enable_timer(enabled= True)
            for index in temp_click_name:
                self.curve[index]['enable'] = value
                self.next()
                self.data_plot.add_curve(self.curve[index]['topic_name'], self.curve[index]['topic_name'], 
                                         self.curve[index]['buff_x_temp'], self.curve[index]['buff_y_temp'])
        else:
            self._if_click['yaw'] = False
            for index in temp_click_name:
                self.curve[index]['enable'] = value
                self.data_plot.remove_curve(self.curve[index]['topic_name']) 
            self.update_plot()
            if True not in self._if_click.values():
                self.enable_timer(enabled= False)  
    def enable_timer(self , enabled = True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
            
    def clean_up_subscribers(self):
        self._motor_subscriber.unregister()
        for key in self.curve:
            self.data_plot.remove_curve(self.curve[key]['topic_name'])
            
        
            
