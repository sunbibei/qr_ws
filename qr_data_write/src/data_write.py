#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

CMD_NAME = "/dragon/joint_angle_command"
STATE_NAME = '/joint_states'

class dataWrite():
    def __init__(self):
        rospy.init_node('data_write', anonymous=True)
        self.start_time = rospy.get_time()
        self.cmd_sub = rospy.Subscriber(CMD_NAME, Float64MultiArray, self.cb_cmd)
        self.state_sub = rospy.Subscriber(STATE_NAME, JointState, self.cb_state)
        #path of pkg
        file_path = rospkg.RosPack().get_path('qr_data_write') + "/data/"
        #document
        self.cmd_doc_name = file_path+"cmd-" + str(self.start_time) + ".txt"
        self.state_doc_name = file_path + "state-" + str(self.start_time) + ".txt"
        rospy.spin()
    
    def cb_cmd(self,msg):
        current_time = rospy.get_time()
        time = current_time - self.start_time
        with open(self.cmd_doc_name, 'a') as f:
            f.write(str(time))
            for i in range(len(msg.data)):
                f.write(',')
                f.write(str(msg.data[i]))
            f.write('\n')
            f.close()

    def cb_state(self,msg):
        state_current_time = rospy.get_time()
        state_time = state_current_time - self.start_time
        with open(self.state_doc_name, 'a') as f:
            f.write(str(state_time))
            for i in range(len(msg.position)):
                f.write(',')
                f.write(str(msg.position[i]))
            f.write('\n')
            f.close()
        
if __name__=="__main__":
    data_write = dataWrite()
