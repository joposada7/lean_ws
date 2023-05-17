#!/usr/bin/env python3
import rospy
from acl_msgs.msg import ViconState

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Vicon State data: %s", data.data)
    print("Vicon State data: %s", data.data)
     
def listener():
    rospy.Subscriber("/1/vicon", ViconState, callback)
    

if __name__ == '__main__':
    rospy.init_node('controller_node', anonymous=True)
    listener()
    rospy.spin()