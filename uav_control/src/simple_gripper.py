#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def gripper():

    rospy.wait_for_service('AttachObject')
    rospy.init_node('gripper', anonymous=True)
    rospy.Subscriber("grip_attach", String, callback)
    att = rospy.ServiceProxy("AttachObject",AttachObject)
    rate = rospy.Rate(10) # 10hz
    rate.sleep()

    if (data.data == "Close")
	resp = att(True, box_name)
    if (data.data == "open")
	execAction = att(False, box_name)

if __name__ == '__main__':
    try:
	gripper()
    except rospy.ROSInterruptException:
        pass
