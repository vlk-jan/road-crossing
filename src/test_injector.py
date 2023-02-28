import rospy
import injector_msgs.msg

def injector():
    pub = rospy.Publisher('injector', injector_msgs, queue_size=10)
    rospy.init_node('injector', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pass