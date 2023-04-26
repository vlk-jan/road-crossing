import rospy
import time
import utm
from road_crossing.msg import injector_msgs

def injector():
    pub = rospy.Publisher('injector', injector_msgs, queue_size=10)
    rospy.init_node('injector', anonymous=True)
    rate = rospy.Rate(10)

    pos1 = (50.0918150, 14.1249011)
    pos1_utm = utm.from_latlon(pos1[0], pos1[1])

    msg = injector_msgs()
    msg.veh_id = 1
    msg.easting = pos1_utm[0]
    msg.northing = pos1_utm[1]
    msg.x_dot = 1.0
    msg.y_dot = 0.0
    msg.x_ddot = 0.0
    msg.y_ddot = 0.0
    msg.length = 2.0
    msg.width = 1.0

    time_consumed = time.time()
    while not rospy.is_shutdown():
        # Update time
        time_consumed = time.time() - time_consumed

        # Update velocity
        msg.x_dot += time_consumed*msg.x_ddot
        msg.y_dot += time_consumed*msg.y_ddot

        # Update position
        msg.easting += time_consumed*msg.x_dot
        msg.northing += time_consumed*msg.y_dot

        # ROS cycle
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        injector()
    except rospy.ROSInterruptException: pass