import rospy
import time
import utm
from road_crossing_msgs.msg import injector_msgs

def injector():
    pub = rospy.Publisher('/road_crossing/injector', injector_msgs, queue_size=10)
    rospy.init_node('injector', anonymous=True)
    rate = rospy.Rate(50)

    pos1 = (50.0930128, 14.1248906)
    pos1_utm = utm.from_latlon(pos1[0], pos1[1])

    msg = injector_msgs()
    msg.veh_id = 1
    msg.easting = pos1_utm[0]
    msg.northing = pos1_utm[1]
    msg.x_dot = 0.0
    msg.y_dot = -1.0
    msg.x_ddot = 0.0
    msg.y_ddot = 0.0
    msg.length = 2.0
    msg.width = 1.0

    time_prev = time.time()
    while not rospy.is_shutdown():
        # Update time
        time_now = time.time()
        time_consumed = time_now - time_prev
        time_prev = time_now

        # Update velocity
        msg.x_dot += time_consumed*msg.x_ddot
        msg.y_dot += time_consumed*msg.y_ddot

        # Update position
        msg.easting += time_consumed*msg.y_dot
        msg.northing += time_consumed*msg.x_dot

        # ROS cycle
        #rospy.loginfo("{}, {}".format(msg.easting, msg.northing))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        injector()
    except rospy.ROSInterruptException: pass
