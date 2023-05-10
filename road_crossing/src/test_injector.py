import rospy
import time
import utm
from road_crossing_msgs.msg import injector_msgs

def injector():
    pub = rospy.Publisher('/road_crossing/injector', injector_msgs, queue_size=10)
    rospy.init_node('injector', anonymous=True)
    rate = rospy.Rate(5)

    pos1 = (50.0930128, 14.1248906)
    pos1_utm = utm.from_latlon(pos1[0], pos1[1])
    pos2 = (50.0929653, 14.1248958)
    pos2_utm = utm.from_latlon(pos2[0], pos2[1])

    msg1 = injector_msgs()
    msg1.veh_id = 1
    msg1.easting = pos1_utm[0]
    msg1.northing = pos1_utm[1]
    msg1.x_dot = 0.0
    msg1.y_dot = -1.5
    msg1.x_ddot = 0.0
    msg1.y_ddot = 0.0
    msg1.length = 4.7
    msg1.width = 2.0

    msg2 = injector_msgs()
    msg2.veh_id = 2
    msg2.easting = pos2_utm[0] - 13
    msg2.northing = pos2_utm[1]
    msg2.x_dot = 0.0
    msg2.y_dot = 1.0
    msg2.x_ddot = 0.0
    msg2.y_ddot = 0.0
    msg2.length = 4.7
    msg2.width = 2.0

    msg3 = injector_msgs()
    msg3.veh_id = 3
    msg3.easting = pos1_utm[0] - 6
    msg3.northing = pos1_utm[1]
    msg3.x_dot = 0.0
    msg3.y_dot = 1.5
    msg3.x_ddot = 0.0
    msg3.y_ddot = 0.0
    msg3.length = 4.7
    msg3.width = 2.0

    time_prev = time.time()
    time_start = time_prev
    while not rospy.is_shutdown():
        # Update time
        time_now = time.time()
        time_consumed = time_now - time_prev
        time_prev = time_now

        # Update velocity
        msg1.x_dot += time_consumed*msg1.x_ddot
        msg1.y_dot += time_consumed*msg1.y_ddot
        msg2.x_dot += time_consumed*msg2.x_ddot
        msg2.y_dot += time_consumed*msg2.y_ddot

        # Update position
        msg1.easting += time_consumed*msg1.y_dot
        msg1.northing += time_consumed*msg1.x_dot
        msg2.easting += time_consumed*msg2.y_dot
        msg2.northing += time_consumed*msg2.x_dot
        msg3.easting += time_consumed*msg2.y_dot
        msg3.northing += time_consumed*msg2.x_dot

        # Detection errors
        if (time_now - time_start > 1.3 and time_now - time_start < 1.35):
            msg1.easting += 10
            msg2.northing -= 6
            pub.publish(msg1)
            pub.publish(msg2)
            pub.publish(msg3)
            msg1.easting -= 10
            msg2.northing += 6
        else:
        # ROS cycle
        #rospy.loginfo("{}, {}".format(msg.easting, msg.northing))
            pub.publish(msg1)
            pub.publish(msg2)
        rate.sleep()

if __name__ == '__main__':
    try:
        injector()
    except rospy.ROSInterruptException: pass
