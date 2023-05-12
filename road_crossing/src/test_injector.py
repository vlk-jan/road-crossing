import rospy
import time
import sys
import subprocess
import utm
import random
import math

from road_crossing_msgs.msg import injector_msgs
from road_crossing.srv import start_algorithm


class Data:
    def __init__(self):
        self.msgs = []
        self.err_veh = 0
        self.to_zero = []
        self.to_max = []
        self.x_dot_positive = []
        self.y_dot_positive = []
        self.max = 0

def kph_to_mps(kph):
    return kph/3.6

def injector(data):
    pub = rospy.Publisher('/road_crossing/injector', injector_msgs, queue_size=10)
    rate = rospy.Rate(5)

    msg_clear = injector_msgs()
    msg_clear.clear = True

    msgs = data.msgs
    cmd_str = "rosrun road_crossing start.py start"

    input("To start press enter")

    subprocess.run(cmd_str, shell=True)
    
    time_prev = time.time()
    time_start = time_prev
    while not rospy.is_shutdown():
        # Update time
        time_now = time.time()
        time_consumed = time_now - time_prev
        time_prev = time_now

        pub.publish(msg_clear)
        for i in range(len(msgs)):
            msg = msgs[i]
            # Update velocity
            msg.x_dot += time_consumed*msg.x_ddot
            msg.y_dot += time_consumed*msg.y_ddot
            if (data.to_zero[i]):
                msg.x_dot = max(msg.x_dot, 0) if data.x_dot_positive[i] else min(msg.x_dot, 0)
                msg.y_dot = max(msg.y_dot, 0) if data.y_dot_positive[i] else min(msg.y_dot, 0)
                if (i != data.err_veh):
                    msg.x_ddot = 0 if msg.x_dot == 0 else msg.x_ddot
                    msg.y_ddot = 0 if msg.y_dot == 0 else msg.y_ddot
            if (data.to_max[i]):
                msg.x_dot = min(msg.x_dot, data.max) if data.x_dot_positive[i] else max(msg.x_dot, -data.max)
                msg.y_dot = min(msg.y_dot, data.max) if data.y_dot_positive[i] else max(msg.y_dot, -data.max)
                if (i != data.err_veh):
                    msg.x_ddot = 0 if msg.x_dot == data.max else msg.x_ddot
                    msg.y_ddot = 0 if msg.y_dot == data.max else msg.y_ddot
            if (msg.x_dot != 0 and msg.y_dot != 0):
                msg.phi = math.atan2(msg.y_dot, msg.x_dot)

            # Update position
            msg.easting += time_consumed*msg.y_dot
            msg.northing += time_consumed*msg.x_dot

            # Detection errors
            if (time_now - time_start > 1.3 and time_now - time_start < 1.45):
                easting_error = random.uniform(-0.5, 0.5)
                northing_error = random.uniform(-0.5, 0.5)
                msg.easting += easting_error
                msg.northing += northing_error
                pub.publish(msg)
                msg.northing -= easting_error
                msg.northing -= northing_error
            else:
                if (i == data.err_veh):
                    continue
                pub.publish(msg)
        rate.sleep()

def get_scene1():
    data = Data()
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
    msg1.phi = math.atan2(msg1.y_dot, msg1.x_dot)

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
    msg2.phi = math.atan2(msg2.y_dot, msg2.x_dot)

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
    msg3.phi = math.atan2(msg3.y_dot, msg3.x_dot)

    data.msgs = [msg1, msg2, msg3]
    data.err_veh = 2
    data.to_zero = [True, True, True]
    data.to_max = [True, True, True]
    data.x_dot_positive = [True, True, True]
    data.y_dot_positive = [False, True, True]
    data.max = kph_to_mps(50)
    return data

def get_scene2():
    data = Data()
    pos = (50.0929653, 14.1248958)
    pos_utm = utm.from_latlon(pos[0], pos[1])

    msg1 = injector_msgs()
    msg1.veh_id = 1
    msg1.easting = pos_utm[0]
    msg1.northing = pos_utm[1]
    msg1.x_dot = 0.0
    msg1.y_dot = -kph_to_mps(20)
    msg1.x_ddot = 0.0
    msg1.y_ddot = 1.7
    msg1.length = 4.7
    msg1.width = 2.0
    msg1.phi = math.atan2(msg1.y_dot, msg1.x_dot)

    msg2 = injector_msgs()
    msg2.veh_id = 2
    msg2.easting = pos_utm[0]
    msg2.northing = pos_utm[1]
    msg2.x_dot = 0.0
    msg2.y_dot = -kph_to_mps(30)
    msg2.x_ddot = 0.0
    msg2.y_ddot = -1.0
    msg2.length = 4.7
    msg2.width = 2.0
    msg2.phi = math.atan2(msg2.y_dot, msg2.x_dot)

    data.msgs = [msg1, msg2]
    data.err_veh = 1
    data.to_zero = [True, True]
    data.to_max = [True, True]
    data.x_dot_positive = [True, True]
    data.y_dot_positive = [False, False]
    data.max = kph_to_mps(30)
    return data

def main(scene):
    rospy.init_node('injector', anonymous=True)

    if (scene == '1'):
        data = get_scene1()
    elif (scene == '2'):
        data = get_scene2()
    else:
        print("Invalid scene")
        exit()

    injector(data)

if (__name__ == '__main__'):
    if (len(sys.argv) != 2):
        print("Usage: test_injector.py <scene_num>")
        exit()
    try:
        main(sys.argv[1])
    except rospy.ROSInterruptException: pass
