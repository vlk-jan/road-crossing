# Testing purposes only

import rospy

from road_crossing.srv import start_algorithm

def start_client(start=True, stop=False):
    rospy.wait_for_service('start_algorithm')
    try:
        start_cli = rospy.ServiceProxy('start_algorithm', start_algorithm)
        resp1 = start_cli(start, stop)
        return resp1.is_running
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Requesting start")
    print("Start successful: %s"%start_client())