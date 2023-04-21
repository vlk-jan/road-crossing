#! /usr/bin/env python
import rospy

import road_cost


if __name__ == '__main__':
    rospy.init_node("road_cost_standalone")

    #min_lat = rospy.get_param("/min_lat")
    #min_long = rospy.get_param("/min_long")
    #max_lat = rospy.get_param("/max_lat")
    #max_long = rospy.get_param("/max_long")

    min_lat = 50.09
    min_long = 14.11
    max_lat = 50.11
    max_long = 14.13

    road_cost_obj = road_cost.RoadCost()
    road_cost_obj.road_cost(min_lat, min_long, max_lat, max_long)
    road_cost_obj.save_road_segments()
    
