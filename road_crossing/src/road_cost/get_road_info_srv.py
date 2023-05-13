#!/usr/bin/env python

import road_cost

if __name__ == "__main__":
    road_info_obj = road_cost.road_data()
    road_info_obj.get_road_info_server()