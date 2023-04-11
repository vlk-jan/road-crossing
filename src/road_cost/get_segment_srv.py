import road_cost


if __name__ == '__main__':
    road_cost_obj = road_cost.RoadCost()
    road_cost_obj.load_road_segments()
    road_cost_obj.get_road_segment_server()
