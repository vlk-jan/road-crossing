from road_cost import RoadCost


if __name__ == '__main__':
    road_cost_obj = RoadCost()
    road_cost_obj.load_road_segments()
    road_cost_obj.get_suitability_server()
