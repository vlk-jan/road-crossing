import road_info

if __name__ == "__main__":
    road_info_obj = road_info.road_data()
    road_info_obj.load()
    road_info_obj.get_road_info_server()