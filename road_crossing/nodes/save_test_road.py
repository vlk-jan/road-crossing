# Testing purposes only

import road_cost


def main():
    road_data = road_cost.road_data()
    road = road_data.add_test_road()
    road_data.add_road(road)
    road_data.save()


if __name__ == '__main__':
    main()
