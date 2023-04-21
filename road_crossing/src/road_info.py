import rospy
from road_crossing.srv import get_road_info, get_road_infoResponse

import shapely.geometry as geometry
import pickle

class road_info:
    def __init__(self):
        self.cross_segment = None
        self.expected_velocity = None
        self.maximal_velocity = None
        self.lane_num = None
        self.road_width = None
        self.road_type = None
        self.peddestrian_crossing = None
        self.peddestrian_crossing_coords = None

class road_data:
    def __init__(self):
        self.data = []

    def add_road(self, road : road_info):
        self.data.append(road)

    def save(self):
        file_name = rospy.get_param("~road_info_file", "road_info.pyc")
        with open(file_name, "wb") as fp:
            pickle.dump(self.data, fp)
        print("INFO: Road info saved")

    def load(self):
        file_name = rospy.get_param("~road_info_file", "road_info.pyc")
        try:
            with open(file_name, "rb") as file:
                self.data = pickle.load(file)
            print("INFO: Road info loaded")
        except FileNotFoundError:
            print("ERROR: Road info not loaded, file not found")
            exit()
        except OSError:
            print("ERROR: Road info not loaded, file open error")
            exit()

    def handle_road_info(self, req):
        location = geometry.Point(req.easting, req.northing)

        min_dist = float('inf')
        index = None

        for i in range(len(self.data)):
            dist = location.distance(self.data[i].road_segment)
            if (min_dist >= dist):
                if (min_dist == dist):
                    print("WARN: Two segments in equal distance")
                    return
                min_dist = dist
                index = i

        if index is not None:
            i = index
            return get_road_infoResponse(self.data[i].cross_segment.coords[0,0], self.data[i].cross_segment.coords[0,1] /
                                         self.data[i].cross_segment.coords[1,0], self.data[i].cross_segment.coords[1,1] /
                                         self.data[i].expected_velocity, self.data[i].maximal_velocity. self.data[i].num_lanes /
                                         self.data[i].road_width, self.road_type, self.data[i].peddestrian_crossing /
                                         self.data[i].peddestrian_crossing.coords[0] if self.data[i].peddestrian_crossing else 0 /
                                         self.data[i].peddestrian_crossing.coords[1] if self.data[i].peddestrian_crossing else 0)

    def get_road_info_server(self):
        rospy.init_node("get_road_info_server")
        s_get_road_ifno = rospy.Service("get_road_info", get_road_info, self.handle_road_suitability)
        rospy.loginfo("Road info server ready")
        rospy.spin()
