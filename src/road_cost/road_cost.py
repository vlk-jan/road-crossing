import rospy
from road_crossing.srv import get_road_segment, get_road_segmentResponse, get_suitability, get_suitabilityResponse

import overpy
from math import floor
import shapely.geometry as geometry
import pickle

import road_detection as rd_det
import road_curvature as rd_cur
import road_elevation as rd_ele
from road_crossing_consts import *


class RoadCost:
    road_segments = []

    def road_cost(self, min_lat, min_long, max_lat, max_long):
        api = overpy.Overpass(url="https://overpass.kumi.systems/api/interpreter")
        way_query = self.get_way_query(min_lat, min_long, max_lat, max_long)
        osm_ways_data = api.query(way_query)
        self.road_segments = self.get_road_cost(osm_ways_data)

    def get_way_query(self, min_lat, min_long, max_lat, max_long):
        query = """(way({}, {}, {}, {}); >; ); out;""".format(min_lat, min_long, max_lat, max_long)
        return query

    def get_road_cost(self, osm_ways_data):
        elev_data_files = ELEV_DATA_FILES
        roads = rd_det.get_roads(osm_ways_data)
        prices = rd_det.road_class_price(roads)
        road_network = rd_det.create_road_network([road[0] for road in roads], False, True)
        intersections = rd_det.find_intersections(road_network)
        junctions = rd_det.find_junctions(intersections, road_network)
        road_network = rd_det.combine_road(junctions, intersections, road_network)
        segments = rd_cur.get_average_radius(road_network)
        rd_cur.rank_segments_curve(segments, junctions)
        ranked_segments = rd_cur.road_cost_for_curve(segments)
        if (elev_data_files is not None):
            elev_data = rd_ele.get_road_network_elevation(road_network, elev_data_files)
            class_TPI = rd_ele.classify_TPI(elev_data)
            elev_cost = rd_ele.road_cost_for_height(class_TPI)
        ranked_segments_2 = [[] for i in range(ROAD_CROSSINGS_RANKS)]
        for segment in ranked_segments:
            cost = (segment[1]+1)/ROAD_CURVATURE_RANKS * CURVATURE_WEIGHT
            for road in prices:
                if segment[0].distance(road[0]) < 1e-9:
                    cost += road[1]/ROAD_CLASS_RANKS * CLASS_WEIGHT
                    break
            if (elev_data_files is not None):
                for elev_segment in elev_cost:
                    if elev_segment[0].distance(segment[0]) < 1e-9:
                        cost += (elev_segment[1])/ROAD_ELEVATION_RANKS * ELEVATION_WEIGHT
            cost /= (CURVATURE_WEIGHT+CLASS_WEIGHT+(ELEVATION_WEIGHT if elev_data_files is not None else 0))
            cost *= ROAD_CROSSINGS_RANKS
            ranked_segments_2[(ROAD_CROSSINGS_RANKS-1) if cost >= (ROAD_CROSSINGS_RANKS-1) else floor(cost)].append(segment[0])
        return ranked_segments_2

    def sub_callback(self, msg):
        self.easting = msg.easting
        self.northing = msg.northing
        self.context_score = msg.context_score
        rospy.loginfo("easting: {}, northing: {}, context_score: {}".format(self.easting, self.northing, self.context_score))

    def score_road(self, easting, northing):
        point = geometry.Point(easting, northing)
        min_dist = float('inf')
        cost = None

        for segment_level in range(len(self.road_segments)):
            for segment in self.road_segments[segment_level]:
                dist = segment.distance(point)
                if (dist < min_dist):
                    min_dist = dist
                    if (dist < 10):
                        cost = ROAD_CROSSINGS_RANKS - segment_level

        rospy.loginfo("min_dist: {}, cost: {}".format(min_dist, cost))
        return cost

    def save_road_segments(self):  # TODO: file name as param
        with open("road_segments.pyc", "wb") as file:
            pickle.dump(self.road_segments, file)
        rospy.INFO("Road segments saved")

    def load_road_segments(self):  # TODO: file name as param & error handling
        with open("road_segments.pyc", "rb") as file:
            self.road_segments = pickle.load(file)
        rospy.INFO("Road segments loaded")

    def handle_road_suitability(self, req):
        easting = req.easting
        northing = req.northing
        valid = False
        suitable = False

        score = self.score_road(req.easting, req.northing)
        if (score is not None):
            valid = True
            if (score + req.context_score >= 20):
                suitable = True

        return get_suitabilityResponse(easting, northing, valid, suitable)

    def get_suitability_server(self):
        rospy.init_node("get_suitability_server")
        s_get_suitability = rospy.Service("get_suitability", get_suitability, self.handle_road_suitability)
        rospy.spin()

    def handle_get_road_segment(self, req):
        point = geometry.Point(req.easting, req.northing)
        min_dist = float('inf')

        easting_1 = 0
        northing_1 = 0
        easting_2 = 0
        northing_2 = 0

        for segment_level in range(len(self.road_segments)):
            for segment in self.road_segments[segment_level]:
                dist = segment.distance(point)
                if (dist < min_dist):
                    min_dist = dist
                    easting_1 = segment.coords[0][0]
                    northing_1 = segment.coords[0][1]
                    easting_2 = segment.coords[1][0]
                    northing_2 = segment.coords[1][1]

        return get_road_segmentResponse(easting_1, northing_1, easting_2, northing_2)

    def get_road_segment_server(self):
        rospy.init_node("get_road_segment_server")
        s_get_segment = rospy.Service("get_road_segment", get_road_segment, self.handle_get_road_segment)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("road_cost_standalone")

    #min_lat = rospy.get_param("/min_lat")
    #min_long = rospy.get_param("/min_long")
    #max_lat = rospy.get_param("/max_lat")
    #max_long = rospy.get_param("/max_long")

    min_lat = 50.09
    min_long = 14.11
    max_lat = 50.11
    max_long = 14.13

    road_cost_obj = RoadCost()
    road_cost_obj.road_cost(min_lat, min_long, max_lat, max_long)
    road_cost_obj.save_road_segments()
