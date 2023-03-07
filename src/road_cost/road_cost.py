import rospy
from road_crossing.msg import UTM_data, place_data

import overpy
from math import floor
import shapely.geometry as geometry

import road_detection as rd_det
import road_curvature as rd_cur
import road_elevation as rd_ele
from road_crossing_consts import *


easting = 0.0
northing = 0.0
context_score = 0

def road_cost(min_lat, min_long, max_lat, max_long):
    api = overpy.Overpass(url="https://lz4.overpass-api.de/api/interpreter")
    way_query = get_way_query(min_lat, min_long, max_lat, max_long)
    osm_ways_data = api.query(way_query)
    return get_road_cost(osm_ways_data)

def get_way_query(min_lat, min_long, max_lat, max_long):
    query = """(way({}, {}, {}, {}); >; ); out;""".format(min_lat, min_long, max_lat, max_long)

    return query

def get_road_cost(osm_ways_data):
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

def sub_callback(msg):
    global easting, northing, context_score
    easting = msg.easting
    northing = msg.northing
    context_score = msg.context_score

def score_road(road_segments, easting, northing):
    point = geometry.Point(easting, northing)
    min_dist = float('inf')
    cost = None
    for segment_level in range(len(road_segments)):
        for segment in segment_level:
            dist = segment.distance(point)
            if (dist < 5 and dist < min_dist):
                min_dist = dist
                cost = ROAD_CLASS_RANKS - segment_level
    return cost

if __name__ == "__main__":
    #min_lat = rospy.get_param("/min_lat")
    #min_long = rospy.get_param("/min_long")
    #max_lat = rospy.get_param("/max_lat")
    #max_long = rospy.get_param("/max_long")
    
    min_lat = 50.08
    min_long = 14.12
    max_lat = 50.1
    max_long = 14.14

    road_segments = road_cost(min_lat, min_long, max_lat, max_long)

    sub = rospy.Subscriber("cur_place", UTM_data, sub_callback)
    pub = rospy.Publisher("place_cost", place_data, queue_size=10)

    while (not rospy.is_shutdown()):
        if (easting != 0.0 and northing != 0.0 and context_score != 0):
            msg = place_data()
            msg.easting = easting
            msg.northing = northing
            road_score = score_road(road_segments, easting, northing)
            if (road_score is None):
                msg.is_valid = False
            else:
                msg.is_valid = True
                score = road_score + context_score
                if (score < 20):
                    msg.suitable = False
                else:
                    msg.suitable = True
            pub.publish(msg)
        rospy.spin()
