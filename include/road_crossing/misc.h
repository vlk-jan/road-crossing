#ifndef MISC
#define MISC

/**
 * @brief Calculate the heading from one point to other. The return is the azimut of observer
 * standing at the first point and looking in the direction of the second one.
 * 
 * @param lat1 Latitude or easting of the first point.
 * @param lon1 Longitude or northing of the first point.
 * @param lat2 Latitude or easting of the second point.
 * @param lon2 Longitude or northing of the second point.
 */
double gps_points_heading(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Calculate the difference between two angles. The result is between [-PI,PI].
 * 
 * @param angle1 The value of the first angle in rads.
 * @param angle2 The value of the second angle in rads.
 */
double angle_diff(double angle1, double angle2);

/**
 * @brief Convert azimuth from NED to ENU format.
 * 
 * @param azimuth The azimuth in NED to be converted to ENU.
 */
double ned_to_enu(double azimuth);

/**
 * @brief Convert angle from degrees to radians.
 * 
 * @param angle Angle to convert.
 */
double deg_to_rad(double angle);

/**
 * @brief Determine the shortest rotation such as to make the first heading perpendicular
 * to the second one.
 * 
 * @param rob_heading The heading of the robot.
 * @param road_heading The heading of the road we want the robot to be perpendicular to.
 */
double comp_heading(double rob_heading, double road_heading);

/**
 * @brief Converts gps coordinates to utm coordinates.
 * 
 * @param lat Latitude of the gps coordinates.
 * @param lon Longitude of the gps coordinates.
 * @param x Easting of the utm coordinates.
 * @param y Northing of the utm coordinates.
 */
void gps_to_utm(double lat, double lon, double &x, double &y);

/**
 * @brief Calculates the score based on the context info given.
 * Currently the context may only be given by the user via rosparam.
 */
int calculate_context_score();

#endif
