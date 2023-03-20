#ifndef misc
#define misc

double gpsPointsHeading(double lat1, double lon1, double lat2, double lon2);

double angleDifference(double angle1, double angle2);

double ned2enu(double angle);

double deg2rad(double angle);

double comp_heading(double rob_heading, double road_heading);

/**
         * @brief Converts gps coordinates to utm coordinates.
         * 
         * @param lat    latitude of the gps coordinates.
         * @param lon    longitude of the gps coordinates.
         * @param x      easting of the utm coordinates.
         * @param y      northing of the utm coordinates.
        */
void gps_to_utm(double lat, double lon, double &x, double &y);

#endif
