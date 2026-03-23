#ifndef CROW_SERVER_GPSPOINT_H
#define CROW_SERVER_GPSPOINT_H
#include "wykobi/wykobi.hpp"
#include <cmath>

// Earth's radius in meters
const double EARTH_RADIUS = 6371000;

namespace mrcpp{
    class gps_point: public wykobi::point2d<double> {
    public:
        // Constructor from point2d - initialize GPS coordinates properly
        gps_point(const wykobi::point2d<double>& p) : wykobi::point2d<double>(p) {
            lat_ = 0.0;
            lon_ = 0.0;
        }

        // Constructor from GPS coordinates - initialize Cartesian coordinates
        gps_point(double lat, double lon)  {
            lat_ = lat;
            lon_ = lon;
            x = 0.0;
            y = 0.0;
        }

        // Convert GPS coordinates to Cartesian relative to reference point
        void toCartesian(double lat_ref, double lon_ref) {
            double lat_diff = lat_ - lat_ref;
            double lon_diff = lon_ - lon_ref;
            y = latitudeDegreesToMeters(lat_diff);
            x = longitudeDegreesToMeters(lon_diff, lat_ref);
        }

        // Convert Cartesian coordinates to GPS relative to reference point
        void toGps(double lat_ref, double lon_ref) {
            double lat_diff = metersToLatitudeDegrees(y);
            double lon_diff = metersToLongitudeDegrees(x, lat_ref);
            lat_ = lat_diff + lat_ref;
            lon_ = lon_diff + lon_ref;
        }

        // Setters for GPS coordinates
        void setLatitude(double lat) { lat_ = lat; }
        void setLongitude(double lon) { lon_ = lon; }
        void setGPS(double lat, double lon) { lat_ = lat; lon_ = lon; }

        // Getters
        double latitude() const { return lat_; }
        double longitude() const { return lon_; }

    private:
        double lat_, lon_;

    protected:
        double metersToLatitudeDegrees(double meters) const
        {
            // Formula to convert meters to latitude degrees
            return (meters / EARTH_RADIUS) * (180.0 / M_PI);
        }

        double metersToLongitudeDegrees(double meters, double latitude) const
        {
            // Convert latitude to radians for calculations
            double latRad = latitude * M_PI / 180.0;

            // Formula to convert meters to longitude degrees at the given latitude
            return (meters / (EARTH_RADIUS * cos(latRad))) * (180.0 / M_PI);
        }

        double longitudeDegreesToMeters(double degrees, double latitude) const
        {
            // Convert latitude to radians
            double latRad = latitude * M_PI / 180.0;

            // Convert longitude degrees to meters at the given latitude
            return (degrees * M_PI / 180.0) * EARTH_RADIUS * cos(latRad);
        }

        double latitudeDegreesToMeters(double degrees) const
        {
            // Convert latitude degrees to meters
            return (degrees * M_PI / 180.0) * EARTH_RADIUS;
        }
    };
}

#endif //CROW_SERVER_GPSPOINT_H