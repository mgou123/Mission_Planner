#ifndef ASV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_TO_POSE_HPP_
#define ASV_BT_PLUGINS__PLUGINS__ACTION__NAVIGATE_TO_POSE_HPP_

#include <string>
#include "geographic_msgs/GeoPoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "bb_msgs/LocomotionAction.h"
#include "bb_msgs/LocomotionGoal.h"
#include "bb_msgs/LocomotionResult.h"
#include "mp_behavior_tree/bt_action_node.hpp"

#include <math.h>
#include <iostream>

#define TO_RADS 0.0174533
#define OUT

// mp wrapper for Locomotion action
namespace mp_behavior_tree
{

class NavigateToGeopose 
: public BtActionNode<bb_msgs::LocomotionAction, 
                      bb_msgs::LocomotionGoal, 
                      bb_msgs::LocomotionResult>
{
public:
    NavigateToGeopose(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<geographic_msgs::GeoPoseStamped>("origin", "Destination geopose"),
            BT::InputPort<geographic_msgs::GeoPoseStamped>("goal", "Destination geopose"),
            BT::InputPort<double>("forward_tolerance", "Tolerance for forward motion"),
            BT::InputPort<double>("sidemove_tolerance", "Tolerance for XY motion"),
            BT::InputPort<double>("yaw_tolerance", "Tolerance for yaw"),
            BT::OutputPort<double>("yaw_output", "Yaw output")
        });
    }

private:
    // WGS-84 geodetic constants
    static constexpr  double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)

    static  constexpr double b = 6356752.314245;     // Derived Earth semiminor axis (m)
    static  constexpr double f = (a - b) / a;        // Ellipsoid Flatness
    static  constexpr double f_inv = 1.0 / f;       // Inverse flattening

    static constexpr double a_sq = a * a;
    static constexpr double b_sq = b * b;
    static constexpr double e_sq = f * (2 - f);

    static constexpr double R = a;

    static double DegreesToRadians(double deg) {
        return deg * TO_RADS;
    }

    static double RadiansToDegrees(double rad) {
        return rad / TO_RADS;
    }

    // Converts WGS-84 Geodetic point (lat, lon, h) to the
    // Distance-Bearing
    static void GeodeticToDistbear(double lat, double lon, double h, double lat0, double lon0, double h0,
                                    OUT double& d, OUT double& theta)
    {
        auto phi1 = DegreesToRadians(lat);
        auto phi0 = DegreesToRadians(lat0);

        auto lam1 = DegreesToRadians(lon);
        auto lam0 = DegreesToRadians(lon0);

        auto delta_phi = phi1 - phi0;
        auto delta_lambda = lam1 - lam0;

        auto a = sin(delta_phi/2) * sin(delta_phi/2) + cos(phi0) * cos(phi1) * sin(delta_lambda) * sin(delta_lambda);
        auto c = 2 * atan2(sqrt(a), sqrt(1 - a));

        d = R * c;

        auto y = sin(delta_lambda) * cos(phi1);
        auto x = cos(phi0) * sin(phi1) - sin(phi0) * cos(phi1) * cos(delta_lambda);
        theta = atan2(y, x);
    }

    // Converts WGS-84 Geodetic point (lat, lon, h) to the 
    // Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
    static void GeodeticToEcef(double lat, double lon, double h,
                                        OUT double& x, OUT double& y, OUT double& z)
    {
        // Convert to radians in notation consistent with the paper:
        auto lambda = DegreesToRadians(lat);
        auto phi = DegreesToRadians(lon);
        auto s = sin(lambda);
        auto N = a / sqrt(1 - e_sq * s * s);

        auto sin_lambda = sin(lambda);
        auto cos_lambda = cos(lambda);
        auto cos_phi = cos(phi);
        auto sin_phi = sin(phi);

        x = (h + N) * cos_lambda * cos_phi;
        y = (h + N) * cos_lambda * sin_phi;
        z = (h + (1 - e_sq) * N) * sin_lambda;
    }

    // Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to 
    // (WGS-84) Geodetic point (lat, lon, h).
    static void EcefToGeodetic(double x, double y, double z,
                                        OUT double& lat, OUT double& lon, OUT double& h)
    {
        auto eps = e_sq / (1.0 - e_sq);
        auto p = sqrt(x * x + y * y);
        auto q = atan2((z * a), (p * b));
        auto sin_q = sin(q);
        auto cos_q = cos(q);
        auto sin_q_3 = sin_q * sin_q * sin_q;
        auto cos_q_3 = cos_q * cos_q * cos_q;
        auto phi = atan2((z + eps * b * sin_q_3), (p - e_sq * a * cos_q_3));
        auto lambda = atan2(y, x);
        auto v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
        h = (p / cos(phi)) - v;

        lat = RadiansToDegrees(phi);
        lon = RadiansToDegrees(lambda);
    }

    // Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to 
    // East-North-Up coordinates in a Local Tangent Plane that is centered at the 
    // (WGS-84) Geodetic point (lat0, lon0, h0).
    static void EcefToEnu(double x, double y, double z,
                                    double lat0, double lon0, double h0,
                                    OUT double& xEast, OUT double& yNorth, OUT double& zUp)
    {
        // Convert to radians in notation consistent with the paper:
        auto lambda = DegreesToRadians(lat0);
        auto phi = DegreesToRadians(lon0);
        auto s = sin(lambda);
        auto N = a / sqrt(1 - e_sq * s * s);

        auto sin_lambda = sin(lambda);
        auto cos_lambda = cos(lambda);
        auto cos_phi = cos(phi);
        auto sin_phi = sin(phi);

        double x0 = (h0 + N) * cos_lambda * cos_phi;
        double y0 = (h0 + N) * cos_lambda * sin_phi;
        double z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

        double xd, yd, zd;
        xd = x - x0;
        yd = y - y0;
        zd = z - z0;

        // This is the matrix multiplication
        xEast = -sin_phi * xd + cos_phi * yd;
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
    }

    // Inverse of EcefToEnu. Converts East-North-Up coordinates (xEast, yNorth, zUp) in a
    // Local Tangent Plane that is centered at the (WGS-84) Geodetic point (lat0, lon0, h0)
    // to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
    static void EnuToEcef(double xEast, double yNorth, double zUp,
                                    double lat0, double lon0, double h0,
                                    OUT double& x, OUT double& y, OUT double& z)
    {
        // Convert to radians in notation consistent with the paper:
        auto lambda = DegreesToRadians(lat0);
        auto phi = DegreesToRadians(lon0);
        auto s = sin(lambda);
        auto N = a / sqrt(1 - e_sq * s * s);

        auto sin_lambda = sin(lambda);
        auto cos_lambda = cos(lambda);
        auto cos_phi = cos(phi);
        auto sin_phi = sin(phi);

        double x0 = (h0 + N) * cos_lambda * cos_phi;
        double y0 = (h0 + N) * cos_lambda * sin_phi;
        double z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

        double xd = -sin_phi * xEast - cos_phi * sin_lambda * yNorth + cos_lambda * cos_phi * zUp;
        double yd = cos_phi * xEast - sin_lambda * sin_phi * yNorth + cos_lambda * sin_phi * zUp;
        double zd = cos_lambda * yNorth + sin_lambda * zUp;

        x = xd + x0;
        y = yd + y0;
        z = zd + z0;
    }

    // Converts the geodetic WGS-84 coordinated (lat, lon, h) to 
    // East-North-Up coordinates in a Local Tangent Plane that is centered at the 
    // (WGS-84) Geodetic point (lat0, lon0, h0).
    static void GeodeticToEnu(double lat, double lon, double h,
                                        double lat0, double lon0, double h0,
                                        OUT double& xEast, OUT double& yNorth, OUT double& zUp)
    {
        double x, y, z;
        GeodeticToEcef(lat, lon, h, x, y, z);
        EcefToEnu(x, y, z, lat0, lon0, h0, xEast, yNorth, zUp);
    }
};

} // namespace mp_behavior_tree

#endif