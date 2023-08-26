#include "msl_ellipsoid_conversion.h"
#include <exception>
#include <GeographicLib/Geoid.hpp>
#include <iostream>

double msl2wgs_alt_converter(double msl_lat, double msl_lon, double msl_alt) {
    // Convert height above egm96(MSL) msl_alt to height above the ellipsoid(WGS84)
    try {
        GeographicLib::Geoid egm96("egm96-5");
        double undulation = GeographicLib::Geoid::GEOIDTOELLIPSOID * egm96(msl_lat, msl_lon);
        double wgs_alt = (msl_alt + undulation);
        return wgs_alt;
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
        return 1;
    }
}

double wgs2msl_alt_converter(double wgs_lat, double wgs_lon, double wgs_alt) {
    // Convert height above the ellipsoid(WGS84) to height above egm96(MSL) msl_alt
    try {
        GeographicLib::Geoid egm96("egm96-5");
        double undulation = GeographicLib::Geoid::GEOIDTOELLIPSOID * egm96(wgs_lat, wgs_lon);
        double msl_alt = (wgs_alt - undulation);
        return msl_alt;
    }
    catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
        return 1;
    }
}