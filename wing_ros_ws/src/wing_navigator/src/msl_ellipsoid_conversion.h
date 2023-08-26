//
// Created by areza on 8/26/23.
//

#ifndef SRC_MSL_ELLIPSOID_CONVERSION_H
#define SRC_MSL_ELLIPSOID_CONVERSION_H
double msl2wgs_alt_converter(double msl_lat, double msl_lon, double msl_alt);
double wgs2msl_alt_converter(double wgs_lat, double wgs_lon, double wgs_alt);
#endif //SRC_MSL_ELLIPSOID_CONVERSION_H
