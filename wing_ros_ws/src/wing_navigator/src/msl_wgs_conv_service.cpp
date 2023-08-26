#include <ros/ros.h>
#include "msl_ellipsoid_conversion.h"
#include <wing_navigator/MSL_WGS_CONV.h>

bool msl_wgs_conv_service_cb(wing_navigator::MSL_WGS_CONV::Request &req,
                             wing_navigator::MSL_WGS_CONV::Response &resp) {
    if(req.msl_in)
        resp.alt_convd = msl2wgs_alt_converter(req.lat, req.lon, req.alt);
    else
        resp.alt_convd = wgs2msl_alt_converter(req.lat, req.lon, req.alt);
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "msl_wgs_conv_service");
    ros::NodeHandle nh;
    ros::ServiceServer msl_wgs_conv_service;
    msl_wgs_conv_service = nh.advertiseService("/msl_wgs_conv_service", msl_wgs_conv_service_cb);
    ros::spin();
    return 0;
}