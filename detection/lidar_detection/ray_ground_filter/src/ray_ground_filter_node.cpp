//
// Created by adam on 18-9-21.
//

#include "ray_ground_filter_core.h"
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/server.h>
#include <ray_ground_filter/RayFilterConfig.h>

void cfg_handle(const ray_ground_filter::RayFilterConfig& config, uint32_t level, RayGroundFilter* filter)
{
    filter->reset_params(config);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_ground_filter_node");

    ros::NodeHandle nh_("~");

    RayGroundFilter filter(nh_);
    filter.run();

    dynamic_reconfigure::Server<ray_ground_filter::RayFilterConfig> cfg_server;
    dynamic_reconfigure::Server<ray_ground_filter::RayFilterConfig>::CallbackType cfg_callback = boost::bind(&cfg_handle, _1, _2, &filter);
    cfg_server.setCallback(cfg_callback);

    ros::spin();
    return 0;
}