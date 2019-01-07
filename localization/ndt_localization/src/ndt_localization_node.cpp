#include "ndt_localization/ndt_localization.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_localization_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  NDTLocalization ndt(nh, pnh);

  ndt.init();

  ros::spin();
  return 0;
}