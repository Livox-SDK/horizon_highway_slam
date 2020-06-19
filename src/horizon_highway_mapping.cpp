#include <horizon/horizon.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "horizon_mapping");
  ros::NodeHandle nodeHandler("~");

  MAPPING_SWIFT mapping_swift(nodeHandler);

  ros::spin();

  return 0;
}

