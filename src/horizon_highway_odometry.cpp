#include <horizon/horizon.h>

int main(int argc, char** argv)
{
  if(argc < 2){
    return 1;
  }

  ros::init(argc, argv, "horizon_slam");
  ros::NodeHandle nodeHandler("~");

  ConfigParam config(nodeHandler);

  ROSINPUT rosinput(nodeHandler);

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();

  ROSINPUT::shutdown();

  if(!ConfigParam::SaveMapPath.empty())
    rosinput.saveMapCloud();

  return 0;
}



