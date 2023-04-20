#include <ros/ros.h>
#include "pointcloud_merger_core.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_merger");
  PointCloudMerger pointcloud_merger;

  pointcloud_merger.run();

  return 0;
}