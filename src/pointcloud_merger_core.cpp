#include "pointcloud_merger_core.h"

#include <unistd.h>

  
//Class Constructor
PointCloudMerger::PointCloudMerger() : 
private_nh_("~")
{ 
  initParam(); 
}

//Class Destructor
PointCloudMerger::~PointCloudMerger(){}

void PointCloudMerger::initParam()
{
  private_nh_.getParam("param_file_path",param_file);
  if (access(param_file.c_str(), F_OK) != -1) {
    ROS_INFO_STREAM("Loaded parameters from" << param_file);
  } else {
    ROS_ERROR_STREAM("Failed to load parameters from " << param_file);
  }
  private_nh_.param<std::string>("pm_input_topic_fir", pm_input_topic_fir, "/top/velodyne_pointcloud");
  private_nh_.param<std::string>("pm_input_topic_sec", pm_input_topic_sec, "/left/filtered_pointcloud");
  private_nh_.param<std::string>("pm_input_topic_third", pm_input_topic_third, "/right/filtered_pointcloud");
  private_nh_.param<std::string>("frame_id", target_frame_id, "base_link");
}

void PointCloudMerger::run()
{
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1(nh_, pm_input_topic_fir, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2(nh_, pm_input_topic_sec, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_3(nh_, pm_input_topic_third, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2, sub_3);
  sync.registerCallback(boost::bind(&PointCloudMerger::PointCloudCallback, this, _1, _2, _3));

  // Publisher
  output_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged/points", 1);

  ros::spin();
}

// void PointCloudMerger::MergeCallback(const ros::TimerEvent& event) {
void PointCloudMerger::PointCloudCallback(  const sensor_msgs::PointCloud2ConstPtr& cloud_msg_fir, 
                                            const sensor_msgs::PointCloud2ConstPtr& cloud_msg_sec, 
                                            const sensor_msgs::PointCloud2ConstPtr& cloud_msg_third)
{  
  ros::Time current_time = ros::Time::now();
  double current_time_ms = current_time.toSec() * 1000;
  double fir_lidar_stamp_ms = cloud_msg_fir->header.stamp.toSec() * 1000;
  double sec_lidar_stamp_ms = cloud_msg_sec->header.stamp.toSec() * 1000;
  double third_lidar_stamp_ms = cloud_msg_third->header.stamp.toSec() * 1000;

  double fir_lidar_time_diff_ms = current_time_ms - fir_lidar_stamp_ms;
  double sec_lidar_time_diff_ms = current_time_ms - sec_lidar_stamp_ms;
  double third_lidar_time_diff_ms = current_time_ms - third_lidar_stamp_ms;

  pcl::PointCloud<pcl::PointXYZ> first_cloud, second_cloud, third_cloud, merged_cloud;
  pcl::PointCloud<pcl::PointXYZ> first_cloud_tf, second_cloud_tf, third_cloud_tf;

  pcl::fromROSMsg(*cloud_msg_fir,   first_cloud);
  static tf::TransformListener tf_listener;
  tf::StampedTransform transform_fir;
  try {
    tf_listener.lookupTransform(target_frame_id, cloud_msg_fir->header.frame_id, ros::Time(0), transform_fir);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  pcl_ros::transformPointCloud(first_cloud, first_cloud_tf, transform_fir);

  pcl::fromROSMsg(*cloud_msg_sec,   second_cloud);
  tf::StampedTransform transform_sec;
  try {
    tf_listener.lookupTransform(target_frame_id, cloud_msg_sec->header.frame_id, ros::Time(0), transform_sec);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  pcl_ros::transformPointCloud(second_cloud, second_cloud_tf, transform_sec);

  pcl::fromROSMsg(*cloud_msg_third, third_cloud);
  tf::StampedTransform transform_third;
  try {
    tf_listener.lookupTransform(target_frame_id, cloud_msg_sec->header.frame_id, ros::Time(0), transform_third);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  pcl_ros::transformPointCloud(third_cloud, third_cloud_tf, transform_third);

  merged_cloud = first_cloud_tf;

  for(unsigned int i=0; i<second_cloud.points.size(); i++){
    merged_cloud.points.push_back(second_cloud.points[i]);
  }
  for(unsigned int i=0; i<third_cloud.points.size(); i++){
    merged_cloud.points.push_back(third_cloud.points[i]);
  }
  merged_cloud.width += second_cloud.width;
  merged_cloud.width += third_cloud.width;

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(merged_cloud));
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  pcl::PointCloud<pcl::PointXYZ> voxel_filtered_cloud;
  voxel_filter.setInputCloud(merged_cloud_ptr);
  voxel_filter.setLeafSize(0.12, 0.12, 0.12);
  voxel_filter.filter(voxel_filtered_cloud);
  

  sensor_msgs::PointCloud2 output_cloud_msg;
  output_cloud_msg.header.stamp = current_time;
  output_cloud_msg.header.frame_id = target_frame_id;
  pcl::toROSMsg(voxel_filtered_cloud, output_cloud_msg);
  output_cloud_pub_.publish(output_cloud_msg);

  double end_time_ms = ros::Time::now().toSec() * 1000;

  ROS_INFO("/* --  --  --  --  --*/");
  ROS_INFO_STREAM("[TOP]  points:"<<first_cloud.points.size()<<", diff_time:"<< fir_lidar_time_diff_ms <<"ms");
  ROS_INFO_STREAM("[LEFT] points:"<<second_cloud.points.size()<<", diff_time:"<< sec_lidar_time_diff_ms <<"ms");
  ROS_INFO_STREAM("[RIGHT] points:"<<third_cloud.points.size()<<", diff_time:"<< third_lidar_time_diff_ms <<"ms");
  ROS_INFO_STREAM("point_cloud_merger exe time : "<< end_time_ms-current_time_ms << "(ms)");
}

