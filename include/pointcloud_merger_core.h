#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <ctime>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PointCloudMerger {
    public:
    PointCloudMerger();
    ~PointCloudMerger();

    void run();

    private:
    // ROS Handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // ROS Publisher
    ros::Publisher output_cloud_pub_;

    // ROS Parameters
    std::string param_file;

    /* input_topic_name */
    std::string pm_input_topic_fir;
    std::string pm_input_topic_sec;
    std::string pm_input_topic_third;
    /* frame id */
    std::string target_frame_id;
    
    void initParam();
    void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_fir, 
                            const sensor_msgs::PointCloud2ConstPtr& cloud_msg_sec, 
                            const sensor_msgs::PointCloud2ConstPtr& cloud_msg_third);
};