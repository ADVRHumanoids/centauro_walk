#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>


class Perception {
public:
    Perception(ros::NodeHandle nh,
               std::vector<std::string> input_topics);

    void setBaseLink(std::string base_link);
    void update();

private:
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg,
                  int i);
    void transform_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
                               std::string frame_id,
                               int i,
                               ros::Time time = ros::Time::now());

    ros::NodeHandle _nh;
    ros::CallbackQueue _queue;
    std::vector<ros::Subscriber> _subs;
    ros::Publisher _pub;
    std::mutex _mtx;

    std::vector<std::shared_ptr<tf::TransformListener>> _listeners;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _input_clouds;
    std::string _base_link;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _merged_cloud, _downsampled_cloud;
};

#endif // PERCEPTION_H
