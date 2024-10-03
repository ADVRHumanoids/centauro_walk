#include "perception.h"

Perception::Perception(ros::NodeHandle nh, std::vector<std::string> input_topics):
_nh(nh),
_merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    _nh.setCallbackQueue(&_queue);

    _input_clouds.resize(input_topics.size());
    _listeners.resize(input_topics.size());

    int i = 0;

    for(auto topic: input_topics)
    {
        std::cout << "Controller: subscribed to " << topic << "\n";

        auto cb = [this, i](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
        {
            callback(msg, i);
        };

        auto sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>(topic, 1, cb);

        _subs.push_back(sub);

        i++;
    }

    _pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("merged_cloud", 10);
}

void Perception::setBaseLink(std::string base_link)
{
    _base_link = base_link;
    _merged_cloud->header.frame_id = _base_link;
}

void Perception::callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg, int i)
{
    if(!_input_clouds[i])
    {
        try
        {
            _listeners[i] = std::make_shared<tf::TransformListener>();
            _listeners[i]->waitForTransform(_base_link, msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        _input_clouds[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    *pc = *msg;
    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*pc, *pc, ind);

    std::lock_guard<std::mutex> lg(_mtx);

    transform_point_cloud(pc, _input_clouds[i], _base_link, i);
}

void Perception::transform_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
                                       std::string frame_id,
                                       int i)
{
    tf::StampedTransform transform;
    _listeners[i]->lookupTransform(frame_id, cloud_in->header.frame_id, ros::Time(0), transform);

    Eigen::Affine3d b_T_cam;
    tf::transformTFToEigen(transform, b_T_cam);
    pcl::transformPointCloud(*cloud_in, *cloud_out, b_T_cam.matrix());

    cloud_out->header.frame_id = frame_id;
}

void Perception::update()
{
    if(!_merged_cloud->empty())
    {
        _merged_cloud->clear();
    }

    for (auto point_cloud : _input_clouds)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(point_cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*downsampled_cloud);

        *_merged_cloud += *downsampled_cloud;
    }

    _pub.publish(_merged_cloud);
}
