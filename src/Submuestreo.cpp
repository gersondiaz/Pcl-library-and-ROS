
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("Nube_flitro_SOR", 10, &cloudHandler::cloudCB, this); // Nube_flitro_SOR
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("Nube_submuestreada", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;
        sensor_msgs::PointCloud2 output;
        pcl::fromROSMsg(input, cloud);

        pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
        voxelSampler.setInputCloud(cloud.makeShared());
        voxelSampler.setLeafSize(0.001f, 0.001f, 0.001f);
        voxelSampler.filter(cloud_downsampled);

        pcl::toROSMsg(cloud_downsampled, output);
        pcl_pub.publish(output);

    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "Nube_submuestreada");

    cloudHandler handler;

    ros::spin();

    return 0;
}

