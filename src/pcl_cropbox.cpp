#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <iostream>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("Pcl_flitro_SOR_final", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_cropbox", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2& input)
    {

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        sensor_msgs::PointCloud2 output;
        pcl::fromROSMsg(input, cloud);

	float minX = -1, minY = -0.230 , minZ = 0.0390;
        float maxX = 1, maxY = 0.0680 , maxZ =  0.225;

        pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setInputCloud(cloud.makeShared());
	boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 0));
	
	boxFilter.filter(cloud_filtered);

        pcl::toROSMsg(cloud_filtered, output);
        pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_cropbox");

    cloudHandler handler;

    ros::spin();

    return 0;
}


