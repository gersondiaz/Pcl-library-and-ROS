#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("Nube_normal", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("Nube_flitro_SOR", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2& input)
    {
	ROS_INFO("Synchronization successful");
	//std::cout<<"Entro a funcion"<<std::endl;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statFilter;
        statFilter.setInputCloud(cloud.makeShared());
        statFilter.setMeanK(600);       // Numero de puntos vecinos que busca 
        statFilter.setStddevMulThresh(0.4); // Distancia 
        statFilter.filter(cloud_filtered);

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
    ros::init(argc, argv, "Filtro_SOR");

    cloudHandler handler;

    ros::spin();

    return 0;
}

