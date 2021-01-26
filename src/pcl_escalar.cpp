#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>
#include <iostream>


class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("Nube_plato", 10, &cloudHandler::cloudCB, this); //Nube_plato
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_escalar", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
	pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_scale;
        pcl::fromROSMsg(input, cloud);
	

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform (0,0) = transform (0,0) * 1.079; //altura
	transform (1,1) = transform (1,1) * 1.070; // ancho
	transform (2,2) = transform (2,2) * 1.071; // ancho
	pcl::transformPointCloud (cloud, cloud_scale, transform); 

	//Publish the new cloud
       sensor_msgs::PointCloud2 output;
       pcl::toROSMsg(cloud_scale, output);
       //output.header.frame_id = "/base_link";
       pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    //float theta = M_PI/4; // The angle of rotation in radians	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Nube_escalada");

    cloudHandler handler;

    ros::spin();

    return 0;
}

