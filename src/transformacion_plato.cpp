
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
        pcl_sub = nh.subscribe("/pcl_transform", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("Nube_plato", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
	pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_transform;
	pcl::PointCloud<pcl::PointXYZ> cloud_rotate;
        pcl::fromROSMsg(input, cloud);
	

	
	 Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  	// Define a translation of 2.5 meters on the x axis.
  	transform_2.translation() << 0.6, 1.2, 0.72;
  	// The same rotation matrix as before; theta radians around Z axis
  	//transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));


	pcl::transformPointCloud (cloud, cloud_transform, transform_2);
  	
	 //Publish the new cloud
       sensor_msgs::PointCloud2 output;
       pcl::toROSMsg(cloud_transform, output);
       pcl_pub.publish(output);
       output.header.frame_id = "world"; 
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    //float theta = M_PI/4; // The angle of rotation in radians	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Nube_en_plato");

    cloudHandler handler;

    ros::spin();

    return 0;
}

