
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
        pcl_sub = nh.subscribe("Submuestreo_final", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_transform", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
	pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_transform;
	pcl::PointCloud<pcl::PointXYZ> cloud_rotate;
        pcl::fromROSMsg(input, cloud);
	//float theta = M_PI/2;
	//Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 
        //transform.translation() << 0.0, 0.0, 0.0;
	//transform.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(DEG2RAD(0.606085), Eigen::Vector3f::UnitX())
            //                                 * Eigen::AngleAxisf(DEG2RAD(0.438108), Eigen::Vector3f::UnitY())
          //                                   * Eigen::AngleAxisf(DEG2RAD(-0.663869), Eigen::Vector3f::UnitZ()); 
       
	//pcl::transformPointCloud (cloud, cloud_transform, transform);
	// Rotación de la nube de puntos
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//transform_1 (0,0) = std::cos(theta);
	//transform_1 (0,2) = sin(theta);
	//transform_1 (2,0) = -sin(theta);
  	//transform_1 (2,2) = std::cos(theta);
	
	// Traslación a coordenadas 0,0,0
	//transform_1 (0,3) = 0;
	//transform_1 (1,3) = 0;
  	//transform_1 (2,3) = 0;

	
	

	// Nube de puntos buenarda
	transform_1 (0,0) = 0.015055626631 ;
  	transform_1 (0,1) = 0.003600066761;
	transform_1 (0,2) = -0.999880194664;
	transform_1 (0,3) = 0.983876466751;
  	transform_1 (1,0) = 0.016639675945;
  	transform_1 (1,1) = -0.999855935574;
	transform_1 (1,2) = -0.003349428531;
	transform_1 (1,3) = -0.503740787506;
	transform_1 (2,0) = -0.999748170376;
  	transform_1 (2,1) = -0.016587253660;
	transform_1 (2,2) = -0.015113359317;
	transform_1 (2,3) = 0.993733763695;
	transform_1 (3,3) = 1;

	pcl::transformPointCloud (cloud, cloud_transform, transform_1);


  	//Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
  	//pcl::compute3DCentroid(cloud, centroid);
  	//Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
  	//centroid_new.head<3>() = transform.rotation() * centroid.head<3>();
  	//transform.translation() = centroid.head<3>() - centroid_new.head<3>();
  	//pcl::transformPointCloud(cloud, cloud_rotate, transform);
  	
	 //Publish the new cloud
       sensor_msgs::PointCloud2 output;
       pcl::toROSMsg(cloud_transform, output);
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
    ros::init(argc, argv, "Rotatin_and_trnasforming");

    cloudHandler handler;

    ros::spin();

    return 0;
}

