#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/passthrough.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("Nube_normal", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("Pass_through", 1);
    }
	

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_passthrough;
        sensor_msgs::PointCloud2 output;
        pcl::fromROSMsg(input, cloud);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
  	pass.setInputCloud(cloud.makeShared());
 	pass.setFilterFieldName ("x");
  	pass.setFilterLimits (0.700, 1.0);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter(cloud_passthrough);

	pcl::toROSMsg(cloud_passthrough, output);
        pcl_pub.publish(output);




    }	
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};


main(int argc, char **argv)
{
    ros::init(argc, argv, "Pass_Through");

    cloudHandler handler;

    ros::spin();

    return 0;
}

