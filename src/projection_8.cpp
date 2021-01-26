
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

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_projected_7", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_projected_8", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_projected	;
        pcl::fromROSMsg(input, cloud);

    //Create a set of planar coefficients with X=Y=0,Z=1
       pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
       coefficients->values.resize (4);
       coefficients->values[0] = 0.9890;
       coefficients->values[1] = -0.0583;
       coefficients->values[2] = -0.1355;
       coefficients->values[3] = -0.7760;   //eje z para poder hacer la  proyección

    //Create the filtering object
       pcl::ProjectInliers<pcl::PointXYZ> proj;
       proj.setModelType (pcl::SACMODEL_PLANE);
       proj.setInputCloud (cloud.makeShared());
       proj.setModelCoefficients (coefficients);
       proj.setCopyAllData(true);
       proj.filter(cloud_projected);
  	
        
    //Publish the new cloud
       sensor_msgs::PointCloud2 output;
       pcl::toROSMsg(cloud_projected, output);
       pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Projecting_8");

    cloudHandler handler;

    ros::spin();

    return 0;
}
