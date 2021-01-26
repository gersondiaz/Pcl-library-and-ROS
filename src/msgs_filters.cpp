
// LIBRERIAS DE ROS, MESSAGE_FILTER Y C++
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
//	LIBRERIAS DE PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>


class Node
{
 public:
  Node()
  //:viewer("Cloud Viewer")
  {
    sub_1_.subscribe(nh_, "/Nube_comparar", 1);
    sub_2_.subscribe(nh_, "/Molde_malo", 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_));
    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2));
    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("Parte_Danada", 1);
   // viewer_timer = nh_.createTimer(ros::Duration(0.1), &Node::timerCB, this);	
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::PointCloud2ConstPtr& input2)
  {
    std::cout<<"Ya entraste a la funcion"<<std::endl;	
   //ROS_INFO("Synchronization successful");
    
    //Leemos a la nube A
    pcl::PointCloud<pcl::PointXYZ> cloud_A;
    pcl::fromROSMsg(*input, cloud_A); 

	
    //Leemos a la nube B
    pcl::PointCloud<pcl::PointXYZ> cloud_B;
    pcl::fromROSMsg(*input2, cloud_B); 
      
  

    //Declaramos la resta de las nubes y procesamos en pcl
    pcl::PointCloud<pcl::PointXYZ> Diferencia;
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr Diferencia (new pcl::PointCloud<pcl::PointXYZ>); 
	
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::SegmentDifferences<pcl::PointXYZ> sdiff;
    sdiff.setInputCloud(cloud_A.makeShared());
    sdiff.setTargetCloud(cloud_B.makeShared());
    sdiff.setSearchMethod(tree);
    sdiff.setDistanceThreshold(0.000024); //0.000024
    sdiff.segment(Diferencia); 

    //Convertimos la nube en mensaje de ros
    sensor_msgs::PointCloud2 output;	 	
    pcl::toROSMsg(Diferencia, output); 
		
    pcl_pub.publish(output); 
    output.header.frame_id = "world"; 
    //viewer.showCloud(Diferencia.makeShared());		
    
  }

 // void timerCB(const ros::TimerEvent&)
 //{
  //if (viewer.wasStopped())
 //{
  //ros::shutdown();
 //}
 
//}


 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  ros::Publisher pcl_pub;
  //pcl::visualization::CloudViewer viewer;
  //ros::Timer viewer_timer;
	
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Resta_nubes");

  Node synchronizer;

  ros::spin();
  
  return 0;
}
