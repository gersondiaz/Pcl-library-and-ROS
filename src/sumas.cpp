
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

  {
    sub_1_.subscribe(nh_, "/Parte_Danada", 1);
    sub_2_.subscribe(nh_, "/Proyeccion_1", 1);
    sub_3_.subscribe(nh_, "/Proyeccion_2", 1);
    sub_4_.subscribe(nh_, "/Proyeccion_3", 1);
    sub_5_.subscribe(nh_, "/Proyeccion_4", 1);
    sub_6_.subscribe(nh_, "/Proyeccion_5", 1);
    sub_7_.subscribe(nh_, "/Proyeccion_6", 1);
    sub_8_.subscribe(nh_, "/Proyeccion_7", 1);
 
    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_, sub_3_, sub_4_, sub_5_, sub_6_, sub_7_, sub_8_));
    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("Suma_total", 1);
   
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::PointCloud2ConstPtr& input2, const sensor_msgs::PointCloud2ConstPtr& input3, const sensor_msgs::PointCloud2ConstPtr& input4, const sensor_msgs::PointCloud2ConstPtr& input5, const sensor_msgs::PointCloud2ConstPtr& input6, const sensor_msgs::PointCloud2ConstPtr& input7, const sensor_msgs::PointCloud2ConstPtr& input8)
  {
    std::cout<<"Ya entraste a la funcion"<<std::endl;	
    //ROS_INFO("Synchronization successful");
    
    //Leemos a la nube A
    pcl::PointCloud<pcl::PointXYZ> cloud_A;
    pcl::fromROSMsg(*input, cloud_A);
    //Leemos a la nube B
    pcl::PointCloud<pcl::PointXYZ> cloud_B;
    pcl::fromROSMsg(*input2, cloud_B);
    //Leemos a la nube C
    pcl::PointCloud<pcl::PointXYZ> cloud_C;
    pcl::fromROSMsg(*input3, cloud_C);
    //Leemos a la nube D
    pcl::PointCloud<pcl::PointXYZ> cloud_D;
    pcl::fromROSMsg(*input4, cloud_D);
    //Leemos a la nube E
    pcl::PointCloud<pcl::PointXYZ> cloud_E;
    pcl::fromROSMsg(*input5, cloud_E);
    //Leemos a la nube F
    pcl::PointCloud<pcl::PointXYZ> cloud_F;
    pcl::fromROSMsg(*input6, cloud_F);
    //Leemos a la nube G
    pcl::PointCloud<pcl::PointXYZ> cloud_G;
    pcl::fromROSMsg(*input7, cloud_G);
    //Leemos a la nube H
    pcl::PointCloud<pcl::PointXYZ> cloud_H;
    pcl::fromROSMsg(*input8, cloud_H);

        
    //Declaramos la resta de las nubes y procesamos en pcl
    pcl::PointCloud<pcl::PointXYZ> Suma_total;
    
    Suma_total = cloud_A;
    Suma_total += cloud_B;
    Suma_total += cloud_C;
    Suma_total += cloud_D;
    Suma_total += cloud_E;
    Suma_total += cloud_F;
    Suma_total += cloud_G;
    Suma_total += cloud_H;
 
    //Convertimos la nube en mensaje de ros
    sensor_msgs::PointCloud2 output;	
    pcl::toROSMsg(Suma_total, output); 		
    pcl_pub.publish(output); 
    output.header.frame_id = "map"; 
    
  }




 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_3_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_4_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_5_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_6_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_7_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_8_;


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  ros::Publisher pcl_pub;

	
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Suma_totales");

  Node synchronizer;

  ros::spin();
  
  return 0;
}
