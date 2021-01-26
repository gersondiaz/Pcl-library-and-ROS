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
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>


class Node
{
 public:
  Node()

  {
    sub_1_.subscribe(nh_, "/Parte_Danada", 1);
    sub_2_.subscribe(nh_, "/model_coef", 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_));
    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2));
    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("Proyeccion_3", 1);

  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& input, const pcl_msgs::ModelCoefficientsConstPtr& input2)
  {
    std::cout<<"Ya entraste a la funcion"<<std::endl;	
 
    //Leemos a la nube A
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_projected;
    pcl::fromROSMsg(*input, cloud); 
    // Leemos a los coeficientes del modelo  
    pcl::ModelCoefficients model_coef;
    pcl_conversions::toPCL(*input2, model_coef);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = model_coef.values[0];
    coefficients->values[1] = model_coef.values[1];
    coefficients->values[2] = model_coef.values[2];
    coefficients->values[3] = model_coef.values[3] + 0.0171;   //eje z para poder hacer la  proyección
     
    //std::cout<<model_coef.values[0] <<std::endl;
    //std::cout<<model_coef.values[1] <<std::endl;
    //std::cout<<model_coef.values[2] <<std::endl;
    //std::cout<<model_coef.values[3] <<std::endl;

    //Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud.makeShared());
    proj.setModelCoefficients(coefficients);
    proj.setCopyAllData(true);
    proj.filter(cloud_projected);
    //std::cout<< coefficients.values[0]<<std::endl;
   
    //Convertimos la nube en mensaje de ros
    sensor_msgs::PointCloud2 output;	 	
    pcl::toROSMsg(cloud_projected, output); 
    pcl_pub.publish(output); 
    output.header.frame_id = "map"; 
 	
    
  }




 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1_;
  message_filters::Subscriber<pcl_msgs::ModelCoefficients> sub_2_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, pcl_msgs::ModelCoefficients> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  ros::Publisher pcl_pub;
 
	
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Proyeccion_3");

  Node synchronizer;

  ros::spin();
  
  return 0;
}
