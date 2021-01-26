
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

main(int argc, char **argv)
{
    ros::init (argc, argv, "Leer_Nube");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("Nube_normal", 1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::io::loadPCDFile ("/home/gerson/catkin_ws/Nubes_molde/molde_miercoles1.pcd", cloud);

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


// molde_seminario3.pcd
// molde_defensa.pcd
// molde_20207
// molde_miercoles1
