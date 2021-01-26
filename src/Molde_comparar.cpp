
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

main(int argc, char **argv)
{
    ros::init (argc, argv, "Leer_Nube");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("Nube_comparar", 1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::io::loadPCDFile ("/home/gerson/catkin_ws/Nubes_molde/moldeCAD_vs_moldeZED/nube_comparar_CAD.pcd", cloud);

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
// molde_031120
// molde_fromcad_plano

// "/home/gerson/catkin_ws/Nubes_molde/nubes_seminario_3/nube_comparar_CAD.pcd"
