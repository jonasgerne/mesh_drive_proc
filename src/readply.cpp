//
// Created by jonasgerstner on 21.02.20.
//
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;

void read(){
    ROS_INFO("Reading ply.");
    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    pcl::PLYReader reader;
    reader.read("/home/jonasgerstner/Downloads/cube.ply", *mesh);

    pcl_msgs::PolygonMesh pcl_mesh_msg;
    pcl_conversions::fromPCL(*mesh, pcl_mesh_msg);
    pcl_mesh_msg.header.stamp = ros::Time::now();
    pub.publish(pcl_mesh_msg);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "ply_reader");
    ros::NodeHandle nh;

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl_msgs::PolygonMesh> ("mesh_from_ply", 1, true);

    read();
    ROS_INFO("Published PolygonMesh");
    // Spin
    ros::spin();
}
