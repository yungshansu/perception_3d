#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"  
ros::Publisher pub;
ros::Publisher pointcloudXYZ;
ros::Publisher pointcloud2_publisher;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
// count PCLXYZ valid points percentage of each frame
float count_valid_percentage(PointCloudXYZ*  cloud){
    int i, j;
    float max_valid, min_valid, percentage;
    float count = 0.0;// the num of invalid points
    float tmp;
    for(i=0;i<cloud->height;i++){
        for(j=0;j<cloud->width;j++){
            tmp = cloud->points[i*j].z;
            // use isnan(arg) to examine tmp(z) valid or not
            if(std::isnan(tmp)){
                count+=1.0;
            }
        }
    }
    //count valid percentage
    percentage = 1.0f - count / (cloud->width * cloud->height);
	ROS_INFO("%.4f",percentage*100);//cout
    return percentage;
}
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	 // Create a container for the data.
 	sensor_msgs::PointCloud2 output;
 	sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;

	// Do data processing here...
	output = *input;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);//convert from PointCloud2 to pcl
	/*
       void fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
       {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(cloud, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
       }
       */

    	pcl::toROSMsg(cloud, pcl_to_ros_pointcloud2);//convert back to PointCloud2
    //count valid percentage of output    
	float percentage;
	// pass PointXYZ to count valid point percentage	
	percentage = count_valid_percentage(&cloud);
	//publish to topics
    	pub.publish (output);
	pointcloudXYZ.publish(cloud);
	pointcloud2_publisher.publish(pcl_to_ros_pointcloud2);
	//ROS_INFO("Success output");//cout
}   
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1); 
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("ros_pointcloudxyz", 1);
     pointcloud2_publisher = nh.advertise<sensor_msgs::PointCloud2> ("pcltoros_pointcloud2", 1);
     // Spin
     ros::spin ();
  }
