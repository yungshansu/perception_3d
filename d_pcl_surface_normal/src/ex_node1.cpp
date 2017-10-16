#include <ros/ros.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <boost/foreach.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
ros::Publisher pointcloudXYZRGB;
ros::Publisher mask_pub;
ros::Publisher result_pub;
using namespace pcl;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
void  cloud_cb (const PointCloudXYZRGB::ConstPtr& input, const ImageConstPtr& rgb)
{
	//convert datatype sensor_msgs::Image to CvImage
	cv_bridge::CvImagePtr cv_ptr;
   	try{
      	    cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      	ROS_ERROR("cv_bridge exception: %s", e.what());
	}	
	
	//convert rgb image to hsv image
	Mat hsvImg;
	cvtColor(cv_ptr->image, hsvImg, CV_BGR2HSV);
	
	//define color red
    Scalar hsv_l(170, 50, 50);
    Scalar hsv_h(180, 255, 255);
    
    //filter hsvImg by color red and generate mask
    Mat mask;
    inRange(hsvImg, hsv_l, hsv_h, mask);

    //convert from Mat to ros image message
	cv_bridge::CvImage out_msg;
	out_msg.header   = rgb->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::MONO8; 
	//publish mask to topic mask
	mask_pub.publish(out_msg.toImageMsg());

	//apply mask on the rgbimage
	for(int i=0;i<mask.rows;i++){
	    for(int j=0;j<mask.cols;j++)
			if((mask.at<uchar>(i,j))==0){
			    cv_ptr->image.at<Vec3b>(i,j)[0]=0;
			    cv_ptr->image.at<Vec3b>(i,j)[1]=0;
			    cv_ptr->image.at<Vec3b>(i,j)[2]=0;
			}		   
	}

	result_pub.publish(cv_ptr->toImageMsg());
	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*input,*cloud);

	//apply mask on input pointcloud
	for(int i=0;i<mask.rows;i++){
		for(int j=0;j<mask.cols;j++)
		  	if((mask.at<uchar>(i,j))==0){
		      	cloud->points[i*mask.cols+j].z= std::numeric_limits<float>::quiet_NaN();
		  	}                
    }    	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> ind;
	pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, ind); 		

	//publish to topics
	pointcloudXYZRGB.publish(*cloud_filtered);

	ROS_INFO("Success output");
	
	hsvImg.release();
	mask.release();
}   
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "ex_node1");
	ros::NodeHandle nh;   

	// Create a ROS subscriber for the input point cloud
	message_filters::Subscriber<PointCloudXYZRGB> pcl_sub(nh, "/camera/depth_registered/points", 1);
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_rect_color", 1);
	typedef sync_policies::ApproximateTime<PointCloudXYZRGB, Image> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pcl_sub, rgb_sub);
	//TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image > sync(pcl_sub, rgb_sub, 10);
	sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

	pointcloudXYZRGB = nh.advertise<PointCloudXYZRGB > ("/camera/ros_pointcloudxyz", 1);

	mask_pub = nh.advertise<Image > ("mask", 1);

	result_pub = nh.advertise<Image > ("result", 1);

	// Spin
	ros::spin ();
  }
