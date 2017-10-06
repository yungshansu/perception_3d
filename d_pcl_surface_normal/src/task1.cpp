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
	ROS_INFO("test");
	cv_bridge::CvImagePtr cv_ptr;
   	try{
		ROS_INFO("try1");
      	    cv_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    	}
    	catch (cv_bridge::Exception& e){
      	ROS_ERROR("cv_bridge exception: %s", e.what());
	}	
	// Update GUI Window
    	//cv::imshow("Image window", cv_ptr->image);
    	//cv::waitKey(3);
	Mat hsvImg;
	cvtColor(cv_ptr->image, hsvImg, CV_BGR2HSV);
	//cv::imshow("Image window", hsvImg);
    	//cv::waitKey(3);
        Scalar hsv_l(170, 50, 50);
        Scalar hsv_h(180, 255, 255);
        Mat bw;
        inRange(hsvImg, hsv_l, hsv_h, bw);

	//cvtColor(hsvImg,cv_ptr->image, CV_HSV2BGR);
	//cv_ptr->image.copyTo(cv_ptr->image,bw );
	

	
		ROS_INFO("try2");
	cv_bridge::CvImage out_msg;
	out_msg.header   = rgb->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	bw.copyTo(out_msg.image);// = bw; // Your cv::Mat
	mask_pub.publish(out_msg.toImageMsg());
	
		ROS_INFO("try3");
	//cvtColor(hsvImg,cv_ptr->image, CV_HSV2BGR);
	//result_pub.publish(cv_ptr->toImageMsg());
	for(int i=0;i<bw.rows;i++){
	    for(int j=0;j<bw.cols;j++)
		if((bw.at<uchar>(i,j))==0){
		    cv_ptr->image.at<Vec3b>(i,j)[0]=0;
		    cv_ptr->image.at<Vec3b>(i,j)[1]=0;
		    cv_ptr->image.at<Vec3b>(i,j)[2]=0;
		}
		   // input->points[j*bw.cols+i].z=100;
	}
	//cvtColor(hsvImg,cv_ptr->image, CV_HSV2BGR);
	result_pub.publish(cv_ptr->toImageMsg());
	
		ROS_INFO("try4");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::copyPointCloud (*input,*cloud);
	for(int i=0;i<bw.rows;i++){
              for(int j=0;j<bw.cols;j++)
                  if((bw.at<uchar>(i,j))==0){
                      cloud->points[i*bw.cols+j].z= std::numeric_limits<float>::quiet_NaN();
                  }   
                     // input->points[j*bw.cols+i].z=100;
        }    
	//pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new  pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 1000)));
        // build the filter
        //pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
        //condrem.setCondition (range_cond);
        //condrem.setInputCloud (cloud);
        //condrem.setKeepOrganized(true);
        // apply filter
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        //condrem.filter (*cloud_filtered);
	std::vector<int> ind;
	printf("Original point Size: %d %d %d\n",cloud_filtered->points.size(),cloud_filtered->width,cloud_filtered->height);	
	pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, ind); 	
	printf("Original point Size: %d %d %d\n",cloud_filtered->points.size(),cloud_filtered->width,cloud_filtered->height);	
	/*Mat res;
	bitwise_and(cv_ptr->image,bw,res);
	out_msg.image=res;
	out_msg.encoding = sensor_msgs::image_encodings::BGR8;
	result_pub.publish(out_msg.toImageMsg());
        */
	//image_pub_.publish(cv_ptr->toImageMsg());
	//imshow("Specific Colour", bw);
	//uchar* hsvDataPtr = hsvImg.data;
    	// Output modified video stream
	//mask_pub.publish(bw);
	// Create a container for the data.
 	//sensor_msgs::PointCloud2 output;
 	//sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;
	// Do data processing here...
	//output = *input;
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        //pcl::fromROSMsg (*input, *cloud);//convert from PointCloud2 to pcl
    	//pcl::toROSMsg(cloud, pcl_to_ros_pointcloud2);//convert back to PointCloud2
	
	//publish to topics
	pointcloudXYZRGB.publish(*cloud_filtered);
	ROS_INFO("Successi output");//cout
	hsvImg.release();
	bw.release();
}   
int main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
     message_filters::Subscriber<PointCloudXYZRGB> pcl_sub(nh, "/camera/depth_registered/points", 1);
     message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_rect_color", 1);
     typedef sync_policies::ApproximateTime<PointCloudXYZRGB, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pcl_sub, rgb_sub);
     //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image > sync(pcl_sub, rgb_sub, 10);
     sync.registerCallback(boost::bind(&cloud_cb, _1, _2));
     ROS_INFO("try"); // Create a ROS publisher for the output point cloud
     pointcloudXYZRGB = nh.advertise<PointCloudXYZRGB > ("/camera/ros_pointcloudxyz", 1);
     
     mask_pub = nh.advertise<Image > ("mask", 1);
     result_pub = nh.advertise<Image > ("result", 1);
     
     // Spin
     ros::spin ();
  }
