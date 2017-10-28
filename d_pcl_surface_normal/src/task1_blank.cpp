#include <string>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <queue>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace pcl;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
ros::Publisher low_curvature;
ros::Publisher surface1;
ros::Publisher surface2;
ros::Publisher surface3;
ros::Publisher vis_pub;
///The smallest scale to use in the DoN filter.[meter]
double scale1;
///The largest scale to use in the DoN filter.[meter]
double scale2;
///The minimum DoN magnitude to threshold by.[meter]
double threshold;
///segment scene into clusters with given distance tolerance using euclidean clustering
double segradius;

void Callback(const PointCloudXYZRGB::ConstPtr& input)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*input,*cloud);

	/*---------------------------------------
	something here
	compute curvature
	...

	referance: ex_node1

	---------------------------------------*/

	// Filter by magnitude
	cout << "Filtering out DoN mag <= " << threshold << "..." << endl;
	/*---------------------------------------
	something here
	filter the low curvature
	...

	publish the result
	low_curvature.publish(*doncloud);
	--------------------------------------
	*/
  	cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  	pcl::search::KdTree<PointXYZRGBNormal>::Ptr segtree (new pcl::search::KdTree<PointXYZRGBNormal>);
  	segtree->setInputCloud (doncloud);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<PointXYZRGBNormal> ec;

  	ec.setClusterTolerance (segradius);
  	ec.setMinClusterSize (50);
  	ec.setMaxClusterSize (100000);
  	ec.setSearchMethod (segtree);
  	ec.setInputCloud (doncloud);
  	ec.extract (cluster_indices);
  	queue<pcl::PointCloud<PointXYZRGBNormal>::Ptr> cluster_que;
  	int j = 0;
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  	{
  		if(j==3) break;
    	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointXYZRGBNormal>);
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    	{
      		cloud_cluster_don->points.push_back (doncloud->points[*pit]);
    	}
    	cloud_cluster_don->header = doncloud->header;
    	cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    	cloud_cluster_don->height = 1;
    	cloud_cluster_don->is_dense = true;
    	cluster_que.push(cloud_cluster_don);
    	
    	cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    	
  	}
	
	
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	queue<pcl::PointCloud<PointXYZRGBNormal>::Ptr> surface_queue;
  	
  	Eigen::Vector4f c[3];
  	Eigen::Vector3f normal[3];
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (50);
  	seg.setDistanceThreshold (0.01);

  	for(int i=0;i<3;i++){
  		seg.setInputCloud (cluster_que.front());
    	seg.segment (*inliers, *coefficients);
    	if (inliers->indices.size () == 0)
    	{
      		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      		break;
    	}

    	// Extract the planar inliers from the input cloud
    	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    	extract.setInputCloud (cluster_que.front());
    	extract.setIndices (inliers);
    	extract.setNegative (false);

    	// Get the points associated with the planar surface
    	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    	extract.filter(*cloud_plane);
    	/*
    	somthing here
    	compute ceter point, surface normal
    	...
		store center point in vector c
		store center point in vector normal
    	*/
    	surface_queue.push(cloud_plane);
    	
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    	cluster_que.pop();
    }
    visualization_msgs::MarkerArray markerArray;
    
  	for(int i=0;i<3;i++){
  		visualization_msgs::Marker marker;
  		marker.type = visualization_msgs::Marker::ARROW;
  		marker.header.frame_id = doncloud->header.frame_id;
  		marker.header.stamp = ros::Time::now();
  		marker.ns = "arrow";
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point p;
      	p.x = c[i][0];
      	p.y = c[i][1];
      	p.z = c[i][2];
     	marker.points.push_back(p);

     	p.x += normal[i][0]/10;
      	p.y += normal[i][1]/10;
      	p.z += normal[i][2]/10;
      	marker.points.push_back(p);

		marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 1.0f;
		marker.color.a = 1.0;
		marker.scale.x = 0.01;
    	marker.scale.y = 0.02;
    	marker.scale.z = 0.01;
		markerArray.markers.push_back(marker);
    }
    vis_pub.publish(markerArray);
	surface1.publish(*surface_queue.front());
	surface_queue.pop();
	surface2.publish(*surface_queue.front());
	surface_queue.pop();
	surface3.publish(*surface_queue.front());
	surface_queue.pop();
	
	cout<<"Success output\n\n";
}

int main(int argc, char **argv)
{
  
	if (argc < 5){
		cerr << "usage: " << argv[0] << "smallscale largescale threshold" << endl;
		exit (EXIT_FAILURE);
	}

	istringstream (argv[1]) >> scale1;       // small scale
	istringstream (argv[2]) >> scale2;       // large scale
	istringstream (argv[3]) >> threshold;    // threshold for DoN magnitude
	istringstream (argv[4]) >> segradius;   // threshold for radius segmentation


	ros::init(argc, argv, "task1");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<PointCloudXYZRGB>("/camera/ros_pointcloudxyz", 10, Callback);

	low_curvature = n.advertise<PointCloudXYZRGB > ("/camera/task1/low_curvature", 1);
	surface1 = n.advertise<PointCloudXYZRGB > ("/camera/task1/surface1", 1);
	surface2 = n.advertise<PointCloudXYZRGB > ("/camera/task1/surface2", 1);
	surface3 = n.advertise<PointCloudXYZRGB > ("/camera/task1/surface3", 1);
	vis_pub = n.advertise<visualization_msgs::MarkerArray>( "center_point_normal", 0);
	ros::spin ();
	return 0;
}