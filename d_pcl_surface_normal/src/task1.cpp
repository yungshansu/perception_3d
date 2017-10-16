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
#include <queue>
using namespace pcl;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
ros::Publisher low_curvature;
ros::Publisher surface1;
ros::Publisher surface2;
ros::Publisher surface3;
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

	// Create a search tree, use KDTreee for non-organized data.
	pcl::search::Search<PointXYZRGB>::Ptr tree;
	if (cloud->isOrganized ()){
		tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
	}
	else{
		tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
	}

	// Set the input pointcloud for the search tree
	tree->setInputCloud (cloud);

	if (scale1 >= scale2){
		cerr << "Error: Large scale must be > small scale!" << endl;
		exit (EXIT_FAILURE);
	}

	// Compute normals using both small and large scales at each point
	pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
	ne.setInputCloud (cloud);
	ne.setSearchMethod (tree);

	ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

	// calculate normals with the small scale
	cout << "Calculating normals for scale..." << scale1 << endl;
	pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

	ne.setRadiusSearch (scale1);
	ne.compute (*normals_small_scale);

	// calculate normals with the large scale
	cout << "Calculating normals for scale..." << scale2 << endl;
	pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

	ne.setRadiusSearch (scale2);
	ne.compute (*normals_large_scale);

	// Create output cloud for DoN results
	PointCloud<PointXYZRGBNormal>::Ptr doncloud (new pcl::PointCloud<PointXYZRGBNormal>);
	copyPointCloud<PointXYZRGB, PointXYZRGBNormal>(*cloud, *doncloud);

	cout << "Calculating DoN... " << endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointXYZRGBNormal> don;
	don.setInputCloud (cloud);
	don.setNormalScaleLarge (normals_large_scale);
	don.setNormalScaleSmall (normals_small_scale);

	if (!don.initCompute ())
	{
		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
		exit (EXIT_FAILURE);
	}

	// Compute DoN
	don.computeFeature (*doncloud);

	// Filter by magnitude
	cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

	// Build the condition for filtering
	pcl::ConditionOr<PointXYZRGBNormal>::Ptr range_cond (
		new pcl::ConditionOr<PointXYZRGBNormal> ()
	);
	//only curvatures which is less than threshold can stay in the pointcloud, LT means less than
	range_cond->addComparison (pcl::FieldComparison<PointXYZRGBNormal>::ConstPtr (
		new pcl::FieldComparison<PointXYZRGBNormal> ("curvature", pcl::ComparisonOps::LT, threshold))	 
	);

	// Build the filter
	pcl::ConditionalRemoval<PointXYZRGBNormal> condrem (range_cond);
	condrem.setInputCloud (doncloud);

	pcl::PointCloud<PointXYZRGBNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointXYZRGBNormal>);

	// Apply filter
	condrem.filter (*doncloud_filtered);

	doncloud = doncloud_filtered;
	low_curvature.publish(*doncloud);
	 // Filter by magnitude
	
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
    	//Save cluster
    	cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    	//stringstream ss;
    	//ss << "don_cluster_" << j << ".pcd";
   	 	//writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
  	}
	cout<<"cluster num : "<<cluster_que.size()<<endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
	//copyPointCloud<PointXYZRGBNormal, PointXYZRGB>(*cluster.front(), *output);
	pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	queue<pcl::PointCloud<PointXYZRGBNormal>::Ptr> surface_queue;
  	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

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
    	extract.filter(*cloud_plane);
    	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr newcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		copyPointCloud<PointXYZRGBNormal, PointXYZRGBNormal>(*cloud_plane, *newcloud);
    	surface_queue.push(newcloud);
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    	cluster_que.pop();
    }
	surface1.publish(*surface_queue.front());
	surface_queue.pop();
	surface2.publish(*surface_queue.front());
	surface_queue.pop();
	surface3.publish(*surface_queue.front());
	surface_queue.pop();
	//low_curvature.publish(*doncloud);
	cout<<"Success output\n";
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
	ros::spin();

	return 0;
}