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

using namespace pcl;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher pointcloudXYZRGB2;

///The smallest scale to use in the DoN filter.[meter]
double scale1;
///The largest scale to use in the DoN filter.[meter]
double scale2;
///The minimum DoN magnitude to threshold by.[meter]
double threshold;

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
	range_cond->addComparison (pcl::FieldComparison<PointXYZRGBNormal>::ConstPtr (
		new pcl::FieldComparison<PointXYZRGBNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
	);

	// Build the filter
	pcl::ConditionalRemoval<PointXYZRGBNormal> condrem (range_cond);
	condrem.setInputCloud (doncloud);

	pcl::PointCloud<PointXYZRGBNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointXYZRGBNormal>);

	// Apply filter
	condrem.filter (*doncloud_filtered);

	doncloud = doncloud_filtered;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud<PointXYZRGBNormal, PointXYZRGB>(*doncloud, *output);


	pointcloudXYZRGB2.publish(*output);
	ROS_INFO("Success output");
}

int main(int argc, char **argv)
{
  
	if (argc < 4){
		cerr << "usage: " << argv[0] << "smallscale largescale threshold" << endl;
		exit (EXIT_FAILURE);
	}

	istringstream (argv[1]) >> scale1;       // small scale
	istringstream (argv[2]) >> scale2;       // large scale
	istringstream (argv[3]) >> threshold;    // threshold for DoN magnitude


	ros::init(argc, argv, "ex_node2");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<PointCloudXYZRGB>("/camera/ros_pointcloudxyz", 10, Callback);

	pointcloudXYZRGB2 = n.advertise<PointCloudXYZRGB > ("/camera/ros_pointcloudxyz2", 1);

	ros::spin();

	return 0;
}