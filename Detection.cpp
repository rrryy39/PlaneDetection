#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::visualization::PCLVisualizer viewer("3D Viewer");

  //Input PCD File
	if (pcl::io::loadPCDFile("BlaserDemo.pcd", *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return -1;
	}
	std::cerr << "number:" << cloud->size() << std::endl;

  //Set Peremeters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
  //Use RANSAC algrithm to determine the parameters
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.001);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Error");
		return -1;
	}

  //Extracting indices
	pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
	extract_indices.setIndices(boost::make_shared<const pcl::PointIndices>(*inliers));
	extract_indices.setInputCloud(cloud);
	extract_indices.filter(*cloud1);
	std::cerr << "output point size:" << cloud1->points.size() << std::endl;
	
  //Save the Output PCD file
  pcl::io::savePCDFile("BlaserPlane.pcd", *cloud1);
	viewer.setBackgroundColor(255, 255, 255);
	viewer.initCameraParameters();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_handler(cloud1, 255, 0, 0);

  //View the PCD file
	viewer.addPointCloud(cloud1, output_handler, "plane points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane points");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
