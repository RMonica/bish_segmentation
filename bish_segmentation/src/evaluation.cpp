#include <bish_segmentation/evaluation.h>

/****************************************************************************************************/

float evaluate(std::string tppath, std::string rppath, std::string test_cloud_name)
{
	pcl::PointCloud<pcl::PointXYZRGBA> ground_truth_cloud = extractPointsXYZRGBA(tppath, test_cloud_name);
	
	std::string tmpname = test_cloud_name.substr(0, test_cloud_name.size()-4).append("-2_LABELED.pcd");
	pcl::PointCloud<pcl::PointXYZRGBA> segmented_cloud = extractPointsXYZRGBA(rppath, tmpname);
	
	if(ground_truth_cloud.size() != segmented_cloud.size())
	{
		ROS_ERROR("Different number of points between clouds (%i, %i)", int(ground_truth_cloud.size()), int(segmented_cloud.size()));
		exit(-1);
	}
	
	unsigned int diff = 0;
	for(unsigned int i=0; i<segmented_cloud.size(); i++)
		if(segmented_cloud[i].r != ground_truth_cloud[i].r || segmented_cloud[i].g != ground_truth_cloud[i].g || segmented_cloud[i].b != ground_truth_cloud[i].b)
			diff++;
	
	return float(segmented_cloud.size() - diff)/float(segmented_cloud.size());
}
