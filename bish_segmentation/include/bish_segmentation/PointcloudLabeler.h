#ifndef POINTCLOUD_LABELER_H
#define POINTCLOUD_LABELER_H

// local
#include "debug.h"
#include "HausdorffDist.h"
#include "HausdorffImageMatch.h"
#include "HausdorffImageSimplify.h"
#include "HausdorffNode.h"
#include "utility.h"

// C++
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <float.h>
#include <fstream>
#include <iostream>
#include <tuple>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include <ros/ros.h>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/
typedef struct CP {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	float a;
	float probability;
	
	CP(): r(0), g(0), b(0), a(0), probability(0) {}
	CP(uint8_t r, uint8_t g, uint8_t b, float a, float prob): r(r), g(g), b(b), a(a), probability(prob) {}
	
	inline bool operator==(const CP& cp)
	{
		if(r != cp.r || g != cp.g || b != cp.b)
			return false;
		
		return true;
	}
	inline bool operator!=(const CP& cp) { return !operator==(cp); }
	
	inline bool operator>(const CP& cp)
	{
		if(operator==(cp))
			return false;
		if(r < cp.r)
			return false;
		if(g < cp.g)
			return false;
		if(b < cp.b)
			return false;
		
		return true;
	}
	inline bool operator<(const CP& cp)
	{
		if(operator==(cp))
			return false;
		
		return !operator>(cp);
	}
	
	inline bool operator>=(const CP& cp) { return operator>(cp) || operator==(cp); }
	inline bool operator<=(const CP& cp) { return operator<(cp) || operator==(cp); }
	
} ColorProbability;

inline std::ostream& operator <<(std::ostream& os, const ColorProbability& cp)
{
	os << "RGBA: " << int(cp.r) << ", " << int(cp.g) << ", " << int(cp.b) << ", " << cp.a << " | probability: " << cp.probability;
	
	return os;
}

/**************************************************/

typedef struct {
	std::string name;
	int distance;
} ImageDistance;


typedef struct {
	std::pair<int, int> row_limits;
	int dstNodeRow_start;
	std::vector<int> line;
	float const_shift;
} DataForProbability;


/****************************************************************************************************/
/****************************************************************************************************/

class PointcloudLabeler
{
public:
  PointcloudLabeler(ros::NodeHandle & nh): m_nh(nh) { }
  ~PointcloudLabeler() { }
	
  bool run(std::string results_path, std::string tppath, std::string rppath, std::string test_cloud_name, std::string tpath, std::string lpath);

private:
	ros::NodeHandle & m_nh;
	cv::Mat getPointProbability(cv::Size size, std::vector<HausdorffNode> srcNodes, std::vector<HausdorffNode> dstNodes);
};


#endif

