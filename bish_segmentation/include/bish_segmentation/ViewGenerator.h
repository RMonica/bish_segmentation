
// local
#include "debug.h"
#include <init_fake_opengl_context/fake_opengl_context.h>
#include "surfel_next_best_view_render_plugin.h"
#include "surfel_renderer.h"
#include "utility.h"


// ROS
#include <ros/ros.h>

// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>

// C++
#include <cmath>
#include <float.h>
#include <iostream>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;

struct boundingBox {
	Eigen::Vector3d min;
	Eigen::Vector3d max;

	boundingBox(const Eigen::Vector3d & myMin, const Eigen::Vector3d & myMax) : min(myMin),  max(myMax) {/* do nothing*/}
};

/****************************************************************************************************/
/****************************************************************************************************/

class ViewGenerator
{
//VARIABLES
public:
	/* nothing*/
	
private:	
	//------------------------- variables
	ros::NodeHandle & m_nh;
  static bool m_opengl_context_initialized;
	std::string m_opengl_context_screen;

	SurfelRenderer::Ptr m_renderer;

	PointSurfelCloud::ConstPtr m_cloud;
	GPUSurfelCloud::ConstPtr m_cloud_gpu;
	
	int image_height, image_width;
	double focal_x, focal_y;
	Eigen::Vector2i size;
	Eigen::Vector2f center;
	Eigen::Vector2f focal;
	Eigen::Vector2f range;
	Eigen::Vector3d angle_step;

	RenderPlugin *render_plugin = NULL;
	const sensor_msgs::JointState *const joint_state = NULL;
	
public:
  typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > Affine3dVector;
  typedef std::vector<SurfelRenderer::GLUintVector> GLUintVectorVector;
  typedef std::vector<SurfelRenderer::GLFloatVector> GLFloatVectorVector;

	ViewGenerator(ros::NodeHandle & nh): m_nh(nh)
	{
		double range_min, range_max;
		double thetaX, thetaY, thetaZ;
		nh.param<int>("image_height", image_height, 512);
		nh.param<int>("image_width", image_width, 512);
		focal_x = double(image_width);
		focal_y = double(image_height);
		nh.param<double>("range_min", range_min, 0.0f);
		nh.param<double>("range_max", range_max, 5.0f);
		nh.param<double>("theta_x", thetaX, 0);
		nh.param<double>("theta_y", thetaY, 0);
		nh.param<double>("theta_z", thetaZ, 0);
		
		size = Eigen::Vector2i(image_width, image_height);
    center = Eigen::Vector2f(double(image_width/2), double(image_height/2));
		focal = Eigen::Vector2f(focal_x, focal_y);
		range = Eigen::Vector2f(range_min, range_max);
		angle_step = Eigen::Vector3d(thetaX, thetaY, thetaZ);
	}

	~ViewGenerator() { /*do nothing*/ }

  int Height() const {return image_height; }
  int Width() const {return image_width; }
	
	/****************************************************************************************************/

	void CheckOpenGLInit()
	{
		if (!m_opengl_context_initialized)
		{
      ROS_INFO("bish_segmentation: ViewGenerator: initializing fake opengl context (Screen: \"%s\").",
			m_opengl_context_screen.c_str());
			InitFakeOpenGLContext(m_opengl_context_screen);
			m_opengl_context_initialized = true;
		}
	}
	
	/****************************************************************************************************/

  void run(std::string ppath, std::string saving_path, const uint64 k_search_count,
           const float point_radius, bool with_color, bool with_depth = true, bool enable_lighting = false,
           const bool back_face_culling = false);

  /****************************************************************************************************/

  void runMem(const PointSurfelCloud & cloud,
              const Affine3dVector & view_poses,
              const bool with_color, const bool with_depth, const bool enable_lighting,
              const bool back_face_culling,
              const std::string saving_path, const std::string saving_source_name);
	
	/****************************************************************************************************/

	void setPointCloud(PointSurfelCloud cloud) { m_cloud_gpu = GPUSurfelCloud::ConstPtr(new GPUSurfelCloud(cloud));	}
	
private:
	boundingBox computeDists(PointSurfelCloud cloud);
	
  PointSurfelCloud extractPointsSurfel(std::string path, std::string filename,
                                       const uint64 k_search_count, const float point_radius);
	
  void saveImage(SurfelRenderer::GLFloatVector imgvector, SurfelRenderer::GLUintVector indexvector,
                 std::string saving_path, std::string filename,
                 double thetaX, double thetaY, double thetaZ, bool with_color);

  void saveImageCustomString(SurfelRenderer::GLFloatVector imgvector, SurfelRenderer::GLUintVector indexvector,
                             std::string saving_path, std::string filename,
                             std::string custom_string, bool with_color);
};

/****************************************************************************************************/




