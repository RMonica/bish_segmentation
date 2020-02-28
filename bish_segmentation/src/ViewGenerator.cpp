#include <bish_segmentation/ViewGenerator.h>

/****************************************************************************************************/

bool ViewGenerator::m_opengl_context_initialized = false;

/****************************************************************************************************/

PointSurfelCloud ViewGenerator::extractPointsSurfel(std::string path, std::string filename,
                                                    const uint64 k_search_count, const float point_radius)
{
	// Open file for reading
	std::string tmpname = std::string(path);
	tmpname.append(filename);

	// Check if file is in OFF format
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_cloud_rgba (new pcl::PointCloud<pcl::PointXYZRGBA>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(tmpname, *tmp_cloud) == -1 ||
      pcl::io::loadPCDFile<pcl::PointXYZRGBA>(tmpname, *tmp_cloud_rgba) == -1) //* load the file
	{
		PCL_ERROR("Could not read file %s \n", filename);
		exit(-1);
	}

	PointSurfelCloud cloud;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(tmp_cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//ne.setRadiusSearch(0.05f);
  ne.setKSearch(k_search_count); // 10
	
	
	ne.compute(*cloud_normals);
	
	
	uint8_t r = 0;
	uint8_t g = 0;
	uint8_t b = 0;
	
	for(unsigned int i=0; i<tmp_cloud_rgba->points.size(); i++)
	{
		pcl::PointSurfel point;
		point.x = tmp_cloud_rgba->points[i].x;
		point.y = tmp_cloud_rgba->points[i].y;
		point.z = tmp_cloud_rgba->points[i].z;

		point.normal_x = cloud_normals->points[i].normal_x;
		point.normal_y = cloud_normals->points[i].normal_y;
		point.normal_z = cloud_normals->points[i].normal_z;
		
		point.rgba = tmp_cloud_rgba->points[i].rgba;
		
    point.radius = point_radius; // 0.005f
		point.confidence = 1.0f;
		cloud.push_back(point);
	}
	
	return cloud;
}

/****************************************************************************************************/

void ViewGenerator::saveImage(SurfelRenderer::GLFloatVector imgvector, SurfelRenderer::GLUintVector indexvector,
                              std::string saving_path, std::string filename, double thetaX, double thetaY, double thetaZ,
                              bool with_color)
{
  std::ostringstream ostr;
  ostr << thetaX << "-" << thetaY << "-" << thetaZ;
  saveImageCustomString(imgvector, indexvector, saving_path, filename, ostr.str(), with_color);
}

void ViewGenerator::saveImageCustomString(SurfelRenderer::GLFloatVector imgvector, SurfelRenderer::GLUintVector indexvector,
                                          std::string saving_path, std::string filename, std::string custom_string,
                                          bool with_color)
{
	cv::Mat rimg(image_height, image_width, CV_8UC3);
	for(int j=0; j<rimg.cols; j++)
		for(int k=0; k<rimg.rows; k++)
			rimg.at<cv::Vec3b>(j,k) = cv::Vec3b(255,255,255);

	if(with_color)
		for(unsigned int i=0; i<imgvector.size()/4; i++)
		{
			if(imgvector[i*4+3] != 0)
        rimg.at<cv::Vec3b>(i/image_width, i%image_width) = cv::Vec3b(imgvector[i*4]*255,
          imgvector[i*4+1]*255, imgvector[i*4+2]*255);
		}
	else
		for(unsigned int i=0; i<indexvector.size(); i++)
		{
			if(indexvector[i] != 0)
			{
				uint8_t b = (indexvector[i]-1) / (256*256);
				uint8_t g = ((indexvector[i]-1) - b*256*256) / 256;
				uint8_t r = (indexvector[i]-1) - b*256*256 - g*256;
				
				rimg.at<cv::Vec3b>(i/image_width, i%image_width) = cv::Vec3b(r, g, b);
			}
		}
	
	std::string tmpname(saving_path);
	if(!fileExists(tmpname))
		mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);

	std::ostringstream sstream;
  sstream << filename.substr(0,filename.size()-4) << "_" << custom_string << ".png";
	tmpname.append(sstream.str());
	cv::cvtColor(rimg, rimg, cv::COLOR_BGR2RGB);
	
	cv::imwrite(tmpname.c_str(),rimg);
}

/****************************************************************************************************/

void ViewGenerator::runMem(const PointSurfelCloud & cloud_in,
                           const Affine3dVector & view_poses,
                           const bool with_color, const bool with_depth, const bool enable_lighting,
                           const bool back_face_culling,
                           const std::string saving_path, const std::string saving_source_name)
{
  boundingBox bb(computeDists(cloud_in));

  PointSurfelCloud cloud;
  Eigen::Affine3d cloud_transform = Eigen::Affine3d::Identity();
  cloud_transform.translation() = -((bb.max + bb.min)/2.0f);
  pcl::transformPointCloud(cloud_in, cloud, Eigen::Affine3f(cloud_transform));
  bb = boundingBox(computeDists(cloud));

  Affine3dVector local_view_poses = view_poses;
  for (Eigen::Affine3d & pose : local_view_poses)
    pose = cloud_transform * pose;

  this->CheckOpenGLInit();

  this->setPointCloud(cloud);

  double maxX = std::max(bb.min[0], bb.max[0]);
  double maxY = std::max(bb.min[1], bb.max[1]);
  double maxZ = std::max(bb.min[2], bb.max[2]);
  double dist = 1.1f * (std::sqrt(std::pow(maxX,2) + std::pow(maxY,2) + std::pow(maxZ,2)) * std::max(focal_x, focal_y)) /
                (double(std::max(image_height,image_width))*0.9f/2.0f);

  for (Eigen::Affine3d & pose : local_view_poses)
  {
    const double dist_already = pose.translation().norm();
    pose = pose * Eigen::Translation3d(0.0f, 0.0f, -(dist - dist_already));
  }

  m_renderer.reset(new SurfelRenderer(m_nh, size, center, focal, range, with_color, with_depth, enable_lighting,
                                      back_face_culling, render_plugin));

  Affine3dVector poses;
  if (!local_view_poses.empty())
    poses = local_view_poses;
  else
  {
    double thetaX = 0.0f;
    do
    {
      double thetaY = 0.0f;
      do
      {
        double thetaZ = 0.0f;
        do
        {
          Eigen::Translation3d translation(0.0f, 0.0f, -dist);
          Eigen::Affine3d pose = Eigen::AngleAxisd(thetaZ*M_PI/180.0f,Eigen::Vector3d(0,0,1)) *
                      Eigen::AngleAxisd(thetaY*M_PI/180.0f,Eigen::Vector3d(0,1,0)) *
                      Eigen::AngleAxisd(thetaX*M_PI/180.0f,Eigen::Vector3d(1,0,0)) * translation;

          poses.push_back(pose);
          thetaZ += angle_step[2];
        } while(thetaZ <= 360.0f - angle_step[2] && angle_step[2] > 0.0f);

        thetaY += angle_step[1];
      } while(thetaY <= 180.0f && angle_step[1] > 0.0f);

      thetaX += angle_step[0];
    } while(thetaX <= 180.0f && angle_step[0] > 0.0f);
  }

  std::string tmpname(saving_path);
  if(!fileExists(tmpname))
    mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
  tmpname += saving_source_name + "/";
  recursiveDelete(tmpname);

  uint64 id = 0;
  for (const Eigen::Affine3d & pose : poses)
  {
    SurfelRenderer::GLUintVector indexvector;
    SurfelRenderer::GLFloatVector imgvector = m_renderer->Render(*m_cloud_gpu, pose.cast<float>(), joint_state, indexvector);
    const std::string custom_string = std::to_string(id);
    id++;
    saveImageCustomString(imgvector, indexvector, tmpname, saving_source_name + ".pcd", custom_string, with_color);
  }
}
	
/****************************************************************************************************/

void ViewGenerator::run(std::string ppath, std::string saving_path, const uint64 k_search_count, const float point_radius,
                        bool with_color, bool with_depth, bool enable_lighting,
                        const bool back_face_culling)
{
	std::vector<std::string> test_cloud_names = getPointcloudNames(ppath.c_str());
	if(test_cloud_names.size() == 0)
	{
		ROS_ERROR("No pointclouds in %s", ppath.c_str());
		exit(-1);
	}
	
	for(unsigned int i=0; i<test_cloud_names.size(); i++)
	{
		std::string tmpname(saving_path);
		if(!fileExists(tmpname))
			mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
		tmpname.append(test_cloud_names[i].substr(0,test_cloud_names[i].size()-4));
		tmpname.append("/");
		
    PointSurfelCloud cloud(extractPointsSurfel(ppath,test_cloud_names[i], k_search_count, point_radius));
		
		if(PRINT_LEVEL >= 2)
			std::cout << test_cloud_names[i] << "\033[1;32m loaded \033[0m"<< int(cloud.size()) << "\033[1;32m points\033[0m" << std::endl;

		boundingBox bb(computeDists(cloud));
		
		Eigen::Affine3d cloud_transform = Eigen::Affine3d::Identity();
		cloud_transform.translation() = -((bb.max + bb.min)/2.0f);
		pcl::transformPointCloud(cloud, cloud, Eigen::Affine3f(cloud_transform));
		bb = boundingBox(computeDists(cloud));

		this->CheckOpenGLInit();

		this->setPointCloud(cloud);
		
		double maxX = std::max(bb.min[0], bb.max[0]);
		double maxY = std::max(bb.min[1], bb.max[1]);
		double maxZ = std::max(bb.min[2], bb.max[2]);
		double dist = 1.1f * (std::sqrt(std::pow(maxX,2) + std::pow(maxY,2) + std::pow(maxZ,2)) * std::max(focal_x, focal_y)) / (double(std::max(image_height,image_width))*0.9f/2.0f);

    m_renderer.reset(new SurfelRenderer(m_nh, size, center, focal, range, with_color, with_depth, enable_lighting,
                                        back_face_culling, render_plugin));
	
		double thetaX = 0.0f;
		do {
			double thetaY = 0.0f;
			do {
				double thetaZ = 0.0f;
				do {
					Eigen::Translation3d translation(0.0f, 0.0f, -dist);
					Eigen::Affine3d pose = Eigen::AngleAxisd(thetaZ*M_PI/180.0f,Eigen::Vector3d(0,0,1)) *
											Eigen::AngleAxisd(thetaY*M_PI/180.0f,Eigen::Vector3d(0,1,0)) *
											Eigen::AngleAxisd(thetaX*M_PI/180.0f,Eigen::Vector3d(1,0,0)) * translation ;

					//save "snapshot" an angles (thetaX, thetaY, thetaZ)
					SurfelRenderer::GLUintVector indexvector;
					SurfelRenderer::GLFloatVector imgvector = m_renderer->Render(*m_cloud_gpu, pose.cast<float>(), joint_state, indexvector);
					saveImage(imgvector, indexvector, tmpname, test_cloud_names[i], thetaX, thetaY, thetaZ, with_color);
					thetaZ += angle_step[2];
				}while(thetaZ <= 360.0f-angle_step[2] && angle_step[2] > 0.0f);
				
				thetaY += angle_step[1];
			}while(thetaY <= 180.0f && angle_step[1] > 0.0f);
			
			thetaX += angle_step[0];
		}while(thetaX <= 180.0f && angle_step[0] > 0.0f);
	}
}

/****************************************************************************************************/

boundingBox ViewGenerator::computeDists(PointSurfelCloud cloud)
{
	boundingBox dists(Eigen::Vector3d(DBL_MAX,DBL_MAX,DBL_MAX),Eigen::Vector3d(-DBL_MAX,-DBL_MAX,-DBL_MAX));

	for(unsigned int i=0; i< cloud.points.size(); i++)
	{
		if(cloud.points[i].x < dists.min[0])
			dists.min[0] = cloud.points[i].x;
		if(cloud.points[i].x > dists.max[0])
			dists.max[0] = cloud.points[i].x;

		if(cloud.points[i].y < dists.min[1])
			dists.min[1] = cloud.points[i].y;
		if(cloud.points[i].y > dists.max[1])
			dists.max[1] = cloud.points[i].y;

		if(cloud.points[i].z < dists.min[2])
			dists.min[2] = cloud.points[i].z;
		if(cloud.points[i].z > dists.max[2])
			dists.max[2] = cloud.points[i].z;
	}

	return dists;
}


