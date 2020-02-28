#include <bish_segmentation/PointcloudLabeler.h>

//#include <bish_segmentation/graph.h>	//TODO sposta in .h
#include <pcl/console/time.h>		//DEBUG

/*
void myBoykov(pcl::PointCloud<pcl::PointXYZRGBA> cloud, std::string rppath, std::string test_cloud_name, std::vector<std::vector<ColorProbability> > labelProb)
{
	
	std::vector<ColorProbability> labels;
	for(unsigned int i=0; i<cloud.size(); i++)
	{
		ColorProbability color(cloud[i].r, cloud[i].g, cloud[i].b, cloud[i].a, 1);
		std::vector<ColorProbability>::iterator it = std::find_if(labels.begin(), labels.end(), [color](ColorProbability cp) {return cp == color;});
		if(it == labels.end())
			labels.push_back(color);
	}
	
  double w = 0.1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	bool changed = true;
	std::vector<int> flows;
	std::vector<int> old_flows;
	for(int n=0; n<100 && changed; n++)
	{
		flows.clear();
		changed = 0;
		for(unsigned int l1=0; l1<labels.size()-1; l1++)
		{
			for(unsigned int l2=l1+1; l2<labels.size(); l2++)
			{
				std::vector<unsigned int> index_map;
				for(unsigned int i=0; i<cloud.size(); i++)
				{
					ColorProbability color(cloud[i].r, cloud[i].g, cloud[i].b, cloud[i].a, 1);
					if(color==labels[l1] || color==labels[l2])
						index_map.push_back(i);
				}
				
				int K = std::min(5, int(index_map.size()));
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				pcl::copyPointCloud(cloud, *cloudXYZ);
				kdtree.setInputCloud(cloudXYZ);
				
				typedef Graph<double, double, double> GraphType;
				GraphType *g = new GraphType(int(index_map.size()), int(index_map.size())*K);
				
				for(unsigned int i=0; i<index_map.size(); i++)
					g->add_node();
				
				std::vector<std::pair<int, int> > edges;
				for(unsigned int i=0; i<index_map.size(); i++)
				{
					double ta;
					double tb;
					if (labelProb.at(index_map[i]).size()==0)
					{
						ta = -std::log10(1.0/double(labels.size()));		//TODO giusto considerare di sparare a caso?
						tb = -std::log10(1.0/double(labels.size()));		//TODO giusto considerare di sparare a caso?
					}
					else
					{
						ColorProbability lbl = labels.at(l1);
						std::vector<ColorProbability>::iterator it = std::find_if(labelProb.at(index_map[i]).begin(), labelProb.at(index_map[i]).end(),
																					[lbl](ColorProbability cp) {return cp == lbl;});
						if(it != labelProb.at(index_map[i]).end())
							ta = -std::log10(double(it->probability));
						else
							ta = -std::log10(0);
						
						lbl = labels.at(l2);
						it = std::find_if(labelProb.at(index_map[i]).begin(), labelProb.at(index_map[i]).end(),
											[lbl](ColorProbability cp) {return cp == lbl;});
						
						if(it != labelProb.at(index_map[i]).end())
							tb = -std::log10(double(it->probability));
						else
							tb = -std::log10(0);
					}
					
					if(kdtree.nearestKSearch(cloudXYZ->points[index_map[i]], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						for(int j=0; j<K; j++)
						{
							std::vector<unsigned int>::iterator index_it = std::find(index_map.begin(), index_map.end(), pointIdxNKNSearch[j]);
							if(index_it != index_map.end())
							{
								int index = std::distance(index_map.begin(), index_it);
								
								std::pair<int, int> e1(int(i), index);
								std::vector<std::pair<int, int> >::iterator edges_it = std::find_if(edges.begin(), edges.end(), [e1](std::pair<int, int> e2) {return e1 == e2;});
								if(edges_it==edges.end() && int(i)!=int(index))
								{
									double d = w * -std::log10(double(pointNKNSquaredDistance[j]));
									g->add_edge(int(i), index, d, d);
									edges.push_back(std::pair<int, int> (int(i), index));
									edges.push_back(std::pair<int, int> (index, int(i)));
								}
							}
							else
							{
								double d = w * -std::log10(double(pointNKNSquaredDistance[j]));
								ta += d;
								tb += d;
							}
						}
					}
					
					g->add_tweights(int(i), ta, tb);
				}
				
				flows.push_back(g->maxflow());
				for(unsigned int i=0; i<index_map.size(); i++)
				{
					ColorProbability actual_color(cloud[index_map[i]].r, cloud[index_map[i]].g, cloud[index_map[i]].b, cloud[index_map[i]].a, 1);
					if(g->what_segment(i)==GraphType::SOURCE)
					{
						cloud[index_map[i]].r = labels[l2].r;
						cloud[index_map[i]].g = labels[l2].g;
						cloud[index_map[i]].b = labels[l2].b;
						cloud[index_map[i]].a = labels[l2].a;
					}
					else
					{
						cloud[index_map[i]].r = labels[l1].r;
						cloud[index_map[i]].g = labels[l1].g;
						cloud[index_map[i]].b = labels[l1].b;
						cloud[index_map[i]].a = labels[l1].a;
					}
				}

				delete g;
			}
		}
		
		if(flows==old_flows)		//TODO cambia
		{
			changed = false;
		}
		else
		{
			old_flows.clear();
			for(unsigned int f=0; f<flows.size(); f++)
				old_flows.push_back(flows[f]);
		}
	}
	
	std::string tmpname(rppath);
	tmpname.append(test_cloud_name.substr(0,test_cloud_name.size()-4));
	tmpname.append("-BOYKOV.pcd");
	pcl::io::savePCDFileASCII (tmpname, cloud);
}
*/

/****************************************************************************************************/

void myEneryMinimization(pcl::PointCloud<pcl::PointXYZRGBA> cloud, std::string rppath, std::string test_cloud_name, std::vector<std::vector<ColorProbability> > labelProb)
{

	pcl::PointCloud<pcl::PointXYZRGBA> old_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	int K = std::min(50, int(cloud.size()));
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	std::vector<ColorProbability> labels;
	for(unsigned int i=0; i<cloud.size(); i++)
	{
		ColorProbability color(cloud[i].r, cloud[i].g, cloud[i].b, cloud[i].a, 1);
		std::vector<ColorProbability>::iterator labels_it = std::find_if(labels.begin(), labels.end(), [color](ColorProbability cp) {return cp == color;});
		if(labels_it == labels.end())
			labels.push_back(color);
	}
	
	double w = 0.1;		//TODO 0.1 è una costante a caso
	pcl::copyPointCloud(cloud, *cloudXYZ);
	kdtree.setInputCloud (cloudXYZ);
	std::vector<bool> change_flag(cloud.size(), false);
	std::vector<bool> old_change_flag(cloud.size(), false);
	bool changed = true;
	for(unsigned int n=0; n< 100 && changed; n++)
	{
		changed = 0;
		
		pcl::copyPointCloud(cloud, old_cloud);
		
		
		for(unsigned int i=0; i<cloud.size(); i++)
		{
			if(kdtree.nearestKSearch(cloudXYZ->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				double min_energy = FLT_MAX;
				ColorProbability actual_color(cloud[i].r, cloud[i].g, cloud[i].b, cloud[i].a, 1);
				ColorProbability min_label = actual_color;
				for(unsigned int l=0; l<labels.size(); l++)
				{
					ColorProbability lbl = labels[l];
					double energy;
					if (labelProb.at(i).size()==0)
						energy = -std::log10(1.0/double(labels.size())+0.1) + std::log10(1.1);		//TODO giusto considerare di sparare a caso?
					else
					{
						std::vector<ColorProbability>::iterator labelProb_it = std::find_if(labelProb.at(i).begin(), labelProb.at(i).end(),
																					[lbl](ColorProbability cp) {return cp == lbl;});
																					
						if(labelProb_it != labelProb.at(i).end())
							energy = -std::log10(double(labelProb_it->probability+0.1)) + std::log10(1.1);
						else
							energy = -std::log10(0.1) + std::log10(1.1);
					}
					
					for(int j=0; j<K; j++)
					{
						ColorProbability color(old_cloud[pointIdxNKNSearch[j]].r, old_cloud[pointIdxNKNSearch[j]].g, old_cloud[pointIdxNKNSearch[j]].b, old_cloud[pointIdxNKNSearch[j]].a, 1);
						//TODO MANTENERE?
						if (labelProb.at(pointIdxNKNSearch[j]).size()==0)
							energy += -std::log10(1.0/double(labels.size())+0.1) + std::log10(1.1);		//TODO giusto considerare di sparare a caso?
						else
						{
							std::vector<ColorProbability>::iterator labelProb_it2 = std::find_if(labelProb.at(pointIdxNKNSearch[j]).begin(), labelProb.at(pointIdxNKNSearch[j]).end(),
																						[color](ColorProbability cp) {return cp == color;});
																						
							if(labelProb_it2 != labelProb.at(pointIdxNKNSearch[j]).end())
								energy += -std::log10(double(labelProb_it2->probability+0.1)) + std::log10(1.1);
							else
								energy += -std::log10(0.1) + std::log10(1.1);
						}
						//END TODO
						
						if(color != lbl)
							energy += w * (-std::log10(double(pointNKNSquaredDistance[j]+0.1)) + std::log10(1.1));
						else
							energy -= w * (-std::log10(double(pointNKNSquaredDistance[j]+0.1)) + std::log10(1.1));
					}
					
					if(energy < min_energy)
					{
						min_energy = energy;
						min_label = lbl;
					}
				}
				
				change_flag[i] = false;
				if(min_label != actual_color)
				{
					change_flag[i] = true;
					
					cloud[i].r = min_label.r;
					cloud[i].g = min_label.g;
					cloud[i].b = min_label.b;
					cloud[i].a = min_label.a;
				}
			}
		}
		
		if(change_flag == old_change_flag)
			changed = false;
		else
			for(unsigned int c=0; c<change_flag.size(); c++)
				old_change_flag[c] = change_flag[c];
	}
	
	std::string tmpname(rppath);
	tmpname.append(test_cloud_name.substr(0,test_cloud_name.size()-4));
	tmpname.append("-2_LABELED.pcd");
	pcl::io::savePCDFileASCII (tmpname, cloud);
}



/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/

cv::Mat PointcloudLabeler::getPointProbability(cv::Size size, std::vector<HausdorffNode> srcNodes, std::vector<HausdorffNode> dstNodes)
{
	HausdorffImageMatch* m_hausdorffImageMatch;
	HausdorffDist* m_hausdorffDist;
	cv::Mat probability = cv::Mat::zeros(size, CV_32F);
	cv::Mat partial_prob = cv::Mat::zeros(probability.rows, 1, CV_32F);
	
	int num1 = srcNodes.size();
	int num2 = dstNodes.size();
	std::vector<std::vector<float> > distmatrix(num1);
	std::vector<std::vector<DataForProbability> > data_for_prob(num1);
	
	for(int jj=0; jj<num1; jj++)
	{
		distmatrix[jj].resize(num2, 0);
		data_for_prob[jj].resize(num2);;
	}
	
	float height = 0;
	for(int jj=0; jj<num1; jj++)
		for(int kk=0; kk<num2; kk++)
		{
			height += srcNodes[jj].height;
			float dist = m_hausdorffDist->computeDist(srcNodes[jj].line, dstNodes[kk].line);
			distmatrix[jj][kk] = dist;		//cs[]
			data_for_prob[jj][kk].row_limits = std::pair<int, int>(srcNodes[jj].start, srcNodes[jj].end);
			data_for_prob[jj][kk].dstNodeRow_start = dstNodes[kk].line[0];
			data_for_prob[jj][kk].line = srcNodes[jj].line;
			data_for_prob[jj][kk].const_shift = float(dstNodes[kk].line.size()) / float(srcNodes[jj].line.size());
		}
		
	float cost = 0;
	std::vector<std::pair<int,int> > pairs = m_hausdorffImageMatch->findMincost(distmatrix, num1-1, num2-1, cost);
	
	cost /= height;		//ci
	for(int i=0; i<pairs.size(); i++)
	{
		int m = pairs[i].first;
		int n = pairs[i].second;
		for(int r=data_for_prob[m][n].row_limits.first-1; r<data_for_prob[m][n].row_limits.second; r++)
			for(std::vector<int>::iterator dfp_it=data_for_prob[m][n].line.begin(); dfp_it<data_for_prob[m][n].line.end(); dfp_it++)
				probability.at<float>(r, *dfp_it-1) = cost + distmatrix[m][n] + std::abs(*dfp_it - std::round(data_for_prob[m][n].dstNodeRow_start +
																							data_for_prob[m][n].const_shift*(*dfp_it-data_for_prob[m][n].line[0])));
	}
	
	return probability;
}

/****************************************************************************************************/

bool PointcloudLabeler::run(std::string results_path, std::string tppath, std::string rppath, std::string test_cloud_name, std::string tpath, std::string lpath)
{
	if(!fileExists(rppath))
		mkdir(rppath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	//read match distances file
	std::string match_file_path(results_path);
	match_file_path.append("matched.txt");
	std::ifstream match_file(match_file_path.c_str());
	
	unsigned long int total_dist = 0;
	std::vector<std::tuple<std::string, std::string, unsigned int> > match;
	if(match_file.is_open())
	{
		std::string test_name, labeled_name, d;
		unsigned int dist;
		while(match_file >> test_name >> labeled_name >> d)
		{
			dist = atoi(d.c_str());
			match.push_back(std::make_tuple(test_name, labeled_name, dist));
			total_dist += dist;
		}
	}
	else
	{
		ROS_ERROR("File '%s' not found", match_file_path.c_str());
		exit(-1);
	}
	
	match_file.close();
	
	pcl::PointCloud<pcl::PointXYZRGBA> cloud(extractPointsXYZRGBA(tppath, test_cloud_name));
	if(PRINT_LEVEL >= 2)
			std::cout << test_cloud_name << "\033[1;32m loaded \033[0m"<< int(cloud.size()) << "\033[1;32m points\033[0m" << std::endl;
	
	HausdorffImageSimplify *simplifier = new HausdorffImageSimplify;
	
	std::vector<std::vector<ColorProbability> > labelProb(int(cloud.size()));
	float sum_of_C = 0.0f;
	for(unsigned int j=0; j<match.size(); j++)
	{
		std::string testpath(std::get<0>(match[j]));
		std::string labeledpath(std::get<1>(match[j]));
		
		std::string test_filename = std::string(tpath);
		test_filename.append(testpath);
		std::string new_label_filename(results_path);
		new_label_filename.append(testpath);
		
		//load test image and labeled image
		cv::Mat test_image = cv::imread(test_filename.c_str(), CV_LOAD_IMAGE_COLOR);
		cv::cvtColor(test_image, test_image, cv::COLOR_BGR2RGB);
		cv::Mat labeled_image = cv::imread(new_label_filename.c_str(), CV_LOAD_IMAGE_COLOR);
		cv::cvtColor(labeled_image, labeled_image, cv::COLOR_BGR2RGB);
		
		//TODO TODO
		std::vector<HausdorffNode> testImgRegions;
		std::vector<HausdorffNode> testImgRegions_rot;
		std::string tmp_path = std::string(tpath).append("nodes/");
		tmp_path.append(testpath.substr(0, testpath.size() - 4));
		std::string tmp = std::string(tmp_path).append(".txt");
		testImgRegions = simplifier->loadNodeInfo(tmp);
		if(BOTH_DIRECTIONS)
		{
			std::string tmp_rot = std::string(tmp_path).append("_rot.txt");
			testImgRegions_rot = simplifier->loadNodeInfo(tmp_rot);
		}
		
		std::vector<HausdorffNode> labeledImgRegions;
		std::vector<HausdorffNode> labeledImgRegions_rot;
		
		std::vector<std::string> dirnames = getDirectoryNames(lpath.c_str());
		if(dirnames.size() == 0)
		{
			ROS_ERROR("No directories in %s", lpath.c_str());
			exit(-1);
		}
		
		std::string new_lpath = lpath;
		
		for(std::string dir_name : dirnames)
		{
			if(dir_name.substr(0, dir_name.size()-1).compare(labeledpath.substr(0,dir_name.size()-1)) != 0)
				continue;
				
			new_lpath = std::string(lpath).append(dir_name);
			std::string tmp_path = std::string(new_lpath).append("nodes/");
			tmp_path.append(labeledpath.substr(0, labeledpath.size() - 4));
			
			std::string tmp = std::string(tmp_path).append(".txt");
			if(!boost::filesystem::exists(tmp))
				continue;
			labeledImgRegions = simplifier->loadNodeInfo(tmp);
				
			if(BOTH_DIRECTIONS)
			{
				std::string tmp_rot = std::string(tmp_path).append("_rot.txt");
				if(!boost::filesystem::exists(tmp_rot))
					continue;
				labeledImgRegions_rot = simplifier->loadNodeInfo(tmp_rot);
			}
			break;
		}
		
		cv::Mat probability = getPointProbability(test_image.size(), testImgRegions, labeledImgRegions).clone();
		if(BOTH_DIRECTIONS)
			probability += getPointProbability(test_image.size(), testImgRegions_rot, labeledImgRegions_rot).clone();
		float sigma = 150.0f;
		
		//TODO TODO
		
		
		//Calculate vector of labels probability for every point in the pointcloud
		for (int r=0; r<test_image.rows; r++)
			for (int c=0; c<test_image.cols; c++)
			{
				cv::Vec3b rgb = test_image.at<cv::Vec3b>(r, c);
				if(rgb[0]<255 || rgb[1]<255 || rgb[2]<255)
				{
					unsigned int index = rgb[0] + rgb[1]*256 + rgb[2]*256*256;
					rgb = labeled_image.at<cv::Vec3b>(r, c);
					float prob = std::exp(-std::pow(probability.at<float>(r,c), 2.0f)/std::pow(sigma, 2.0f));
					ColorProbability color(rgb[0], rgb[1], rgb[2], 1, prob);
					
					std::vector<ColorProbability>::iterator labelProb_it = std::find_if(labelProb.at(index).begin(), labelProb.at(index).end(),
																					[color](ColorProbability cp) {return cp == color;});
					if(labelProb_it != labelProb.at(index).end())
						labelProb_it->probability +=  color.probability;
					else
						labelProb.at(index).push_back(color);
					
					sum_of_C +=  color.probability;
				}
			}
			
	}
	
	for(unsigned int l=0; l<labelProb.size(); l++)
    for(std::vector<ColorProbability>::iterator labelProb_it=labelProb.at(l).begin();
        labelProb_it < labelProb.at(l).end(); labelProb_it++)
			labelProb_it->probability /= sum_of_C;
	
	//assign label to every point in the cloud
	for(unsigned int l=0; l < labelProb.size(); l++)
    sort(labelProb.at(l).begin(), labelProb.at(l).end(),
         [](const ColorProbability & a, const ColorProbability & b){ return a.probability > b.probability; });
	
  pcl::PointIndices::Ptr unlabeled_indices(new pcl::PointIndices());
	for(unsigned int l=0; l < labelProb.size(); l++)
	{
		if(labelProb.at(l).size() > 0)
		{
			cloud.at(l).r = labelProb.at(l).at(0).r;
			cloud.at(l).g = labelProb.at(l).at(0).g;
			cloud.at(l).b = labelProb.at(l).at(0).b;
			cloud.at(l).a = labelProb.at(l).at(0).a;
		}
		else
      unlabeled_indices->indices.push_back(l);
	}
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	*cloudPtr = cloud;
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudCleared(new pcl::PointCloud<pcl::PointXYZRGBA>);
	extract.setInputCloud(cloudPtr);
  extract.setIndices(unlabeled_indices);
	extract.setNegative(true);
	extract.filter(*cloudCleared);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud, *cloudXYZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudClearedXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloudCleared, *cloudClearedXYZ);
  if (cloudClearedXYZ->empty())
    return false;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloudClearedXYZ);
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
  for(unsigned int l=0; l < unlabeled_indices->indices.size(); l++)
	{
    int idx = unlabeled_indices->indices[l];
    if(kdtree.nearestKSearch(cloudXYZ->points[idx], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
      cloud.at(idx).r = cloudCleared->at(pointIdxNKNSearch[0]).r;
      cloud.at(idx).g = cloudCleared->at(pointIdxNKNSearch[0]).g;
      cloud.at(idx).b = cloudCleared->at(pointIdxNKNSearch[0]).b;
      cloud.at(idx).a = cloudCleared->at(pointIdxNKNSearch[0]).a;
		}
		else
		{
      cloud.at(idx).r = 255;
      cloud.at(idx).g = 255;
      cloud.at(idx).b = 255;
      cloud.at(idx).a = 1;
		}
	}
	
	std::string tmpname(rppath);
	tmpname.append(test_cloud_name.substr(0, test_cloud_name.size()-4));
	tmpname.append("-LABELED.pcd");
  pcl::io::savePCDFileBinary(tmpname, cloud);
	
	//TODO graph cut
	//myBoykov(cloud, rppath, test_cloud_name, labelProb);
	myEneryMinimization(cloud, rppath, test_cloud_name, labelProb);

  return true;
}


