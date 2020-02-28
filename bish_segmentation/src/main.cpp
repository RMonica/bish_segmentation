// local
#include <bish_segmentation/batchProcessor.h>
#include <bish_segmentation/evaluation.h>
#include <bish_segmentation/PointcloudLabeler.h>
#include <bish_segmentation/utility.h>
#include <bish_segmentation/ViewGenerator.h>

// ROS
#include <ros/ros.h>

//C++
#include <climits>
#include <ctime>
#include <iostream>
#include <tuple>


//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SHELLSCRIPT "\
#/bin/bash \n\
killall rosmaster\
"

/****************************************************************************************************/

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bish_segmentation");
	ros::NodeHandle nh("~");
	
	std::string lpath;	//labeled images
	std::string protopath;	//prototypes
	std::string tppath;	//test pointclouds
	std::string tpath;	//test images
	std::string results_path;	//resulting images
	std::string rppath;	//resulting pointclouds
	nh.param<std::string>("labeled_images_path", lpath, "~/data/labeled_images/");
	nh.param<std::string>("prototypes_path", protopath, "~/data/prototypes/");
	nh.param<std::string>("test_pointclouds_path", tppath, "~/data/test_pointclouds/");
	nh.param<std::string>("test_images_path", tpath, "~/data/test_images/");
	nh.param<std::string>("results_path", results_path, "~/data/results/");
	nh.param<std::string>("results_pointclouds_path", rppath, "~/data/results_pointclouds/");
	
	int test_segment_number;
	nh.param<int>("test_segment_number", test_segment_number, 50);
	
	bool generate_test_flag;
	nh.param<bool>("generate_test_flag", generate_test_flag, true);
	
  bool skip_segmentation;		//Used only for testing
  nh.param<bool>("skip_segmentation", skip_segmentation, false);

  double point_radius;
  nh.param<double>("point_radius", point_radius, 0.005);

  int k_search_count;
  nh.param<int>("normal_k_search_count", k_search_count, 10);
	
	//Check comparison dataset dir
	std::vector<std::string> classdirs = getDirectoryNames(lpath.c_str());
	if(classdirs.size() == 0)
	{
		ROS_ERROR("Comparison dataset folder '%s' is empty", lpath.c_str());
		exit(-1);
	}

	//Clean and recreate results dir
	if(fileExists(results_path))
		recursiveDelete(results_path);
	mkdir(results_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
  recursiveDelete(tpath);
	
	time_t start, end;
	
	std::cout << std::endl << std::endl;
	
	//Load/Generate images
	if(generate_test_flag || !fileExists(tpath))
	{
		if(!fileExists(tppath))
		{
			ROS_ERROR("%s does not exist", tppath.c_str());
			exit(-1);
		}
		
		recursiveDelete(tpath);
		if(PRINT_LEVEL >= 1)
		{
			time(&start);
			std::cout << "\033[1;36m==============================\033[0m" << std::endl;
			std::cout << "\033[1;32mGenerating test images . . .\033[0m" << std::endl;
		}
		ViewGenerator* vg = new ViewGenerator(nh);
		
    vg->run(tppath, tpath, k_search_count, point_radius, false);
		
		delete vg;
		
		if(PRINT_LEVEL >= 1)
		{
			std::cout << std::endl << "\033[1;32m. . . generation completed\033[0m" << std::endl;
			
			time(&end);
			double gen_time = difftime(end, start);
			std::cout << "\033[0;36m------------------------------" << std::endl;
			std::cout << "Images generation time: \033[0m" << gen_time << "\033[0;36m s" << std::endl; 
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		}
	}
	
	//check test dir
	std::vector<std::string> testdirs = getDirectoryNames(tpath.c_str());
	if(testdirs.size() == 0)
	{
		ROS_ERROR("Comparison dataset folder '%s' is empty", tpath.c_str());
		exit(-1);
	}
	
  //Segmentation
	if(PRINT_LEVEL >= 0)
	{
		time(&start);
		std::cout << "\033[1;36m==============================\033[0m" << std::endl;
    std::cout << "\033[1;32mSegmentation . . .\033[0m" << std::endl << std::endl;
	}
	PointcloudLabeler* labeler = new PointcloudLabeler(nh);
	batchProcessor* processor = new batchProcessor;
	for(std::string dir_name : testdirs)
	{
		time_t cloud_start, cloud_end, match_start, match_end, transf_1_start, transf_1_end, transf_2_start, transf_2_end;
		
		time(&cloud_start);
		std::string test_path = std::string(tpath).append(dir_name);
		processor->imageSimplify(test_path, test_segment_number);
		
		if(PRINT_LEVEL >= 1)
		{
			time(&match_start);
			std::cout << std::endl;
			std::cout << "\033[1;36m==============================\033[0m" << std::endl;
			std::cout << "\033[1;32mMatching . . .\033[0m" << std::endl;
		}
		std::vector<std::tuple<std::string, std::string, unsigned int> > matching;
		std::string match_class;
		unsigned long int min_dist = ULONG_MAX;
		bool prototype_used = false;	//used to not redo the retrieval if the best was found directly in the dataset
		for(std::string class_name : classdirs)
		{
			std::string tmp_res_path = std::string(results_path).append(class_name);
			if(!fileExists(tmp_res_path))
				mkdir(tmp_res_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
			tmp_res_path.append(dir_name);
			
			bool proto = true;
			std::string tmp_lpath = std::string(protopath).append(class_name);
			if(!fileExists(tmp_lpath) || getDirectoryNames(tmp_lpath.c_str()).size()==0)
			{
				ROS_WARN("Missing prototype for %s, searching in the dataset", class_name.c_str());
				tmp_lpath = std::string(lpath).append(class_name);
				proto = false;
				
				if(getDirectoryNames(tmp_lpath.c_str()).size()==0)
				{
					ROS_WARN("No data neither prototype for class %s", class_name.c_str());
					continue;
				}
			}
			
			processor->batchRetrieval(test_path, tmp_lpath, tmp_res_path);
			
			unsigned long int total_dist;
			std::vector<std::tuple<std::string, std::string, unsigned int> > match = processor->readMatchedFile(tmp_res_path, &total_dist);
			
			if(total_dist >= min_dist)
				continue;
			
			min_dist = total_dist;
			matching = match;
			match_class = std::string(class_name);
			prototype_used = proto;
		}
		
		if(match_class.empty())
		{
			ROS_ERROR("No match found, check dataset and prototype");
			exit(-1);
		}
		
		if(prototype_used)
		{
			std::string tmp_lpath = std::string(lpath).append(match_class);
			std::string tmp_res_path = std::string(results_path).append(match_class);
			if(fileExists(tmp_res_path))
				recursiveDelete(tmp_res_path);
			mkdir(tmp_res_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
			tmp_res_path.append(dir_name);
			
			processor->batchRetrieval(test_path, tmp_lpath, tmp_res_path);
			
			unsigned long int total_dist;
			matching = processor->readMatchedFile(tmp_res_path, &total_dist);
		}
		
		if(PRINT_LEVEL >= 1)
		{
			std::cout << std::endl << "\033[1;32m. . . matching completed\033[0m" << std::endl;
			time(&match_end);
			double match_time = difftime(match_end, match_start);
			std::cout << "\033[0;36m------------------------------" << std::endl;
			std::cout << "Matching time: \033[0m" << match_time << "\033[0;36m s" << std::endl; 
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		}
		
		if(PRINT_LEVEL >= 0)
		{
			std::cout << "\033[1;36m==============================\033[0m" << std::endl;
			std::cout << "\033[1;32m"<< dir_name << " classified as " << match_class <<"\033[0m" << std::endl;
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		}
		
    if(skip_segmentation)		//Used only for test
			continue;
		
		if(PRINT_LEVEL >= 3)
		{
			std::cout << "\033[0;36m------------------------------\033[0m" << std::endl;
			for (int i = 0; i < matching.size(); i++)
				std::cout << std::get<0>(matching[i]) << "\033[0;36m ==> \033[0m" << std::get<1>(matching[i]) << "\033[0;36m : \033[0m" << std::get<2>(matching[i]) << std::endl;
			std::cout << "\033[0;36m------------------------------\033[0m" << std::endl << std::endl;
		}

		std::string labeled_path = std::string(lpath).append(match_class);
		std::string res_path = std::string(results_path).append(match_class);
		res_path.append(dir_name);
		
		if(PRINT_LEVEL >= 1)
		{
			time(&transf_1_start);
			std::cout << "\033[1;36m==============================\033[0m" << std::endl;
			std::cout << "\033[1;32mLabel transfering on images . . .\033[0m" << std::endl;
		}
		for (int i = 0; i < matching.size(); i++)
			processor->matchTransfer(labeled_path, test_path, res_path, std::get<0>(matching[i]), std::get<1>(matching[i]));
		if(PRINT_LEVEL >= 1)
		{
			std::cout << "\033[1;32m. . . transfering completed\033[0m" << std::endl;
			time(&transf_1_end);
			double transf_1_time = difftime(transf_1_end, transf_1_start);
			std::cout << "\033[0;36m------------------------------" << std::endl;
			std::cout << "Transfering on images time: \033[0m" << transf_1_time << "\033[0;36m s" << std::endl; 
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		}
		
		if(PRINT_LEVEL >= 1)
		{
			time(&transf_2_start);
			std::cout << "\033[1;36m==============================\033[0m" << std::endl;
			std::cout << "\033[1;32mLabel transfering on pointcloud . . .\033[0m" << std::endl;
		}
		std::string test_cloud_name = std::string(dir_name.substr(0,dir_name.size()-1)).append(".pcd");
		labeler->run(res_path, tppath, rppath, test_cloud_name, test_path, labeled_path);
		if(PRINT_LEVEL >= 1)
		{
			std::cout << "\033[1;32m. . . transfering completed\033[0m" << std::endl;
			time(&transf_2_end);
			double transf_2_time = difftime(transf_2_end, transf_2_start);
			std::cout << "\033[0;36m------------------------------" << std::endl;
			std::cout << "Transfering on pointcloud time: \033[0m" << transf_2_time << "\033[0;36m s" << std::endl; 
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		
      std::cout << "\033[1;32m. . . segmentation of \033[0m" << dir_name << "\033[1;32m completed\033[0m" << std::endl;
			time(&cloud_end);
			double cloud_time = difftime(cloud_end, cloud_start);
			std::cout << "\033[0;36m------------------------------" << std::endl;
      std::cout << "Segmentation time for \033[0m" << dir_name << "\033[0;36m : \033[0m" << cloud_time << "\033[0;36m s" << std::endl;
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		}
		
		if(PRINT_LEVEL >= 0)
		{
			std::cout << "\033[1;36m==============================\033[0m" << std::endl;
			std::cout << "\033[1;32m"<< test_cloud_name << " --> " << float(100*evaluate(tppath, rppath, test_cloud_name)) <<"\033[0m" << std::endl;
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		}
	}
	
	delete processor;
	delete labeler;
	
	if(PRINT_LEVEL >= 0)
	{
		time(&end);
		double coseg_time = difftime(end, start);
		std::cout << "\033[0;36m------------------------------" << std::endl;
    std::cout << "Total segmentation time: \033[0m" << coseg_time << "\033[0;36m s" << std::endl;
		std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
	}
	
  //system(SHELLSCRIPT);		//TODO TESTING
	
	return 0;
}
