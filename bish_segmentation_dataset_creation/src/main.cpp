// local
#include <bish_segmentation/batchProcessor.h>
#include <bish_segmentation/utility.h>
#include <bish_segmentation/ViewGenerator.h>

// Boost
#include <boost/filesystem.hpp>

// C++
#include <climits>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <tuple>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ROS
#include <ros/ros.h>

/****************************************************************************************************/
bool copyDir(boost::filesystem::path const & source, boost::filesystem::path const & destination)
{
    // Check whether the function call is valid
    if(!boost::filesystem::exists(source) || !boost::filesystem::is_directory(source))
    {
        std::cerr << "Source directory " << source.string() << " does not exist or is not a directory." << '\n';
        return false;
    }
    if(boost::filesystem::exists(destination))
    {
        std::cerr << "Destination directory " << destination.string()<< " already exists." << '\n';
        return false;
    }
    // Create the destination directory
    if(!boost::filesystem::create_directory(destination))
    {
        std::cerr << "Unable to create destination directory" << destination.string() << '\n';
        return false;
    }
    
    // Iterate through the source directory
    for(boost::filesystem::directory_iterator file(source); file != boost::filesystem::directory_iterator(); file++)
    {
        boost::filesystem::path current(file->path());
        if(boost::filesystem::is_directory(current))
        {
            // Found directory: Recursion
            if(!copyDir(current, (destination / current.filename())))
                return false;
        }
        else
            boost::filesystem::copy_file(current, destination / current.filename());
    }
    return true;
}

/****************************************************************************************************/
std::vector<std::string> prototyping(std::string path, batchProcessor* processor,
                                     const uint64 prototypes_count)
{
	if(!fileExists(path))
	{
		ROS_ERROR("%s does not exist", path.c_str());
		exit(-1);
	}
	
  std::vector<std::string> best_prototypes(prototypes_count, "");
	
	std::vector<std::string> dirs = getDirectoryNames(path.c_str());
	if(dirs.size()==0)
		return best_prototypes;
	
	HausdorffImageMatch *matcher = new HausdorffImageMatch;
	
	float dist_matrix[dirs.size()][dirs.size()];
	for(unsigned int d1=0; d1<dirs.size(); d1++)
		for(unsigned int d2=0; d2<dirs.size(); d2++)
			dist_matrix[d1][d2] = NAN;
	
	for(unsigned int d1=0; d1<dirs.size(); d1++)
	{
		std::string first_path = std::string(path).append(dirs[d1]);
		
		std::vector<std::vector<HausdorffNode> > firstImgRegions = processor->batchLoadNodes(first_path);
		std::vector<std::vector<HausdorffNode> > firstImgRegions_rot;
		if(BOTH_DIRECTIONS)
			firstImgRegions_rot = processor->batchLoadNodes(first_path, std::vector<int>(), "_rot");		//ADD
		
		for(unsigned int d2=0; d2<dirs.size(); d2++)
		{
			float dist = 0;
			std::string second_path = std::string(path).append(dirs[d2]);
			if(second_path.compare(first_path) == 0)
			{
				dist_matrix[d1][d2] = 0;
				dist_matrix[d2][d1] = 0;	//dovrebbe essere uguale ma per sicurezza...
				continue;
			}
			
			std::vector<std::vector<HausdorffNode> > secondImgRegions = processor->batchLoadNodes(second_path);
			std::vector<std::vector<HausdorffNode> > secondImgRegions_rot;
			if(BOTH_DIRECTIONS)
				secondImgRegions_rot = processor->batchLoadNodes(second_path, std::vector<int>(), "_rot");		//ADD
			
			
			std::vector<std::pair<int, float> > matchparis;
			for(unsigned int i=0; i<firstImgRegions.size(); i++)
			{
				std::vector<std::pair<int, float> > imatchparis;
				for(unsigned int j=0; j<secondImgRegions.size(); j++)
				{
					float cost = 0;
					float cost_rot = 0;		//ADD
					
					std::vector<std::pair<int, int> > pairs = matcher->run(firstImgRegions[i], secondImgRegions[j], cost);
					if(BOTH_DIRECTIONS)
						std::vector<std::pair<int, int> > pairs_rot = matcher->run(firstImgRegions_rot[i], secondImgRegions_rot[j], cost_rot);		//ADD

					imatchparis.push_back(std::pair<int, float>(j, cost + cost_rot));		//MOD aggiunto +cost_rot
				}

				sortPairwithIndex(imatchparis);

				matchparis.push_back(imatchparis[0]);
				dist += imatchparis[0].second;
			}
			
			if (std::isnan(dist_matrix[d1][d2]) || std::isnan(dist_matrix[d2][d1]))
			{
				dist_matrix[d1][d2] = dist;
				dist_matrix[d2][d1] = dist;
			}
			else
			{
				dist_matrix[d1][d2] = std::min(dist, dist_matrix[d1][d2]);
				dist_matrix[d2][d1] = std::min(dist, dist_matrix[d2][d1]);
			}
		}
	}
	
	std::cout << std::endl << "\033[1;32mCreated matrix of distances\033[0m" << std::endl;
	
	srand(time(NULL));
  std::vector<std::vector<unsigned int> > clusters(prototypes_count);
  std::vector<int> c(prototypes_count);
  for (uint64 i = 0; i < clusters.size(); i++)
  {
    do
      c[i] = rand() % dirs.size();
    while(std::find(c.begin(), c.begin() + i, c[i]) != (c.begin() + i));
  }

  std::vector<int> old_c(prototypes_count, -1);
	
	bool iterate = true;
	int count = 0;
	while(iterate)
	{
    for (uint64 i = 0; i < prototypes_count; i++)
    {
      clusters[i].clear();
      clusters[i].push_back(c[i]);
    }

    for(unsigned int i = 0; i < dirs.size(); i++)
		{
      if(std::find(c.begin(), c.end(), i) != c.end())
				continue;
			
      const int ci = std::distance(c.begin(),
        std::min_element(c.begin(), c.end(), [&dist_matrix, i](const int a, const int b) -> bool {
        return dist_matrix[a][i] < dist_matrix[b][i];
      }));
      clusters[ci].push_back(i);
		}
		
    old_c = c;
		
    int min_average;
    for (int ci = 0; ci < clusters.size(); ci++)
    {
      min_average = INT_MAX;
      for(unsigned int nc : clusters[ci])
      {
        int average = 0;
        for(unsigned int e : clusters[ci])
          average += dist_matrix[nc][e];
        average /= clusters[ci].size();

        if(average < min_average)
        {
          min_average = average;
          c[ci] = nc;
        }
      }
    }
		
    if(c == old_c)
			count++;
		else
			count = 0;
		if(count >= 5)
			iterate = false;
	}

  for (int i = 0; i < best_prototypes.size(); i++)
    best_prototypes.at(i) = dirs.at(c[i]);
	
	return best_prototypes;
}



/****************************************************************************************************/

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "bish_segmentation_dataset_creation");
	ros::NodeHandle nh("~");
	
	std::string lppath;	//labeled pointclouds
	std::string lpath;	//labeled images
	std::string protopath;	//prototypes
	nh.param<std::string>("labeled_pointclouds_path", lppath, "~/data/labeled_pointclouds/");
	nh.param<std::string>("labeled_images_path", lpath, "~/data/labeled_images/");
	nh.param<std::string>("prototypes_path", protopath, "~/data/prototypes/");
	
	int labeled_segment_number;
	int test_segment_number;
	nh.param<int>("labeled_segment_number", labeled_segment_number, 25);

  double point_radius;
  nh.param<double>("point_radius", point_radius, 0.005);

  int k_search_count;
  nh.param<int>("normal_k_search_count", k_search_count, 10);

  int prototypes_count;
  nh.param<int>("prototypes_count", prototypes_count, 2);
	
	if(!fileExists(lppath))
	{
		ROS_ERROR("%s does not exist", lppath.c_str());
		exit(-1);
	}
	
	if(fileExists(lpath))
		recursiveDelete(lpath);
	mkdir(lpath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	
	if(fileExists(protopath))
		recursiveDelete(protopath);
	mkdir(protopath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	
	time_t start, end;
	
	//Generate images
	ViewGenerator* vg = new ViewGenerator(nh);
	std::vector<std::string> classdirs = getDirectoryNames(lppath.c_str());
	if(classdirs.size() == 0)
	{
		ROS_ERROR("Dataset pointclouds folder '%s' is empty", lppath.c_str());
		exit(-1);
	}
	
	time(&start);
	std::cout << std::endl << "\033[1;32mGenerating labeled images . . .\033[0m" << std::endl;
	for(std::string class_name : classdirs)
	{
		std::string tmp_path = std::string(lpath).append(class_name);
		std::string tmp_lppath = std::string(lppath).append(class_name);
		if(fileExists(tmp_path))
			recursiveDelete(tmp_path);
    vg->run(tmp_lppath, tmp_path, k_search_count, point_radius, true);
	}
	std::cout << std::endl << "\033[1;32m. . . generation completed\033[0m" << std::endl;
	
	delete vg;
	
	//Segmentation
	batchProcessor* processor = new batchProcessor;
	
	std::cout << std::endl << "\033[1;32mSegmentation and prototyping . . .\033[0m" << std::endl;
	for(std::string class_name : classdirs)
	{
		std::cout << std::endl << class_name << std::endl;
		std::string tmp_path = std::string(lpath).append(class_name);
		std::vector<std::string> ldirs = getDirectoryNames(tmp_path.c_str());
		if(ldirs.size()==0)
			continue;
		
		for(std::string dir_name : ldirs)
			processor->imageSimplify(std::string(tmp_path).append(dir_name), labeled_segment_number);
		
		std::vector<std::string> best_prototypes;
		time_t start_proto, end_proto;
		time(&start_proto);
    best_prototypes = prototyping(tmp_path, processor, prototypes_count);
    if(std::find(best_prototypes.begin(), best_prototypes.end(), "") != best_prototypes.end())
		{
			ROS_WARN("Prototypes not found for class %s", class_name.c_str());
			continue;
		}
		
    std::cout << "\033[1;32m\nSelected as prototypes:\033[0m\n";
    for(std::string proto : best_prototypes)
    {
      std::cout << proto << "\n";
    }
    std::cout << std::endl;
		
		time(&end_proto);
		double proto_time = difftime(end_proto, start_proto);
			std::cout << "\033[0;36m------------------------------" << std::endl;
			std::cout << "Prototype selection time: \033[0m" << proto_time << "\033[0;36m s" << std::endl; 
			std::cout << "\033[1;36m==============================\033[0m" << std::endl << std::endl;
		
		
		std::string dst_folder = std::string(protopath).append(class_name);
		if(fileExists(dst_folder))
			recursiveDelete(dst_folder);
		mkdir(dst_folder.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
		
		for(std::string proto : best_prototypes)
		{
			std::string src = std::string(tmp_path).append(proto);
			std::string dst = std::string(dst_folder).append(proto);
		
			if(fileExists(dst))
				recursiveDelete(dst);
		
			boost::filesystem::path source(src);
			boost::filesystem::path destination(dst);
			std::cout << std::endl << "\033[1;32mCopying " << source << " to " << destination << " . . .\033[0m" << std::endl;
			copyDir(source, destination);
			std::cout << std::endl << "\033[1;32m. . . copied\033[0m" << std::endl;
		}
	}
	std::cout << std::endl << "\033[1;32m. . . segmentation and prototyping completed\033[0m" << std::endl;
	
	delete processor;
	
	time(&end);
	double gen_time = difftime(end, start);
	std::cout << "\033[0;36m------------------------------" << std::endl;
	std::cout << "Total generation time: \033[0m" << gen_time << "\033[0;36m s" << std::endl; 
	std::cout << "==============================\033[0m" << std::endl << std::endl;
	
	return 0;
}
