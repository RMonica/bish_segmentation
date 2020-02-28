#include <bish_segmentation/utility.h>

/****************************************************************************************************/

pcl::PointCloud<pcl::PointXYZRGBA> extractPointsXYZRGBA(std::string path, std::string filename)
{
	// Open file for reading
	std::string tmpname = std::string(path);
	tmpname.append(filename);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_cloud_rgba (new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(tmpname, *tmp_cloud_rgba) == -1) //* load the file
	{
		PCL_ERROR ("Could not read file %s \n", filename);
		exit(-1);
	}

	return *tmp_cloud_rgba;
}

/****************************************************************************************************/

bool fileExists(const std::string& fileName)
{
	std::fstream file;
	file.open(fileName.c_str(), std::ios::in);
	if (file.is_open())
	{
		file.close();
		return true;
	}
	
	file.close();
	return false;
}

/****************************************************************************************************/

std::vector<std::string> getDirectoryNames(const char *path)
{
	std::vector<std::string> filenames(0);
	if(!fileExists(std::string(path)))
	{
		ROS_WARN("%s does not exist", path);
		return filenames;
	}
	
	for(boost::filesystem::directory_iterator it(path); it!=boost::filesystem::directory_iterator(); it++)
		if(is_directory(it->path()))
			filenames.push_back(std::string(it->path().filename().string()).append("/"));
	
	return filenames;
}

/****************************************************************************************************/

std::vector<std::string> getFileNames(const char *path)
{
	std::vector<std::string> filenames(0);
	if(!fileExists(std::string(path)))
	{
		ROS_WARN("%s does not exist", path);
		return filenames;
	}
	
	for(boost::filesystem::directory_iterator it(path); it!=boost::filesystem::directory_iterator(); it++)
	{
		std::string extension = boost::algorithm::to_lower_copy(it->path().extension().string());
    if(!is_directory(it->path()) && (extension == ".png" || extension == ".bmp"))
			filenames.push_back(it->path().filename().string());
	}
	
	return filenames;
}

/****************************************************************************************************/

std::vector<std::string> getPointcloudNames(const char *path)
{
	std::vector<std::string> filenames(0);
	if(!fileExists(std::string(path)))
	{
		ROS_WARN("%s does not exist", path);
		return filenames;
	}
	
	for(boost::filesystem::directory_iterator it(path); it!=boost::filesystem::directory_iterator(); it++)
	{
		std::string extension = boost::algorithm::to_lower_copy(it->path().extension().string());
		if(!is_directory(it->path()) && extension.compare(".pcd")==0)
			filenames.push_back(it->path().filename().string());
	}
	
	return filenames;
}

/****************************************************************************************************/

std::vector<int> sortPairwithIndex(std::vector<std::pair<int,float> > &data)
{
	std::vector<int> index;
	int n = data.size();
	index.resize(n);
	for(int i=0;i<n;i++)
		index[i] = i;
	bool swapped = false;
	do{
		swapped = false;
		for(int i=0; i<n-1; i++)
		{
			if((data[i].second)>(data[i+1].second)){
				std::pair<int,float> tmp = data[i];
				data[i] = data[i+1];
				data[i+1] =  tmp;
				int tmpi = index[i];
				index[i] =  index[i+1];
				index[i+1] = tmpi;
				swapped = true;
			}
		}
		n--;
	}while(swapped);

	return index;
}

/****************************************************************************************************/

void recursiveDelete(std::string path)
{
	if(!fileExists(path))
		return;
		
	std::vector<std::string> filenames;
	for(boost::filesystem::directory_iterator it(path.c_str()); it!=boost::filesystem::directory_iterator(); it++)
		if(is_directory(it->path()))
		{
			recursiveDelete(it->path().string());
			boost::filesystem::remove(it->path().string());
		}
		else
			boost::filesystem::remove(it->path().string());
	boost::filesystem::remove(path);
	
	return;
}


