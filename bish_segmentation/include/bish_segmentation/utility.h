#ifndef UTILITY_H
#define UTILITY_H

// local
#include "debug.h"

// Boost
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

// C++
#include <algorithm>
#include <cassert>
#include <dirent.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <sys/types.h>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include <ros/ros.h>


#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

pcl::PointCloud<pcl::PointXYZRGBA> extractPointsXYZRGBA(std::string path, std::string filename);

bool fileExists(const std::string& fileName);

std::vector<std::string> getDirectoryNames(const char *path);

std::vector<std::string> getFileNames(const char *path);

std::vector<std::string> getPointcloudNames(const char *path);

void recursiveDelete(std::string path);

std::vector<int> sortPairwithIndex(std::vector<std::pair<int,float> > &data);

template<typename T>
std::vector<int> sortWithIndex(std::vector<T> data);

/****************************************************************************************************/

template<typename T>
std::vector<int> sortWithIndex(std::vector<T> data)
{
	std::vector<int> index;
	int n = data.size();
	index.resize(n);
	
	for(int i=0; i<n; i++)
		index[i] = i;
		
	bool swapped = false;
	do{
		swapped = false;
		for(int i=0; i<n-1; i++)
		{
			if(data[i] > data[i+1]){
				T tmp = data[i];
				data[i] = data[i+1];
				data[i+1] = tmp;
				int tmpi = index[i];
				index[i] = index[i+1];
				index[i+1] = tmpi;
				swapped = true;
			}
		}
		n--;
	}while(swapped);

	return index;
}

#endif

