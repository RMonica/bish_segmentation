#ifndef HAUSDORFF_IMAGE_PROCESSOR_H
#define HAUSDORFF_IMAGE_PROCESSOR_H

// local
#include "debug.h"
#include "HausdorffDist.h"
#include "HausdorffNode.h"
#include "utility.h"

// C++
#include <vector>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

class HausdorffImageProcessor
{
//VARIABLES
public:
	cv::Mat m_originalImage;
	cv::Mat m_grayImage;
	cv::Mat m_grayImage_rot;		//ADD
	std::string m_filename;
	HausdorffDist m_hausdorffDist;
private:
	/*nothing*/

//METHODS
public:
	HausdorffImageProcessor() {/*do nothing*/};
	~HausdorffImageProcessor() {/*do nothing*/};
	
	bool loadImage(std::string filename);
	
	cv::Mat getInputImage() {return m_originalImage;}
private:
	cv::Mat convertToGray(cv::Mat, bool isShapeBlack = true);
};

#endif
