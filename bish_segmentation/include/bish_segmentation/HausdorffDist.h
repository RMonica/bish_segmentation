#ifndef HAUSDORFF_DIST_H
#define HAUSDORFF_DIST_H

// local
#include "debug.h"
#include "HausdorffNode.h"
#include "utility.h"

// C++
#include <ctime>
#include <vector>

// Opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

class HausdorffDist
{
//VARIABLES
public:
	/*nothing*/
private:
	/*nothing*/
	
//METHODS
public:
	HausdorffDist(void) {/*do nothing*/};
	~HausdorffDist(void) {/*do nothing*/};

	int computeDist(std::vector<int> A, std::vector<int> B);
private:
	int compute_dist_fast_more_more(std::vector<std::pair<bool,int> > AEnd, std::vector<std::pair<bool,int> > BEnd);

	std::vector<int> detecHoles(std::vector<int> postion,std::vector<std::pair<bool,int> >& endInShape, std::vector<std::pair<bool,int> >& endInHole);

	int pointInSegment(std::vector<std::pair<bool,int> > AEnd, int mid);
};

#endif
