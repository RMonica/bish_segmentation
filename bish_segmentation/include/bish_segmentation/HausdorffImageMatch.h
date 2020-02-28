#ifndef HAUSDORFF_IMAGE_MATCH_H
#define HAUSDORFF_IMAGE_MATCH_H

//local
#include "debug.h"
#include "HausdorffImageProcessor.h"

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

static std::vector<float> initialRegDis = std::vector<float>();

/****************************************************************************************************/

class HausdorffImageMatch: public HausdorffImageProcessor
{
//VARIABLES
public:
	/*nothing*/
private:
	/*nothing*/
	
//METHODS
public:
	HausdorffImageMatch() {/*do nothing*/};
	~HausdorffImageMatch() {/*do nothing*/};
	
	std::vector<std::pair<int,int> > run(std::vector<HausdorffNode> srcNodes, std::vector<HausdorffNode> dstNodes, float& sumcost, std::vector<float>& regionDists = initialRegDis);
	std::vector<std::pair<int, int> > findMincost(std::vector<std::vector<float> > cost, int m, int n, float& mcost);
private:
};

#endif
