#ifndef HAUSDORFF_NODE_H
#define HAUSDORFF_NODE_H

// local
#include "debug.h"

// C++
#include <vector>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

typedef struct HausdorffNode
{
//VARIABLES
public:
	int start;
	int end;
	float uppersistence;
	float downpersistence;
	float minpersistence;
	bool merged;
	int representiveIndex;
	std::vector<int> line;
	int height;

//METHODS
public:
	HausdorffNode(void)
	{
		height = 0;
		merged = false;
		representiveIndex = -1;
	}
	
	HausdorffNode(int s, int e, int h, std::vector<int> data)
	{
		start = s;
		end = e;
		height = h;
		line = data;
	}
}HausdorffNode;

/****************************************************************************************************/

typedef struct Scanline
{
//VARIABLES
public:
	std::vector<std::pair<bool, int> > shapes;
	std::vector<std::pair<bool, int> > holes;
}Scanline;

/****************************************************************************************************/

typedef struct Segment{
//VARIABLES
public:
	int start;
	int end;
	bool continous;
	float scalefactor;
	
//METHONDS
public:
	Segment(int s,int e, bool c)
	{
		start = s;
		end = e;
		continous = c;
	}
}Segment;

#endif


