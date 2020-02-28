#ifndef SCOLOR_MAP_H
#define SCOLOR_MAP_H

// local
#include "debug.h"

// C++
#include <vector>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif


/****************************************************************************************************/

typedef cv::Vec3f Color;

/****************************************************************************************************/

class SColorMap
{
//VARIABLES
public:
	std::vector<Color> m_lstClassColors;
	std::vector<Color> m_lstBoxColors;
private:
	Color *m_classColors;
	int m_classColorNum;
	Color *m_boxColors;
	int m_boxColorNum;

//METHODS
public:
	SColorMap(void);
	~SColorMap(void){/*do nothing*/};

	inline Color GetClassColor(const int i) {return m_classColors[i];}
	
	inline Color GetBoxColor(const int i) {return m_boxColors[i];}

	inline void SetClassColor(const int i, Color color) {m_classColors[i] = color;}

	inline void SetBoxColor(const int i, Color color) {m_boxColors[i] = color;}
private:
	/*nothing*/
};

#endif
