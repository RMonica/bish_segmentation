#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>


/****************************************************************************************************/
#define DEBUG	std::cout << "DEBUG " << __LINE__ << std::endl;

/**
 *	0	essential information like classification, accuracy
 *	1	program operations, operations time
 *	2	image processed, point loaded
 *	3	extra information like matching pairs
**/
#define PRINT_LEVEL 0
#define BOTH_DIRECTIONS true


#endif

