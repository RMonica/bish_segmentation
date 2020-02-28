#ifndef EVALUATION_H
#define EVALUATION_H

// local
#include "debug.h"
#include "utility.h"

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

float evaluate(std::string tppath, std::string rppath, std::string test_cloud_name);

#endif
