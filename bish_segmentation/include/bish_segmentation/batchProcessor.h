#ifndef BATCH_PROCESSOR_H
#define BATCH_PROCESSOR_H

// local
#include "debug.h"
#include "HausdorffImageMatch.h"
#include "HausdorffImageProcessor.h"
#include "HausdorffImageSimplify.h"
#include "HausdorffNode.h"
#include "utility.h"

// Boost
#include <boost/filesystem.hpp> 

// C++
#include <omp.h>
#include <sys/stat.h>
#include <tuple>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

class batchProcessor
{
//VARIABLES
public:
	/*nothing*/
private:
	int width;
	int height;

//METHODS
public:
	batchProcessor() {/*do nothing*/};
	~batchProcessor() {/*do nothing*/};

	std::vector<std::vector<HausdorffNode>> imageSimplify(std::string path, int numseg, bool bmerge=true, bool save_segment_image=false);

	void batchRetrieval(std::string testpath, std::string labelpath, std::string matchpath, std::vector<int> labelobjs = std::vector<int>());

	std::vector<std::tuple<std::string, std::string, unsigned int> > readMatchedFile(std::string file, unsigned long int* total_dist = new unsigned long int(0));

	void matchTransfer(std::string lpath, std::string tpath, std::string results_path, std::string testpath, std::string labeledpath);
	
	std::vector<std::vector<HausdorffNode> > batchLoadNodes(std::string path, std::vector<int> objs = std::vector<int>(), std::string suff = std::string());		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata
private:
	
	std::vector<std::string> getFileNamesFromLabelObj(std::vector<int> labelobj);
};

#endif
