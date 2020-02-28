#ifndef HAUSDORFF_IMAGE_SIMPLIFY_H
#define HAUSDORFF_IMAGE_SIMPLIFY_H

// local
#include "debug.h"
#include "HausdorffImageProcessor.h"
#include "HausdorffNode.h"
#include "SColorMap.h"

// C++
#include <fstream>
#include <sstream>
#include <sys/stat.h>

#ifndef PRINT_LEVEL
#define PRINT_LEVEL 0
#endif

/****************************************************************************************************/

class HausdorffImageSimplify:public HausdorffImageProcessor
{
//VARIABLES
public:
	/*nothing*/
private:
	std::vector<HausdorffNode> m_nodes;
	std::vector<HausdorffNode> m_nodes_rot;		//ADD
	std::vector<std::vector<int> > m_rowset;
	std::vector<std::vector<int> > m_rowset_rot;		//ADD
	std::map<int, float> m_distmap;
	std::map<int, float> m_distmap_rot;		//ADD
	int m_minNumLines;
	int m_maxNumLines;
	int m_oversegThreshold;
	std::string m_imagename;

//METHODS
public:
	HausdorffImageSimplify();
	~HausdorffImageSimplify() {/*do nothing*/};
	
	void run(std::string filename, int maxclust, bool bmerge = true, int minNumLines = -1);
	
	std::vector<std::vector<int> > getRowset() {return m_rowset;};
	
	std::vector<HausdorffNode> getNodes() {return m_nodes;}
	
	void saveSegmentImg(std::string name = std::string());	//MOD
	
	void saveNodeInfo();
	
	void saveNodeInfo_rot();		//ADD
	
	void transferLabels(std::string lpath, std::string tpath, std::string results_path, std::string testfile, std::string labeledfile, std::vector<HausdorffNode> tRegion, std::vector<HausdorffNode> lRegion, std::vector<HausdorffNode> tRegion_rot, std::vector<HausdorffNode> lRegion_rot, std::vector<std::pair<int, int> > pairs, std::vector<std::pair<int, int> > pairs_rot);		//ADD
	void transferLabels(std::string lpath, std::string tpath, std::string results_path, std::string testfile, std::string labeledfile, std::vector<HausdorffNode> tRegion, std::vector<HausdorffNode> lRegion, std::vector<std::pair<int, int> > pairs);		//ADD ADD

	std::vector<HausdorffNode> loadNodeInfo(std::string nodefile);
private:
	void linkage(int maxclus, std::map<int, float> *p_m_distmap, std::vector<std::vector<int> > *p_m_rowset, std::vector<HausdorffNode> *p_m_nodes);		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata (NOTA maxclust c'era gia)
	
	void extractSets(cv::Mat grayImage, std::vector<std::vector<int> > *p_m_rowset);		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata
	
	std::vector<HausdorffNode> constructInitialNodes(std::map<int,float> *p_m_distmap);		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata
	
	void computeRCDists();
	
	int findMinNode(std::vector<HausdorffNode> nodes,bool overseg);
	
	void mergeNodes(std::vector<HausdorffNode> *p_m_nodes, std::map<int, float> *p_m_distmap, std::vector<std::vector<int> > *p_m_rowset);		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata
	
	std::vector<int> sortNodes(std::vector<HausdorffNode>  nodes);
	
	void mergeTwoNodes(std::vector<HausdorffNode>& nodes, int index, std::vector<std::vector<int> > *p_m_rowset);		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata (NOTA nodes e index c'erano gi�)
};

#endif
