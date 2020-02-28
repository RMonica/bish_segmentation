#include <bish_segmentation/batchProcessor.h>

/****************************************************************************************************/

std::vector<std::string> batchProcessor::getFileNamesFromLabelObj(std::vector<int>labelobjs)
{
	std::vector<std::string> imageNames;
	int numofeach = 60;
	if(labelobjs.size() != 0)
	{
		for(unsigned int ii=0; ii<labelobjs.size(); ii++)
		{
			int i = labelobjs[ii];
			char objname[255];
			sprintf(objname,"%d",i);

			for(int j=0; j<numofeach; j++)
			{
				int jid = j*360/numofeach;
				char jbuf[255];
				sprintf(jbuf,"%d",jid);
				std::string name(objname);
				name.append(".");
				name.append(jbuf);
        name.append(".png");
				imageNames.push_back(name);
			}
		}
	}

	return imageNames;
}

/****************************************************************************************************/

std::vector<std::vector<HausdorffNode> > batchProcessor::batchLoadNodes(std::string path, std::vector<int>labelobj, std::string suff)
{
	HausdorffImageSimplify* simplifer = new HausdorffImageSimplify;
	std::vector<std::string> imageNames;
	if(labelobj.size() != 0)
		imageNames = getFileNamesFromLabelObj(labelobj);
	else
		imageNames = getFileNames(path.c_str());

	std::vector<std::vector<HausdorffNode> > allImgNodes;

	for(unsigned int i=0; i<imageNames.size(); i++)
	{
		std::string filename(path);
		filename.append("nodes/");
		filename.append(imageNames[i].substr(0,imageNames[i].size()-4));
		filename.append(suff);
		filename.append(".txt");
		std::vector<HausdorffNode> nodes = simplifer->loadNodeInfo(filename);
		allImgNodes.push_back(nodes);
	}

	delete simplifer;

	return allImgNodes;
}

/****************************************************************************************************/

std::vector<std::vector<HausdorffNode> > batchProcessor::imageSimplify(std::string path, int numseg, bool bmerge, bool save_segment_image)
{
	HausdorffImageSimplify* simplifer = new HausdorffImageSimplify;
	std::vector<std::string> imageNames = getFileNames(path.c_str());
	std::vector<std::vector<HausdorffNode> > allImgNodes;
	boost::filesystem::path p(path.c_str());
	boost::filesystem::path dir = p.parent_path();
	boost::filesystem::path::iterator it = dir.end();
	it--;
	it--;
	if(PRINT_LEVEL >= 1)
		std::cout << "\033[0;36mIn folder: \033[0m" << (it)->filename().string() << "/" << (++it)->filename().string() << "/" << std::endl;

	for(unsigned int i=0; i<imageNames.size(); i++)
	{
		std::string filename(path);
		filename.append(imageNames[i]);
		simplifer->run(filename,numseg,bmerge);
		
		simplifer->saveNodeInfo();
		if(BOTH_DIRECTIONS)
			simplifer->saveNodeInfo_rot();		//ADD
		if(save_segment_image)
			simplifer->saveSegmentImg();		//MOD

		allImgNodes.push_back(simplifer->getNodes());
	}

	delete simplifer;

	return allImgNodes;
}

/****************************************************************************************************/

void batchProcessor::batchRetrieval(std::string tpath, std::string lpath, std::string results_path, std::vector<int> labelobjs)
{
	std::vector<std::vector<HausdorffNode> > testImgRegions = batchLoadNodes(tpath);
	std::vector<std::vector<HausdorffNode> > testImgRegions_rot;
	if(BOTH_DIRECTIONS)
		testImgRegions_rot = batchLoadNodes(tpath, std::vector<int>(), "_rot");		//ADD
	
	std::vector<std::vector<HausdorffNode> > labeledImgRegions;
	std::vector<std::vector<HausdorffNode> > labeledImgRegions_rot;		//ADD
	
	std::vector<std::string> dirnames = getDirectoryNames(lpath.c_str());
	if(dirnames.size() == 0)
	{
		ROS_ERROR("No directories in %s", lpath.c_str());
		exit(-1);
	}
	
	for(std::string dir_name : dirnames)
	{
		std::string tmp_name = std::string(lpath).append(dir_name);
		std::vector<std::vector<HausdorffNode> > tmpRegions = batchLoadNodes(tmp_name,labelobjs);
		labeledImgRegions.insert(labeledImgRegions.end(), tmpRegions.begin(), tmpRegions.end());
		
		if(BOTH_DIRECTIONS)
		{
			std::vector<std::vector<HausdorffNode> > tmpRegions_rot = batchLoadNodes(tmp_name, labelobjs, "_rot");
			labeledImgRegions_rot.insert(labeledImgRegions_rot.end(), tmpRegions_rot.begin(), tmpRegions_rot.end());
		}
	}
	
	std::vector<std::string> testedImg = getFileNames(tpath.c_str());
	
	std::vector<std::string> labeledImg;
	if(labelobjs.size() == 0)
	{
		for(std::string dir_name : dirnames)
		{
			std::vector<std::string> tmpLabeled = getFileNames(std::string(lpath).append(dir_name).c_str());
			labeledImg.insert(labeledImg.end(), tmpLabeled.begin(), tmpLabeled.end());
		}
	}
	else
		labeledImg = getFileNamesFromLabelObj(labelobjs);

	if(!fileExists(results_path))
		mkdir(results_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	
	HausdorffImageMatch *matcher = new HausdorffImageMatch;

	std::vector<std::pair<int, float> > matchparis;
	for(unsigned int i=0; i<testImgRegions.size(); i++)
	{
		std::vector<std::pair<int, float> > imatchparis;
		for(unsigned int j=0; j<labeledImgRegions.size(); j++)
		{
			if(labeledImg[j] == testedImg[i])
				continue;

			float cost = 0;
			float cost_rot = 0;		//ADD
			
			std::vector<std::pair<int, int> > pairs = matcher->run(testImgRegions[i], labeledImgRegions[j], cost);
			if(BOTH_DIRECTIONS)
				std::vector<std::pair<int, int> > pairs_rot = matcher->run(testImgRegions_rot[i], labeledImgRegions_rot[j], cost_rot);		//ADD

			imatchparis.push_back(std::pair<int, float>(j, cost + cost_rot));		//MOD aggiunto +cost_rot
		}

		sortPairwithIndex(imatchparis);

		matchparis.push_back(imatchparis[0]);
	}

	std::vector<int> pindex = sortPairwithIndex(matchparis);

	std::string logname(results_path);
	if(!fileExists(logname))
		mkdir(logname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);

	logname.append("matched.txt");

	std::ofstream outlog(logname.c_str());
	for(unsigned int i=0; i<matchparis.size(); i++)
	{
		int index = pindex[i];
		outlog << testedImg[index] << "\t" << labeledImg[matchparis[i].first] << "\t" << matchparis[i].second << std::endl;
	}

	outlog.close();
	delete matcher;
}

/****************************************************************************************************/

std::vector<std::tuple<std::string, std::string, unsigned int> > batchProcessor::readMatchedFile(std::string file, unsigned long int* total_dist)
{
	std::string logname(file);
	logname.append("matched.txt");
	std::ifstream matchedin(logname.c_str());

	/*std::vector<std::pair<std::string, std::string> >  matches;
	std::string nodeinfo;

	while(!matchedin.eof()){
		//read data from file
		std::string testImg, labeledImg;
		std::getline(matchedin, nodeinfo);
		if(nodeinfo.size() < 1)
			break;

		std::istringstream(nodeinfo, std::ios_base::in) >> labeledImg;
		char* tmpsrc;
		char* tmpdst;
		tmpsrc = (char*)malloc(255);
		tmpdst = (char*)malloc(255);
		sscanf(nodeinfo.c_str(),"%[^\t\n] %s",tmpsrc,tmpdst);

		matches.push_back(std::pair<std::string, std::string>(tmpsrc, tmpdst));
		free(tmpsrc);
		free(tmpdst);
	}
	matchedin.close();*/
	
	*total_dist = 0;
	std::vector<std::tuple<std::string, std::string, unsigned int> > matches;
	if(matchedin.is_open())
	{
		std::string test_name, labeled_name, d;
		unsigned int dist;
		while(matchedin >> test_name >> labeled_name >> d)
		{
			dist = atoi(d.c_str());
			matches.push_back(std::make_tuple(test_name, labeled_name, dist));
			*total_dist += dist;
		}
	}
	else
	{
		ROS_ERROR("File '%s' not found", logname.c_str());
		exit(-1);
	}
	
	matchedin.close();
	return matches;
}

/****************************************************************************************************/

void batchProcessor::matchTransfer(std::string lpath, std::string tpath, std::string results_path, std::string testpath, std::string labeledpath) {
	HausdorffImageSimplify *simplifier = new HausdorffImageSimplify;
	
	std::vector<HausdorffNode> testImgRegions;
	std::vector<HausdorffNode> testImgRegions_rot;

	std::string tmp_path = std::string(tpath).append("nodes/");
	tmp_path.append(testpath.substr(0, testpath.size() - 4));
	std::string tmp = std::string(tmp_path).append(".txt");
	testImgRegions = simplifier->loadNodeInfo(tmp);
	if(BOTH_DIRECTIONS)
	{
		std::string tmp_rot = std::string(tmp_path).append("_rot.txt");
		testImgRegions_rot = simplifier->loadNodeInfo(tmp_rot);
	}
	
	std::vector<HausdorffNode> labeledImgRegions;
	std::vector<HausdorffNode> labeledImgRegions_rot;
	std::vector<std::string> dirnames = getDirectoryNames(lpath.c_str());
	if(dirnames.size() == 0)
	{
		ROS_ERROR("No directories in %s", lpath.c_str());
		exit(-1);
	}
	
	std::string new_lpath = lpath;
	for(std::string dir_name : dirnames)
	{
		new_lpath = std::string(lpath).append(dir_name);
		std::string tmp_path = std::string(new_lpath).append("nodes/");
		tmp_path.append(labeledpath.substr(0, labeledpath.size() - 4));
		
		std::string tmp = std::string(tmp_path).append(".txt");
		if(!boost::filesystem::exists(tmp))
			continue;
		labeledImgRegions = simplifier->loadNodeInfo(tmp);
			
		if(BOTH_DIRECTIONS)
		{
			std::string tmp_rot = std::string(tmp_path).append("_rot.txt");
			if(!boost::filesystem::exists(tmp_rot))
				continue;
			labeledImgRegions_rot = simplifier->loadNodeInfo(tmp_rot);
		}
		break;
	}

	float cost = 0;
	float cost_rot = 0;
	HausdorffImageMatch *matcher = new HausdorffImageMatch;
	std::vector<std::pair<int, int> > pairs = matcher->run(testImgRegions, labeledImgRegions, cost);
	std::vector<std::pair<int, int> > pairs_rot;
	if(BOTH_DIRECTIONS)
		pairs_rot = matcher->run(testImgRegions_rot, labeledImgRegions_rot, cost_rot);
	
	if(BOTH_DIRECTIONS)
		simplifier->transferLabels(new_lpath, tpath, results_path, testpath, labeledpath, testImgRegions, labeledImgRegions, testImgRegions_rot, labeledImgRegions_rot, pairs, pairs_rot);
	else
		simplifier->transferLabels(new_lpath, tpath, results_path, testpath, labeledpath, testImgRegions, labeledImgRegions, pairs);

	delete simplifier;
}
