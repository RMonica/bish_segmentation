#include <bish_segmentation/HausdorffImageSimplify.h>

/****************************************************************************************************/

extern SColorMap g_colorMap;

/****************************************************************************************************/

HausdorffImageSimplify::HausdorffImageSimplify()
{
	m_nodes.clear();
	m_minNumLines = 8;
	m_oversegThreshold = 30;
	m_maxNumLines = 30;

}

/****************************************************************************************************/

void HausdorffImageSimplify::run(std::string filename,int maxclust,bool bmerge,int minNumLines)
{
	if(minNumLines!=-1)
		m_minNumLines = minNumLines;

	m_rowset.clear();
	if(BOTH_DIRECTIONS)
		m_rowset_rot.clear();		//ADD
	m_nodes.clear();
	if(BOTH_DIRECTIONS)
		m_nodes_rot.clear();		//ADD
	m_distmap.clear();
	if(BOTH_DIRECTIONS)
		m_distmap_rot.clear();		//ADD

	m_imagename = filename;
	int pos = m_imagename.find_last_of("/");
	if(PRINT_LEVEL >= 2)
		std::cout << "\033[0;36m\tprocessing image \033[0m"<< m_imagename.substr(pos+1,m_imagename.size()) << std::endl;
	loadImage(filename);		//load normal and rotated image
	extractSets(m_grayImage, &m_rowset);		//extract only black lines of the normal image ("removing white background and holes")
	if(BOTH_DIRECTIONS)
		extractSets(m_grayImage_rot, &m_rowset_rot);		//extract only black lines of the rotated image ("removing white background and holes")
	computeRCDists();		//compute dists of normal and rotated image
	linkage(maxclust, &m_distmap, &m_rowset, &m_nodes);
	if(BOTH_DIRECTIONS)
		linkage(maxclust, &m_distmap_rot, &m_rowset_rot, &m_nodes_rot);

	if (bmerge) {
		mergeNodes(&m_nodes, &m_distmap, &m_rowset);
		if(BOTH_DIRECTIONS)
			mergeNodes(&m_nodes_rot, &m_distmap_rot, &m_rowset_rot);
	}
}

/****************************************************************************************************/

void HausdorffImageSimplify::extractSets(cv::Mat grayImage, std::vector<std::vector<int> > *rowset)//MOD aggiunto parametro cos da poterla usare sia per l'immagine normale sia per quella ruotata
{

	for(int i=0;i<grayImage.rows;i++)
	{
		std::vector<int> tmp;
		for(int j=1;j<grayImage.cols-1;j++)
		{
			if(grayImage.at<uchar>(i,j)==0)
				tmp.push_back(j);
		}
		rowset->push_back(tmp);
	}
}

/****************************************************************************************************/

void HausdorffImageSimplify::computeRCDists()
{

	for(unsigned int i=1;i<m_rowset.size();i++)
	{
		if(m_rowset[i].size()!=0&&m_rowset[i-1].size()!=0)
		{
			float dist = m_hausdorffDist.computeDist(m_rowset[i],m_rowset[i-1]);
			m_distmap.insert(std::pair<int, float>(i,dist));
		}
	}

	//ADD
	if(BOTH_DIRECTIONS)
	{
		for (unsigned int i = 1; i<m_rowset_rot.size(); i++)
		{
			if (m_rowset_rot[i].size() != 0 && m_rowset_rot[i - 1].size() != 0)
			{
				float dist_rot = m_hausdorffDist.computeDist(m_rowset_rot[i], m_rowset_rot[i - 1]);
				m_distmap_rot.insert(std::pair<int, float>(i, dist_rot));
			}
		}
	}
	//END ADD
}

/****************************************************************************************************/

void HausdorffImageSimplify::mergeTwoNodes(std::vector<HausdorffNode>  &nodes, int index, std::vector<std::vector<int> > *p_m_rowset)		//MOD aggiunto parametro cos da poterla usare sia per l'immagine normale sia per quella ruotata (NOTA nodes e index c'erano gi�)
{
	int up = index-1;
	while(up>=0&&nodes[up].merged)
		up--;
	if(up<0)
		up = 0;

	int upprev = up-1;
	while(upprev>=0&&nodes[upprev].merged)
		upprev--;
	if(upprev<0)
		upprev = 0;


	unsigned int below = index+1;
	while(below<nodes.size()&&nodes[below].merged)
		below++;
	unsigned int belownext = below+1;
	while(belownext<nodes.size()&&nodes[belownext].merged)
		belownext++;

	if(below>=nodes.size())
		below = nodes.size()-1;

	int newindex = index;

	if(nodes[index].uppersistence<nodes[index].downpersistence)
		newindex = up;
	else
		newindex = below;

	int start =  nodes[newindex].start<nodes[index].start?nodes[newindex].start:nodes[index].start;
	int end =    nodes[newindex].end>nodes[index].end? nodes[newindex].end:nodes[index].end;
	int newmid = (start+end)/2;
	if((*p_m_rowset)[newmid].size()==0)
	{
		int upmid = newmid--;
		int bemid = newmid++;
		while((*p_m_rowset)[upmid].size()==0)
			upmid--;
		while((*p_m_rowset)[bemid].size()==0)
			bemid++;
		int updist = abs(upmid-newmid);
		int bedist = abs(bemid-newmid);
		if(updist>bedist)
			newmid = bemid;
		else
			newmid = upmid;
	}


	if(nodes[index].uppersistence<nodes[index].downpersistence)//��up node �ϲ�
	{

		int minindex =  newmid;
		nodes[up].representiveIndex = minindex;

		//����up��uppersistance��upprev��downpersistence
		if(up>0&&upprev>=0)
		{
			float lens = abs(newmid-nodes[upprev].representiveIndex+1);
			lens = sqrt(lens);
			nodes[up].uppersistence = lens*m_hausdorffDist.computeDist((*p_m_rowset)[newmid], (*p_m_rowset)[nodes[upprev].representiveIndex]);

			nodes[upprev].downpersistence = nodes[up].uppersistence;
			nodes[upprev].minpersistence = nodes[upprev].uppersistence>nodes[upprev].downpersistence?nodes[upprev].downpersistence :nodes[upprev].uppersistence;

		}
		else
			nodes[up].uppersistence = FLT_MAX;

		//����up��downpersistence��below��uppersistence
		if(below<nodes.size()-1)
		{
			float lens = abs(newmid-nodes[below].representiveIndex+1);
			lens = sqrt(lens);
			nodes[up].downpersistence = lens*m_hausdorffDist.computeDist((*p_m_rowset)[newmid], (*p_m_rowset)[nodes[below].representiveIndex]);
			nodes[below].uppersistence = nodes[up].downpersistence;
			nodes[below].minpersistence = nodes[below].uppersistence>nodes[below].downpersistence?nodes[below].downpersistence :nodes[below].uppersistence;
		}
		else
			nodes[up].downpersistence = FLT_MAX;

		nodes[up].minpersistence = nodes[up].uppersistence>nodes[up].downpersistence?nodes[up].downpersistence :nodes[up].uppersistence;
		nodes[up].end = nodes[index].end;
		nodes[up].height = nodes[up].end-nodes[up].start+1;
		nodes[index].merged=true;
		//count++;
	}
	else //��below nodes �ϲ�
	{
		nodes[index].merged=true;
		nodes[below].representiveIndex = newmid;

		///����up��downpersistence��below��uppersistence
		if(up>0)
		{
			float lens = abs(newmid-nodes[up].representiveIndex+1);
			lens = sqrt(lens);
			nodes[below].uppersistence = lens*m_hausdorffDist.computeDist((*p_m_rowset)[newmid], (*p_m_rowset)[nodes[up].representiveIndex]);
			nodes[up].downpersistence = nodes[below].uppersistence;
			nodes[up].minpersistence = nodes[up].uppersistence>nodes[up].downpersistence?nodes[up].downpersistence :nodes[up].uppersistence;
		}
		else
			nodes[below].uppersistence = FLT_MAX;


		///����below��downpersistence��belownext��uppersistence
		if(below<nodes.size()-1&&belownext<nodes.size())
		{
			float lens = abs(newmid-nodes[belownext].representiveIndex+1);
			lens = sqrt(lens);
			nodes[below].downpersistence = lens*m_hausdorffDist.computeDist((*p_m_rowset)[newmid], (*p_m_rowset)[nodes[belownext].representiveIndex]);
			nodes[belownext].uppersistence = nodes[below].downpersistence;
			nodes[belownext].minpersistence = nodes[belownext].uppersistence>nodes[belownext].downpersistence?nodes[belownext].downpersistence :nodes[belownext].uppersistence;
		}
		else
			nodes[below].downpersistence = FLT_MAX;
		
		nodes[below].minpersistence = nodes[below].uppersistence>nodes[below].downpersistence?nodes[below].downpersistence :nodes[below].uppersistence;
		nodes[below].start = nodes[index].start;
		nodes[below].height = nodes[below].end-nodes[below].start+1;
		//count++;
	}
}

/****************************************************************************************************/

void HausdorffImageSimplify::linkage(int maxclust, std::map<int, float> *p_m_distmap, std::vector<std::vector<int> > *p_m_rowset, std::vector<HausdorffNode> *p_m_nodes)		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata (NOTA maxclust c'era gi�)
{
	bool overseg = false;
	if(maxclust >= m_oversegThreshold)
		overseg = true;

	std::vector<HausdorffNode>  nodes = constructInitialNodes(p_m_distmap);
	int numdata = nodes.size();
	std::vector<bool> merged(numdata-1,false);
	int count = 0;

	std::map<int, float>::iterator it = p_m_distmap->begin();

	std::map<int, float>::reverse_iterator rit = p_m_distmap->rbegin();

	while(count<numdata-maxclust)
	{
		int index = findMinNode(nodes,overseg);
		if(index!=-1)
		{
			mergeTwoNodes(nodes,index,  p_m_rowset);
			count++;
		}
	}

	p_m_nodes->clear();
	for(unsigned int i=0; i<nodes.size(); i++)
	{
		if(!nodes[i].merged)
		{
			nodes[i].line = (*p_m_rowset)[nodes[i].representiveIndex];
			nodes[i].height = nodes[i].end-nodes[i].start+1;
			p_m_nodes->push_back(nodes[i]);
		}
	}
}

/****************************************************************************************************/

std::vector<HausdorffNode> HausdorffImageSimplify::constructInitialNodes(std::map<int, float> *distmap)			//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata
{
	std::vector<HausdorffNode> nodes;
	HausdorffNode tmp;
	for (std::map<int, float>::iterator it = distmap->begin(); it != distmap->end(); it++)
	{
		tmp.start = (*it).first;
		tmp.end = (*it).first;
		nodes.push_back(tmp);
	}
	if (nodes.size()>0)
		nodes[0].start = nodes[0].end - 1;


	if (nodes.size()>0)
	{
		nodes[0].uppersistence = FLT_MAX;
		nodes[0].downpersistence = (*distmap)[nodes[0].end + 1];
	}
	if (nodes.size()>1)
	{

		for (unsigned int i = 1; i<nodes.size() - 1; i++)
		{
			nodes[i].uppersistence = (*distmap)[nodes[i].start];;
			nodes[i].downpersistence = (*distmap)[nodes[i].end + 1];
		}
		nodes[nodes.size() - 1].uppersistence = (*distmap)[nodes[nodes.size() - 1].start];
		nodes[nodes.size() - 1].downpersistence = FLT_MAX;
	}

	for (unsigned int i = 0; i<nodes.size(); i++)
	{
		nodes[i].minpersistence = nodes[i].uppersistence>nodes[i].downpersistence ? nodes[i].downpersistence : nodes[i].uppersistence;
		nodes[i].representiveIndex = nodes[i].start;
		nodes[i].height = 1;
	}

	return nodes;
}

/****************************************************************************************************/

int HausdorffImageSimplify::findMinNode(std::vector<HausdorffNode> nodes, bool overseg)
{
	int index = -1;
	float minP =FLT_MAX;
	if(overseg)
	{
		for(unsigned int i=0; i<nodes.size(); i++)
		{
			if(nodes[i].minpersistence<minP&&!nodes[i].merged&&nodes[i].height<=m_maxNumLines)
			{
				minP = nodes[i].minpersistence;
				index = i;
			}
		}
	}
	else
	{
		for(unsigned int i=0; i<nodes.size(); i++)
		{
			if(nodes[i].minpersistence<minP&&!nodes[i].merged)
			{
				minP = nodes[i].minpersistence;
				index = i;
			}
		}
	}
	return index;
}

/****************************************************************************************************/

void HausdorffImageSimplify::mergeNodes(std::vector<HausdorffNode> *p_m_nodes, std::map<int, float> *p_m_distmap, std::vector<std::vector<int> > *p_m_rowset)		//MOD aggiunto parametro cos� da poterla usare sia per l'immagine normale sia per quella ruotata
{
	if (p_m_nodes->size() == 0) return;

	std::vector<HausdorffNode> nodes = (*p_m_nodes) ;
	std::vector<int> sortIndex = sortNodes(nodes);
	for (unsigned int i = 0; i<nodes.size(); i++)
		nodes[i].merged = false;


	std::map<int, float>::iterator it = p_m_distmap->begin();

	std::map<int, float>::reverse_iterator rit = p_m_distmap->rbegin();
	
	for(unsigned int i = 0; i<nodes.size(); i++)
	{
		int ii = sortIndex[i];

		int len = nodes[ii].end - nodes[ii].start + 1;
		if (len>m_minNumLines || nodes[ii].merged)
			continue;
		mergeTwoNodes(nodes, ii, p_m_rowset);
	}
	
	std::vector<HausdorffNode> fnodes;
	for (unsigned int i = 0; i<nodes.size(); i++)
	{
		if (!nodes[i].merged)
		{
			nodes[i].line = (*p_m_rowset)[nodes[i].representiveIndex];
			nodes[i].height = nodes[i].end - nodes[i].start + 1;
			fnodes.push_back(nodes[i]);
		}
	}

	(*p_m_nodes) = fnodes;
}

/****************************************************************************************************/

std::vector<int> HausdorffImageSimplify::sortNodes(std::vector<HausdorffNode> nodes)
{
	std::vector<float> persistences;
	for(unsigned int i=0; i<nodes.size(); i++)
		persistences.push_back(nodes[i].minpersistence);

	return sortWithIndex(persistences);

}


/****************************************************************************************************/

//ADD
void HausdorffImageSimplify::saveSegmentImg(std::string name)		//MOD
{
	cv::Mat rimg(m_grayImage.size(), CV_8UC3);
	for(int r=0; r<rimg.rows; r++)
		for(int c=0; c<rimg.cols; c++)
			rimg.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 255, 255);


	for (unsigned int i = 0; i<m_nodes.size(); i++)
	{
		int start = m_nodes[i].start;
		int end = m_nodes[i].end;

		for (int j = start; j <= end; j++)
		{
			if(BOTH_DIRECTIONS)
			{
				for (unsigned int ir = 0; ir < m_nodes_rot.size(); ir++)
				{
					int start_rot = m_nodes_rot[ir].start;
					int end_rot = m_nodes_rot[ir].end;

					for (int k = start_rot; k <= end_rot; k++)
					{
						if (m_grayImage.at<uchar>(j, k) == 0)
						{
							Color rgb = g_colorMap.GetClassColor(i % 10);
							Color rgb2 = g_colorMap.GetClassColor(ir % 10);

							uchar r = ((rgb[2] + rgb2[2]) / 2 + rgb[2]) * 255;
							uchar g = ((rgb[1] + rgb2[1]) / 2 + rgb[1]) * 255;
							uchar b = ((rgb[0] + rgb2[0]) / 2 + rgb[0]) * 255;

							rimg.at<cv::Vec3b>(j, k) = cv::Vec3b(r, g, b);
						}
					}
				}
			}
			else
			{
				for (int k = 0; k < m_grayImage.cols; k++)
					{
						if (m_grayImage.at<uchar>(j, k) == 0)
						{
							Color rgb = g_colorMap.GetClassColor(i % 10);

							uchar r = (rgb[2]) * 255;
							uchar g = (rgb[1]) * 255;
							uchar b = (rgb[0]) * 255;

							rimg.at<cv::Vec3b>(j, k) = cv::Vec3b(r, g, b);
						}
					}
			}
			
		}
	}


	if (name.size()<2)
	{
		int pos = m_imagename.find_last_of("/");
		std::string dir = m_imagename.substr(0, pos);
		std::string name = m_imagename.substr(pos + 1, m_imagename.size() - pos);
		char buf[255];
		if(BOTH_DIRECTIONS)
			sprintf(buf,"%d",int((m_nodes.size()*m_nodes_rot.size())));
		else
			sprintf(buf,"%d",int((m_nodes.size())));
		std::string tmpname = dir;
		tmpname.append("/light/");
		if (!fileExists(tmpname))
			mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
		tmpname.append(name);
		tmpname.append("-");
		tmpname.append(buf);
    tmpname.append("-l_tog.png");
		cv::imwrite(tmpname.c_str(), rimg);
	}
	else
		cv::imwrite(name.c_str(), rimg);
}
//END ADD

/****************************************************************************************************/

void HausdorffImageSimplify::saveNodeInfo()
{
	int pos = m_imagename.find_last_of("/");
	std::string dir = m_imagename.substr(0, pos);
	std::string name = m_imagename.substr(pos + 1, m_imagename.size() - pos);
	std::string tmpname = dir;
	tmpname.append("/nodes/");
	if (!fileExists(tmpname))
		mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	tmpname.append(name.substr(0, name.size() - 4));
	tmpname.append(".txt");

	std::ofstream nodefile(tmpname.c_str());
	for (unsigned int i = 0; i<m_nodes.size(); i++)
	{
		nodefile << m_nodes[i].start << '\t' << m_nodes[i].end << '\t' << m_nodes[i].height << '\t' << m_nodes[i].line.size() << std::endl;
		for (unsigned int j = 0; j<m_nodes[i].line.size(); j++)
			nodefile << m_nodes[i].line[j] << ' ';
		nodefile << std::endl;
	}
	nodefile.close();
}

/****************************************************************************************************/

//ADD
void HausdorffImageSimplify::saveNodeInfo_rot()
{
	int pos = m_imagename.find_last_of("/");
	std::string dir = m_imagename.substr(0, pos);
	std::string name = m_imagename.substr(pos + 1, m_imagename.size() - pos);
	std::string tmpname = dir;
	tmpname.append("/nodes/");
	if (!fileExists(tmpname))
		mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	tmpname.append(name.substr(0, name.size() - 4));
	tmpname.append("_rot.txt");

	std::ofstream nodefile(tmpname.c_str());
	for (unsigned int i = 0; i<m_nodes_rot.size(); i++)
	{
		nodefile << m_nodes_rot[i].start << '\t' << m_nodes_rot[i].end << '\t' << m_nodes_rot[i].height << '\t' << m_nodes_rot[i].line.size() << std::endl;
		for (unsigned int j = 0; j<m_nodes_rot[i].line.size(); j++)
			nodefile << m_nodes_rot[i].line[j] << ' ';
		nodefile << std::endl;
	}
	nodefile.close();
}
//END ADD

/****************************************************************************************************/

std::vector<HausdorffNode> HausdorffImageSimplify::loadNodeInfo(std::string nodefile)
{
	std::ifstream nodein(nodefile.c_str());

  if (!nodein)
    ROS_ERROR("error while loading: %s", nodefile.c_str());

	std::vector<HausdorffNode>  nodes;
	std::string nodeinfo;


  while(nodein){

		//read data from file
		int start, end, height, numPtInline;
		std::getline(nodein, nodeinfo);
		if(nodeinfo.size() < 1)
			break;
		std::istringstream(nodeinfo, std::ios_base::in) >> start >> end >> height>> numPtInline;
		std::getline(nodein,nodeinfo);
		//sscanf (nodeinfo," %d %d %d %d ", &start, &end, &height,&numPtInline);

		std::vector<int> pts(numPtInline,-1);
		std::stringstream ss(nodeinfo);
		int j = 0; // the current word index
		while (j < numPtInline) {
			ss >> pts[j]; // here i is as above: the current line index
			j++;
		}

		nodes.push_back(HausdorffNode(start, end, height, pts));
	}

	nodein.close();
	
	return nodes;
}

/****************************************************************************************************/

//ADD
void HausdorffImageSimplify::transferLabels(std::string lpath, std::string tpath, std::string results_path, std::string testfile, std::string labeledfile, std::vector<HausdorffNode> tRegion, std::vector<HausdorffNode> lRegion, std::vector<HausdorffNode> tRegion_rot, std::vector<HausdorffNode> lRegion_rot, std::vector<std::pair<int, int> > pairs, std::vector<std::pair<int, int> > pairs_rot)
{
	
	loadImage(std::string(tpath).append(testfile));

	cv::Mat image = cv::imread(std::string(lpath).append(labeledfile), 1);

	cv::Mat rimg(m_grayImage.size(), CV_8UC3);
	for (int r = 0; r<rimg.rows; r++)
		for (int c = 0; c<rimg.cols; c++)
			rimg.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 255, 255);

	std::vector<std::pair<std::pair<int, int>, std::pair<std::pair<int, int>, std::pair<int, int> > > > tmp;
	for (unsigned int p = 0; p < pairs.size(); p++)
	{
		for (unsigned int pr = 0; pr < pairs_rot.size(); pr++)
		{
			int start = tRegion[pairs[p].first].start;
			int end = tRegion[pairs[p].first].end;
			int start_rot = tRegion_rot[pairs_rot[pr].first].start;
			int end_rot = tRegion_rot[pairs_rot[pr].first].end;
			bool exist = false;
			for(unsigned int t = 0; t < tmp.size(); t++)
			{
				if(tmp[t].first == std::pair<int, int>(pairs[p].second, pairs_rot[pr].second))
				{
					if(start < tmp[t].second.first.first)
						tmp[t].second.first.first = start;
					if(end > tmp[t].second.first.second)
						tmp[t].second.first.second = end;

					if(start_rot < tmp[t].second.second.first)
						tmp[t].second.second.first = start_rot;
					if(end_rot > tmp[t].second.second.second)
						tmp[t].second.second.second = end_rot;

					exist = true;
					break;
				}
			}

			if(!exist)
				tmp.push_back(std::pair<std::pair<int, int>, std::pair<std::pair<int, int>, std::pair<int, int> > >(std::pair<int, int>(pairs[p].second, pairs_rot[pr].second), std::pair<std::pair<int, int>, std::pair<int, int> >(std::pair<int, int>(start, end), std::pair<int, int>(start_rot, end_rot))));
		}
	}

	for(unsigned int t=0; t < tmp.size(); t++)
	{
		int start_sec = lRegion[tmp[t].first.first].start;
		int end_sec = lRegion[tmp[t].first.first].end;
		int start_rot_sec = lRegion_rot[tmp[t].first.second].start;
		int end_rot_sec = lRegion_rot[tmp[t].first.second].end;

		int start = tmp[t].second.first.first;
		int end = tmp[t].second.first.second;
		int start_rot = tmp[t].second.second.first;
		int end_rot = tmp[t].second.second.second;

		float stretchV = abs((float)((end - start)+1) / (float)((end_sec - start_sec)+1));
		float stretchH = abs((float)((end_rot - start_rot)+1) / (float)((end_rot_sec - start_rot_sec)+1));

		for(int i=start; i<=end; i++)
		{
			int ii = floor((abs((float)((i-start)) / stretchV)) + start_sec);
		 	for(int j=start_rot; j<=end_rot; j++)
			{
				int jj = floor((abs((float)((j-start_rot)) / stretchH)) + start_rot_sec);

				if(m_grayImage.at<uchar>(i, j) == 0)
				{
					cv::Vec3b rgb = image.at<cv::Vec3b>(ii,jj);

					//filler for white pixels
					float r = 1;
					float theta = -1;
					float angleStep = 1;
					int x;
					int y;
					while(rgb[2]>= 250 && rgb[1]>= 250 && rgb[0]>= 250)
					{
						theta += angleStep;
						if(theta >= 360)
						{
							theta = 0;
							r++;
						}

						x = ii + floor((r * cos(theta / 180 * M_PI)));
						y = jj + floor((r * sin(theta / 180 * M_PI)));

						if(x < 0 || y < 0 || x > 512 || y > 512)
							continue;

						rgb = image.at<cv::Vec3b>(x,y);
					}

					rimg.at<cv::Vec3b>(i, j) = rgb;
				}
			}
		}
	}

	std::string tmpname = results_path;
	if (!fileExists(tmpname))
		mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	tmpname.append(testfile);
	cv::imwrite(tmpname.c_str(), rimg);
}
//END ADD





/****************************************************************************************************/

//ADD (mod sopra)
void HausdorffImageSimplify::transferLabels(std::string lpath, std::string tpath, std::string results_path, std::string testfile, std::string labeledfile, std::vector<HausdorffNode> tRegion, std::vector<HausdorffNode> lRegion, std::vector<std::pair<int, int> > pairs)
{
	loadImage(std::string(tpath).append(testfile));

	cv::Mat image = cv::imread(std::string(lpath).append(labeledfile), 1);

	cv::Mat rimg(m_grayImage.size(), CV_8UC3);
	for (int r = 0; r<rimg.rows; r++)
		for (int c = 0; c<rimg.cols; c++)
			rimg.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 255, 255);

	std::vector<std::pair<int, std::pair<int, int> > > tmp;
	for (unsigned int p = 0; p < pairs.size(); p++)
	{
		int start = tRegion[pairs[p].first].start;
		int end = tRegion[pairs[p].first].end;
		bool exist = false;
		for(unsigned int t = 0; t < tmp.size(); t++)
		{
			if(tmp[t].first == pairs[p].second)
			{
				if(start < tmp[t].second.first)
					tmp[t].second.first = start;
				if(end > tmp[t].second.second)
					tmp[t].second.second = end;

				exist = true;
				break;
			}
		}

		if(!exist)
			tmp.push_back(std::pair<int, std::pair<int, int> > (pairs[p].second, std::pair<int, int>(start, end)));
	}
	
	for(unsigned int t=0; t < tmp.size(); t++)
	{
		int start_sec = lRegion[tmp[t].first].start;
		int end_sec = lRegion[tmp[t].first].end;
		
		int start = tmp[t].second.first;
		int end = tmp[t].second.second;

		float stretchV = abs((float)((end - start)+1) / (float)((end_sec - start_sec)+1));

		for(int i=start; i<=end; i++)
		{
			int ii = floor((abs((float)((i-start)) / stretchV)) + start_sec);
		 	for(int j=0; j<=image.cols; j++)
			{
				if(m_grayImage.at<uchar>(i, j) == 0)
				{
					cv::Vec3b rgb = image.at<cv::Vec3b>(ii,j);

					//filler for white pixels
					float r = 1;
					float theta = -1;
					float angleStep = 1;
					int x;
					int y;
					while(rgb[2]>= 250 && rgb[1]>= 250 && rgb[0]>= 250)
					{
						theta += angleStep;
						if(theta >= 360)
						{
							theta = 0;
							r++;
						}

						x = ii + floor((r * cos(theta / 180 * M_PI)));
						y = j + floor((r * sin(theta / 180 * M_PI)));

						if(x < 0 || y < 0 || x > 512 || y > 512)
							continue;

						rgb = image.at<cv::Vec3b>(x,y);
					}

					rimg.at<cv::Vec3b>(i, j) = rgb;
				}
			}
		}
	}

	std::string tmpname = results_path;
	if (!fileExists(tmpname))
		mkdir(tmpname.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
	tmpname.append(testfile);
	cv::imwrite(tmpname.c_str(), rimg);
}
//END ADD
