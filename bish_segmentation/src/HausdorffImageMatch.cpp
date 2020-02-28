#include <bish_segmentation/HausdorffImageMatch.h>

/****************************************************************************************************/

std::vector<std::pair<int,int> > HausdorffImageMatch::run(std::vector<HausdorffNode> srcNodes, std::vector<HausdorffNode> dstNodes,float& sumcost, std::vector<float>& regionDists)
{
	int num1 = srcNodes.size();
	int num2 = dstNodes.size();
	std::vector<std::vector<float> > distmatrix(num1);
	for(int jj=0; jj<num1; jj++)
		distmatrix[jj].resize(num2,0);

	for(int jj=0; jj<num1; jj++)
		for(int kk=0; kk<num2; kk++)
		{
			float dist = m_hausdorffDist.computeDist(srcNodes[jj].line,dstNodes[kk].line);	
			//float scalefactor = srcNodes[jj].height/(float)dstNodes[kk].height;
			distmatrix[jj][kk] = dist;// *srcNodes[jj].height*scalefactor;
		}

	std::vector<std::pair<int,int> > pairs = findMincost(distmatrix, num1-1, num2-1, sumcost);
	for(int i=0; i<pairs.size(); i++)
	{
		int m = pairs[i].first;
		int n = pairs[i].second;
		regionDists.push_back(distmatrix[m][n]);
	}
	return pairs;
}

/****************************************************************************************************/

std::vector<std::pair<int,int> > HausdorffImageMatch::findMincost(std::vector<std::vector<float> > cost, int m, int n, float& mcost)
{
  if (cost.empty())
    return std::vector<std::pair<int,int> >();

	int R = cost.size();
  int C = cost[0].size();
	// Instead of following line, we can use int tc[m+1][n+1] or 
	// dynamically allocate memory to save space. The following line is
	// used to keep te program simple and make it working on all compilers.
	std::vector<std::vector<float> > tc(R); 
	std::vector<std::vector<std::pair<int,int> > > path(R);

	for(int i=0; i<R; i++)
	{
		path[i].resize(C);
		tc[i].resize(C);
	}

	tc[0][0] = cost[0][0];

	/* Initialize first column of total cost(tc) array */
	for(int i=1; i<=m; i++)
	{
		tc[i][0] = tc[i-1][0] + cost[i][0];
		path[i][0] = std::pair<int, int>(i-1,0);
	}

	/* Construct rest of the tc array */
	for(int i=1; i<=m; i++)
		for(int j=1; j<=n; j++)	
		{
			if (j >= i)
			{		//ADD
				path[i][j] = std::pair<int, int>(i-1, j-1);
				tc[i][j] = tc[i-1][j-1] + cost[i][j];
				continue;
			}

			float tmpmin =  std::min(tc[i-1][j-1], tc[i-1][j]);

			if(tmpmin == tc[i-1][j])
			{
				if(path[i-1][j] == std::pair<int, int>(i-1,j-1))
				{
					path[i][j] = std::pair<int, int>(i-1,j-1);
					tmpmin = tc[i-1][j-1];
				}
				else
					path[i][j] = std::pair<int, int>(i-1,j);
			}
			else if(tmpmin == tc[i-1][j-1])
				path[i][j] = std::pair<int, int>(i-1,j-1);

			tc[i][j] = tmpmin + cost[i][j];
		}

		mcost = tc[m][n];

		std::vector<std::pair<int, int> > spath;
		std::pair<int, int> tmp(m,n);
		bool finish = false;
		while(!finish)
		{
			spath.push_back(tmp);
			m=tmp.first;
			n=tmp.second;
			tmp = path[m][n];

			if(tmp.first == 0 && tmp.second == 0)
			{
				finish=true;
				spath.push_back(std::pair<int, int>(0,0));
			}
		}

		return spath;
}


