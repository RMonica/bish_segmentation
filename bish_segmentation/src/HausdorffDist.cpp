#include <bish_segmentation/HausdorffDist.h>

/****************************************************************************************************/

std::vector<int> HausdorffDist::detecHoles(std::vector<int> postion, std::vector<std::pair<bool,int> >& endInShape, std::vector<std::pair<bool,int> >& endInHole)
{
	if(postion.size() == 0)
		return std::vector<int>();

	if(postion.size() == 1)
		endInShape.push_back(std::pair<bool, int>(false, postion[0]));

	std::vector<int> holes;

	endInHole.push_back(std::pair<bool, int>(false, postion[0]-1));
	holes.push_back(postion[0]-1);

	int segmentEnd = postion[0];
	bool hasstart = false;
	for(int i=1; i<postion.size(); i++)
	{
		if(postion[i] == postion[i-1]+1)
		{
			segmentEnd = postion[i];
			if(!hasstart)
			{
				endInShape.push_back(std::pair<bool, int>(true, postion[i-1]));
				hasstart =	true;
			}
		}
		else
		{
			endInShape.push_back(std::pair<bool, int>(hasstart, segmentEnd));

			hasstart = false;
			if(segmentEnd+1 == postion[i]-1)
				endInHole.push_back(std::pair<bool, int>(false, postion[i]-1));
			else
			{
				endInHole.push_back(std::pair<bool, int>(true, segmentEnd+1));
				endInHole.push_back(std::pair<bool, int>(true, postion[i]-1));
			}

			for(int j=segmentEnd+1; j<postion[i]; j++)
				holes.push_back(j);

			segmentEnd = postion[i];
		}
	}

	if(postion.size() > 1)
		endInShape.push_back(std::pair<bool, int>(hasstart, postion[postion.size()-1]));

	endInHole.push_back(std::pair<bool, int>(false, postion[postion.size()-1]+1));
	holes.push_back(postion[postion.size()-1]+1);

	return holes;
}

/****************************************************************************************************/

int HausdorffDist::computeDist(std::vector<int> A, std::vector<int> B)
{
	std::vector<std::pair<bool,int> >  endInA,endInB;
	std::vector<std::pair<bool,int> > endInAHole, endInBHole;
	std::vector<int> holesInA = detecHoles(A,endInA,endInAHole);
	std::vector<int> holesInB = detecHoles(B,endInB,endInBHole);

	int fmmdist1 = compute_dist_fast_more_more(endInA,endInB);
	int fmmdist2 = compute_dist_fast_more_more(endInB,endInA);
	int fmmdisthole1 = compute_dist_fast_more_more(endInAHole,endInBHole);
	int fmmdisthole2 = compute_dist_fast_more_more(endInBHole,endInAHole);

	int ffmmdist1 = fmmdist1 > fmmdisthole1 ? fmmdist1 : fmmdisthole1;
	int ffmmdist2 = fmmdist2 > fmmdisthole2 ? fmmdist2 : fmmdisthole2;

	return (ffmmdist1 > ffmmdist2 ? ffmmdist1 : ffmmdist2);
}

/****************************************************************************************************/

int HausdorffDist::pointInSegment(std::vector<std::pair<bool,int> > AEnd,int mid)
{
	int n = AEnd.size();
	int j = 0;
	while (j < n)
	{
		if(!AEnd[j].first)
		{
			if(mid == AEnd[j].second)
				return j;
			else
				j++;
		}
		else
		{
			int start = AEnd[j].second;
			int end = AEnd[j+1].second;

			if(mid >= start && mid <= end)
				return j;
			else
				j += 2;
		}
	}

	return -1;
}

/****************************************************************************************************/

int HausdorffDist::compute_dist_fast_more_more(std::vector<std::pair<bool,int> > AEnd, std::vector<std::pair<bool,int> > BEnd)
{
	int m = AEnd.size();
	int n = BEnd.size();
	std::vector<int> diffs;


	for(int i=0; i<m; i++)
	{
		int diff = INT_MAX;
		int j = 0;

		while (j < n)
		{
			int tmp = INT_MAX;

			if(BEnd[j].first && j+1 < n)
			{
				int start = BEnd[j++].second;
				int diffTos = (AEnd[i].second - start);

				if(diffTos <= 0)
					tmp = diffTos;
				else
				{
					int end = BEnd[j++].second;
					int diffToe = (AEnd[i].second - end);

					if(diffToe <= 0)
						tmp = 0;
					else
						tmp = diffToe;
				}
			}
			else
				tmp = AEnd[i].second-BEnd[j++].second;

			int abstmp = abs(tmp);
			if(abstmp < diff)
				diff = abstmp;

			if(tmp <= 0)
				break;
		}

		diffs.push_back(diff);
	}

	int j = 0;
	while(j < n-1)
	{
		if(BEnd[j].first)
		{
			int end = BEnd[j+1].second;
			if(j+2 < n)
			{
				int nextend = BEnd[j+2].second;
				int mid = (nextend+end)/2;

				int midInA = pointInSegment(AEnd,mid);
				if(midInA != -1)
					diffs.push_back(mid-end);
			}

			j += 2;
		}
		else
		{
			int end = BEnd[j].second;
			int nextend = BEnd[j+1].second;
			int mid = (nextend+end)/2;
			int midInA = pointInSegment(AEnd,mid);
			if(midInA != -1)
				diffs.push_back(mid-end);
			j++;
		}
	}

	int maxdiff = diffs[0];
	for(int i=1; i<diffs.size(); i++)
	{
		if(diffs[i] > maxdiff)
			maxdiff = diffs[i];
	}

	return maxdiff;
}


