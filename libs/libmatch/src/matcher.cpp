#include <limits>
#include "matcher.h"

#include "feature.h"
using namespace std;

const float MATCH_REJECT_NEXT_RATIO = 0.8;

#ifdef _MSC_VER
// necessary to define here since flann doesn't provide serialization for size_t as unsigned long long
namespace flann 
{
  namespace serialization 
  {
    BASIC_TYPE_SERIALIZER(size_t);
  }
}
#endif

const int FLANN_NR_KDTREE = 6;

MatchData FeatureMatcher::match() const 
{
	static const float REJECT_RATIO_SQR = MATCH_REJECT_NEXT_RATIO * MATCH_REJECT_NEXT_RATIO;
	//TotalTimer tm("matcheconst int FLANN_NR_KDTREE = 6;r");

	int l1 = feat1.size(), l2 = feat2.size();
	// loop over the smaller one to speed up
	bool rev = l1 > l2;
	const vector<Descriptor> *pf1, *pf2;
	if (rev) 
	{
		swap(l1, l2);
		pf1 = &feat2, pf2 = &feat1;
	} 
	else 
	{
		pf1 = &feat1, pf2 = &feat2;
	}

	MatchData ret;

#pragma omp parallel for schedule(dynamic)
	for(int k=0;k<l1;++k) 
	{
		const Descriptor& i = (*pf1)[k];
		int min_idx = -1;

		float min = numeric_limits<float>::max(),
					next_min = min;
		for(int kk=0;kk<l2;++kk) 
		{
			float dist = i.euclidean_sqr((*pf2)[kk], next_min);
			if (dist < min) 
			{
				next_min = min;
				min = dist;
				min_idx = kk;
			} 
			else 
			{
				update_min(next_min, dist);
			}
		}
		
		if (min > REJECT_RATIO_SQR * next_min)
			continue;

		//m_assert(min_idx != -1);
#pragma omp critical
		ret.data.emplace_back(k, min_idx);
	}
	if (rev)
		ret.reverse();
	return ret;
}

void PairWiseMatcher::build() 
{
	//GuardedTimer tm("BuildTrees");
	for (auto& feat: feats)	
	{
		float* buf = new float[feat.size() * D];
		bufs.emplace_back(buf);

		for(size_t i=0;i<feat.size();++i)
		{
			float* row = buf + D * i;
			memcpy(row, feat[i].descriptor.data(), D * sizeof(float));
		}
		
		flann::Matrix<float> points(buf, feat.size(), D);
		trees.emplace_back(points, flann::KDTreeIndexParams(FLANN_NR_KDTREE));	// TODO param
	}

#pragma omp parallel for schedule(dynamic)
	for(int i=0;i<(int)trees.size();++i)
		trees[i].buildIndex();
}

MatchData PairWiseMatcher::match(int i, int j) const 
{
	static const float REJECT_RATIO_SQR = MATCH_REJECT_NEXT_RATIO * MATCH_REJECT_NEXT_RATIO;
	MatchData ret;
	auto source = feats.at(i),target = feats.at(j);
	auto& t = trees[j];

	float* buf = new float[source.size() * D];
	for(size_t i=0;i<source.size();++i) 
	{
		float* row = buf + D * i;
		memcpy(row, source[i].descriptor.data(), D * sizeof(float));
	}
	
	flann::Matrix<float> query(buf, source.size(), D);

	flann::Matrix<int> indices(new int[source.size() * 2], source.size(), 2);
	flann::Matrix<float> dists(new float[source.size() * 2], source.size(), 2);
	t.knnSearch(query, indices, dists, 2, flann::SearchParams(128));	// TODO param
	for(size_t i=0;i<source.size();++i) 
	{
		int mini = indices[i][0];
		float mind = dists[i][0], mind2 = dists[i][1];
		if (mind > REJECT_RATIO_SQR * mind2)
			continue;
		ret.data.emplace_back(i, mini);
	}
	delete[] indices.ptr();
	delete[] dists.ptr();
	delete[] buf;
	return ret;
}
