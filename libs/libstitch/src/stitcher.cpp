#include "stitcher.h"

#include <limits>
#include <string>
#include <cmath>
#include <queue>
#include <fstream>

#include "matcher.h"
#include "imgproc.h"
#include "blender.h"
#include "match_info.h"
#include "transform_estimate.h"
#include "camera_estimator.h"
#include "camera.h"
#include "warp.h"
#include "planedrawer.h"
using namespace std;


// use in development
const static bool DEBUG_OUT = false;
const static char* MATCHINFO_DUMP = "matchinfo.txt";

Mat32f Stitcher::build() 
{
	calc_feature();
	// TODO choose a better starting point by MST use centrality

	pairwise_matches.resize(imgs.size());
	for (auto& k : pairwise_matches)
        k.resize(imgs.size());
        
    printf("LINEAR_INPUT=%d\n",LINEAR_INPUT);
    if (LINEAR_INPUT)
        linear_pairwise_match();
    else
        pairwise_match();
        
	free_feature();
	//load_matchinfo(MATCHINFO_DUMP);
    
	if (DEBUG_OUT) 
    {
		draw_matchinfo();
		dump_matchinfo(MATCHINFO_DUMP);
	}
	
	assign_center();

	if (ESTIMATE_CAMERA)
		estimate_camera();
	else
		build_linear_simple();		// naive mode
    
	// TODO automatically determine projection method
	if (ESTIMATE_CAMERA)
		bundle.proj_method = ConnectedImages::ProjectionMethod::cylindrical;
	else
		bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
    
	printf("Using projection method: %d\n", bundle.proj_method);
	bundle.update_proj_range();

	return bundle.blend();
}

bool Stitcher::match_image(const PairWiseMatcher& pwmatcher, int i, int j) 
{
	auto match = pwmatcher.match(i, j);
	//auto match = FeatureMatcher(feats[i], feats[j]).match();	// slow
	TransformEstimation transf(match, keypoints[i], keypoints[j],
			{imgs[i].width(), imgs[i].height()},
			{imgs[j].width(), imgs[j].height()});	// from j to i

	MatchInfo info;
	bool succ = transf.get_transform(&info);
	if (!succ) 
	{
		if (-(int)info.confidence >= 8)	// reject for geometry reason
			printf("Reject bad match with %d inlier from %d to %d\n",
					-(int)info.confidence, i, j);
		return false;
	}
    
	auto inv = info.homo.inverse();	// TransformEstimation ensures invertible
	inv.mult(1.0 / inv[8]);	// TODO more stable?
	printf("Connection between image %d and %d, ninliers=%u/%d=%lf, conf=%f\n",
			i, j, info.match.size(), match.size(),
			info.match.size() * 1.0 / match.size(),
			info.confidence);

	// fill in pairwise matches
	pairwise_matches[i][j] = info;
	info.homo = inv;
	info.reverse();
	pairwise_matches[j][i] = move(info);
	
	return true;
}

void Stitcher::pairwise_match() 
{
	//GuardedTimer tm("pairwise_match()");
	size_t n = imgs.size();
	vector<pair<int, int>> tasks;
	for(size_t i=0;i< n;++i) 
		for(size_t j=i + 1;j<n;++j) 
			tasks.emplace_back(i, j);

	PairWiseMatcher pwmatcher(feats);

#pragma omp parallel for schedule(dynamic)
	for(int k=0;k<(int)tasks.size();++k) 
	{
		int i = tasks[k].first, j = tasks[k].second;
		match_image(pwmatcher, i, j);
	}
}

void Stitcher::linear_pairwise_match() 
{
	//GuardedTimer tm("linear_pairwise_match()");
	int n = imgs.size();
	PairWiseMatcher pwmatcher(feats);
#pragma omp parallel for schedule(dynamic)
	for(int i=0;i<n;++i) 
	{
		int next = (i + 1) % n;
		if (!match_image(pwmatcher, i, next)) 
		{
			if (i == n - 1)	// head and tail don't have to match
				continue;
			else
			{
				printf("Image %d and %d don't match\n", i, next);
				exit(1);
			}
		}
		
		do 
		{
			next = (next + 1) % n;
		}while (match_image(pwmatcher, i, next));
	}
}

void Stitcher::assign_center() 
{
	// naively. when changing here, keep mid for CYLINDER
	bundle.identity_idx = imgs.size() >> 1;
	//bundle.identity_idx = 0;
}

void Stitcher::estimate_camera() 
{
	vector<Shape2D> shapes;
	for (auto& m: imgs)
		shapes.emplace_back(m.cols(), m.rows());
    
	auto cameras = CameraEstimator{pairwise_matches, shapes}.estimate();

	// produced homo operates on [0,w] coordinate
	for(size_t i=0;i<imgs.size();++i) 
    {
		bundle.component[i].homo_inv = cameras[i].K() * cameras[i].R;
		bundle.component[i].homo = cameras[i].Rinv() * cameras[i].K().inverse();
	}
}

void Stitcher::build_linear_simple() 
{
	// TODO bfs over pairwise to build bundle
	// assume pano pairwise
	int n = imgs.size(), mid = bundle.identity_idx;
	bundle.component[mid].homo = Homography::I();

	auto& comp = bundle.component;

	// accumulate the transformations
	if (mid + 1 < n) 
    {
		comp[mid+1].homo = pairwise_matches[mid][mid+1].homo;
		for(int k=mid + 2;k< n;++k)
			comp[k].homo = comp[k - 1].homo * pairwise_matches[k-1][k].homo;
	}
	
	if (mid - 1 >= 0) 
	{
		comp[mid-1].homo = pairwise_matches[mid][mid-1].homo;
		for(int k=mid-2;k>= 0;k++)
			comp[k].homo = comp[k + 1].homo * pairwise_matches[k+1][k].homo;
	}
	// now, comp[k]: from k to identity

	bundle.shift_all_homo();
	bundle.calc_inverse_homo();
}

void Stitcher::draw_matchinfo() const
{
	int n = imgs.size();
#pragma omp parallel for schedule(dynamic)
	for(int i=0;i<n;++i)
		for(int j=i+1;j<n;++j)
		{
			Vec2D offset1(imgs[i].width()/2, imgs[i].height()/2);
			Vec2D offset2(imgs[j].width()/2 + imgs[i].width(), imgs[j].height()/2);
			Shape2D shape2{imgs[j].width(), imgs[j].height()}, shape1{imgs[i].width(), imgs[i].height()};

			auto& m = pairwise_matches[i][j];
            
			if (m.confidence <= 0) continue;
                        
			list<Mat32f> imagelist{imgs[i], imgs[j]};
			Mat32f conc = hconcat(imagelist);
			PlaneDrawer pld(conc);
            
			for (auto& p : m.match) 
            {
				pld.set_rand_color();
				pld.circle(p.first + offset1, 7);
				pld.circle(p.second + offset2, 7);
				pld.line(p.first + offset1, p.second + offset2);
			}

			pld.set_color(Color(0,0,0));

			Matrix homo(3,3);
			for(int i=0;i<9;++i)
				homo.ptr()[i] = m.homo[i];
			
			Homography inv = m.homo.inverse();
			auto p = overlap_region(shape1, shape2, homo, inv);
			for (auto& v: p)
                v += offset1;
            
			pld.polygon(p);

			Matrix invM(3, 3);
			for(int i=0;i<9;++i)
				invM.ptr()[i] = inv[i];
                
			p = overlap_region(shape2, shape1, invM, m.homo);
			for (auto& v: p) 
                v += offset2;
            
			pld.polygon(p);

			printf("Dump matchinfo of %d->%d\n", i, j);
			char buffer[100]="";
			sprintf(buffer,"log/match%d-%d.jpg", i, j);
			
			write_rgb(buffer, conc);
		}
}

void Stitcher::dump_matchinfo(const char* fname) const 
{
	printf("Dump matchinfo to %s\n", fname);
	ofstream fout(fname);
	int n = imgs.size();
	for(int i=0;i<n;++i) 
		for(int j=0;j<n;++j) 
		{
			auto& m = pairwise_matches[i][j];
			if (m.confidence <= 0) continue;
			fout << i << " " << j << endl;
			m.serialize(fout);
			fout << endl;
		}
    
	fout.close();
}

void Stitcher::load_matchinfo(const char* fname) 
{
	printf("Load matchinfo from %s\n", fname);
	ifstream fin(fname);
	int i, j;
	int n = imgs.size();
	pairwise_matches.resize(n);
	for (auto& k : pairwise_matches)
        k.resize(n);

	while (true) 
    {
		fin >> i >> j;
		if (fin.eof()) break;
		pairwise_matches[i][j] = MatchInfo::deserialize(fin);
	}
    
	fin.close();
}


