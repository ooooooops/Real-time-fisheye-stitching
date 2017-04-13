#include "cylstitcher.h"
#include "config.h"
#include "imgproc.h"
#include "matcher.h"
#include "transform_estimate.h"
#include "blender.h"
#include "match_info.h"
#include "warp.h"
#include <vector>
#include <limits>  

const float SLOPE_PLAIN = 8e-3;

Mat32f CylinderStitcher::build()
{
	calc_feature();
    
	// naively. when changing here, keep mid for CYLINDER
	bundle.identity_idx = imgs.size() >> 1;
	build_warp();
	free_feature();
        
    
	bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
	bundle.update_proj_range();
    
    
	auto ret = bundle.blend();
    
    
	return perspective_correction(ret);
}

void CylinderStitcher::build_warp() 
{
	//GuardedTimer tm("build_warp()");
	int n = imgs.size(), mid = bundle.identity_idx;
    
	for(int i=0;i<n;++i)
		bundle.component[i].homo = Homography::I();


	std::vector<MatchData> matches;		// matches[k]: k,k+1
	PairWiseMatcher pwmatcher(feats);
	matches.resize(n-1);

#pragma omp parallel for schedule(dynamic)
	for(int k=0;k<n - 1;++k)
		matches[k] = pwmatcher.match(k, (k + 1) % n);//KNN Match
        

	std::vector<Homography> bestmat;

	float minslope = std::numeric_limits<float>::max();
	float bestfactor = 1;
	if (n - mid > 1) 
    {
		float newfactor = 1;
		// XXX: ugly
		float slope = update_h_factor(newfactor, minslope, bestfactor, bestmat, matches);
		if (bestmat.empty())
		{
			printf("Failed to find hfactor");
			exit(1);
		}
    
		float centerx1 = 0, centerx2 = bestmat[0].trans2d(0, 0).x;
		float order = (centerx2 > centerx1 ? 1 : -1);
		for(int k=0;k<3;++k) 
		{
			if (fabs(slope) < SLOPE_PLAIN) break;
			newfactor += (slope < 0 ? order : -order) / (5 * pow(2, k));
			slope = update_h_factor(newfactor, minslope, bestfactor, bestmat, matches);
		}
	}
	printf("Best hfactor: %lf\n", bestfactor);


	CylinderWarper warper(bestfactor);
#pragma omp parallel for schedule(dynamic)
	for(int k=0;k<n;++k)
		warper.warp(imgs[k], keypoints[k]);

	// accumulate
	for(int k= mid + 1;k< n;++k)
		bundle.component[k].homo = std::move(bestmat[k - mid - 1]);
	
#pragma omp parallel for schedule(dynamic)
	for(int i=mid - 1; i>= 0; i--) 
	{
		matches[i].reverse();
		MatchInfo info;
		bool succ = TransformEstimation(
				matches[i], keypoints[i + 1], keypoints[i],
				{imgs[i+1].width(), imgs[i+1].height()},
				{imgs[i].width(), imgs[i].height()}).get_transform(&info);
		// Can match before, but not here. This would be a bug.
		if (! succ)
		{
			printf("Failed to match between image %d and %d.", i, i+1);
			exit(1);
		}
		// homo: operate on half-shifted coor
		bundle.component[i].homo = info.homo;
	}
	for(int i=mid - 2;i>=0;i--)
		bundle.component[i].homo = bundle.component[i + 1].homo * bundle.component[i].homo;

	bundle.shift_all_homo();
	bundle.calc_inverse_homo();
}

float CylinderStitcher::update_h_factor(float nowfactor,
		float & minslope,
		float & bestfactor,
		std::vector<Homography>& mat,
		const std::vector<MatchData>& matches) 
{
	const int n = imgs.size(), mid = bundle.identity_idx;
	const int start = mid, end = n, len = end - start;

	std::vector<Mat32f> nowimgs;
	std::vector<std::vector<Vec2D>> nowkpts;
	for(int k=start;k<end;++k) 
	{
		nowimgs.push_back(imgs[k].clone());
		nowkpts.push_back(keypoints[k]);
	}			// nowfeats[0] == feats[mid]

	CylinderWarper warper(nowfactor);
#pragma omp parallel for schedule(dynamic)
	for(int k=0;k<len;++k)
		warper.warp(nowimgs[k], nowkpts[k]);

	std::vector<Homography> nowmat;		// size = len - 1
	nowmat.resize(len - 1);
	bool failed = false;

#pragma omp parallel for schedule(dynamic)
	for(int k=1;k<len;++k) 
	{
		MatchInfo info;
		bool succ = TransformEstimation(
				matches[k - 1 + mid], nowkpts[k - 1], nowkpts[k],
				{nowimgs[k-1].width(), nowimgs[k-1].height()},
				{nowimgs[k].width(), nowimgs[k].height()}).get_transform(&info);

		if (! succ)
			failed = true;
		//error_exit("The two image doesn't match. Failed");
		nowmat[k-1] = info.homo;
	}
	
	if (failed) return 0;

	for(int k=1;k<len - 1;++k)
		nowmat[k] = nowmat[k - 1] * nowmat[k];	// transform to nowimgs[0] == imgs[mid]

	// check the slope of the result image
	Vec2D center2 = nowmat.back().trans2d(0, 0);
	const float slope = center2.y/ center2.x;
	printf("slope: %lf\n", slope);
	if (minslope > fabs(slope)) 
	{
		minslope = fabs(slope);
		bestfactor = nowfactor;
		mat = move(nowmat);
	}
	
	return slope;
}

Mat32f CylinderStitcher::perspective_correction(const Mat32f& img) 
{
	int w = img.width(), h = img.height();
	int refw = imgs[bundle.identity_idx].width();
    int refh = imgs[bundle.identity_idx].height();
	auto homo2proj = bundle.get_homo2proj();
	Vec2D proj_min = bundle.proj_range.min;

	std::vector<Vec2D> corners;
	auto cur = &(bundle.component.front());
	auto to_ref_coor = [&](Vec2D v) 
	{
		v.x *= cur->imgptr->width(), v.y *= cur->imgptr->height();
		Vec homo = cur->homo.trans(v);
		homo.x /= refw, homo.y /= refh;
		homo.x += 0.5 * homo.z, homo.y += 0.5 * homo.z;
		Vec2D t_corner = homo2proj(homo);
		t_corner.x *= refw, t_corner.y *= refh;
		t_corner = t_corner - proj_min;
		corners.push_back(t_corner);
	};
	
	to_ref_coor(Vec2D(-0.5, -0.5));
	to_ref_coor(Vec2D(-0.5, 0.5));
	cur = &(bundle.component.back());
	to_ref_coor(Vec2D(0.5, -0.5));
	to_ref_coor(Vec2D(0.5, 0.5));

	// stretch the four corner to rectangle
	std::vector<Vec2D> corners_std = {Vec2D(0, 0), Vec2D(0, h),
									  Vec2D(w, 0), Vec2D(w, h)};
	Matrix m = getPerspectiveTransform(corners, corners_std);
	Homography inv(m);

	LinearBlender blender;
	blender.add_image(Coor(0,0), Coor(w,h), img,[=](Coor c) -> Vec2D {
		return inv.trans2d(Vec2D(c.x, c.y));
	});
	
	return blender.run();
}

