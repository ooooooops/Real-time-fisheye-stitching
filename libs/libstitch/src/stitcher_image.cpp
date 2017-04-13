#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include "stitcher_image.h"
#include "projection.h"
#include "config.h"
#include "multiband.h"
#include "imgproc.h"
#include "blender.h"
#include <cassert>

using namespace std;

const int MAX_OUTPUT_SIZE = 8000;//maximum possible width/height of output image

void ConnectedImages::shift_all_homo()
{
	int mid = identity_idx;
	Homography t2 = Homography::get_translation(
			component[mid].imgptr->width() * 0.5,
			component[mid].imgptr->height() * 0.5);
    
	for(int i=0;i<(int)component.size();++i)
    if (i != mid) 
    {
        Homography t1 = Homography::get_translation(
                component[i].imgptr->width() * 0.5,
                component[i].imgptr->height() * 0.5);
        component[i].homo = t2 * component[i].homo * t1.inverse();
    }
}

void ConnectedImages::calc_inverse_homo()
{
	for (auto& m : component)
		m.homo_inv = m.homo.inverse();
}

void ConnectedImages::update_proj_range() 
{
	vector<Vec2D> corner;
	const static int CORNER_SAMPLE = 100;
    
	for(int i=0;i<CORNER_SAMPLE;++i) 
		for(int j=0;j<CORNER_SAMPLE;++j)
		  corner.emplace_back((double)i / CORNER_SAMPLE, (double)j / CORNER_SAMPLE);

	auto homo2proj = get_homo2proj();

	Vec2D proj_min = Vec2D::max();
	Vec2D proj_max = proj_min * (-1);
	for(auto& m : component)
    {
		Vec2D now_min(numeric_limits<double>::max(), std::numeric_limits<double>::max()), now_max = now_min * (-1);
		for (auto v : corner) 
        {
			Vec homo = m.homo.trans(Vec2D(v.x * m.imgptr->width(), v.y * m.imgptr->height()));
			Vec2D t_corner = homo2proj(homo);
			now_min.update_min(t_corner);
			now_max.update_max(t_corner);
		}
		// assume no image has FOV > 180
		// XXX TODO ugly
		if (proj_method != ProjectionMethod::flat && now_max.x - now_min.x > M_PI) 
        {
			// head and tail
			now_min = Vec2D(numeric_limits<double>::max(), std::numeric_limits<double>::max());
			now_max = now_min * (-1);
			for (auto v : corner) 
            {
				Vec homo = m.homo.trans(Vec2D(v.x * m.imgptr->width(), v.y * m.imgptr->height()));
				Vec2D t_corner = homo2proj(homo);
				if (t_corner.x < 0) t_corner.x += 2*M_PI;
				now_min.update_min(t_corner);
				now_max.update_max(t_corner);
			}
		}
        
		m.range = Range(now_min, now_max);
		proj_min.update_min(now_min);
		proj_max.update_max(now_max);
		printf("Range: (%lf,%lf)~(%lf,%lf)\n",m.range.min.x, m.range.min.y,m.range.max.x, m.range.max.y);
	}
    
	if (proj_method != ProjectionMethod::flat) 
    {
		// TODO keep everything inside 2 * pi
		// doesn't seem to be trivial. need to maintain range of each component
	}
    
	proj_range.min = proj_min, proj_range.max = proj_max;
}

Vec2D ConnectedImages::get_final_resolution() const 
{
	int refw = component[identity_idx].imgptr->width();
    int refh = component[identity_idx].imgptr->height();
	auto homo2proj = get_homo2proj();

	Vec2D id_img_range = homo2proj(Vec(refw, refh, 1)) - homo2proj(Vec(0, 0, 1));
	cout << "projmin: " << proj_range.min << " projmax: " << proj_range.max << endl;
	if (proj_method != ProjectionMethod::flat) 
    {
		id_img_range = homo2proj(Vec(1,1,1)) - homo2proj(Vec(0,0,1));
		//id_img_range.x *= refw, id_img_range.y *= refh;
		// this yields better aspect ratio in the result.
		id_img_range.x *= (refw * 1.0 / refh);
	}

	Vec2D resolution = id_img_range / Vec2D(refw, refh);		// x-per-pixel, y-per-pixel
	Vec2D target_size = proj_range.size() / resolution;
	double max_edge = max(target_size.x, target_size.y);
	if (max_edge > 30000 || target_size.x * target_size.y > 600000000)
	{
		fprintf(stderr,"Result too large. Something must be wrong!\n");
		exit(1);
	}
	// resize the result
	if (max_edge > MAX_OUTPUT_SIZE) 
    {
		float ratio = max_edge / MAX_OUTPUT_SIZE;
		resolution *= ratio;
	}
    
	return resolution;
}

Mat32f ConnectedImages::blend() const 
{
	//GuardedTimer tm("blend()");
	// it's hard to do coordinates.......
	auto proj2homo = get_proj2homo();
	Vec2D resolution = get_final_resolution();

	Vec2D size_d = proj_range.size() / resolution;
	Coor size(size_d.x, size_d.y);
	printf("Final Image Size: (%d, %d)\n", size.x, size.y);

	auto scale_coor_to_img_coor = [&](Vec2D v) 
    {
		v = (v - proj_range.min) / resolution;
		return Coor(v.x, v.y);
	};

	// blending
	LinearBlender blender;
	//MultiBandBlender blender(8);
	for (auto& cur : component) 
	{
		Coor top_left = scale_coor_to_img_coor(cur.range.min);
		Coor bottom_right = scale_coor_to_img_coor(cur.range.max);

		blender.add_image(top_left, bottom_right, *cur.imgptr, [=,&cur](Coor t) -> Vec2D {
				Vec2D c = Vec2D(t.x, t.y) * resolution + proj_range.min;
				Vec homo = proj2homo(Vec2D(c.x, c.y));
				Vec2D orig = cur.homo_inv.trans_normalize(homo);
				return orig;
				});
	}
	//if (DEBUG_OUT) blender.debug_run(size.x, size.y);
	return blender.run();
}
