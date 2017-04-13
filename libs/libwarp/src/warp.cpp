#include "warp.h"
#include "imgproc.h"
using namespace std;

//# focal length in 35mm format. used in CYLINDER mode
float FOCAL_LENGTH =37;//from jk's camera
//float FOCAL_LENGTH 24; // my sony camera
//float FOCAL_LENGTH 25.83; //for Mi3 phone, focal length * 7.38

#define between(a, b, c) (((a) >= (b)) && ((a) <= (c - 1)))

Vec2D CylinderProject::proj(const Vec& p) const {
	double x = atan((p.x - center.x) / r);
	double y = (p.y - center.y) / (hypot(p.x - center.x, r));
	return Vec2D(x, y);
}

Vec2D CylinderProject::proj_r(const Vec2D& p) const {
	double x = r * tan(p.x) + center.x;
	double y = p.y * r / cos(p.x) + center.y;
	return Vec2D(x, y);
}

Mat32f CylinderProject::project(const Mat32f& img, vector<Vec2D>& pts) const {
	Vec2D min(numeric_limits<double>::max(), numeric_limits<double>::max()),
		  max(0, 0);
	for(int i=0;i<img.height();++i) 
		for(int j=0;j< img.width();++j) 
		{			// TODO finally: only use rect corners
			Vec2D newcoor = proj(Vec2D(j, i));
			min.update_min(newcoor), max.update_max(newcoor);
		}

	max = max * sizefactor, min = min * sizefactor;
	Vec2D realsize = max - min,
		  offset = min * (-1);
	Coor size = Coor(realsize.x, realsize.y);
	double sizefactor_inv = 1.0 / sizefactor;

	Mat32f mat(size.y, size.x, 3);
	fill(mat, Color::NO);
#pragma omp parallel for schedule(dynamic)
	for(int i=0;i<mat.height();++i) 
		for(int j=0;j<mat.width();++j) 
		{
			Vec2D oricoor = proj_r((Vec2D(j, i) - offset) * sizefactor_inv);
			if (between(oricoor.x, 0, img.width()) && between(oricoor.y, 0, img.height())) {
				Color c = interpolate(img, oricoor.y, oricoor.x);
				float* p = mat.ptr(i, j);
				p[0] = c.x, p[1] = c.y, p[2] = c.z;
			}
		}

	for (auto & f : pts) {
		Vec2D coor(f.x + img.width() / 2, f.y + img.height() / 2);
		f = proj(coor) * sizefactor + offset;
		f.x -= mat.width() / 2;
		f.y -= mat.height() / 2;
	}
	return mat;
}

void CylinderWarper::warp(Mat32f& mat, std::vector<Vec2D>& kpts) const {
	// 43.266 = hypot(36, 24)
	int r = hypot(mat.width(), mat.height()) * (FOCAL_LENGTH / 43.266);
	Vec cen(mat.width() / 2, mat.height() / 2 * h_factor, r);
	CylinderProject cyl(r, cen, r);
	mat = cyl.project(mat, kpts);
}

