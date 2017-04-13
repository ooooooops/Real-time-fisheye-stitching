
#pragma once
#include "geometry.h"
#include "mat.h"
#include "feature.h"


class CylinderProject {
	public:
		Vec center;
		int r;
		int sizefactor;

		CylinderProject(int m_r, const Vec& m_center, int m_size):
			center(m_center), r(m_r),
			sizefactor(m_size){}

		CylinderProject(const CylinderProject&) = delete;
		CylinderProject& operator = (const CylinderProject&) = delete;

		Mat32f project(const Mat32f& img, std::vector<Vec2D>& pts) const;

	private:
		// return (angle with x) and (angle vertical)
		Vec2D proj(const Vec& p) const;

		inline Vec2D proj(const Vec2D& p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec2D proj_r(const Vec2D& p) const;
};

class CylinderWarper {
	public:
		const double h_factor;
		explicit CylinderWarper(double m_hfactor):
			h_factor(m_hfactor) {}

		void warp(Mat32f& mat, std::vector<Vec2D>& kpts) const;

		inline void warp(Mat32f& mat) const {
			std::vector<Vec2D> a;
			warp(mat, a);
		}
};

