

#include "blender.h"

#include <iostream>
#include "config.h"
#include "imgproc.h"
#include "lut.h"
using namespace std;


void LinearBlender::add_image(
			const Coor& upper_left,
			const Coor& bottom_right,
			const Mat32f &img,
			std::function<Vec2D(Coor)> coor_func) {
	images.emplace_back(ImageToBlend{Range{upper_left, bottom_right}, img, coor_func});
	target_size.update_max(bottom_right);
}

Mat32f LinearBlender::run() {

	/*//////////////////test/////////////////////*/
	{
		int mm = 0;
		for(auto& img: images)
		{
			char buf[256];
			sprintf(buf, "check%d.jpg", mm++);
			write_rgb(buf,img.img);
		}
	}
	//////end      images中保存的是柱面投影后的图
	
	Mat32f target(target_size.y, target_size.x, 3);
	if (CYLINDER > 0)
	{
		if(1 == images.size()) lut.resize(target_size.y, target_size.x);
		else lut_coor.resize(target_size.y, target_size.x);
	}
	else{
		lut.resize(target_size.y, target_size.x);
	}
	fill(target, Color::BLACK);
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < target.height(); i ++) {
		float *row = target.ptr(i);
		for (int j = 0; j < target.width(); j ++) {
			Color isum = Color::BLACK;
			float wsum = 0;
			for (int n = 0; n < images.size(); n++)
			{
				auto& img = images[n];
				double width = img.img.cols(), height = img.img.rows();
				if (img.range.contain(i, j)) {
					Vec2D img_coor = img.map_coor(i, j);
					if (!img_coor.isNaN()) {
						float r = img_coor.y, c = img_coor.x;
						auto color = interpolate(img.img, r, c);
						if (color.x < 0) continue;
						float w;
						if (LINEAR_INPUT)
							// x-axis linear interpolation
							w = 0.5 - fabs(c / img.img.width() - 0.5);
						else
						    w = (0.5 - fabs(c / img.img.width() - 0.5)) * (0.5 - fabs(r / img.img.height() - 0.5));

						isum += color * w;
						wsum += w;

						if (CYLINDER > 0){
							//printf("CYLINDER\n");
							if(1 != images.size())
							{
								LUTTriple t;
								t.coor = img_coor;
								t.id = n;
								t.w = w;
								if(1 == n) t.coor.rot90_counter_clock(width, height);
								else if(2 == n) t.coor.rot90_clock(width, height);
								else if(3 == n) t.coor.rot180(width, height);
								lut_coor.ptr(i, j)->ele.push_back(t);
							}
							else
							{
								lut.ptr(i, j)->ele.resize(lut_coor.ptr(r, c)->ele.size());
								lut.ptr(i, j)->ele.insert(lut.ptr(i, j)->ele.begin(), lut_coor.ptr(r, c)->ele.begin(), 
									lut_coor.ptr(r, c)->ele.end());
							}
						}
						else
						{
							//printf("CYLINDER ELSE\n");
							LUTTriple t;
							t.coor = img_coor;
							t.id = n;
							t.w = w;
							//printf("%d:%g,%g\n",n,width,height);
							if(1 == n) t.coor.rot90_counter_clock(width, height);
							else if(2 == n) t.coor.rot90_clock(width, height);
							else if(3 == n) t.coor.rot180(width, height);
							lut.ptr(i, j)->ele.push_back(t);
						}
					}
				}
			}
			if (wsum > 0)	// keep original Color::NO
				(isum / wsum).write_to(row + j * 3);
		}
	}
	return target;
}

void LinearBlender::debug_run(int w, int h) {
#pragma omp parallel for schedule(dynamic)
	for(int k=0;k<(int)images.size();++k) {
		auto& img = images[k];
		Mat32f target(h, w, 3);
		fill(target, Color::NO);
		for (int i = 0; i < target.height(); i ++) {
			float *row = target.ptr(i);
			for (int j = 0; j < target.width(); j ++) {
				Color isum = Color::BLACK;
				if (img.range.contain(i, j)) {
					Vec2D img_coor = img.map_coor(i, j);
					if (!img_coor.isNaN()) {
						float r = img_coor.y, c = img_coor.x;
						isum = interpolate(img.img, r, c);
					}
				}
				isum.write_to(row + j * 3);
			}
		}
		
		printf("Debug rendering %02d image\n", k);
		char buffer[100]="";
		sprintf(buffer,"log/blend-%02d.jpg", k);
		write_rgb(buffer, target);
	}
}

