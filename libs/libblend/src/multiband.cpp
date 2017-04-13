

#include "multiband.h"
#include "imgproc.h"
#include "gaussian.h"


void MultiBandBlender::add_image(const Coor& upper_left,const Coor& bottom_right,
		const Mat32f &img,std::function<Vec2D(Coor)> coor_func) 
{
	Range range{upper_left, bottom_right};
	Mat<WeightedPixel> wimg(range.height(), range.width(), 1);
	for (int i = 0; i < range.height(); ++i)
		for (int j = 0; j < range.width(); ++j) 
		{
			Coor target_coor{j + range.min.x, i + range.min.y};
			Vec2D orig_coor = coor_func(target_coor);
			Color c = interpolate(img, orig_coor.y, orig_coor.x);
			wimg.at(i, j).c = c;
			if (c.x < 0) 
			{	// Color::NO
				wimg.at(i, j).w = 0;
			} 
			else 
			{
				orig_coor.x = orig_coor.x / img.width() - 0.5;
				orig_coor.y = orig_coor.y / img.height() - 0.5;
				wimg.at(i, j).w = std::max((0.5f - fabs(orig_coor.x)) * (0.5f - fabs(orig_coor.y)),	0.0) + 1e-6;
				// ext? eps?
			}
		}
	images.emplace_back(ImageToBlend{range, std::move(wimg)});
	target_size.update_max(bottom_right);
}

Mat32f MultiBandBlender::run() {
	update_weight_map();
	Mat32f target(target_size.y, target_size.x, 3);
	fill(target, Color::NO);

	next_lvl_images.resize(images.size());
	for (int level = 0; level < band_level; level ++) {
		create_next_level(level);
		//debug_level(level);
		for(int i=0;i<target_size.y;++i) 
			for(int j=0;j<target_size.x;++j) {
			Color isum(0, 0, 0);
			float wsum = 0;
			for(size_t imgid=0;imgid<images.size();++imgid)  {
				auto& img_cur = images[imgid];
				if (img_cur.range.contain(i, j)) {
					auto& ccur = img_cur.color_on_target(j, i);
					if (ccur.get_min() < 0) continue;
					auto & img_next = next_lvl_images[imgid];
					float w = img_cur.weight_on_target(j, i);
					if (w <= 0) continue;
					isum += (ccur - img_next.color_on_target(j, i)) * w;
					wsum += w;
				}
			}
			if (wsum < 1e-6)
				continue;
			isum /= wsum;
			float* p = target.ptr(i, j);
			if (*p < 0) {
				isum.write_to(p);
			} else {
				p[0] += isum.x, p[1] += isum.y, p[2] += isum.z;
			}
		}
		swap(next_lvl_images, images);
	}

	// XXX
	for(int i=0;i<target.rows();++i)
		for(int j=0;j<target.cols();++j) 
		{
			float* p = target.ptr(i, j);
			update_min(p[0], 1.0f);
			update_min(p[1], 1.0f);
			update_min(p[2], 1.0f);
		}
		
	return target;
}

void MultiBandBlender::update_weight_map() {
	for(int i=0;i<target_size.y;++i) 
		for(int j=0;j<target_size.x;++j)
		{
			float max = 0;
			for (auto& img : images)
				if (img.range.contain(i, j))
					update_max(max, img.weight_on_target(j, i));
			max -= 1e-6;
			for (auto& img : images) if (img.range.contain(i, j)) {
				float& w = img.weight_on_target(j, i);
				w = (w >= max);
			}
		}
}

void MultiBandBlender::create_next_level(int level) 
{
	//m_assert(next_lvl_images.size() == images.size());
	if (level == band_level - 1) 
	{
		for(size_t k=0;k<images.size();++k) 
		{
			auto& img = images[k].img;
			next_lvl_images[k].range = images[k].range;
			// don't have to do this when band_level > 1
			auto& wimg = (next_lvl_images[k].img = Mat<WeightedPixel>(img.rows(), img.cols(), 1));
			for(int i=0;i<img.rows();++i) 
				for(int j=0;j<img.cols();++i)
				{
					wimg.at(i, j).c = Color::BLACK;
					wimg.at(i, j).w = 0;
				}
			//memset(wimg.ptr(), 0, sizeof(float) * 4 * img.rows() * img.cols());
		}
	} 
	else 
	{
		GaussianBlur blurer(sqrt(level * 2 + 1.0) * 3);
		for(size_t i=0;i<images.size();++i) 
		{
			next_lvl_images[i].range = images[i].range;
			next_lvl_images[i].img = blurer.blur(images[i].img);
		}
	}
}

void MultiBandBlender::debug_level(int level) const {
	int imgid = 0;
	for (auto& t: next_lvl_images) 
	{
		auto& wimg = t.img;
		Mat32f img(wimg.rows(), wimg.cols(), 3);
		Mat32f weight(wimg.rows(), wimg.cols(), 3);
		for(int i=0;i<wimg.rows();++i) 
			for(int j=0;j<wimg.cols();++j) 
			{
				wimg.at(i, j).c.write_to(img.ptr(i, j));
				float* p = weight.ptr(i, j);
				p[0] = p[1] = p[2] = wimg.at(i, j).w;
		    }
		
		char buffer1[100]="";
		char buffer2[100]="";
		
		sprintf(buffer1,"log/multiband%d-%d.jpg", imgid, level);
		write_rgb(buffer1, img);
		
		sprintf(buffer2,"log/multibandw%d-%d.jpg", imgid, level);
		write_rgb(buffer2, weight);
		
		imgid ++;
	}
}

