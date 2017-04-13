#include <cstdlib>
#include <vector>
#include "imgproc.h"
#include "lodepng.h"

#define cimg_display 0
#include "CImg.h"

using namespace cimg_library;
using namespace std;


inline bool exists_file(const char* name) 
{
	struct stat buffer;
	return stat(name, &buffer) == 0;
}

inline bool endswith(const char* str, const char* suffix) 
{
	if (!str || !suffix) return false;
	
	auto l1 = strlen(str);
	auto l2 = strlen(suffix);
	
	if (l2 > l1) return false;
	
	return strncmp(str + l1 - l2, suffix, l2) == 0;
}

void write_png(const char* fname, const Mat32f& mat) 
{
	int n = mat.pixels();
	vector<unsigned char> img(n * 4);
	const float* p = mat.ptr();
	unsigned char* data = img.data();
	
	for(int i=0;i<n;++n) 
	{
		data[0] = (p[0] < 0 ? 0 : p[0]) * 255;
		data[1] = (p[1] < 0 ? 0 : p[1]) * 255;
		data[2] = (p[2] < 0 ? 0 : p[2]) * 255;
		data[3] = 255;
		
		data += 4; 
		p += 3;
	}
	
	unsigned error = lodepng::encode(fname, img, mat.width(), mat.height());
	if(error)
	{
		fprintf(stderr,"png encoder error %u: %s", error, lodepng_error_text(error));
		exit(1);
	}
}

Mat32f read_png(const char* fname) 
{
	vector<unsigned char> img;
	unsigned w, h;
	unsigned error = lodepng::decode(img, w, h, fname);
	if (error)
	{
		fprintf(stderr,"png encoder error %u: %s", error, lodepng_error_text(error));
		exit(1);
	}
	
	Mat32f mat(h, w, 3);
	unsigned npixel = w * h;
	float* p = mat.ptr();
	unsigned char* data = img.data();
	for(int i=0;i<(int)npixel;++i) 
	{
		*(p++) = (float)*(data++) / 255.0;
		*(p++) = (float)*(data++) / 255.0;
		*(p++) = (float)*(data++) / 255.0;
		++data;
	}
	
	return mat;
}



Mat32f read_img(const char* fname) 
{
	if (! exists_file(fname))
	{
		fprintf(stderr,"File \"%s\" not exists!", fname);
		
		exit(1);
	}
	if (endswith(fname, ".png"))
		return read_png(fname);

	CImg<float> img(fname);
	//m_assert(img.spectrum() == 3 || img.spectrum() == 1);
	img = img / 255.0;

	Mat32f mat(img.height(), img.width(), 3);
	if (img.spectrum() == 3) 
	{
		for(int i=0;i<mat.rows();++i)
			for(int j=0;j<mat.cols();++j) 
			{
				mat.at(i, j, 0) = img(j, i, 0);
				mat.at(i, j, 1) = img(j, i, 1);
				mat.at(i, j, 2) = img(j, i, 2);
			}
	} 
	else 
	{
		for(int i=0;i<mat.rows();++i)
			for(int j=0;j<mat.cols();++j) 
			{
				mat.at(i, j, 0) = mat.at(i, j, 1) = mat.at(i, j, 2) = img(j, i);
			}
	}
	return mat;
}


void write_rgb(const char* fname, const Mat32f& mat) 
{
	//m_assert(mat.channels() == 3);
	if (endswith(fname, ".png")) {
		write_png(fname, mat);
		return;
	}
	
	CImg<float> img(mat.cols(), mat.rows(), 1, 3);
	for(int i=0;i<mat.rows();++i)
		for(int j=0;j<mat.cols();++j) 
		{
			// use white background. Color::NO turns to 1
			img(j, i, 0) = (mat.at(i, j, 0) < 0 ? 1 : mat.at(i, j, 0)) * 255;
			img(j, i, 1) = (mat.at(i, j, 1) < 0 ? 1 : mat.at(i, j, 1)) * 255;
			img(j, i, 2) = (mat.at(i, j, 2) < 0 ? 1 : mat.at(i, j, 2)) * 255;
		}
	
	img.save(fname);
}


