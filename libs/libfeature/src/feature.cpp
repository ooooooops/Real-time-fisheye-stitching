
#include "feature.h"

#include "extrema.h"
#include "orientation.h"
#include "sift.h"
#include "dog.h"
#include "brief.h"
#include "imgproc.h"
using namespace std;

const int SIFT_WORKING_SIZE = 800;//working resolution for sift
const int NUM_OCTAVE = 3;
const int NUM_SCALE = 7;

// keep unchanged
const int BRIEF_PATH_SIZE = 9;
const int BRIEF_NR_PAIR = 256;


// return half-shifted image coordinate
vector<Descriptor> FeatureDetector::detect_feature(const Mat32f& img) const 
{
	auto ret = do_detect_feature(img);
	// convert scale-coordinate to half-offset image coordinate
	for (auto& d: ret) 
    {
		d.coor.x = (d.coor.x - 0.5) * img.width();
		d.coor.y = (d.coor.y - 0.5) * img.height();
       // printf("(x,y)=(%f,%f)\n",d.coor.x,d.coor.y);
	}
    //printf("\n");
	return ret;
}

// return [0, 1] coordinate
vector<Descriptor> SIFTDetector::do_detect_feature(const Mat32f& mat) const 
{
	// perform sift at this resolution
	float ratio = SIFT_WORKING_SIZE * 2.0f / (mat.width() + mat.height());
	Mat32f resized(mat.rows() * ratio, mat.cols() * ratio, 3);
	resize(mat, resized);

	ScaleSpace ss(resized, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	OrientationAssign ort(sp, ss, keyp);
	keyp = ort.work();
	SIFT sift(ss, keyp);
	auto descp = sift.get_descriptor();
	return descp;
}

BRIEFDetector::BRIEFDetector() 
{
	pattern.reset(new BriefPattern(	BRIEF::gen_brief_pattern(BRIEF_PATH_SIZE, BRIEF_NR_PAIR)));
}

BRIEFDetector::~BRIEFDetector() {}

vector<Descriptor> BRIEFDetector::do_detect_feature(const Mat32f& mat) const 
{
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	//OrientationAssign ort(sp, ss, keyp);
	//keyp = ort.work();
	BRIEF brief(mat, keyp, *pattern);

	auto ret = brief.get_descriptor();
    
	return ret;
}


