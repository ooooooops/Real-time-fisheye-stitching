
#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>	 
#include <cassert>
#include <stdio.h>
#include <opencv2/opencv.hpp>


#include "mat.h"


#include "extrema.h"
#include "matcher.h"
#include "orientation.h"

#include "config.h"
#include "geometry.h"
#include "imgproc.h"
#include "planedrawer.h"
#include "polygon.h"
#include "cylstitcher.h"
#include "match_info.h"
#include "stitcher.h"
#include "transform_estimate.h"
#include "warp.h"
#include "imgproc.h"
#include "lut.h"



void stitch(std::vector<Mat32f>& imgs, Mat32f& res);

typedef struct{
	cv::Mat* img;
	int type;
	
	int *crop_minx , *crop_maxx , *crop_miny , *crop_maxy ;
	int *fill_minx , *fill_maxx , *fill_miny , *fill_maxy ;
}TrackbarParams;
