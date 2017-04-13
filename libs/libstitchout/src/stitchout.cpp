
#include "stitchout.h"

int  CROP = 1;


void init_config() 
{
//[general modes]: if no modes is set, will use naive mode
	CYLINDER =0;//default 0	
	ESTIMATE_CAMERA = 1;//default 1
	TRANS = 0;
	
	int flag=CYLINDER + TRANS + ESTIMATE_CAMERA;
	
	printf("flag %d, int(CYLINDER)= %d, int(TRANS)=%d, int(ESTIMATE_CAMERA)=%d\n",flag,CYLINDER,TRANS,ESTIMATE_CAMERA);
	
	if (flag >= 2)
	{
		fprintf(stderr,"You set too many modes...\n");
		exit(1);
	}
	
	if (CYLINDER > 0)
	{
		printf("Run with cylinder mode.\n");
	} 
	else if (TRANS > 0)
	{
		printf("Run with translation mode.\n");
	}
	else if (ESTIMATE_CAMERA > 0)
	{
		printf("Run with camera estimation mode.\n");
	}
	else
		printf("Run with naive mode.\n");

	//if no modes above is set, use naive mode
	LINEAR_INPUT = 0;// set this option when input is ordered
	if (!LINEAR_INPUT && !ESTIMATE_CAMERA)
	{
		fprintf(stderr,"Require LINEAR_INPUT under this mode!\n");
		exit(1);
	}

//[keypoint related parameters]:    
	SCALE_FACTOR = 1.4142135623;
	GAUSS_SIGMA = 1.4142135623;
	GAUSS_WINDOW_FACTOR = 4;//larger value gives less feature
}

Mat32f Mat2Mat32f_(cv::Mat& mat)
{
	//cv::Mat temp;
	//mat.convertTo(temp, CV_32FC3, 1.0/255);
	
	int w = mat.cols;
	int h = mat.rows;
	//int c = 3;
	
	Mat32f res(h, w, 3);

	//#define MAT(ROW, COL) ((float*)(temp.ptr<float>((ROW)) + (COL)* sizeof(float) * c))
	for(int y = 0; y < h; y++)
	{
		float *row_dst = res.ptr(y);
		float *row_src = mat.ptr<float>(y);
		for(int x = 0; x < w; x++)
		{
			*(row_dst + x * 3 + 0) = *(row_src + x * 3 + 2);
			*(row_dst + x * 3 + 1) = *(row_src + x * 3 + 1);
			*(row_dst + x * 3 + 2) = *(row_src + x * 3 + 0);
		}
	}
	
	return res;
}

void Mat32f2Mat_(const Mat32f& src, cv::Mat& dst)
{
	
	int w = src.width();
	int h = src.height();
	int c = src.channels();
	
	dst = cv::Mat::zeros(h,w,CV_32FC3);
	
	
	#define IM(ROW, COL) (float*)(dst.data + dst.step  * (ROW) + (COL)* sizeof(float) * 3)
	
	for(int y = 0; y < h; y++)
	{
		const float *row = src.ptr(y);
		for(int x = 0; x < w; x++)
		{
			*(IM(y, x) + 2) = *(row + x * 3);
			*(IM(y, x) + 1) = *(row + x * 3 + 1);
			*(IM(y, x) + 0) = *(row + x * 3 + 2);
		}
	}
}

inline void lut_stitch(std::vector<Mat32f>& imgs, Mat32f& res)
{
	
	//cv::Mat im;
	//Mat32f2Mat_(imgs[4], im);
	//cv::imshow("fill", im);cv::waitKey();
	
	int w = lut.cols, h = lut.rows;

	res.reset(h, w, 3);
	fill(res, Color::BLACK);
	
	#pragma omp parallel for schedule(dynamic)
	for(int y = 0; y < h; y++)
	{
		//#pragma omp parallel for schedule(dynamic)//openmp at this place will reduce fps
		for(int x = 0; x < w; x++)
		{
			IMGPT* impt = lut.ptr(y, x);
			float* p = res.ptr(y, x);
			size_t elesize = impt->ele.size();
			if(elesize > 0)
			{
				Color csum = Color::BLACK;
				float wsum = 0;
				if(1 == elesize)
				{
					int id = impt->ele[0].id;
					Vec2D coor = impt->ele[0].coor;
					csum = interpolate(imgs[id],coor.y,coor.x);
					if(csum.x < 0) continue;
					p[0] = csum.x;
					p[1] = csum.y;
					p[2] = csum.z;
				}
				else
				{
					for(size_t i = 0; i < elesize; i++)
					{
						int id = impt->ele[i].id;
						Vec2D coor = impt->ele[i].coor;
						Color c = interpolate(imgs[id],coor.y,coor.x);
						if(c.x < 0) continue;
						float w = impt->ele[i].w;
						csum += c * w;
						wsum += w;
					}
					if(wsum > 0)
					{
						//(csum / wsum).write_to(p);
						csum /= wsum;
						p[0] = csum.x;
						p[1] = csum.y;
						p[2] = csum.z;
					}
				}
			}
		}
	}
}

Mat32f rebuild_lut(std::vector<Mat32f>& imgs)
{
	//*********************************************************
	srand(time(NULL));//?????是否用于KNN？？？？？？？ or Other ?????
	init_config();
	
//*********************************************************		
	Mat32f res;
	if (CYLINDER > 0) 
	{
		printf("CYLINDER mode \n");
		CylinderStitcher p(imgs);
		res = p.build();
	}
	else 
	{
		printf("naive mode \n");
		Stitcher p(imgs);
		res = p.build();
	}
	return res;
}

void onTrackbar(int pos, void* userdata)
{
	const std::string win_img = "stitch result";
	const std::string win_tranckbar = "trackbar";
	static int log_count = 0;
	
	TrackbarParams* params = (TrackbarParams*)userdata;
	//int crop_minx = 0, crop_maxx = params->img->cols-1, crop_miny = 0, crop_maxy = params->img->rows-1;
	//int fill_minx = 0, fill_maxx = params->img->cols-1, fill_miny = 0, fill_maxy = params->img->rows-1;
	auto draw_lines = [&](){
		cv::Mat img_copy;
		cv::resize(*(params->img), img_copy, cv::Size(params->img->cols/2, params->img->rows/2));
		
		//printf("w:h=%d:%d\n", img_copy.cols, img_copy.rows);
		//printf("pt1:(%d,%d),pt2:(%d,%d)\n", *(params->crop_minx)/2, *(params->crop_miny)/2, *(params->crop_maxx)/2, *(params->crop_miny)/2);
		
		
		cv::line(img_copy, cv::Point(*(params->crop_minx)/2, *(params->crop_miny)/2), cv::Point(*(params->crop_minx)/2, *(params->crop_maxy)/2), cvScalarAll(255));
		cv::line(img_copy, cv::Point(*(params->crop_maxx)/2, *(params->crop_miny)/2), cv::Point(*(params->crop_maxx)/2, *(params->crop_maxy)/2), cvScalarAll(255));
		cv::line(img_copy, cv::Point(*(params->crop_minx)/2, *(params->crop_miny)/2), cv::Point(*(params->crop_maxx)/2, *(params->crop_miny)/2), cvScalarAll(255));
		cv::line(img_copy, cv::Point(*(params->crop_minx)/2, *(params->crop_maxy)/2), cv::Point(*(params->crop_maxx)/2, *(params->crop_maxy)/2), cvScalarAll(255));
		
		cv::line(img_copy, cv::Point(*(params->fill_minx)/2, *(params->fill_miny)/2), cv::Point(*(params->fill_minx)/2, *(params->fill_maxy)/2), cvScalarAll(255));
		cv::line(img_copy, cv::Point(*(params->fill_maxx)/2, *(params->fill_miny)/2), cv::Point(*(params->fill_maxx)/2, *(params->fill_maxy)/2), cvScalarAll(255));
		cv::line(img_copy, cv::Point(*(params->fill_minx)/2, *(params->fill_miny)/2), cv::Point(*(params->fill_maxx)/2, *(params->fill_miny)/2), cvScalarAll(255));
		cv::line(img_copy, cv::Point(*(params->fill_minx)/2, *(params->fill_maxy)/2), cv::Point(*(params->fill_maxx)/2, *(params->fill_maxy)/2), cvScalarAll(255));
		
		cv::imshow(win_img.c_str(), img_copy);
	};
	
	switch(params->type)
	{
	case 4:
		if(pos > *(params->crop_maxx) || pos > *(params->fill_minx))
		{
			cv::setTrackbarPos("裁剪左边界", win_tranckbar, *(params->crop_minx));
			fprintf(stderr, "%d:裁剪左边界不能超过填充左边界和裁剪右边界.\n", log_count++);
		}
		else
		{
			*(params->crop_minx) = pos;
		}
		draw_lines();
		break;
		
	case 5:
		if(pos < *(params->crop_minx) || pos < *(params->fill_maxx))
		{
			cv::setTrackbarPos("裁剪右边界", win_tranckbar, *(params->crop_maxx));
			fprintf(stderr, "%d:裁剪右边界不能超过填充右边界和裁剪左边界.\n", log_count++);
		}
		else
		{
			*(params->crop_maxx) = pos;
		}
		draw_lines();
		break;
		
	case 6:
		if(pos > *(params->crop_maxy) || pos > *(params->fill_miny))
		{
			cv::setTrackbarPos("裁剪上边界", win_tranckbar, *(params->crop_miny));
			fprintf(stderr, "%d:裁剪上边界不能超过填充上边界和裁剪下边界.\n", log_count++);
		}
		else
		{
			*(params->crop_miny) = pos;
		}
		draw_lines();
		break;
		
	case 7:
		if(pos < *(params->crop_miny) || pos < *(params->fill_maxy))
		{
			cv::setTrackbarPos("裁剪下边界", win_tranckbar, *(params->crop_maxy));
			fprintf(stderr, "%d:裁剪下边界不能超过填充下边界和裁剪上边界.\n", log_count++);
		}
		else
		{
			*(params->crop_maxy) = pos;
		}
		draw_lines();
		break;
		
	case 0:
		if(pos < *(params->crop_minx) || pos > *(params->fill_maxx))
		{
			cv::setTrackbarPos("填充左边界", win_tranckbar, *(params->fill_minx));
			fprintf(stderr, "%d:填充左边界不能超过填充右边界和裁剪左边界.\n", log_count++);
		}
		else
		{
			*(params->fill_minx) = pos;
		}
		draw_lines();
		break;
		
	case 1:
		if(pos > *(params->crop_maxx)  || pos < *(params->fill_minx))
		{
			cv::setTrackbarPos("填充右边界", win_tranckbar, *(params->fill_maxx));
			fprintf(stderr, "%d:填充右边界不能超过填充左边界和裁剪右边界.\n", log_count++);
		}
		else
		{
			*(params->fill_maxx) = pos;
		}
		draw_lines();
		break;
		
	case 2:
		if(pos < *(params->crop_miny) || pos > *(params->fill_maxy))
		{
			cv::setTrackbarPos("填充上边界", win_tranckbar, *(params->fill_miny));
			fprintf(stderr, "%d:填充上边界不能超过填充下边界和裁剪上边界.\n", log_count++);
		}
		else
		{
			*(params->fill_miny) = pos;
		}
		draw_lines();
		break;
		
	case 3:
		if(pos > *(params->crop_maxy) || pos < *(params->fill_miny))
		{
			cv::setTrackbarPos("填充下边界", win_tranckbar, *(params->fill_maxy));
			fprintf(stderr, "%d:填充下边界不能超过填充上边界和裁剪下边界.\n", log_count++);
		}
		else
		{
			*(params->fill_maxy) = pos;
		}
		draw_lines();
		break;
	default:
		fprintf(stderr, "TrackbarParams error.\n");
		break;
	}
}

Mat32f cut(const Mat32f& mat/*, std::vector<Mat32f>& imgs*/)
{
	fprintf(stderr, "select boundry...\n");
	
	cv::Mat img;
	Mat32f2Mat_(mat, img);
	
	const std::string win_img = "stitch result";
	cv::namedWindow(win_img);
	
	//cv::Mat tmp;
	//cv::resize(img, tmp, cv::Size(img.cols/2, img.rows/2));
	//cv::imshow(win_img.c_str(), tmp);
	
	const std::string win_tranckbar = "trackbar";
	cv::namedWindow(win_tranckbar);
	
	int crop_minx = 0, crop_maxx = img.cols-1, crop_miny = 0, crop_maxy = img.rows-1;
	int fill_minx = 0, fill_maxx = img.cols-1, fill_miny = 0, fill_maxy = img.rows-1;
	
	auto setParams = [&](TrackbarParams& p, int type){
		p.img = &img;
		p.type = type;
		p.crop_minx = &crop_minx, p.crop_maxx = &crop_maxx, p.crop_miny = &crop_miny, p.crop_maxy = &crop_maxy;
		p.fill_minx = &fill_minx, p.fill_maxx = &fill_maxx, p.fill_miny = &fill_miny, p.fill_maxy = &fill_maxy;
	};
	
	TrackbarParams tp; setParams(tp,0);
	onTrackbar(0, &tp);
	
	TrackbarParams tp0; setParams(tp0,0);
	cv::createTrackbar("填充左边界", win_tranckbar, &fill_minx, img.cols-1, onTrackbar, &tp0);
	
	TrackbarParams tp1; setParams(tp1,1);
	cv::createTrackbar("填充右边界", win_tranckbar, &fill_maxx, img.cols-1, onTrackbar, &tp1);
	
	TrackbarParams tp2; setParams(tp2,2);
	cv::createTrackbar("填充上边界", win_tranckbar, &fill_miny, img.rows-1, onTrackbar, &tp2);
	
	TrackbarParams tp3; setParams(tp3,3);
	cv::createTrackbar("填充下边界", win_tranckbar, &fill_maxy, img.rows-1, onTrackbar, &tp3);
	
	TrackbarParams tp4; setParams(tp4,4);
	cv::createTrackbar("裁剪左边界", win_tranckbar, &crop_minx, img.cols-1, onTrackbar, &tp4);
	
	TrackbarParams tp5; setParams(tp5,5);
	cv::createTrackbar("裁剪右边界", win_tranckbar, &crop_maxx, img.cols-1, onTrackbar, &tp5);
	
	TrackbarParams tp6; setParams(tp6,6);
	cv::createTrackbar("裁剪上边界", win_tranckbar, &crop_miny, img.rows-1, onTrackbar, &tp6);
	
	TrackbarParams tp7; setParams(tp7,7);
	cv::createTrackbar("裁剪下边界", win_tranckbar, &crop_maxy, img.rows-1, onTrackbar, &tp7);
	
	cv::waitKey();
	fprintf(stderr, "select boundry...end\n");
	
	
	cv::Rect fill_range(fill_minx-crop_minx, fill_miny-crop_miny, fill_maxx-fill_minx+1, fill_maxy-fill_miny+1);
	
	auto is_fill_pt = [&](int x, int y){
		if(x >= fill_range.x && x < (fill_range.x+fill_range.width) && y >= fill_range.y && y < (fill_range.y+fill_range.height))
			return true;
		return false;
	};
	
	
	//printf("fill_range:x,y,w,h:%d,%d,%d,%d\n", fill_range.x,fill_range.y,fill_range.width,fill_range.height);
	
	int newh = crop_maxy-crop_miny+1, neww = crop_maxx-crop_minx+1;
	Mat32f ret(newh, neww, 3);
	IMGPT* newlut = (IMGPT*)malloc(neww*newh*sizeof(IMGPT));
	for(int y = 0; y < newh; y++)
	{
		{
			const float* src = mat.ptr(y+crop_miny, crop_minx);
			float* dst = ret.ptr(y);
			memcpy(dst, src, neww*3*sizeof(float));
		}
		
		{
			const IMGPT* src = lut.ptr(y+crop_miny,crop_minx);
			IMGPT* dst = newlut + y * neww;
			for(int x = 0; x < neww; x++)
			{
				if(is_fill_pt(x,y))
				{
					dst[x].ele.clear();
					LUTTriple pt;
					pt.id = 4;//(int)imgs.size();
					//printf("x:y=%d,%d\n",x,y);//getchar();
					//if(pt.id!=4) printf("id:%d\n", pt.id);
					pt.coor.x = x - fill_range.x;
					pt.coor.y = y - fill_range.y;
					pt.w = 1.0;
					//printf("x:y=%lf,%lf\n",pt.coor.x,pt.coor.y);//getchar();
					dst[x].ele.push_back(pt);
				}
				else if(src[x].ele.size() > 0)
				{
					dst[x].ele.clear();
					dst[x].ele.insert(dst[x].ele.end(), src[x].ele.begin(), src[x].ele.end());
				}
			}
		}
	}
	if(lut.data) free(lut.data);
	lut.data = newlut;
	lut.rows = newh;
	lut.cols = neww;
	
	
	
	char *path1 = "fillout.jpg";
	if(access(path1, F_OK) != 0)
	{
		fprintf(stderr, "fillout image does not exist. rename it to fillout.png and rerun.\n");
		exit(-1);
	}
	cv::Mat im0 = cv::imread(path1), im1;
	//im0.convertTo(im1, CV_32FC3, 1.0/255);
	printf("resize:%d,%d\n",fill_range.width,fill_range.height);
	cv::resize(im0, im1, cv::Size(fill_range.width+1,fill_range.height+1));
	cv::imwrite("fillout_resize.jpg", im1);
	
	return ret;
}

void stitch(std::vector<Mat32f>& imgs, Mat32f& res) 
{
	//printf("stitch\n");
	
	static bool lut_loaded = false;
	static bool rebuild = true;
	static Mat32f fill_img;

	//check if there is lut
	if(access("lut", F_OK) == 0) rebuild = false;
	if(rebuild)
	{
		
		//image 1
		cv::Mat temp11, temp12;
		Mat32f2Mat_(imgs[1], temp11);
		cv::transpose(temp11, temp12);
		cv::flip(temp12, temp11, 1);
		imgs[1] = Mat2Mat32f_(temp11);
		
		//image 2
		cv::Mat temp21, temp22;
		Mat32f2Mat_(imgs[2], temp21);
		cv::transpose(temp21, temp22);
		cv::flip(temp22, temp21, 0);
		imgs[2] = Mat2Mat32f_(temp21);
		
		//image 3
		cv::Mat temp31, temp32;
		Mat32f2Mat_(imgs[3], temp31);
		cv::flip(temp31, temp32, -1);
		imgs[3] = Mat2Mat32f_(temp32);
		
		
		fprintf(stderr, "rebuilding the lut...\n");
		res = rebuild_lut(imgs);
		fprintf(stderr, "rebuilding the lut...end\n");
		
		if (CROP) 
		{
			int oldw = res.width(), oldh = res.height();
			res = cut(res);
			printf("Crop from %dx%d to %dx%d\n", oldh, oldw, res.height(), res.width());
		}
		//write_rgb("out.jpg",res);

		lut.write_lut();
		lut_loaded = true;
		char *path1 = "fillout_resize.jpg";
		if(access(path1, F_OK) != 0)
		{
			fprintf(stderr, "fillout image does not exist. delete lut and rerun.\n");
			exit(-1);
		}
		fill_img = read_img(path1);
		return;
	}
	
	
	if(!lut_loaded)
	{
		//fprintf(stderr, "loading lut...\n");
		char *path = "lut";
		lut.load_lut(path);
		lut_loaded = true;
		
		char *path1 = "fillout_resize.jpg";
		if(access(path1, F_OK) != 0)
		{
			fprintf(stderr, "fillout image does not exist. delete lut and rerun.\n");
			exit(-1);
		}
		fprintf(stderr, "loading fillout image...\n");
		fill_img = read_img(path1);
		fprintf(stderr, "loading fillout image...end\n");
	}
	
	if(CROP)
	{
		imgs.emplace_back(fill_img);
	}
	lut_stitch(imgs, res);
	
}

