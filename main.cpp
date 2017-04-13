#include <cctype>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <thread>


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching/stitcher.hpp"

#include "mat.h"
#include "stitchout.h"

bool loadParams(std::vector<cv::Mat>& vcameraMatrix, std::vector<cv::Mat>& vdistCoeffs)
{
	fprintf(stderr, "loadParams.\n");
	cv::FileStorage fs("camera.yml", cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		fprintf(stderr, "%s:%d:loadParams falied. 'camera.yml' does not exist\n", __FILE__, __LINE__);
		return false;
	}

	int n_camera = (int)fs["n_camera"];
	vcameraMatrix.resize(n_camera);
	vdistCoeffs.resize(n_camera);
	for (int i = 0; i < n_camera; i++)
	{
		char buf1[100];
		sprintf(buf1, "cam_%d_matrix", i);
		char buf2[100];
		sprintf(buf2, "cam_%d_distcoeffs", i);
		fs[buf1] >> vcameraMatrix[i];
		fs[buf2] >> vdistCoeffs[i];
	}
	fs.release();

	return true;
}

inline Mat32f Mat2Mat32f(cv::Mat& mat)
{
	cv::Mat temp;
	mat.convertTo(temp, CV_32FC3, 1.0/255);
	
	int w = mat.cols;
	int h = mat.rows;
	int c = 3;
	
	Mat32f res(h, w, 3);

	//#define MAT(ROW, COL) ((float*)(temp.ptr<float>((ROW)) + (COL)* sizeof(float) * c))
	for(int y = 0; y < h; y++)
	{
		float *row_dst = res.ptr(y);
		float *row_src = temp.ptr<float>(y);
		for(int x = 0; x < w; x++)
		{
			*(row_dst + x * 3 + 0) = *(row_src + x * 3 + 2);
			*(row_dst + x * 3 + 1) = *(row_src + x * 3 + 1);
			*(row_dst + x * 3 + 2) = *(row_src + x * 3 + 0);
		}
		//memcpy(row_dst, row_src, w * sizeof(float) * 3);
	}
	
	return res;
}

inline void Mat32f2Mat(const Mat32f& mat, cv::Mat& im)
{
	
	int w = mat.width();
	int h = mat.height();
	int c = mat.channels();
	
	im = cv::Mat::zeros(h,w,CV_32FC3);
	
	
	#define IM(ROW, COL) (float*)(im.data + im.step  * (ROW) + (COL)* sizeof(float) * 3)
	
	for(int y = 0; y < h; y++)
	{
		const float *row = mat.ptr(y);
		for(int x = 0; x < w; x++)
		{
			*(IM(y, x) + 2) = *(row + x * 3 + 0);
			*(IM(y, x) + 1) = *(row + x * 3 + 1);
			*(IM(y, x) + 0) = *(row + x * 3 + 2);
		}
		//memcpy(IM(y, 0), row, w * sizeof(float)*3);
	}
}


void checkParams(int argc, char* argv[])
{
	std::string path(argv[0]);
	size_t pos = path.find_last_of('/');
	if(std::string::npos == pos)
	{
		printf("excutable file's path error.\n");
		exit(0);
	}
	std::string file_name = path.substr(pos+1,path.length()-pos-1);
	if(argc < 2 || 0 == atoi(argv[1]))
	{
		printf("Usage:./%s <n_cams>\n", file_name.c_str());
		exit(0);
	}
}

int main(int argc, char* argv[])
{
	
	checkParams(argc, argv);
	const int n_cams = atoi(argv[1]);
	//int boardSize_w = atoi(argv[2]);
	//int boardSize_h = atoi(argv[3]);
	//double crop_ratio = atof(argv[4]);

	std::vector<cv::Mat> frames(n_cams);
	std::vector<cv::VideoCapture> caps(n_cams);
	std::vector<cv::Mat> vcameraMatrix;
	std::vector<cv::Mat> vdistCoeffs;
	
	bool load_succeed = loadParams(vcameraMatrix, vdistCoeffs);


	for (int k = 0; k < n_cams; k++)
	{
		//clock_t timestamp = 0;
		while (!caps[k].isOpened())
		{
			fprintf(stderr, "trying to open cam %d...\n", k);
			caps[k].open(k);
			//timestamp = clock();
		}
		/*if(2 == k)
		{
			caps[k].set(CV_CAP_PROP_FRAME_WIDTH, 480);
			caps[k].set(CV_CAP_PROP_FRAME_HEIGHT, 640);
		}*/
		fprintf(stderr, "capture %d:width:%d, height:%d\n", k, 
			(int)caps[k].get(CV_CAP_PROP_FRAME_WIDTH), (int)caps[k].get(CV_CAP_PROP_FRAME_HEIGHT));
	}


	fprintf(stderr, "press g to stitch and press q quit.\n");
	bool start_stitch = false;

	std::vector<bool> undistorted(n_cams, false);
	struct timeval t_begin, t_end, all_begin, all_end;// = 0, ts2 = 0, ts3 = 0;
	int elapse0 = 0, elapse1 = 0, elapse2 = 0, elapse3 = 0, elapse4 = 0, elapse5 = 0;

	while (true) {
		gettimeofday(&all_begin, NULL);
		//timestamp = clock(); ts2 = timestamp; ts3 = timestamp;
		//std::vector<std::thread> threads;
		
		gettimeofday(&t_begin, NULL);
		//#pragma omp parallel for schedule(dynamic)
		for (int k = 0; k < n_cams; k++)
		{
			caps[k] >> frames[k];
		}
		
		#pragma omp parallel for schedule(dynamic) //grand promotion
		for (int k = 0; k < n_cams; k++)
		{
			cv::Mat t1;
			undistort(frames[k], t1, vcameraMatrix[k], vdistCoeffs[k]);
			frames[k] = t1;
			undistorted[k] = true;
		}
		gettimeofday(&t_end, NULL);
		elapse1 = (t_end.tv_sec - t_begin.tv_sec)*1000 + (t_end.tv_usec-t_begin.tv_usec)/1000;
		//fprintf(stderr, "capture:%dms\t", elapse);
		//fprintf(stderr, "read:%dms\n", int((clock()-timestamp+0.0)/CLOCKS_PER_SEC*1000));getchar();
		//timestamp = clock();

		gettimeofday(&t_begin, NULL);
		for (int k = 0; k < n_cams; k++)
		{
			if (undistorted[k])
			{
				char buf[20];
				sprintf(buf, "cam%d", k);
				cv::imshow(buf, frames[k]);
			}
			else
			{
				fprintf(stderr, "image %d have not been undistorted.\n", k);
			}
		}
		gettimeofday(&t_end, NULL);
		elapse2 = (t_end.tv_sec - t_begin.tv_sec)*1000 + (t_end.tv_usec-t_begin.tv_usec)/1000;
		//fprintf(stderr, "show:%dms\t", elapse);


		if (start_stitch)
		{
			int k = 0;
			for (k = 0; k < n_cams; k++)
			{
				if (!undistorted[k])
				{
					fprintf(stderr, "image %d have not been undistorted.\n", k);
					break;
				}
			}
			if (k < n_cams) continue;

			std::vector<Mat32f> imgs;

			gettimeofday(&t_begin, NULL);
			for(size_t i = 0; i < 4; i++)
			{
				imgs.emplace_back(Mat2Mat32f(frames[i]));
			}
			gettimeofday(&t_end, NULL);
			elapse3 = (t_end.tv_sec - t_begin.tv_sec)*1000 + (t_end.tv_usec-t_begin.tv_usec)/1000;
			//fprintf(stderr, "convert:%dms\t", elapse);

			
			Mat32f res;
			
			gettimeofday(&t_begin, NULL);
			stitch(imgs,res);
			gettimeofday(&t_end, NULL);
			
			elapse4 = (t_end.tv_sec - t_begin.tv_sec)*1000 + (t_end.tv_usec-t_begin.tv_usec)/1000;
			//fprintf(stderr, "stitch:%dms\t", elapse);

			
			cv::Mat im;
			Mat32f2Mat(res, im);		
			cv::imshow("stitched", im);


			for (int k = 0; k < n_cams; k++)
			{
				undistorted[k] = false;
			}
			
		}

		char get_key = 0xff & cv::waitKey(1);
		if (get_key == 'g' || get_key == 'G')
		{
				start_stitch = true;
		}
		if (get_key == 'q' || get_key == 'Q')//27-escape
		{
			cv::destroyAllWindows();
			break;
		}	
		if(get_key == 's' || get_key == 'S')
		{
			int k = 0;
			for (k = 0; k < n_cams; k++)
			{
				if (!undistorted[k])
				{
					fprintf(stderr, "image %d have not been undistorted. can not get snapshot\n", k);
					break;
				}
			}
			if (k < n_cams) continue;
			for(int i = 0; i < frames.size(); i++)
			{
				char buf[20];
				sprintf(buf, "snapshot%d.jpg", i);
				cv::imwrite(buf, frames[i]);
			}
		}
		
		gettimeofday(&all_end, NULL);
		elapse5 = (all_end.tv_sec - all_begin.tv_sec)*1000 + (all_end.tv_usec-all_begin.tv_usec)/1000 - 1;
		fprintf(stderr, "cap:%dms\tshow:%dms\tconvert:%dms\tstitch:%dms\tfps--:%.2f\n", elapse1, elapse2, elapse3, elapse4, (1000+0.0)/elapse5);
	}

	return 0;
}
