#pragma once
#include <vector>
#include <fstream>
#include <string>
#include <unistd.h> //access
#include "mat.h"
#include "geometry.h"

struct LUTTriple
{
	int id;//which image
	Vec2D coor;//coordinates in id-th image
	float w;//the weight of color of the id-th image in the position of coor
};

typedef struct
{
	//Vec2D coor;//just for single image
	std::vector<struct LUTTriple> ele;
}IMGPT;

class LUT
{
	public:
	LUT():rows(0), cols(0), data(0){};
	
	LUT(int r, int c): rows(r), cols(c), data((IMGPT*)malloc(r*c*sizeof(IMGPT)))
	{
		//memset(data, v, r*c*sizeof(IMGPT));
		init();
	}
	
	~LUT()
	{
		if(data)
		{
			free(data);
			data = NULL;
		}
	}
	
	void init()
	{
		for(int y = 0; y < rows; y++)
		{
			for(int x = 0; x < cols; x++)
			{
				ptr(y, x)->ele.clear();
			}
		}
	}
	
	void write_lut();//write lut to disk
	void load_lut(const char* lut_path);
	void resize(int r, int c)
	{
		fprintf(stderr, "resizing LUT...\n");
		if(data)
		{	
			free(data);
		}
		data = (IMGPT*)malloc(r * c * sizeof(IMGPT));
		//memset(data, v, r * c * sizeof(int));
		rows = r;
		cols = c;
		
		init();
	}
	
	IMGPT *ptr(int r, int c = 0)
	{
		if(!data) return NULL;
		if(r < 0 || r >= rows || c < 0 || c >= cols)
		{
			//fprintf(stderr, "%s:%d:params error.\n", __FILE__, __LINE__);
			//return NULL;
			exit(-1);
		}
		return data + r * cols + c;
	}
	
	int rows, cols;
	IMGPT *data;
};

typedef struct
{
	int x, y, width, height;
}LutInvRoI;//LUT的兴趣区域ROI=LUT-LutInvRoI


extern LUT lut_coor;
extern LUT lut;
extern LutInvRoI inv_roi;

bool in_inv_roi(int x, int y);//检查点(x,y)是否在inv_roi(目前inv_roi是头文件中声明的外部变量extern LutInvRoI inv_roi)中，是则返回true，否则返回false
bool set_inv_roi(int x, int y, int w, int h);

