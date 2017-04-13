#include "lut.h"

LUT lut_coor;
LUT lut;
LutInvRoI inv_roi = {0,0,0,0};
//LUT lut_color;
//std::vector<LUT> luts_coor(4);
void LUT::write_lut()
{
	fprintf(stderr, "writing lut...\n");
	fprintf(stderr, "lut width:%d, height:%d\n", cols, rows);
	if(cols == 0 || rows == 0) return;
	FILE *fp = fopen("lut", "wt");
	if(!fp) return;
	int cnt = 0;
	for(int y = 0; y < rows; y++)
	{
		fprintf(stderr, "progress:%f%%\r",(y+0.0)/rows*100);
		for(int x = 0; x < cols; x++)
		{
			//printf("y:%d,x:%d",y,x);
			IMGPT *p = lut.ptr(y, x);
			
			if(0 == p->ele.size())
			{
				cnt++;
				fprintf(fp, "- ");
				continue;
			}
			//printf("ppppp\n");
			for(int k = 0; k < p->ele.size(); k++)
			{
				int id = p->ele[k].id;
				Vec2D coor = p->ele[k].coor;
				float w = p->ele[k].w;
				
				char buf[256];
				sprintf(buf, "%d,%lf,%lf,%f|",id,coor.x,coor.y,w);
				fprintf(fp, buf);
			}
			
			fprintf(fp, " ");
		}
		//fprintf(stderr, "\n");
		fprintf(fp, "\n");
	}
	fprintf(stderr, "%d have none correspondence\n", cnt);
	fclose(fp);
}



void LUT::load_lut(const char* path)
{
	if(access(path, F_OK) != 0)
	{
		fprintf(stderr, "lut file error.please check the path and the authority.\n");
		exit(-1);
	}
	
	using namespace std;
	fprintf(stderr, "loading lut...\n");

	ifstream ifs(path);
	auto splite = [](std::string s, char c)->vector<string>{
		while(' ' == *(s.begin())) s.erase(s.begin());
		while(' ' == *(s.end() - 1)) s.erase(s.end() - 1);
		vector<string> res;
		int pos = 0;
		int pre = 0;
		while( (pos = s.find(c, pre)) != string::npos)
		{
			res.push_back(s.substr(pre, pos-pre));
			pre = pos + 1;
		}
		if(c == ' ') res.push_back(s.substr(pre, s.length() - pre));
		return res;
	};
	
	//step 1. get the size of the lut
	string line;
	rows = 0;
	while(getline(ifs, line) && line.length() > 0)
	{
		vector<string> vs = splite(line, ' ');
		if(cols == 0)
		{
			cols = vs.size();
		}
		if(cols != (int)vs.size())
		{
			fprintf(stderr, "lut format error.please rebuild the lut.\n");
			exit(-1);
		}	
		rows++;
	}
	
	//step 2. allocate memory
	data = (IMGPT*)malloc(rows * cols * sizeof(IMGPT));
	
	//step 3. read data
	ifstream ifs1(path);
	for(int y = 0; y < rows; y++)
	{
		getline(ifs1, line);
		if(line.length() == 0) continue;
		vector<string> vs = splite(line, ' ');
		for(int x = 0; x < cols; x++)
		{
			string& lut_term = vs[x];//获得一个像素点的lut
			if("-" == lut_term) continue;
			vector<string> term = splite(lut_term, '|');
			for(int i = 0; i < (int)term.size(); i++)
			{
				LUTTriple tri;
				sscanf(term[i].c_str(), "%d,%lf,%lf,%f|", &tri.id, &tri.coor.x, &tri.coor.y, &tri.w);
				ptr(y, x)->ele.push_back(tri);
			}
		}
	}
	
	//step 4. close file stream
	if(ifs1.is_open())
	{
		ifs1.close();
	}
	if(ifs.is_open())
	{
		ifs.close();
	}
	
	fprintf(stderr, "lut size:cols = %d, rows = %d\n", cols, rows);
}

bool in_inv_roi(int x, int y)//检查点(x,y)是否在inv_roi(目前inv_roi是头文件中声明的外部变量extern LutInvRoI inv_roi)中，是则返回true，否则返回false
{
	//1.检查inv_roi的有效性
	if(inv_roi.x < 0 || inv_roi.y < 0 || inv_roi.width <= 0 || inv_roi.height <= 0)
	{
		fprintf(stderr, "%s:%d:inv_roi err.\n", __FILE__, __LINE__);
		return false;
	}
	//2.查点(x,y)是否在inv_roi内
	if(x >= inv_roi.x && x <= (inv_roi.x + inv_roi.width) && y >= inv_roi.y && y <= (inv_roi.y + inv_roi.height))
		return true;
	else return false;
}

bool set_inv_roi(int x, int y, int w, int h)
{
	//需要抠掉的区域
	inv_roi.x = x;
	inv_roi.y = y;
	inv_roi.width = w;
	inv_roi.height = h;
}
