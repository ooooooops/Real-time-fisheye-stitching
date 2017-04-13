

#include "stitcherbase.h"
//#include "timer.hh"


void StitcherBase::calc_feature() 
{
	//GuardedTimer tm("calc_feature()");
	feats.resize(imgs.size());
	keypoints.resize(imgs.size());

	// detect feature
#pragma omp parallel for schedule(dynamic)
	for(unsigned int k=0;k<imgs.size();++k)
	{
		feats[k] = feature_det->detect_feature(imgs[k]);
		if (feats[k].size() == 0)	// TODO delete the image
		{
			printf("Cannot find feature in image %u!\n", k);
			exit(1);
		}
		
		printf("Image %u has %u features\n", k, feats[k].size());
        
		keypoints[k].resize(feats[k].size());         
		for(unsigned int i=0;i<feats[k].size();++i)
			keypoints[k][i] = feats[k][i].coor;
	}
}

void StitcherBase::free_feature()
{
	feats.clear(); 
    feats.shrink_to_fit();	// free memory for feature
    
	keypoints.clear(); 
    keypoints.shrink_to_fit();	// free memory for feature
}


