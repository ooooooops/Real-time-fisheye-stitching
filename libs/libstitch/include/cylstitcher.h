#pragma once

#include "stitcherbase.h"
#include "stitcher_image.h"
#include <vector>

// forward declaration
class Homography;
class MatchData;
struct MatchInfo;

// a stitcher which warps images to cyilnder before stitching
class CylinderStitcher : public StitcherBase 
{
	public:
		template< typename U, typename X = disable_if_same_or_derived<CylinderStitcher, U> >
        CylinderStitcher(U&& i) : StitcherBase(std::forward<U>(i))
        {
            bundle.component.resize(imgs.size());
            for(size_t i=0;i<imgs.size();++i)
                bundle.component[i].imgptr = &imgs[i];
        }

		virtual Mat32f build();
		

	protected:
		ConnectedImages bundle;
        
		// build panorama with cylindrical pre-warping
		void build_warp();
        
		// in cylindrical mode, search warping parameter for straightening
		float update_h_factor(float, float&, float&,std::vector<Homography>&,const std::vector<MatchData>&);
        
		// in cylindrical mode, perspective correction on the final image
		Mat32f perspective_correction(const Mat32f&);
};


