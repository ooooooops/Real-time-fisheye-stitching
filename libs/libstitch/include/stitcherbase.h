#pragma once
#include <vector>
#include <memory>
#include "mat.h"
#include "geometry.h"
#include "feature.h"

class StitcherBase 
{
	protected:
		// helper template to prevent generic constructor being used as copy constructor
		template<typename A, typename B>
			using disable_if_same_or_derived =
			typename std::enable_if<
			!std::is_base_of<A, typename std::remove_reference<B>::type
			>::value
			>::type;

		std::vector<Mat32f> imgs;
		// feature and keypoints of each image
		std::vector<std::vector<Descriptor>> feats;	// [-w/2,w/2]
		std::vector<std::vector<Vec2D>> keypoints;	// store coordinates in [-w/2,w/2]

		// feature detector
		std::unique_ptr<FeatureDetector> feature_det;

		// get feature descriptor and keypoints for each image
		void calc_feature();
		void free_feature();

	public:
		// universal reference constructor to initialize imgs
		template<typename U, typename X =
        disable_if_same_or_derived<StitcherBase, U>>
        StitcherBase(U&& i) : imgs(std::forward<U>(i)) 
        {
            if (imgs.size() <= 1)
            {
                printf("Cannot stitch with only %u images.", imgs.size());
                exit(1);
            }

            feature_det.reset(new SIFTDetector);
        }

		StitcherBase(const StitcherBase&) = delete;
		StitcherBase& operator = (const StitcherBase&) = delete;

		virtual Mat32f build() = 0;

		virtual ~StitcherBase() = default;
};




