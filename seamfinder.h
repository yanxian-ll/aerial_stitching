#ifndef SEAMFINDER_H
#define SEAMFINDER_H

#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>

enum SeamFinderType
{
    SEAM_NONE,      // 0
    SEAM_GRAPH_CUT, // 1
    SEAM_VORONOI,   // 2
    SEAM_DP         // 3
};


enum BlendType
{
    BLEND_NONE,      // 0: 直接copy
    BLEND_FEATHER,   // 1: FeatherBlender
    BLEND_MULTIBAND  // 2: MultiBandBlender
};

cv::Mat seamMerge(
    cv::Mat &dom,
    cv::Mat &warped,
    cv::Rect &warpedRect,
    SeamFinderType seamType = SEAM_GRAPH_CUT,
    BlendType blendType = BLEND_FEATHER,
    std::string debug_file = "",
    int maxSeamWidth = 256,
    float blendStrength = 5.0f, // Feather强度
    int numBands = 5            // MultiBand层数
);

#endif   // SEAMFINDER_H

