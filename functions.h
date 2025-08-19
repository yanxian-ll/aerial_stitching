#ifndef FUNCTION_H
#define FUNCTION_H

#include <opencv2/opencv.hpp>       
#include <opencv2/features2d.hpp>    
#include <opencv2/calib3d.hpp>      

#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <ctime>
#include <climits>

// ORB特征匹配
void orbMatching(const cv::Mat& img1, const cv::Mat& img2, 
                 std::vector<cv::Point2f>& srcPoints, 
                 std::vector<cv::Point2f>& dstPoints,
                 float match_ratio = 0.7);

// Ransac 过滤
void thresholdFilter(const std::vector<cv::Point2f>& src_pts, 
                         const std::vector<cv::Point2f>& dst_pts,
                         std::vector<cv::Point2f>& src_inliers,
                         std::vector<cv::Point2f>& dst_inliers,
                         float delta_theta_deg = 30,
                         float k_sigma = 5.0f);

// 提取 Homography 矩阵
cv::Mat findHomographyTransform(const std::vector<cv::Point2f>& srcPoints, 
                const std::vector<cv::Point2f>& dstPoints);

// 提取相似性矩阵
cv::Mat findSimilarityTransform(const std::vector<cv::Point2f>& srcPoints, 
                                const std::vector<cv::Point2f>& dstPoints);

// 只提取平移向量
cv::Mat findTranslationTransform(const std::vector<cv::Point2f>& srcPoints, 
                                 const std::vector<cv::Point2f>& dstPoints);

// 检测矩阵是否合理
bool isTransformValid(const cv::Mat& H_match, const cv::Mat& H_gps, cv::Size imgSize, 
    float max_translation=100, float max_rotation_angle=30, float max_scale=0.3, float max_scale_ratio=0.1);

// 根据矩阵转换图像
void getWarppedAcc2(cv::Mat &warped, cv::Rect &warpedRect, cv::Mat *original, cv::Mat H);

#endif  // FUNCTION_H
