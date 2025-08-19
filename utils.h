#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>       
#include <opencv2/features2d.hpp>   

#include <filesystem>            
#include <string>
#include <vector>
#include <iostream>


// 获取相对路径
std::string get_relative_path(const std::string& fullPathStr, const std::string& rootPathStr);

// 保存特征匹配结果
void saveMatches(const cv::Mat &img1, 
                 const cv::Mat &img2,
                 const std::vector<cv::Point2f> &srcPoints,
                 const std::vector<cv::Point2f> &dstPoints,
                 const std::string &outputPath);
// 保存变换矩阵
void saveTransformMatrix(const std::string &filePath, const cv::Mat &matrix);
// 保存变换后图像
void saveWarped(cv::Mat &warped, cv::Mat currentImage, cv::Mat mask2, const std::string &outputPath);

#endif   // UTILS_H
