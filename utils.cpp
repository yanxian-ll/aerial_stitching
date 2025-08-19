#include "utils.h"


std::string get_relative_path(const std::string& fullPathStr, const std::string& rootPathStr)
{
    if (rootPathStr.empty())
    {
        return fullPathStr; // 没有设置根目录
    }

    std::filesystem::path full_path = std::filesystem::weakly_canonical(fullPathStr);
    std::filesystem::path root_path = std::filesystem::weakly_canonical(rootPathStr);

    // 确保是前缀关系
    if (full_path.string().find(root_path.string()) == 0)
    {
        return full_path.lexically_relative(root_path).string();
    }
    else
    {
        return full_path.string(); // 没有匹配到前缀
    }
}


void saveMatches(const cv::Mat &img1, 
                 const cv::Mat &img2,
                 const std::vector<cv::Point2f> &srcPoints,
                 const std::vector<cv::Point2f> &dstPoints,
                 const std::string &outputPath)
{
    cv::Mat visImage;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    std::vector<cv::DMatch> cvMatches;
 
    // 转换点对数据为KeyPoint格式 
    for (size_t i = 0; i < srcPoints.size();  ++i)
    {
        keypoints1.emplace_back(srcPoints[i],  0); // 使用默认参数创建KeyPoint
        keypoints2.emplace_back(dstPoints[i],  0);
        cvMatches.emplace_back(i,  i, 0); // 创建1:1匹配关系
    }
 
    // 以下保持原有实现不变
    cv::drawMatches(img1, keypoints1, img2, keypoints2, cvMatches, visImage,
                    cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
 
    cv::putText(visImage, "Matches: " + std::to_string(srcPoints.size()), 
                cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2);
 
    cv::imwrite(outputPath, visImage);
}


void saveTransformMatrix(const std::string &filePath, const cv::Mat &matrix)
{
    cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
    fs << "TransformMatrix" << matrix;
    fs.release();
}


void saveWarped(cv::Mat &warped, cv::Mat currentImage, cv::Mat mask2, const std::string &outputPath)
{
    int base_height = warped.rows; // 以warped图像高度为基准
    cv::Mat combined;

    cv::Mat resized_current;
    if (!currentImage.empty() && base_height > 0)
    {
        int new_width = static_cast<int>((static_cast<float>(base_height) / currentImage.rows) * currentImage.cols);
        cv::resize(currentImage, resized_current, cv::Size(new_width, base_height));
    }
    else
    {
        resized_current = cv::Mat::zeros(base_height, currentImage.cols, currentImage.type());
    }

    std::vector<cv::Mat> images_to_concat;
    images_to_concat.push_back(resized_current);
    images_to_concat.push_back(warped);

    cv::Mat mask_display;
    if (mask2.empty())
    {
        mask2 = cv::Mat::zeros(warped.size(), CV_8U); // 创建黑色占位
    }
    if (warped.channels() == 3)
    {
        cv::cvtColor(mask2, mask_display, cv::COLOR_GRAY2BGR);
    }
    else if (warped.channels() == 4)
    {
        cv::cvtColor(mask2, mask_display, cv::COLOR_GRAY2BGRA);
    }
    else
    {
        mask_display = mask2;
    }
    images_to_concat.push_back(mask_display);

    cv::hconcat(images_to_concat, combined);

    std::vector<std::string> labels = {"Original", "Warped", "Mask"};
    int label_pos_y = 30;
    int segment_width = combined.cols / 3;

    for (size_t i = 0; i < labels.size(); ++i)
    {
        cv::putText(
            combined,
            labels[i],
            cv::Point(i * segment_width + 10, label_pos_y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            (i == 2) ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 0), // 掩膜标签用白色
            2);
    }

    if (!cv::imwrite(outputPath, combined))
    {
        std::cerr << "Error: Could not save combined image to: " << outputPath << std::endl;
    }
}
