#include "functions.h"
#include "debug_utils.h"

// 特征匹配
void orbMatching(const cv::Mat& img1, const cv::Mat& img2, 
                 std::vector<cv::Point2f>& srcPoints, 
                 std::vector<cv::Point2f>& dstPoints,
                 float match_ratio)
{
    // 转换为灰度图 
    clock_t start = clock();
    cv::Mat gray1, gray2;
    if (img1.channels()  == 3) {
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    } else if (img1.channels()  == 4) {
        cv::cvtColor(img1, gray1, cv::COLOR_BGRA2GRAY);
    } else {
        gray1 = img1;
    }
    
    if (img2.channels()  == 3) {
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
    } else if (img2.channels()  == 4) {
        cv::cvtColor(img2, gray2, cv::COLOR_BGRA2GRAY);
    } else {
        gray2 = img2;
    }
 
    // ORB特征检测 
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2500, 1.12f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;
    orb->detectAndCompute(gray1, cv::noArray(), kp1, des1);
    orb->detectAndCompute(gray2, cv::noArray(), kp2, des2);
 
    // 特征匹配 
    cv::BFMatcher bf(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> matches_knn;
    bf.knnMatch(des1,  des2, matches_knn, 2);
 
    // 比率测试筛选 
    std::vector<cv::DMatch> good_matches;
    for (const auto& match_pair : matches_knn) {
        if (match_pair.size() < 2) continue;
        if (match_pair[0].distance < match_ratio * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]); 
        }
    }
 
    // 提取匹配点
    srcPoints.clear(); 
    dstPoints.clear(); 
    for (const auto& m : good_matches) {
        srcPoints.push_back(kp1[m.queryIdx].pt); 
        dstPoints.push_back(kp2[m.trainIdx].pt); 
    }

    LOG_INFO("Matching took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
}
 
// 过滤函数 
void thresholdFilter(const std::vector<cv::Point2f>& src_pts, 
                         const std::vector<cv::Point2f>& dst_pts,
                         std::vector<cv::Point2f>& src_inliers,
                         std::vector<cv::Point2f>& dst_inliers,
                         float delta_theta_deg,
                         float k_sigma)   // k 倍标准差
{
    clock_t start = clock();
    if (src_pts.empty() || dst_pts.empty() || src_pts.size() != dst_pts.size())
        return;

    const size_t N = src_pts.size();
    std::vector<float> dxs, dys;
    dxs.reserve(N);
    dys.reserve(N);

    double sum_cos = 0.0, sum_sin = 0.0; // 用于方向均值

    // 计算每对点的位移和方向
    for (size_t i = 0; i < N; i++) {
        float dx = dst_pts[i].x - src_pts[i].x;
        float dy = dst_pts[i].y - src_pts[i].y;
        dxs.push_back(dx);
        dys.push_back(dy);

        float theta = std::atan2(dy, dx);
        sum_cos += std::cos(theta);
        sum_sin += std::sin(theta);
    }

    auto mean_of = [](const std::vector<float>& v) {
        return std::accumulate(v.begin(), v.end(), 0.0f) / v.size();
    };

    auto stddev_of = [&](const std::vector<float>& v, float mean) {
        float sum = 0.0f;
        for (float val : v) sum += (val - mean) * (val - mean);
        return std::sqrt(sum / v.size());
    };

    // 平移均值
    float mean_dx = mean_of(dxs);
    float mean_dy = mean_of(dys);

    // 平移标准差 -> 阈值
    float std_dx = stddev_of(dxs, mean_dx);
    float std_dy = stddev_of(dys, mean_dy);
    float delta_x = k_sigma * std_dx;
    float delta_y = k_sigma * std_dy;

    // 4. 方向均值（向量平均法）
    float mean_theta = std::atan2(sum_sin, sum_cos);
    float delta_theta = delta_theta_deg * static_cast<float>(CV_PI) / 180.0f;

    // 5. 筛选内点
    src_inliers.clear();
    dst_inliers.clear();
    for (size_t i = 0; i < N; i++) {
        float dx = dxs[i];
        float dy = dys[i];
        float theta = std::atan2(dy, dx);

        // 角度差考虑周期性
        float dtheta = std::fabs(theta - mean_theta);
        if (dtheta > CV_PI) dtheta = 2 * CV_PI - dtheta;

        if (std::fabs(dx - mean_dx) <= delta_x &&
            std::fabs(dy - mean_dy) <= delta_y &&
            dtheta <= delta_theta)
        {
            src_inliers.push_back(src_pts[i]);
            dst_inliers.push_back(dst_pts[i]);
        }
    }

    LOG_INFO("Filter took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
    // LOG_DEBUG("mean_dx=" + std::to_string(mean_dx) +
    //          ", mean_dy=" + std::to_string(mean_dy) +
    //          ", std_dx=" + std::to_string(std_dx) +
    //          ", std_dy=" + std::to_string(std_dy) +
    //          ", mean_theta(deg)=" + std::to_string(mean_theta * 180.0f / CV_PI) +
    //          ", kept=" + std::to_string(src_inliers.size()) + "/" + std::to_string(N));
}


cv::Mat findHomographyTransform(const std::vector<cv::Point2f>& srcPoints, 
                const std::vector<cv::Point2f>& dstPoints)
{
    clock_t start = clock();
    cv::Mat H = cv::findHomography(srcPoints, dstPoints, cv::RANSAC); // 3x3矩阵
    LOG_INFO("Homography took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
    return H;
}


// 相似变换函数 
cv::Mat findSimilarityTransform(const std::vector<cv::Point2f>& srcPoints, 
                                const std::vector<cv::Point2f>& dstPoints)
{
    clock_t start = clock();
    if (srcPoints.size()  < 4 || dstPoints.size()  < 4) 
        return cv::Mat::eye(3, 3, CV_64F);
 
    // 使用RANSAC估计相似变换
    cv::Mat inliers;
    cv::Mat transform = cv::estimateAffinePartial2D(
        srcPoints, dstPoints, inliers, cv::RANSAC, 3.0, 2000, 0.99
    );
 
    if (transform.empty())  {
        return cv::Mat::eye(3, 3, CV_64F);
    }
 
    // 转换为齐次矩阵
    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    transform.rowRange(0, 2).colRange(0, 3).copyTo(H.rowRange(0, 2));
    LOG_INFO("Similarity took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
    return H;
}


// 只提取平移向量
cv::Mat findTranslationTransform(const std::vector<cv::Point2f>& srcPoints, 
                                 const std::vector<cv::Point2f>& dstPoints)
{
    clock_t start = clock();
    if (srcPoints.empty() || dstPoints.empty() || srcPoints.size() != dstPoints.size())
        return cv::Mat::eye(3, 3, CV_64F);

    // 计算平移量 (用质心差)
    cv::Point2f srcCenter(0.f, 0.f), dstCenter(0.f, 0.f);
    for (size_t i = 0; i < srcPoints.size(); ++i) {
        srcCenter += srcPoints[i];
        dstCenter += dstPoints[i];
    }
    srcCenter.x /= srcPoints.size();
    srcCenter.y /= srcPoints.size();
    dstCenter.x /= dstPoints.size();
    dstCenter.y /= dstPoints.size();

    double tx = static_cast<double>(dstCenter.x - srcCenter.x);
    double ty = static_cast<double>(dstCenter.y - srcCenter.y);

    // 构造仅平移的齐次矩阵
    cv::Mat T = cv::Mat::eye(3, 3, CV_64F);
    T.at<double>(0, 2) = tx;
    T.at<double>(1, 2) = ty;
    LOG_INFO("Translation took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
    return T;
}


inline std::string matToString(const cv::Mat& m,
                               cv::Formatter::FormatType fmt = cv::Formatter::FMT_DEFAULT)
{
    std::ostringstream oss;
    oss << cv::format(m, fmt);
    return oss.str();
}

// 判断匹配变换矩阵相较于GPS变换是否合理的函数
bool isTransformValid(const cv::Mat& H_match, const cv::Mat& H_gps, cv::Size imgSize, 
    float max_translation, float max_rotation_angle, float max_scale, float max_scale_ratio) 
{
    LOG_DEBUG("H_gps: \n" + matToString(H_gps));
    LOG_DEBUG("H_match: \n" + matToString(H_match));

    // -------------------------
    // 1. 检查平移
    // -------------------------
    double tx_diff = H_match.at<double>(0,2) - H_gps.at<double>(0,2); 
    double ty_diff = H_match.at<double>(1,2) - H_gps.at<double>(1,2); 
    double translation_diff = std::sqrt(tx_diff*tx_diff + ty_diff*ty_diff);
    // LOG_DEBUG("Translation tx: " + std::to_string(tx_diff) + ", ty" + std::to_string(ty_diff) + ", t: " + std::to_string(translation_diff));
    if (translation_diff > max_translation) {
        return false;
    }

    // -------------------------
    // 2. 检查旋转
    // -------------------------
    cv::Mat A = H_match(cv::Rect(0, 0, 2, 2));
    cv::SVD svd(A);
    cv::Mat R = svd.u * svd.vt;
    double sx = svd.w.at<double>(0);
    double sy = svd.w.at<double>(1);
    double angle = std::atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180.0 / CV_PI;
    // LOG_DEBUG("Scale sx: " + std::to_string(sx) + ", sy: " + std::to_string(sy) + ", Angle: " + std::to_string(angle));
    if (std::abs(angle) > max_rotation_angle) {
        return false;
    }

    if ((std::abs(sx - 1.0) > max_scale) || (std::abs(sy - 1.0) > max_scale))
    {
        return false;
    }

    // -------------------------
    // 3. 检查图像变换后大小变化
    // -------------------------
    // 定义图像四个角点 (顺时针)
    std::vector<cv::Point2f> corners_src = {
        cv::Point2f(0, 0),
        cv::Point2f((float)imgSize.width, 0),
        cv::Point2f((float)imgSize.width, (float)imgSize.height),
        cv::Point2f(0, (float)imgSize.height)
    };

    std::vector<cv::Point2f> corners_dst;
    cv::perspectiveTransform(corners_src, corners_dst, H_match);
    cv::Rect2f bbox_dst = cv::boundingRect(corners_dst);

    float orig_area = (float)imgSize.width * imgSize.height;
    float new_area  = bbox_dst.width * bbox_dst.height;
    float scale_ratio = new_area / orig_area;
    // LOG_DEBUG("Area orig: " + std::to_string(orig_area) + ", new: " + std::to_string(new_area) + ", scale: " + std::to_string(scale_ratio));
    if (scale_ratio < (1.0f - max_scale_ratio) || scale_ratio > (1.0f + max_scale_ratio)) {
        return false;
    }
    return true;
}


void getWarppedAcc2(cv::Mat &warped, cv::Rect &warpedRect, cv::Mat *original, cv::Mat H)
{
    clock_t start = clock();
    int h = original->rows;
    int w = original->cols;
    int channel = original->channels();

    // 计算变换后图像边界
    std::vector<cv::Point2f> corners = {
        cv::Point2f(0, 0), cv::Point2f(w, 0),
        cv::Point2f(w, h), cv::Point2f(0, h)};
    std::vector<cv::Point2f> warpedCorners;
    cv::perspectiveTransform(corners, warpedCorners, H);

    // 计算最小包围盒
    warpedRect = cv::boundingRect(warpedCorners);
    int new_width = warpedRect.width;
    int new_height = warpedRect.height;
    // std::cout << "wraped Rect: " << warpedRect.x << ", " << warpedRect.y << ", "
    //           << warpedRect.width << ", " << warpedRect.height << std::endl;

    // 调整单应性矩阵（添加平移）
    cv::Mat T = (cv::Mat_<double>(3, 3) << 1, 0, -warpedRect.x,
                 0, 1, -warpedRect.y,
                 0, 0, 1);
    cv::Mat H_adjusted = T * H;

    cv::warpPerspective(
        *original,                       // 输入图像
        warped,                          // 输出图像
        H_adjusted,                      // 组合后的单应性矩阵
        cv::Size(new_width, new_height), // 输出尺寸
        cv::INTER_LINEAR,                // 双线性插值
        cv::BORDER_CONSTANT,             // 边界填充
        cv::Scalar(0, 0, 0)              // 填充黑色
    );

    LOG_INFO("Warping took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
}


