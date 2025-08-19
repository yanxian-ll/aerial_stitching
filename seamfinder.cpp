#include "seamfinder.h"
#include "debug_utils.h"

cv::Mat seamMerge(
    cv::Mat &dom,
    cv::Mat &warped,
    cv::Rect &warpedRect,
    SeamFinderType seamType,
    BlendType blendType,
    std::string debug_file,
    int maxSeamWidth,
    float blendStrength,
    int numBands)
{

    auto saveDebug = [&](const std::string &suffix, const cv::Mat &img) {
        if (!debug_file.empty()) {
            if (!cv::imwrite(debug_file + suffix, img)) {
                LOG_ERROR("Failed to save debug image: " + debug_file + suffix);
            }
        }
    };

    auto extractMaskAndRGB = [](cv::Mat &img, cv::Mat &mask)
    {
        if (img.channels() == 4)
        {
            std::vector<cv::Mat> channels;
            cv::split(img, channels);
            mask = channels[3];
            cv::merge(std::vector<cv::Mat>(channels.begin(), channels.begin() + 3), img);
        }
        else
        {
            mask = cv::Mat(img.size(), CV_8UC1, cv::Scalar(255));
        }
    };

    auto mergeBGRMask = [](const cv::Mat &bgr, const cv::Mat &mask, cv::Mat &bgra)
    {
        std::vector<cv::Mat> bgr_a;
        std::vector<cv::Mat> bgr_split(3);
        cv::split(bgr, bgr_split);
        bgr_a = {bgr_split[0], bgr_split[1], bgr_split[2], mask};
        cv::merge(bgr_a, bgra);
    };

    clock_t start = clock();
    int channels_dom = dom.channels();

    cv::Mat mask1, mask2;
    extractMaskAndRGB(dom, mask1);
    extractMaskAndRGB(warped, mask2);

    cv::Rect domRect(0, 0, dom.cols, dom.rows);
    cv::Rect unionRect = domRect | warpedRect;

    // 创建全尺寸画布
    cv::Mat img1_full(unionRect.height, unionRect.width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat img2_full(unionRect.height, unionRect.width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask1_full(unionRect.height, unionRect.width, CV_8U, cv::Scalar(0));
    cv::Mat mask2_full(unionRect.height, unionRect.width, CV_8U, cv::Scalar(0));

    // 偏移量
    cv::Point domOffset(domRect.x - unionRect.x, domRect.y - unionRect.y);
    cv::Point warpedOffset(warpedRect.x - unionRect.x, warpedRect.y - unionRect.y);

    dom.copyTo(img1_full(cv::Rect(domOffset, dom.size())));
    mask1.copyTo(mask1_full(cv::Rect(domOffset, mask1.size())));
    warped.copyTo(img2_full(cv::Rect(warpedOffset, warped.size())));
    mask2.copyTo(mask2_full(cv::Rect(warpedOffset, mask2.size())));

    // 计算重叠区域
    cv::Rect overlapRect = domRect & warpedRect;

    if (overlapRect.area() > 0 && seamType != SEAM_NONE)
    {
        // 转换坐标系
        overlapRect.x -= unionRect.x;
        overlapRect.y -= unionRect.y;

        cv::Mat img1_overlap = img1_full(overlapRect);
        cv::Mat img2_overlap = img2_full(overlapRect);
        cv::Mat mask1_overlap = mask1_full(overlapRect);
        cv::Mat mask2_overlap = mask2_full(overlapRect);

        saveDebug("_img1_overlap.png", img1_overlap);
        saveDebug("_img2_overlap.png", img2_overlap);

        // 如果太大，需要先缩放
        double scale = 1.0;
        if (img1_overlap.cols > maxSeamWidth)
        {
            scale = static_cast<double>(maxSeamWidth) / img1_overlap.cols;
        }

        cv::Mat img1_proc, img2_proc, mask1_proc, mask2_proc;
        if (scale < 1.0)
        {
            cv::resize(img1_overlap, img1_proc, cv::Size(), scale, scale, cv::INTER_LINEAR);
            cv::resize(img2_overlap, img2_proc, cv::Size(), scale, scale, cv::INTER_LINEAR);
            cv::resize(mask1_overlap, mask1_proc, cv::Size(), scale, scale, cv::INTER_NEAREST);
            cv::resize(mask2_overlap, mask2_proc, cv::Size(), scale, scale, cv::INTER_NEAREST);
        }
        else
        {
            img1_proc = img1_overlap;
            img2_proc = img2_overlap;
            mask1_proc = mask1_overlap;
            mask2_proc = mask2_overlap;
        }

        cv::Mat img1_f32, img2_f32;
        img1_proc.convertTo(img1_f32, CV_32F);
        img2_proc.convertTo(img2_f32, CV_32F);

        std::vector<cv::UMat> imgs_umats, masks_umats;
        imgs_umats.push_back(img1_f32.getUMat(cv::ACCESS_READ));
        imgs_umats.push_back(img2_f32.getUMat(cv::ACCESS_READ));
        masks_umats.push_back(mask1_proc.getUMat(cv::ACCESS_READ));
        masks_umats.push_back(mask2_proc.getUMat(cv::ACCESS_READ));

        // 根据参数选择缝合线算法
        cv::Ptr<cv::detail::SeamFinder> seamFinder;
        switch (seamType)
        {
        case SEAM_GRAPH_CUT:
            seamFinder = cv::makePtr<cv::detail::GraphCutSeamFinder>(
                cv::detail::GraphCutSeamFinderBase::COST_COLOR, 10000.f, 1000.f);
            break;
        case SEAM_VORONOI:
            seamFinder = cv::makePtr<cv::detail::VoronoiSeamFinder>();
            break;
        case SEAM_DP:
            seamFinder = cv::makePtr<cv::detail::DpSeamFinder>("COLOR");
            break;
        default:
            seamFinder = cv::makePtr<cv::detail::GraphCutSeamFinder>(
                cv::detail::GraphCutSeamFinderBase::COST_COLOR, 10000.f, 1000.f);
            break;
        }

        // 计算接缝
        std::vector<cv::Point> corners = {cv::Point(0, 0), cv::Point(0, 0)};
        seamFinder->find(imgs_umats, corners, masks_umats);

        cv::Mat mask1_after, mask2_after;
        masks_umats[0].copyTo(mask1_after);
        masks_umats[1].copyTo(mask2_after);

        // 更新掩码
        if (scale < 1.0)
        {
            cv::Mat mask1_resized, mask2_resized;
            cv::resize(mask1_after, mask1_resized, mask1_overlap.size(), 0, 0, cv::INTER_NEAREST);
            cv::resize(mask2_after, mask2_resized, mask2_overlap.size(), 0, 0, cv::INTER_NEAREST);
            mask1_resized.copyTo(mask1_overlap);
            mask2_resized.copyTo(mask2_overlap);
        }
        else
        {
            mask1_after.copyTo(mask1_overlap);
            mask2_after.copyTo(mask2_overlap);
        }

        saveDebug("_mask1_overlap_graphcut.png", mask1_overlap);
        saveDebug("_mask2_overlap_graphcut.png", mask2_overlap);
    }

    // 根据blendType融合
    cv::Mat result_bgra;
    if (blendType == BLEND_NONE)
    {
        cv::Mat img1_full_bgra, img2_full_bgra;
        mergeBGRMask(img1_full, mask1_full, img1_full_bgra);
        mergeBGRMask(img2_full, mask2_full, img2_full_bgra);

        result_bgra = cv::Mat(unionRect.height, unionRect.width, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        img1_full_bgra.copyTo(result_bgra, mask1_full);
        img2_full_bgra.copyTo(result_bgra, mask2_full);
    }
    else
    {
        // 转 16S
        cv::Mat img1_16s, img2_16s;
        img1_full.convertTo(img1_16s, CV_16S);
        img2_full.convertTo(img2_16s, CV_16S);

        cv::Ptr<cv::detail::Blender> blender;
        if (blendType == BLEND_FEATHER)
        {
            cv::Ptr<cv::detail::FeatherBlender> fb = cv::makePtr<cv::detail::FeatherBlender>();
            fb->setSharpness(blendStrength);
            blender = fb;
        }
        else if (blendType == BLEND_MULTIBAND)
        {
            cv::Ptr<cv::detail::MultiBandBlender> mb = cv::makePtr<cv::detail::MultiBandBlender>(false, numBands);
            blender = mb;
        }

        blender->prepare(cv::Rect(0, 0, unionRect.width, unionRect.height));
        blender->feed(img1_16s, mask1_full, cv::Point(0, 0));
        blender->feed(img2_16s, mask2_full, cv::Point(0, 0));

        cv::Mat result_s16, result_mask;
        blender->blend(result_s16, result_mask);

        result_s16.convertTo(result_bgra, CV_8UC4);
        std::vector<cv::Mat> rgba(4);
        cv::split(result_bgra, rgba);
        rgba[3] = result_mask;
        cv::merge(rgba, result_bgra);
    }

    if (channels_dom == 4)
    {
        LOG_INFO("Stitching took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
        return result_bgra;
    }
    else
    {
        cv::Mat canvas_bgr;
        cv::cvtColor(result_bgra, canvas_bgr, cv::COLOR_BGRA2BGR);
        LOG_INFO("Stitching took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
        return canvas_bgr;
    }
}
