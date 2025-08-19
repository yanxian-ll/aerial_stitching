#ifndef IMAGE_H
#define IMAGE_H

#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <gdal_priv.h>
#include <cpl_conv.h>
#include <cpl_multiproc.h> // 多线程支持
#include <cpl_vsi.h>       // 包含文件系统函数

#include "metadata.h"
#include "debug_utils.h"

void saveDOMAsGeoTIFF(
    const cv::Mat &dom,
    const std::string &outputPath,
    int epsg,
    double origin_x,
    double origin_y,
    double gsd);

int GenerateTiles(
    const std::string &input_file,
    const std::string &output_dir,
    const std::string &profile = "geodetic",   // wgs84
    const std::string &resampling = "average", // 重采样
    const std::string &zoom = "17-19",
    bool resume = true, // 更快
    const std::string &nodata = "0,0,0",
    bool tmscompatible = true, // tms格式
    bool xyz = false,          //
    bool exclude = true,       // 排除透明tile
    bool quiet = true,         // 不打印log
    int processes = -1,        //
    int tilesize = 256);       // 瓦块大小

class Image
{
public:
    Image() = default;

    bool readImage(const std::string &path, int conversion_code = -1)
    {
        image_path = path;

        clock_t start = clock();
        image = cv::imread(path, cv::IMREAD_COLOR);
        if (image.empty())
        {
            LOG_ERROR("Could not load image: " + path);
            return false;
        }
        if (conversion_code >= 0)
        {
            cv::cvtColor(image, image, conversion_code);
        }

        LOG_INFO("ReadImage Metadata took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
        return true;
    }

    bool readMetadata(const std::string &path)
    {
        image_path = path;

        clock_t start = clock();
        if (!metadata.extract(path))
        {
            LOG_ERROR("Could not load metadata: " + path);
            return false;
        }
        LOG_INFO("Extract Metadata took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
        return true;
    }

    bool read(const std::string &path, int conversion_code = -1)
    {
        image_path = path;

        if (!readImage(path, conversion_code))
        {
            return false;
        }
        if (!readMetadata(path))
        {
            return false;
        }

        return true;
    }

    bool empty() const
    {
        return image.empty();
    }

    // 添加深拷贝函数
    Image clone() const
    {
        Image cloned;
        cloned.image_path = this->image_path;
        cloned.image = this->image.clone();
        cloned.metadata = this->metadata;
        return cloned;
    }

public:
    std::string image_path;
    cv::Mat image;
    Metadata metadata;

public:
    // 缩放图像到指定分辨率
    void resize(int width, int height)
    {
        cv::resize(image, image, cv::Size(width, height), 0, 0, cv::INTER_CUBIC);
        // 根据缩放系数，更新metadata的GSD
        double scale_x = static_cast<double>(width) / metadata.image_width;
        double scale_y = static_cast<double>(height) / metadata.image_height;
        metadata.gsd /= (scale_x + scale_y) / 2.0;
        metadata.x_res /= scale_x;
        metadata.y_res /= scale_y;
        // 更新metadata宽高
        metadata.image_width = width;
        metadata.image_height = height;

        // std::cout<< "Resized image to: " << width << "x" << height << std::endl;
        // std::cout<< "Updated GSD: " << metadata.gsd << std::endl;
    }

    // 缩放图像到指定分辨率
    // 这里假设分辨率模式是1080p的比例
    void resize_1080(int resolution_mode)
    {
        clock_t start = clock();
        int new_width = static_cast<int>(image.cols * resolution_mode / 1080.0);
        int new_height = static_cast<int>(image.rows * resolution_mode / 1080.0);
        resize(new_width, new_height);
        LOG_INFO("ResizeImage took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
    }

    void coarse_ortho()
    {
        /* 执行单张图像的粗正射校正

        参数:
            img: 输入图像 (BGR 或灰度图)
            gyaw: 云台偏航角(度)，顺时针为正方向
            scale: 图像缩放比例，默认为1.0

        返回:
            校正后的图像，边界填充黑色

        算法说明:
            1. 计算旋转中心为图像中心
            2. 根据偏航角计算旋转矩阵
            3. 调整图像尺寸以适应旋转后的内容
            4. 执行仿射变换完成校正
        */

        clock_t start = clock();
        // 获取图像尺寸
        const int w = image.cols;
        const int h = image.rows;
        const cv::Point2f center(w / 2.0f, h / 2.0f);

        // 计算旋转矩阵（注意角度取负以实现逆时针旋转）
        double gyaw = metadata.gimbal_yaw;
        cv::Mat M = cv::getRotationMatrix2D(center, -gyaw, 1.0);

        // 提取旋转矩阵的数值
        const double cos_val = std::abs(M.at<double>(0, 0));
        const double sin_val = std::abs(M.at<double>(0, 1));

        // 计算旋转后的新尺寸
        const int new_w = static_cast<int>((h * sin_val) + (w * cos_val));
        const int new_h = static_cast<int>((h * cos_val) + (w * sin_val));

        // 调整旋转矩阵的平移分量
        M.at<double>(0, 2) += (new_w - w) / 2.0;
        M.at<double>(1, 2) += (new_h - h) / 2.0;

        // 执行仿射变换
        cv::Mat result;
        cv::warpAffine(
            image,                  // 输入图像
            image,                  // 输出图像
            M,                      // 变换矩阵
            cv::Size(new_w, new_h), // 输出尺寸
            cv::INTER_NEAREST,      // 使用最近邻插值
            cv::BORDER_CONSTANT,    // 边界填充
            cv::Scalar(0, 0, 0)     // 填充黑色
        );

        // 更新metadata
        metadata.image_width = new_w;
        metadata.image_height = new_h;
        metadata.calculateImageBounds();
        LOG_INFO("CoarseOrtho took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
    }
};

void saveDOMAsGeoTIFF(const cv::Mat &dom, const std::string &outputPath,
                      int epsg, double origin_x, double origin_y, double gsd)
{
    clock_t start = clock();
    GDALAllRegister();

    // 获取DOM尺寸
    int width = dom.cols;
    int height = dom.rows;
    int channels = dom.channels();

    // 确定输出波段数（忽略alpha通道）
    int outputBands = (channels == 4) ? 3 : channels;

    // 创建输出数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    GDALDataset *dataset = driver->Create(
        outputPath.c_str(),
        width, height,
        outputBands,
        GDT_Byte,
        nullptr);

    if (!dataset)
    {
        LOG_ERROR("Error: Could not create GeoTIFF: " + outputPath);
        return;
    }

    // 设置地理变换参数
    double adfGeoTransform[6] = {
        origin_x, // 左上角X坐标
        gsd,      // 东西方向像素分辨率
        0,        // 旋转系数
        origin_y, // 左上角Y坐标
        0,        // 旋转系数
        -gsd      // 南北方向像素分辨率（负值）
    };
    dataset->SetGeoTransform(adfGeoTransform);

    // 设置投影坐标系（UTM）
    std::string proj = "EPSG:" + std::to_string(epsg);
    OGRSpatialReference oSRS;
    oSRS.SetFromUserInput(proj.c_str());
    char *pszProjection = nullptr;
    oSRS.exportToWkt(&pszProjection);
    dataset->SetProjection(pszProjection);
    CPLFree(pszProjection);

    // 准备图像数据（转换为RGB顺序）
    cv::Mat outputImage;
    if (channels == 4)
    {
        cv::cvtColor(dom, outputImage, cv::COLOR_BGRA2RGB);
    }
    else if (channels == 3)
    {
        cv::cvtColor(dom, outputImage, cv::COLOR_BGR2RGB);
    }
    else
    {
        dom.copyTo(outputImage);
    }

    // 写入波段数据
    for (int band = 0; band < outputBands; ++band)
    {
        GDALRasterBand *rasterBand = dataset->GetRasterBand(band + 1);
        CPLErr err = rasterBand->RasterIO(
            GF_Write, 0, 0, width, height,
            outputImage.data + band,
            width, height, GDT_Byte,
            outputBands,        // 像素间距
            outputBands * width // 行间距
        );
        if (err != CE_None)
        {
            std::cerr << "Error: RasterIO write failed with code " << err << std::endl;
            break;
        }
    }

    // 清理资源
    GDALClose(dataset);
    LOG_INFO("SaveTIFF took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
}

int GenerateTiles(
    const std::string &input_file,
    const std::string &output_dir,
    const std::string &profile,
    const std::string &resampling,
    const std::string &zoom,
    bool resume,
    const std::string &nodata,
    bool tmscompatible,
    bool xyz,
    bool exclude,
    bool quiet,
    int processes,
    int tilesize)
{
    if (processes <= 0) {
        int hw_threads = std::thread::hardware_concurrency();
        processes = std::max(1, hw_threads / 2); // 用一半核心
    }

    clock_t start = clock();
    std::ostringstream cmd;
    cmd << "python3 /usr/local/bin/gdal2tiles.py ";

    // 添加带值的参数
    if (!profile.empty())
        cmd << "--profile " << profile << " ";
    if (!resampling.empty())
        cmd << "--resampling " << resampling << " ";
    if (!zoom.empty())
        cmd << "--zoom " << zoom << " ";
    if (!nodata.empty())
        cmd << "--srcnodata " << nodata << " ";
    if (tilesize > 0)
        cmd << "--tilesize " << tilesize << " ";
    if (processes > 0)
        cmd << "--processes " << processes << " ";

    // 添加布尔开关
    if (resume)
        cmd << "--resume ";
    if (xyz)
        cmd << "--xyz ";
    if (tmscompatible)
        cmd << "--tmscompatible ";
    if (exclude)
        cmd << "--exclude ";
    if (quiet)
        cmd << "--quiet ";
    
    // 不需要kml和webviewer
    cmd << "--no-kml --webviewer 'none' ";

    // 添加输入/输出路径（处理空格问题）
    cmd << "\"" << input_file << "\" ";
    if (!output_dir.empty())
        cmd << "\"" << output_dir << "\"";

    // // print cmd
    // std::cout << cmd.str() <<std::endl;

    int exit_code = std::system(cmd.str().c_str());
    LOG_INFO("GenerateTiles took: " + std::to_string(float(clock() - start) / CLOCKS_PER_SEC) + " seconds");
    return exit_code;
}

#endif // IMAGE_H