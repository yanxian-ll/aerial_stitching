#include <iostream>
#include <cmath>
#include <cctype>
#include <algorithm>

#include "metadata.h"
#include "debug_utils.h"

Metadata::Metadata() = default;

Metadata::~Metadata() = default;

bool Metadata::extract(const std::string &image_path)
{
    this->image_path = image_path;

    try
    {
        // 读取图像元数据
        auto image = Exiv2::ImageFactory::open(image_path);

        image->readMetadata();
        Exiv2::ExifData &exifData = image->exifData();
        Exiv2::XmpData &xmpData = image->xmpData();

        parseExifData(exifData, xmpData);
        convertToUTM();

        // 打印metadata信息
        printMetadata();

        return true;
    }
    catch (const Exiv2::Error &e)
    {
        LOG_ERROR("Exiv2 error: " + std::string(e.what()));
        return false;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("Exiv2 error: " + std::string(e.what()));
        return false;
    }
}

bool Metadata::is_nadir()
{

    // 检查云台横滚角，只支持0°或180°两种状态
    if (abs(gimbal_roll - 0) > 0.5 && abs(gimbal_roll - 180) > 0.5)
    {
        // std::cout << "Skip image:" << image_path << ", roll of gimbal " << gimbal_roll << " only support 0° or 180°" << std::endl;
        return false;
    }

    // 正射模式需要云台俯仰角为-90°(垂直向下)
    if (abs(gimbal_pitch + 90) > 0.5)
    {
        // std::cout << "Skip image:" << image_path << ", pitch of gimbal " << gimbal_pitch << " must be -90°" << std::endl;
        return false;
    }

    // 根据云台横滚角调整偏航角:
    // - 180°状态需要将偏航角旋转180°
    if (abs(gimbal_roll - 180) < 0.5)
    {
        gimbal_yaw = fmod(gimbal_yaw + 180, 360);
    }
    return true;
}

// 解析函数
void Metadata::parseExifData(const Exiv2::ExifData &exifData, const Exiv2::XmpData &xmpData)
{
    // 辅助lambda：安全获取值
    auto getXmpValue = [&](const std::string &key, double default_val = 0.0) -> double
    {
        auto pos = xmpData.findKey(Exiv2::XmpKey(key));
        return (pos != xmpData.end()) ? parseRational(pos->toRational()) : default_val;
    };

    // // 打印所有keys
    // for (const auto& key : exifData) {
    //     std::cout << key.key() << ": " << key.toString() << std::endl;
    // }

    // for (const auto& key : xmpData) {
    //     std::cout << key.key() << ": " << key.toString() << std::endl;
    // }

    // 注册命名空间，避免 Exiv2 报错
    Exiv2::XmpProperties::registerNs("http://www.dji.com/drone-dji/1.0/", "drone-dji");
    Exiv2::XmpProperties::registerNs("http://www.dji.com/drone/1.0/", "drone");

    // 解析姿态角数据
    const std::vector<std::string> gimbalPrefixes = {"Xmp.drone-dji", "Xmp.drone"};
    for (const auto &prefix : gimbalPrefixes)
    {
        if (xmpData.findKey(Exiv2::XmpKey(prefix + ".GimbalYawDegree")) != xmpData.end())
        {
            gimbal_yaw = getXmpValue(prefix + ".GimbalYawDegree");
            gimbal_pitch = getXmpValue(prefix + ".GimbalPitchDegree");
            gimbal_roll = getXmpValue(prefix + ".GimbalRollDegree");
            // std::cout << "Read Gimbal Yaw: " << gimbal_yaw << ", Pitch: " << gimbal_pitch << ", Roll: " << gimbal_roll << std::endl;
            break;
        }
    }

    // 解析GPS数据
    if (xmpData.findKey(Exiv2::XmpKey("Xmp.drone-dji.AbsoluteAltitude")) != xmpData.end()){
        alt = getXmpValue("Xmp.drone-dji.AbsoluteAltitude");
    }
    else if (xmpData.findKey(Exiv2::XmpKey("Xmp.drone.AbsoluteAltitude")) != xmpData.end()){
        alt = getXmpValue("Xmp.drone.AbsoluteAltitude");
    }
    else{
        auto pos = exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSAltitude"));
        if (pos != exifData.end()){
            alt = static_cast<double>(parseRational(pos->toRational()));
        }
    }

    if (xmpData.findKey(Exiv2::XmpKey("Xmp.drone-dji.GpsLongitude")) != xmpData.end())
    {
        lon = getXmpValue("Xmp.drone-dji.GpsLongitude");
        lat = getXmpValue("Xmp.drone-dji.GpsLatitude");
    }
    else if (xmpData.findKey(Exiv2::XmpKey("Xmp.drone.GpsLongitude")) != xmpData.end())
    {
        lon = getXmpValue("Xmp.drone.GpsLongitude");
        lat = getXmpValue("Xmp.drone.GpsLatitude");
    }
    else
    {
        // 尝试从EXIF中获取GPS信息
        auto pos = exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitude"));
        if (pos != exifData.end()){            
            lon = static_cast<double>(convertGpsToDecimal(pos->toString()));
        }
        pos = exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitude"));
        if (pos != exifData.end()){
            lat = static_cast<double>(convertGpsToDecimal(pos->toString()));
        }
    }

    // 解析图像尺寸
    const std::vector<std::string> widthKeys = {
        "Exif.Photo.PixelXDimension",
        "Exif.Image.ImageWidth"};
    const std::vector<std::string> widthXmpKeys = {
        "Xmp.drone.ExifImageWidth",
        "Xmp.drone-dji.ExifImageWidth"};
    const std::vector<std::string> heightKeys = {
        "Exif.Photo.PixelYDimension",
        "Exif.Image.ImageLength"};
    const std::vector<std::string> heightXmpKeys = {
        "Xmp.drone.ExifImageHeight",
        "Xmp.drone-dji.ExifImageHeight"};

    // 优先EXIF
    for (const auto &key : widthKeys)
    {
        auto pos = exifData.findKey(Exiv2::ExifKey(key));
        if (pos != exifData.end())
        {
            image_width = static_cast<int>(parseRational(pos->toRational()));
            break;
        }
    }
    // 若EXIF未找到，查XMP
    if (image_width == 0)
    {
        for (const auto &key : widthXmpKeys)
        {
            auto pos = xmpData.findKey(Exiv2::XmpKey(key));
            if (pos != xmpData.end())
            {
                image_width = static_cast<int>(parseRational(pos->toRational()));
                break;
            }
        }
    }

    for (const auto &key : heightKeys)
    {
        auto pos = exifData.findKey(Exiv2::ExifKey(key));
        if (pos != exifData.end())
        {
            image_height = static_cast<int>(parseRational(pos->toRational()));
            break;
        }
    }
    if (image_height == 0)
    {
        for (const auto &key : heightXmpKeys)
        {
            auto pos = xmpData.findKey(Exiv2::XmpKey(key));
            if (pos != xmpData.end())
            {
                image_height = static_cast<int>(parseRational(pos->toRational()));
                break;
            }
        }
    }
    // std::cout << "Read Height: " << image_height << ", Width: " << image_width << std::endl;

    // 计算地面采样距离(GSD)
    double focal_length{-1}, relative_alt{-1};
    const std::vector<std::string> gsdPrefixes = {"Xmp.drone-dji", "Xmp.drone"};
    for (const auto &prefix : gsdPrefixes)
    {
        if (xmpData.findKey(Exiv2::XmpKey(prefix + ".CalibratedFocalLength")) != xmpData.end())
        {
            focal_length = getXmpValue(prefix + ".CalibratedFocalLength", -1.0);
            // std::cout << "Read Xmp.drone-dji.CalibratedFocalLength: " 
            //     << focal_length << std::endl;
            break;
        }
    }
    
    if (focal_length == -1)
    {   
        double f_mm{-1}, f_eq35{-1};
        auto pos = exifData.findKey(Exiv2::ExifKey("Exif.Photo.FocalLength"));
        if (pos != exifData.end())
            f_mm = static_cast<double>(parseRational(pos->toRational()));

        pos = exifData.findKey(Exiv2::ExifKey("Exif.Photo.FocalLengthIn35mmFilm"));
        if (pos != exifData.end())
            f_eq35 = static_cast<double>(pos->toLong());

        // std::cout << "Read FocalLength (mm): " << f_mm 
        //       << ", FocalLengthIn35mmFilm: " << f_eq35 << std::endl;
        
        if (f_mm > 0 && f_eq35  > 0) {
            const double diag35mm = std::sqrt(36.0 * 36.0 + 24.0 * 24.0); // ≈ 43.266mm
            double sensor_diag_mm = diag35mm * (f_mm / f_eq35);
            double img_diag_px = std::sqrt(image_width * image_width +
                                        image_height * image_height);
            double pixel_size_mm = sensor_diag_mm / img_diag_px;
            focal_length = f_mm / pixel_size_mm;
            // std::cout << "Computed focal length in pixels: " << focal_length << std::endl;
        }
    }

    for (const auto &prefix : gsdPrefixes)
    {
        if (xmpData.findKey(Exiv2::XmpKey(prefix + ".RelativeAltitude")) != xmpData.end())
        {
            relative_alt = getXmpValue(prefix + ".RelativeAltitude", -1.0);
            // std::cout << "Read Relative Altitude: " << relative_alt << std::endl;
            break;
        }
    }

    if (relative_alt == -1) 
    {
        relative_alt = alt;
        // std::cout << "Cann't read relative altitude, use altitude.\n";
    }

    if (focal_length > 0 && relative_alt > 0)
    {
        gsd = relative_alt / focal_length;
        x_res = gsd;
        y_res = -gsd; // 假设x和y分辨率相同
        // std::cout << "Read GSD: " << gsd << std::endl;
    }
    else
    {
        // TODO: 
    }
}

// 转换为UTM坐标系
void Metadata::convertToUTM()
{
    // 自动计算UTM带号
    int zone = static_cast<int>((lon + 180.0) / 6) + 1;
    if (zone > 60) zone = 60;
    bool is_north = lat >= 0;
    epsg = is_north ? 32600 + zone : 32700 + zone;

    PJ_CONTEXT *ctx = proj_context_create();
    // const char* src_crs = "OGC:CRS84"  // 经度、纬度顺序的WGS84
    const char* src_crs = "+proj=longlat +datum=WGS84 +no_defs +type=crs";
    PJ *utm_converter = proj_create_crs_to_crs(
        ctx,
        src_crs, 
        ("EPSG:" + std::to_string(epsg)).c_str(),
        nullptr);

    if (!utm_converter)
    {
        proj_context_destroy(ctx);
        throw std::runtime_error("Failed to create UTM converter");
    }

    PJ_COORD coord = proj_coord(lon, lat, alt, 0);
    PJ_COORD result = proj_trans(utm_converter, PJ_FWD, coord);

    utm_x = result.xyz.x;
    utm_y = result.xyz.y;

    proj_destroy(utm_converter);
    proj_context_destroy(ctx);
}

// 计算图像地理范围
void Metadata::calculateImageBounds()
{
    // 左上角坐标
    x0 = utm_x - gsd * (image_width / 2.0);
    y0 = utm_y + gsd * (image_height / 2.0);

    // 右下角坐标
    x1 = utm_x + gsd * (image_width / 2.0);
    y1 = utm_y - gsd * (image_height / 2.0);
}

// 辅助函数：解析有理数
double Metadata::parseRational(const Exiv2::Rational &rational) const
{
    return static_cast<double>(rational.first) / rational.second;
}

// Function to convert a GPS coordinate string like "28/1 39/1 1062/100" to a decimal degree
double Metadata::convertGpsToDecimal(const std::string& gpsString) {
    size_t firstSpace = gpsString.find(' ');
    size_t secondSpace = gpsString.find(' ', firstSpace + 1);

    // Extract the degrees, minutes, and seconds parts
    std::string degreesStr = gpsString.substr(0, firstSpace);
    std::string minutesStr = gpsString.substr(firstSpace + 1, secondSpace - firstSpace - 1);
    std::string secondsStr = gpsString.substr(secondSpace + 1);

    // Convert to decimal values (degrees, minutes, seconds)
    double degrees = std::stod(degreesStr.substr(0, degreesStr.find('/'))) / std::stod(degreesStr.substr(degreesStr.find('/') + 1));
    double minutes = std::stod(minutesStr.substr(0, minutesStr.find('/'))) / std::stod(minutesStr.substr(minutesStr.find('/') + 1)) / 60.0;
    double seconds = std::stod(secondsStr.substr(0, secondsStr.find('/'))) / std::stod(secondsStr.substr(secondsStr.find('/') + 1)) / 3600.0;
    return degrees + minutes + seconds;
}

// 辅助函数：转换GPS坐标
double Metadata::convertGPSCoordinate(const Exiv2::Rational *coordinates, char direction) const
{
    double degrees = parseRational(coordinates[0]);
    double minutes = parseRational(coordinates[1]);
    double seconds = parseRational(coordinates[2]);

    double decimal = degrees + minutes / 60.0 + seconds / 3600.0;

    // 处理方向
    if (direction == 'S' || direction == 'W')
    {
        decimal = -decimal;
    }

    return decimal;
}

void Metadata::printMetadata() const
{
    LOG_INFO("\n-------------------------------------------------------");
    LOG_INFO("Metadata: " + image_path);
    LOG_INFO("-------------------------------------------------------");
    LOG_INFO("  Latitude: " + std::to_string(lat));
    LOG_INFO("  Longitude: " + std::to_string(lon));
    LOG_INFO("  Altitude: " + std::to_string(alt));
    LOG_INFO("  Gimbal Yaw: " + std::to_string(gimbal_yaw));
    LOG_INFO("  Gimbal Pitch: " + std::to_string(gimbal_pitch));
    LOG_INFO("  Gimbal Roll: " + std::to_string(gimbal_roll));
    LOG_INFO("  X Resolution: " + std::to_string(x_res));
    LOG_INFO("  Y Resolution: " + std::to_string(y_res));
    LOG_INFO("  EPSG Code: " + std::to_string(epsg));
    LOG_INFO("  UTM X: " + std::to_string(utm_x));
    LOG_INFO("  UTM Y: " + std::to_string(utm_y));
    LOG_INFO("  GSD: " + std::to_string(gsd));
    LOG_INFO("  Image Width: " + std::to_string(image_width));
    LOG_INFO("  Image Height: " + std::to_string(image_height));
    LOG_INFO("-------------------------------------------------------\n");

}

// ----------------------------------------------------------------
json build_image_json(const Metadata &metadata,
                      const int &project_id,
                      const std::string &image_path)
{
    // 创建 PROJ 上下文
    PJ_CONTEXT *C = proj_context_create();
    if (!C) throw std::runtime_error("Failed to create PROJ context.");

    // 构造源/目标 CRS
    const std::string src_epsg = "EPSG:" + std::to_string(metadata.epsg);
    const char *dst_epsg = "EPSG:4326";         // 地理坐标（WGS84）

    PJ *P = proj_create_crs_to_crs(C, src_epsg.c_str(), dst_epsg, nullptr);
    if (!P)
    {
        proj_context_destroy(C);
        throw std::runtime_error("Failed to create CRS transformer from " + src_epsg + " to EPSG:4326");
    }

    // 规范化（确保输出为 lon,lat 顺序，输入为常见的 easting,northing 顺序）
    if (PJ *P_vis = proj_normalize_for_visualization(C, P))
    {
        proj_destroy(P);
        P = P_vis;
    }

    auto forward_xy_to_lonlat_deg = [&](double x, double y) -> std::array<double, 2>
    {
        PJ_COORD in = proj_coord(x, y, 0, 0);
        PJ_COORD out = proj_trans(P, PJ_FWD, in);

        if (!std::isfinite(out.lp.lam) || !std::isfinite(out.lp.phi))
        {
            int err = proj_errno(P);
            std::string msg = "[PROJ] transform failed";
#ifdef PROJ_VERSION_MAJOR
#if PROJ_VERSION_MAJOR >= 6
            if (err)
                msg += (": " + std::string(proj_errno_string(err)));
#endif
#endif
            throw std::runtime_error(msg);
        }

        double lon = out.lp.lam;
        double lat = out.lp.phi;
        return {lon, lat}; // 注意顺序：(lon, lat)
    };

    //  角点 & 中心点（注意：x,y 必须与 metadata.epsg 匹配）
    const double x_center = (metadata.x0 + metadata.x1) * 0.5;
    const double y_center = (metadata.y0 + metadata.y1) * 0.5;

    std::vector<std::pair<double, double>> xy = {
        {metadata.x0, metadata.y0}, // 左下
        {metadata.x1, metadata.y0}, // 右下
        {metadata.x1, metadata.y1}, // 右上
        {metadata.x0, metadata.y1}, // 左上
        {x_center, y_center}        // 中心
    };

    std::vector<std::array<double, 2>> trans_lla; // (lon, lat)
    trans_lla.reserve(xy.size());

    std::vector<double> lons, lats;
    lons.reserve(xy.size());
    lats.reserve(xy.size());

    for (const auto &p : xy)
    {
        auto ll = forward_xy_to_lonlat_deg(p.first, p.second);
        trans_lla.push_back(ll);
        lons.push_back(ll[0]);
        lats.push_back(ll[1]);
    }

    // // 打印（固定小数，两位）
    // std::cout << "epsg: " << src_epsg <<std::endl;
    // std::cout << std::fixed << std::setprecision(2);
    // std::cout << "utm center: " << x_center << ' ' << y_center << std::endl;

    // const auto &center_ll = trans_lla.back(); // 最后一个是中心点
    // std::cout << "latlon center: " << center_ll[1] << ' ' << center_ll[0] << std::endl;

    // 计算 bbox_lla: [min_lon, min_lat, max_lon, max_lat]
    auto [min_lon_it, max_lon_it] = std::minmax_element(lons.begin(), lons.end());
    auto [min_lat_it, max_lat_it] = std::minmax_element(lats.begin(), lats.end());
    std::array<double, 4> bbox_lla = {*min_lon_it, *min_lat_it, *max_lon_it, *max_lat_it};

    // 时间戳
    double now_seconds = std::chrono::duration<double>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();

    // 组装 JSON
    json j;
    j["project_id"] = project_id;
    j["bbox_lla"] = bbox_lla; // [min_lon, min_lat, max_lon, max_lat]
    j["camera_pitch"] = metadata.gimbal_pitch;
    j["camera_roll"] = metadata.gimbal_roll;
    j["camera_yaw"] = metadata.gimbal_yaw;
    j["image_path"] = image_path;
    j["image_time"] = now_seconds;
    j["platform_alt"] = metadata.alt;
    j["platform_lat"] = metadata.lat;
    j["platform_lon"] = metadata.lon;
    j["trans_lla"] = trans_lla; // [(lon,lat), ... 共5个点]

    // 释放资源
    proj_destroy(P);
    proj_context_destroy(C);

    return j;
}

json build_tile_json(const Metadata &metadata,
                     const int &project_id,
                     const std::string &image_path)
{
    // 建立坐标转换：EPSG:<src> -> EPSG:4326 ----
    PJ_CONTEXT *C = proj_context_create();
    if (!C) throw std::runtime_error("Failed to create PROJ context.");

    const std::string src_epsg = "EPSG:" + std::to_string(metadata.epsg);
    const char *dst_epsg = "EPSG:4326";

    PJ *P = proj_create_crs_to_crs(C, src_epsg.c_str(), dst_epsg, nullptr);
    if (!P)
    {
        proj_context_destroy(C);
        throw std::runtime_error("Failed to create CRS transformer from " + src_epsg + " to EPSG:4326");
    }

    // 规范化以便可视化（确保输出经纬为常见顺序 lon,lat）
    if (PJ *P_vis = proj_normalize_for_visualization(C, P))
    {
        proj_destroy(P);
        P = P_vis;
    }

    auto forward_xy_to_lonlat_deg = [&](double x, double y) -> std::array<double, 2>
    {
        PJ_COORD in = proj_coord(x, y, 0, 0);
        PJ_COORD out = proj_trans(P, PJ_FWD, in);

        if (!std::isfinite(out.lp.lam) || !std::isfinite(out.lp.phi))
        {
            int err = proj_errno(P);
            std::string msg = "[PROJ] transform failed";
#ifdef PROJ_VERSION_MAJOR
#if PROJ_VERSION_MAJOR >= 6
            if (err)
                msg += (": " + std::string(proj_errno_string(err)));
#endif
#endif
            throw std::runtime_error(msg);
        }

        double lon = out.lp.lam;
        double lat = out.lp.phi;
        return {lon, lat}; // 注意顺序：(lon, lat)
    };

    std::vector<std::pair<double, double>> xy = {
        {metadata.x0, metadata.y0}, // 左下
        {metadata.x1, metadata.y0}, // 右下
        {metadata.x1, metadata.y1}, // 右上
        {metadata.x0, metadata.y1}, // 左上
    };

    std::vector<std::array<double, 2>> trans_lla; // (lon, lat)
    trans_lla.reserve(xy.size());
    std::vector<double> lons, lats;
    lons.reserve(xy.size());
    lats.reserve(xy.size());

    for (const auto &p : xy)
    {
        auto ll = forward_xy_to_lonlat_deg(p.first, p.second);
        trans_lla.push_back(ll);
        lons.push_back(ll[0]);
        lats.push_back(ll[1]);
    }

    // ---- 计算 bbox_lla: [min_lon, min_lat, max_lon, max_lat] ----
    auto [min_lon_it, max_lon_it] = std::minmax_element(lons.begin(), lons.end());
    auto [min_lat_it, max_lat_it] = std::minmax_element(lats.begin(), lats.end());
    std::array<double, 4> bbox_lla = {*min_lon_it, *min_lat_it, *max_lon_it, *max_lat_it};

    // 时间戳
    double now_seconds = std::chrono::duration<double>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();

    json j;
    j["project_id"] = project_id;
    j["bbox_lla"] = bbox_lla; // [min_lon, min_lat, max_lon, max_lat]
    j["tiles_path"] = image_path;
    j["tiles_time"] = now_seconds;

    proj_destroy(P);
    proj_context_destroy(C);

    return j;
}


