#ifndef METADATA_H 
#define METADATA_H 
 
#include <string>
#include <map>
#include <vector>
#include <exiv2/exiv2.hpp> 
#include <proj.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;


class Metadata {
public:
    // 构造函数/析构函数 
    Metadata();
    ~Metadata();

    Metadata(const Metadata&) = default;
    Metadata& operator=(const Metadata&) = default;
 
    // 主提取函数 
    bool extract(const std::string& image_path);

    // 判断是否为下视拍摄
    bool is_nadir();
  
    // 调试输出
    void printMetadata() const;

    // 元数据存储
    std::string image_path;
    double gimbal_yaw = 0.0;
    double gimbal_pitch = 0.0;
    double gimbal_roll = 0.0;
    double lon = 0.0;
    double lat = 0.0;
    double alt = 0.0;
    double gsd = 0.0;
    double x_res = 0.0;
    double y_res = 0.0;
    double x_rotation = 0.0;
    double y_rotation = 0.0;
    int epsg = 0;
    double utm_x = 0.0;
    double utm_y = 0.0;
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    int image_width = 0;
    int image_height = 0;
 
    void parseExifData(const Exiv2::ExifData& exifData, const Exiv2::XmpData& xmpData);
    void convertToUTM();
    void calculateImageBounds();
 
    // 辅助函数 
    double parseRational(const Exiv2::Rational& rational) const;
    double convertGpsToDecimal(const std::string& gpsString);
    double convertGPSCoordinate(const Exiv2::Rational* coordinates, char direction) const;
};


// 每张图像需要发送的 json
json build_image_json(const Metadata& metadata,
                      const int& project_id,
                      const std::string& image_path);

// DOM需要发送的 json
json build_tile_json(const Metadata& metadata,
                      const int& project_id,
                      const std::string& image_path);
                      
 
#endif // METADATA_H
