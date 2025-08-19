#include "metadata.h"
#include <iostream>

// g++ -std=c++17 test_metadata.cpp metadata.cpp debug_utils.cpp -o metadata_reader -lexiv2 -lproj

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <image_path>" << std::endl;
        return 1;
    }

    std::string image_path = argv[1];

    Metadata meta;
    if (!meta.extract(image_path)) {
        std::cerr << "❌ 提取失败: " << image_path << std::endl;
        return 1;
    }

    // 判断是否为下视影像
    if (meta.is_nadir()) {
        std::cout << "✅ 该影像为下视 (nadir) 拍摄" << std::endl;
    } else {
        std::cout << "⚠️ 该影像不是标准下视拍摄" << std::endl;
    }

    // 计算地理范围
    meta.calculateImageBounds();
    std::cout << "Image Bounds (UTM): " << std::endl;
    std::cout << "  左上角: (" << meta.x0 << ", " << meta.y0 << ")" << std::endl;
    std::cout << "  右下角: (" << meta.x1 << ", " << meta.y1 << ")" << std::endl;

    return 0;
}
