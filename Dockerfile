# ------------- 构建基础 -------------
FROM ubuntu:22.04 AS builder

ARG DEBIAN_FRONTEND=noninteractive

# 安装依赖和构建工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git curl build-essential cmake pkg-config \
    libboost-system-dev libboost-dev \
    libopencv-dev \
    libexiv2-dev \
    libproj-dev \
    libgdal-dev \
    nlohmann-json3-dev \
    tzdata \
 && rm -rf /var/lib/apt/lists/* \
 && ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata

# 构建并安装 AMQP-CPP
COPY ./AMQP-CPP /tmp/amqpcpp
RUN cmake -S /tmp/amqpcpp -B /tmp/amqpcpp/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DAMQP-CPP_BUILD_SHARED=OFF \
    -DAMQP-CPP_LINUX_TCP=ON \
 && cmake --build /tmp/amqpcpp/build -j"$(nproc)" \
 && cmake --install /tmp/amqpcpp/build \
 && rm -rf /tmp/amqpcpp \
 && ldconfig

# 复制源代码和CMake配置文件
WORKDIR /usr/src/app/
COPY ./*.h /usr/src/app/
COPY ./*.cpp /usr/src/app/
COPY ./CMakeLists.txt /usr/src/app/CMakeLists.txt

# 使用 CMake 构建并安装
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build -j"$(nproc)" \
 && cmake --install build


# ------------- 运行时环境 -------------
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# 只安装运行时依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    libopencv-stitching4.5d libopencv-calib3d4.5d \
    libopencv-imgcodecs4.5d libopencv-imgproc4.5d \
    libopencv-features2d4.5d libopencv-core4.5d \
    libopencv-flann4.5d \
    libexiv2-27 \
    libproj22 \
    libgdal30 \
    libjson-c5 \
    python3 python3-gdal gdal-bin \
 && rm -rf /var/lib/apt/lists/* \
 && ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata

# gdal2tiles.py 可执行程序 
COPY --from=builder /usr/local/bin/* /usr/local/bin/
COPY ./gdal2tiles.py /usr/local/bin/gdal2tiles.py

WORKDIR /usr/src/app/
COPY  ./main.sh /usr/src/app/main.sh 
RUN chmod +x /usr/src/app/main.sh
COPY ./simulate_aerial.py /usr/src/app/simulate_aerial.py

# ------------- 环境变量 -------------
ENV INPUT_DIR=/app/input
ENV OUTPUT_DIR=/app/output

ENV RABBIT_HOST=rabbitmq-host
ENV RABBIT_PORT=5672
ENV RABBIT_USERNAME=admin
ENV RABBIT_PASSWORD=secret

ENV RABBIT_QUEUE=kuaipin
ENV RABBIT_EXCHANGE=topic.kuaipin
ENV RABBIT_ROUTINGKEY=kuaipin.zn

ENV VOLUME_ROOT=/data
