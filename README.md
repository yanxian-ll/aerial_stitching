# Aerial Stitching

## Overview
**Aerial Stitching** is a C++/OpenCV-based image correction and stitching service designed to process aerial images in real time.  
It integrates with **RabbitMQ** for distributed message passing and supports a wide range of configuration options for resolution, seam handling, blending, and performance tuning.  

The project is containerized using **Docker** and **Docker Compose**, making deployment easy and portable.

---

## Features
- 📷 **Image Stitching & Correction** using OpenCV
- 🔗 **RabbitMQ Integration** for message-driven workflows
- ⚡ **Configurable Parameters** (resolution, seam width, blend modes, thresholds, etc.)
- 🐳 **Dockerized Build & Runtime Environment**
- 📂 **Automatic Project ID Generation**
- 📝 **Logging Support** (per-project logs stored in the output directory)
- ⏱ **Idle Timeout Handling** for graceful shutdown

---

## Build & Run

### 1. Clone Repository
```bash
git clone https://github.com/yanxian-ll/aerial_stitching
cd aerial_stitching
```

### 2. Build with Docker Compose 
```bash
docker compose up --build
```

This will:
- Start a RabbitMQ server (with management UI at http://localhost:15672,default user: admin, password: secret)
- Build and run the aerial_stitching container


### 3. Command-Line Options

The main binary supports the following options:
```bash
Usage: /usr/local/bin/fastcorrector [OPTIONS]
```
