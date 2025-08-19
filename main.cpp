#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_set>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <thread>
#include <random>
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <getopt.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctime>
#include <csignal>
#include <future>
#include <atomic>
#include <deque>
#include <memory>

// Linux inotify (optional, with graceful fallback)
#include <sys/inotify.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <cerrno>
#include <limits.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "image.h"
#include "rabbitmq.h"
#include "seamfinder.h"
#include "functions.h"
#include "utils.h"
#include "debug_utils.h"


// ======================== Global variables ========================
const int MAX_QUEUE_SIZE = 10; // Loader -> Processor queue capacity
std::shared_mutex domMutex;    // shared for ROI read, exclusive for write
static std::atomic<bool> exitFlag{false};  // 退出标志
std::atomic<std::chrono::steady_clock::time_point> lastImageTime;  // 记录最后一张图像的时间
std::atomic<std::chrono::steady_clock::time_point> lastSaveTime;   // 最后保存时间
std::unordered_set<std::string> processedFiles;  // 记录已经处理过的图像

cv::Mat globalDOM;
Metadata globalMetadata;
double global_x0, global_y0, global_x1, global_y1; // UTM bounds
double global_gsd;                                 // meters per pixel
double global_origin_x, global_origin_y;           // DOM top-left UTM

RabbitMQManager mq;  // rabbitMQ
std::string global_volume_root = get_env("VOLUME_ROOT", "");

// ======================== Helpers ========================
static inline void set_exit_flag(int) { exitFlag.store(true); }
static inline std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b; if (a.back() == '/') return a + b; return a + "/" + b;
}


// ======================== Bounded queue ========================
template <typename T>
class BoundedQueue {
public:
    BoundedQueue(size_t cap) : cap_(cap) {}

    void push(T&& v) {
        std::unique_lock<std::mutex> lk(m_);
        // 如果 exitFlag 已经置 true，就直接丢弃
        if (exitFlag.load()) {
            return;
        }

        not_full_.wait(lk, [&]{ return q_.size() < cap_ || exitFlag.load(); });
        if (exitFlag.load()) {
            return;
        }

        q_.emplace(std::move(v));
        not_empty_.notify_one();
    }

    bool pop(T& v) {
        std::unique_lock<std::mutex> lk(m_);
        not_empty_.wait(lk, [&]{ return !q_.empty() || exitFlag.load(); });

        if (exitFlag.load() && q_.empty()) {
            return false;
        }

        if (q_.empty()) {
            return false;
        }

        v = std::move(q_.front());
        q_.pop();
        not_full_.notify_one();
        return true;
    }

    void clear() {
        std::lock_guard<std::mutex> lk(m_);
        std::queue<T> empty;
        std::swap(q_, empty);
        not_empty_.notify_all();  // <--- 唤醒 pop()
        not_full_.notify_all();   // <--- 避免 push 卡住
    }

private:
    std::mutex m_;
    std::condition_variable not_full_, not_empty_;
    std::queue<T> q_;
    size_t cap_;
};



// ======================== 线程池（固定线程数） ========================
class ThreadPool {
public:
    explicit ThreadPool(size_t n) : stop_(false), exitFlag(false), active_jobs_(0) {
        for (size_t i = 0; i < n; ++i) {
            workers_.emplace_back([this]() { this->worker(); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(m_);
            stop_ = true;
        }
        cv_.notify_all();

        for (auto &t : workers_) {
            if (t.joinable()) t.join();
        }
    }

    void enqueue(std::function<void()> job) {
        {
            std::lock_guard<std::mutex> lk(m_);
            q_.push_back(std::move(job));
        }
        cv_.notify_one();
    }

    void wait_for_tasks() {
        while (true) {
            if (q_.empty() && active_jobs() == 0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    size_t pending() {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }

    size_t active_jobs() {
        return active_jobs_.load();
    }

    void stop() {
        exitFlag = true;
        cv_.notify_all();
    }

private:
    void worker() {
        while (true) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [this] { return stop_ || exitFlag || !q_.empty(); });

                if ((stop_ || exitFlag) && q_.empty()) {
                    return;
                }

                job = std::move(q_.front());
                q_.pop_front();
            }

            active_jobs_.fetch_add(1, std::memory_order_relaxed);
            try {
                job();
            } catch (const std::exception &e) {
                LOG_INFO(std::string("job exception: ") + e.what());
            } catch (...) {
                LOG_INFO("job unknown exception");
            }
            active_jobs_.fetch_sub(1, std::memory_order_relaxed);
        }
    }

private:
    std::vector<std::thread> workers_;
    std::deque<std::function<void()>> q_;
    std::mutex m_;
    std::condition_variable cv_;
    bool stop_;
    std::atomic<bool> exitFlag;
    std::atomic<size_t> active_jobs_;
};


// ======================== Save task ========================
struct SaveTask {
    cv::Mat dom;       // copy at enqueue time to decouple from live DOM
    Metadata meta;
    std::string name;  // base filename (without extension)
    double origin_x;
    double origin_y;
    double gsd;
};


// ======================== Queues & Sync ========================
static BoundedQueue<Image> g_load2proc(MAX_QUEUE_SIZE);
static std::mutex g_commitMutex;                                 // 串行提交
static std::shared_ptr<SaveTask> g_latestTask;
static std::mutex g_taskMutex;


// ======================== monitorFolder ========================
static bool monitor_with_inotify(const std::string &folderPath,
                                 const std::string &outputFolder,
                                 int resolution_mode,
                                 int project_id) {
    int in_fd = inotify_init1(IN_NONBLOCK);
    if (in_fd < 0) return false;

    int wd = inotify_add_watch(in_fd, folderPath.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
    if (wd < 0) { close(in_fd); return false; }

    int ep = epoll_create1(0);
    if (ep < 0) { inotify_rm_watch(in_fd, wd); close(in_fd); return false; }

    epoll_event ev{.events = EPOLLIN, .data = {.fd = in_fd}};
    if (epoll_ctl(ep, EPOLL_CTL_ADD, in_fd, &ev) < 0) {
        close(ep); inotify_rm_watch(in_fd, wd); close(in_fd); return false;
    }

    const size_t BUF_LEN = 16 * (sizeof(struct inotify_event) + NAME_MAX + 1);
    std::vector<char> buf(BUF_LEN);

    while (!exitFlag.load()) {
        epoll_event events[4];
        int nfds = epoll_wait(ep, events, 4, 300);
        if (nfds < 0) { if (errno == EINTR) continue; break; }
        if (nfds == 0) continue;

        int len = read(in_fd, buf.data(), BUF_LEN);
        if (len <= 0) continue;

        for (char* ptr = buf.data(); ptr < buf.data() + len; ) {
            struct inotify_event* event = reinterpret_cast<struct inotify_event*>(ptr);
            ptr += sizeof(struct inotify_event) + event->len;
            if (event->len == 0) continue;
            if (!(event->mask & (IN_CLOSE_WRITE | IN_MOVED_TO))) continue;

            std::string filename = event->name;
            if (processedFiles.find(filename) != processedFiles.end()) continue;
            processedFiles.insert(filename);

            std::string fullpath = joinPath(folderPath, filename);

            Image image;
            if (!image.readMetadata(fullpath)) continue;
            if (!image.metadata.is_nadir()) continue;
            if (!image.readImage(fullpath, cv::COLOR_BGR2BGRA)) continue;

            image.resize_1080(resolution_mode);
            image.coarse_ortho();

            std::string out_png = joinPath(joinPath(outputFolder, "images"),
                                   std::filesystem::path(filename).stem().string() + ".png");
            cv::imwrite(out_png, image.image);

            mq.send_model_progress(build_image_json(image.metadata, project_id,
                               get_relative_path(out_png, global_volume_root)));

            g_load2proc.push(std::move(image));

            // 更新最后接收图像时间
            lastImageTime.store(std::chrono::steady_clock::now());
        }
    }

    epoll_ctl(ep, EPOLL_CTL_DEL, in_fd, nullptr);
    close(ep);
    inotify_rm_watch(in_fd, wd);
    close(in_fd);
    return true;
}

void monitorFolder(const std::string &folderPath,
                   const std::string &outputFolder,
                   int resolution_mode,
                   int project_id) {
    // inotify 优先；失败则轮询兜底
    if (monitor_with_inotify(folderPath, outputFolder, resolution_mode, project_id)) return;
    
    while (!exitFlag.load()) {
        DIR *dir = opendir(folderPath.c_str());
        if (dir) {
            std::vector<std::pair<std::string, time_t>> files;
            struct dirent *ent;
            while ((ent = readdir(dir)) != NULL) {
                if (ent->d_type == DT_REG) {
                    std::string filename = ent->d_name;
                    if (processedFiles.find(filename) != processedFiles.end()) continue;
                    std::string fullpath = joinPath(folderPath, filename);
                    struct stat st;
                    if (stat(fullpath.c_str(), &st) == 0) {
                        files.emplace_back(filename, st.st_mtime);
                    }
                }
            }
            closedir(dir);

            std::sort(files.begin(), files.end(),
                     [](auto &a, auto &b){ return a.first < b.first; });

            for (const auto &file : files) {
                if (exitFlag.load()) break;
                std::string filename = file.first;
                processedFiles.insert(filename);
                std::string fullpath = joinPath(folderPath, filename);

                Image image;
                if (!image.readMetadata(fullpath)) continue;
                if (!image.metadata.is_nadir()) continue;
                if (!image.readImage(fullpath, cv::COLOR_BGR2BGRA)) continue;

                image.resize_1080(resolution_mode);
                image.coarse_ortho();

                std::string out_png = joinPath(joinPath(outputFolder, "images"),
                                        std::filesystem::path(filename).stem().string() + ".png");
                cv::imwrite(out_png, image.image);

                mq.send_model_progress(build_image_json(image.metadata, project_id,
                                   get_relative_path(out_png, global_volume_root)));

                g_load2proc.push(std::move(image));

                // 更新最后接收图像时间
                lastImageTime.store(std::chrono::steady_clock::now());
            }
        } else {
            LOG_ERROR("Error opening directory: " + folderPath);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

// ======================== exit Idle watcher ========================
void exitIdleWatcherThread(int idle_timeout) {
    using namespace std::chrono;
    auto lastWarned = 0;

    while (!exitFlag.load()) {
        auto now = steady_clock::now();
        int idleSec = (int)duration_cast<seconds>(now - lastImageTime.load()).count();

        if (idleSec >= idle_timeout) {
            LOG_INFO("No new images for " + std::to_string(idleSec) +
                      " seconds. Exiting program.");
            exitFlag.store(true);
            break;
        }

        if (idleSec >= lastWarned + 5) {
            lastWarned = (idleSec / 5) * 5;
            LOG_INFO("Warning: No new images for " + std::to_string(lastWarned) + " seconds.");            
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

// ======================== save Idle watcher ========================
void saveIdleWatcherThread(int idle_timeout) {
    using namespace std::chrono;
    while (!exitFlag.load()) {
        auto now = steady_clock::now();
        int idle_sec = duration_cast<seconds>(now - lastSaveTime.load()).count();
        
        // 超过阈值且DOM非空时触发保存
        if (idle_sec >= idle_timeout) {
            std::lock_guard<std::mutex> commitLock(g_commitMutex);
            
            bool domValid = false;
            {
                std::shared_lock<std::shared_mutex> rlk(domMutex);
                domValid = !globalDOM.empty();
            }
            
            if (domValid) {
                auto task = std::make_shared<SaveTask>();
                task->name = "idle_save_" + std::to_string(
                    duration_cast<seconds>(now.time_since_epoch()).count()
                );
                
                // 获取DOM快照
                {
                    std::shared_lock<std::shared_mutex> rlk(domMutex);
                    task->dom = globalDOM.clone();
                    task->meta = globalMetadata;
                    task->origin_x = global_origin_x;
                    task->origin_y = global_origin_y;
                    task->gsd = global_gsd;
                }
                
                // 提交保存任务
                {
                    std::lock_guard<std::mutex> lk(g_taskMutex);
                    g_latestTask = task;
                }
                
                LOG_INFO("Idle save triggered after " + 
                         std::to_string(idle_sec) + " seconds");
                // 也更新一下
                lastSaveTime.store(std::chrono::steady_clock::now());
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

// ======================== saverThread ========================
void saverThread(const std::string &outputFolder, int project_id) {
    using namespace std::chrono_literals;
    while (!exitFlag.load()) {
        std::shared_ptr<SaveTask> taskPtr;
        {
            std::lock_guard<std::mutex> lk(g_taskMutex);
            taskPtr = g_latestTask;
            g_latestTask.reset();
        }
        if (!taskPtr) { std::this_thread::sleep_for(200ms); continue; }
        const SaveTask &task = *taskPtr;
        if (task.dom.empty()) { continue; }

        std::string geotiff_path = joinPath(joinPath(outputFolder, "tiles"),
                                            task.name + "_dom.tif");
        saveDOMAsGeoTIFF(task.dom, geotiff_path, task.meta.epsg,
                         task.origin_x, task.origin_y, task.gsd);
        GenerateTiles(geotiff_path, joinPath(outputFolder, "tiles"));

        // 更新最后保存时间
        lastSaveTime.store(std::chrono::steady_clock::now());

        mq.send_model_progress(
            build_tile_json(task.meta, project_id,
                get_relative_path(geotiff_path, global_volume_root)));
    }
}



// ======================== processImages ========================
void processImages(const std::string &outputFolder,
                   int /*single_mem_cpy*/,
                   int seam_mode,
                   int blend_mode,
                   int save_interval,
                   float match_threshold,
                   int max_seam_width,
                   int /*project_id*/)
{
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const size_t poolSize = hw > 2 ? hw - 1 : 1;
    ThreadPool pool(poolSize);

    std::atomic<int> committedCount{0};
    using Clock = std::chrono::steady_clock;
    const auto maxIdle = std::chrono::seconds(20);
    bool firstImage = true;

    const size_t MAX_INFLIGHT = std::max<size_t>(2, poolSize * 2);

    while (!exitFlag.load()) {
        Image image;
        if (!g_load2proc.pop(image)) 
        {
            break;
        }

        const std::string image_path = image.image_path;
        const std::string image_filename =
            std::filesystem::path(image_path).filename().replace_extension("").string();

        cv::Mat& currentImageRef = image.image; // 将在本作用域拷贝
        Metadata currentMetadata = image.metadata;

        clock_t loopStart = clock();

        if (firstImage) {
            {
                std::unique_lock<std::shared_mutex> wlk(domMutex);
                global_origin_x = currentMetadata.x0;
                global_origin_y = currentMetadata.y0;

                global_x0 = currentMetadata.x0;
                global_y0 = currentMetadata.y0;
                global_x1 = currentMetadata.x1;
                global_y1 = currentMetadata.y1;
                global_gsd = currentMetadata.gsd;

                globalDOM = currentImageRef.clone();
                globalMetadata = currentMetadata;
            }
            firstImage = false;

            auto task = std::make_shared<SaveTask>();
            task->name = image_filename;
            {
                std::shared_lock<std::shared_mutex> rlk(domMutex);
                task->dom = globalDOM.clone();
                task->meta = globalMetadata;
                task->origin_x = global_origin_x;
                task->origin_y = global_origin_y;
                task->gsd = global_gsd;
            }
            
            {
                std::lock_guard<std::mutex> lk(g_taskMutex);
                g_latestTask = task;
            }
            
            committedCount.fetch_add(1, std::memory_order_relaxed);
            LOG_INFO("Process Took: " + std::to_string(float(clock() - loopStart) / CLOCKS_PER_SEC) + " seconds");
            LOG_INFO("-------------------------------------------------------\n");
            continue;
        }

        // 克隆影像，避免 image 出作用域后悬挂
        cv::Mat currentImage = currentImageRef.clone();

        while (pool.pending() >= MAX_INFLIGHT && !exitFlag.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        pool.enqueue([currentImage, currentMetadata, image_filename,
                      &outputFolder, &committedCount,
                      seam_mode, blend_mode, save_interval, match_threshold, maxIdle, max_seam_width]() mutable
        {
            // 读取地理状态快照（读锁）
            double snap_x0, snap_y0, snap_gsd;
            {
                std::shared_lock<std::shared_mutex> rlk(domMutex);
                snap_x0 = global_x0;
                snap_y0 = global_y0;
                snap_gsd = global_gsd;
            }

            // 1) 由 GPS 估计像素平移
            double dx = (currentMetadata.x0 - snap_x0) / snap_gsd;
            double dy = (snap_y0 - currentMetadata.y0) / snap_gsd;

            double left   = (currentMetadata.x0 - snap_x0) / snap_gsd;
            double top    = (snap_y0 - currentMetadata.y0) / snap_gsd;
            double right  = (currentMetadata.x1 - snap_x0) / snap_gsd;
            double bottom = (snap_y0 - currentMetadata.y1) / snap_gsd;

            cv::Rect estimatedRect(
                (int)std::round(left),
                (int)std::round(top),
                (int)std::round(right - left),
                (int)std::round(bottom - top));

            // 2) 匹配用 DOM ROI（读锁 + clone）
            cv::Mat domRegion;
            cv::Rect domROI;
            {
                std::shared_lock<std::shared_mutex> rlk(domMutex);
                cv::Rect domBounds(0, 0, globalDOM.cols, globalDOM.rows);
                domROI = estimatedRect & domBounds;
                if (domROI.area() > 100) domRegion = globalDOM(domROI).clone();
                else { domROI = domBounds; domRegion = globalDOM.clone(); }
            }

            // 3) 估计 H（平移），必要时 ORB/RANSAC 修正
            cv::Mat H = (cv::Mat_<double>(3,3) << 1,0,dx,  0,1,dy,  0,0,1);
            const double tiny = 1.5;
            bool needMatch = (std::abs(dx) + std::abs(dy)) > tiny;

            if (needMatch) {
                std::vector<cv::Point2f> srcPts, dstPts, srcIn, dstIn;
                orbMatching(domRegion, currentImage, srcPts, dstPts, match_threshold);
                // saveMatches(domRegion, currentImage, srcPts, dstPts, joinPath(outputFolder, "tiles/" + image_filename + "_orb.png"));

                // 根据filter后判断
                thresholdFilter(srcPts, dstPts, srcIn, dstIn);

                cv::Mat H_;
                cv::Mat T = (cv::Mat_<double>(3,3) << 1,0,domROI.x, 0,1,domROI.y, 0,0,1);
                if (srcIn.size() > 20)
                {
                    // ransac
                    H_ = findHomographyTransform(dstPts, srcPts);
                }
                else if (srcIn.size() > 10)
                {
                    // ransac
                    H_ = findSimilarityTransform(dstPts, srcPts);
                }
                else if (!srcIn.empty())
                {
                    H_ = findTranslationTransform(dstIn, srcIn);
                }
                
                if (!srcIn.empty()) {
                    H_ = T * H_;
                    if (isTransformValid(H_, H, currentImage.size())) H = H_;
                }
            }

            // 4) Warp
            cv::Mat warped;
            cv::Rect warpedRect;
            getWarppedAcc2(warped, warpedRect, &currentImage, H);

            // 5) 提交阶段：接缝 + DOM 更新 + 边界更新
            {
                std::lock_guard<std::mutex> commitLock(g_commitMutex);
                cv::Mat stitched;
                {
                    std::shared_lock<std::shared_mutex> rlk(domMutex);
                    stitched = seamMerge(
                        globalDOM, warped, warpedRect,
                        (SeamFinderType)seam_mode, (BlendType)blend_mode,
                        "", max_seam_width
                    );

                    // stitched = seamMerge(globalDOM, warped, warpedRect,
                    //                      (SeamFinderType)seam_mode, (BlendType)blend_mode,
                    //                      joinPath(joinPath(outputFolder, "tiles"), image_filename));
                }

                {
                    std::unique_lock<std::shared_mutex> wlk(domMutex);
                    globalDOM = std::move(stitched);

                    double new_x0 = global_origin_x + warpedRect.x * global_gsd;
                    double new_y0 = global_origin_y - warpedRect.y * global_gsd;
                    double new_x1 = new_x0 + warpedRect.width  * global_gsd;
                    double new_y1 = new_y0 - warpedRect.height * global_gsd;

                    global_x0 = std::min(global_x0, new_x0);
                    global_y0 = std::max(global_y0, new_y0);
                    global_x1 = std::max(global_x1, new_x1);
                    global_y1 = std::min(global_y1, new_y1);

                    if (warpedRect.x < 0) global_origin_x = new_x0;
                    if (warpedRect.y < 0) global_origin_y = new_y0;

                    globalMetadata.x0 = global_x0;
                    globalMetadata.y0 = global_y0;
                    globalMetadata.x1 = global_x1;
                    globalMetadata.y1 = global_y1;
                    globalMetadata.gsd = global_gsd;
                }

                // 保存触发：每 N 张或空闲超时
                committedCount.fetch_add(1, std::memory_order_relaxed);
                bool needSave = false;
                if (committedCount.load(std::memory_order_relaxed) % save_interval == 0) {
                    needSave = true;
                }

                if (needSave) {
                    auto task = std::make_shared<SaveTask>();
                    task->name = image_filename;
                    {
                        std::shared_lock<std::shared_mutex> rlk(domMutex);
                        task->dom = globalDOM.clone();
                        task->meta = globalMetadata;
                        task->origin_x = global_origin_x;
                        task->origin_y = global_origin_y;
                        task->gsd = global_gsd;
                    }

                    {
                        std::lock_guard<std::mutex> lk(g_taskMutex);
                        g_latestTask = task;
                    }
                }
            }
        });

        pool.wait_for_tasks();

        LOG_INFO("Process Took: " + std::to_string(float(clock() - loopStart) / CLOCKS_PER_SEC) + " seconds");
        LOG_INFO("-----------------------------------------------------------\n");
    }
}

// ======================== CLI & main ========================
static void print_help(const char *program_name) {
    std::cerr << "Usage: " << program_name << " [OPTIONS]\n"
              << "Options:\n"
              << "  -i, --input_path PATH      Specify input folder path (required)\n"
              << "  -o, --output_path PATH     Specify output folder path (required)\n"
              << "  -p, --project_id ID        Set project ID (integer, auto-generated if not provided)\n"
              << "  -r, --resolution MODE      Set resolution mode 50-500 (default=150)\n"
              << "  -s, --seam_mode MODE       Seam mode: 0-NONE,1-GRAPH_CUT,2-VORONOI,3-DP (default=1)\n"
              << "  -w, --max_seam_width INT   Maximum seam width (default=256)"
              << "  -b, --blend_mode MODE      Blend mode: 0-NONE,1-FEATHER,2-MULTIBAND (default=0)\n"
              << "  -m, --single_mem_cpy       Enable single memory copy mode (flag)\n"
              << "  -n, --save_interval N      Save DOM every N images (default=10)\n"
              << "  -t, --match_threshold VAL  Match Threshold 0-1 (default=0.65)\n"
              << "  -x, --idle_timeout SECS    Exit if no new image arrives in SECS (default=30), 0-disabled\n"
              << "  -h, --help                 Show this help message\n"
              << "\nExample:\n"
              << "  " << program_name << " -i /input -o /output -p 1234 -r 200 -n 20\n"
              << "  " << program_name << " --input_path /input --output_path /output --project_id 1234\n";
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT,  set_exit_flag);
    std::signal(SIGTERM, set_exit_flag);

    static struct option longopts[] = {
        {"input_path", required_argument, NULL, 'i'},
        {"output_path", required_argument, NULL, 'o'},
        {"project_id", required_argument, NULL, 'p'},
        {"seam_mode", required_argument, NULL, 's'},
        {"max_seam_width", required_argument, NULL, 'w'},
        {"blend_mode", required_argument, NULL, 'b'},
        {"resolution", required_argument, NULL, 'r'},
        {"save_interval", required_argument, NULL, 'n'},
        {"match_threshold", required_argument, NULL, 't'},
        {"single_mem_cpy", no_argument,       NULL, 'm'},
        {"idle_timeout", required_argument, NULL, 'x'},
        {"help", no_argument,                 NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int max_seam_width = 256;
    int idle_timeout = 30;
    float match_threshold = 0.65f;
    int save_interval = 10;
    int single_mem_cpy = 0;
    int seam_mode = 1;
    int blend_mode = 0;
    int resolution_mode = 150;
    int project_id = -1;
    std::string input_folder;
    std::string output_folder;

    int c;
    while ((c = getopt_long(argc, argv, "i:o:p:s:w:b:r:m:n:t:x:h", longopts, NULL)) != -1) {
        switch (c) {
        case 'i': input_folder = optarg; break;
        case 'o': output_folder = optarg; break;
        case 'p': project_id = std::stoi(optarg); break;
        case 's': seam_mode = std::stoi(optarg); if (seam_mode < 0 || seam_mode > 3) std::cerr << "Error: Invalid seam mode. Must be 0..3.\n"; break;
        case 'w': max_seam_width = std::stoi(optarg); break;
        case 'b': blend_mode = std::stoi(optarg); if (blend_mode < 0 || blend_mode > 2) std::cerr << "Error: Invalid blend mode. Must be 0..2.\n"; break;
        case 'r': resolution_mode = std::stoi(optarg); break;
        case 'm': single_mem_cpy = 1; break;
        case 'n': save_interval = std::max(1, std::stoi(optarg)); break;
        case 't': match_threshold = std::stof(optarg); break;
        case 'x': idle_timeout = std::stoi(optarg); break;
        case 'h': print_help(argv[0]); return 0;
        default:  print_help(argv[0]); return 1;
        }
    }

    // 输入相对路径
    output_folder = joinPath(global_volume_root, output_folder);
    input_folder = joinPath(global_volume_root, input_folder);

    if (input_folder.empty() || output_folder.empty()) {
        std::cerr << "Error: Input and output folder paths are required.\n\n";
        print_help(argv[0]);
        return 1;
    }

    std::filesystem::create_directories(joinPath(output_folder, "images"));
    std::filesystem::create_directories(joinPath(output_folder, "tiles"));

    // 设置 log
    DebugLogger::getInstance().setLogFile(output_folder + "/log.log");

    if (project_id == -1) {
        std::random_device rd; std::mt19937 gen(rd()); std::uniform_int_distribution<int> dis(10000, 99999);
        project_id = dis(gen);
        LOG_INFO("Generated project ID: " + std::to_string(project_id));
    }

    if (!mq.init_rabbitmq_infrastructure()) return 1;

    cv::setUseOptimized(true);
    cv::setNumThreads(std::max(1u, std::thread::hardware_concurrency()));

    LOG_INFO("\nRunning with parameters:");
    LOG_INFO("Input folder: " + input_folder);
    LOG_INFO("Output folder: " + output_folder);
    LOG_INFO("Project ID: " + std::to_string(project_id));
    LOG_INFO("Seam mode: " + std::to_string(seam_mode));
    LOG_INFO("Blend mode: " + std::to_string(blend_mode));
    LOG_INFO("Resolution mode: " + std::to_string(resolution_mode));
    LOG_INFO("Save interval: " + std::to_string(save_interval));
    LOG_INFO("Single memory copy mode: " + std::to_string(single_mem_cpy));
    LOG_INFO("Match Threshold: " + std::to_string(match_threshold));
    LOG_INFO("VOLUME ROOT: " + global_volume_root);

    lastImageTime.store(std::chrono::steady_clock::now());
    lastSaveTime.store(std::chrono::steady_clock::now());

    int save_idle_timeout = idle_timeout - 10;  // 比退出慢10秒，预留保存dom时间
    if (save_idle_timeout < 10) save_idle_timeout = 10;

    std::thread t_monitor(monitorFolder, input_folder, output_folder, resolution_mode, project_id);
    std::thread t_process(processImages, output_folder, single_mem_cpy, seam_mode, blend_mode, save_interval, match_threshold, max_seam_width, project_id);
    std::thread t_saver(saverThread, output_folder, project_id);
    std::thread t_save_idle(saveIdleWatcherThread, save_idle_timeout);

    std::thread t_idle;
    if (idle_timeout > 0) {
        t_idle = std::thread(exitIdleWatcherThread, idle_timeout);
    }

    LOG_INFO("Monitoring folder: " + input_folder + "\n");

    if (t_idle.joinable()) {
        t_idle.join();
    }

    if (t_save_idle.joinable()) t_save_idle.join();

    // 这里 exitFlag 已经 true，确保清空队列避免卡住
    g_load2proc.clear();

    t_monitor.join();
    t_process.join();
    t_saver.join();

    // 停止 rabbitMQ
    mq.stop();

    LOG_INFO("Exiting main..."); 
    return 0;
}
