#include <iostream>
#include <chrono>
#include <vector>
#include <set>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "CameraApi.h"
#include "volleyball_vision/yolo_detector.hpp"

struct CameraThreadContext {
    int handle{-1};
    tSdkCameraDevInfo dev_info;
    std::string name;
    std::atomic<bool> running{true};
    std::mutex mtx;
    cv::Mat latest_frame;
    std::atomic<double> fps{0.0};
    uint64_t frame_count{0};
};

struct YoloThreadContext {
    volleyball_vision::YoloDetector detector;
    std::atomic<bool> enabled{true};
    std::atomic<bool> running{true};
    std::mutex mtx;
    std::vector<volleyball_vision::Detection> latest_dets_l;
    std::vector<volleyball_vision::Detection> latest_dets_r;
    std::atomic<double> fps{0.0};
    std::atomic<double> latency_ms{0.0};
};

void cameraCaptureThread(CameraThreadContext* ctx) {
    int max_w = 1280;
    int max_h = 1024;
    unsigned char* rgb_buf = (unsigned char*)malloc(max_w * max_h * 3);

    auto last_fps_time = std::chrono::steady_clock::now();
    int fps_counter = 0;

    while (ctx->running) {
        tSdkFrameHead head;
        BYTE* raw_buf = nullptr;

        // Non-blocking / short timeout (30ms)
        CameraSdkStatus st = CameraGetImageBuffer(ctx->handle, &head, &raw_buf, 30);
        if (st == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(ctx->handle, raw_buf, rgb_buf, &head);

            cv::Mat frame(head.iHeight, head.iWidth, CV_8UC3, rgb_buf);
            {
                std::lock_guard<std::mutex> lock(ctx->mtx);
                ctx->latest_frame = frame.clone();
            }

            CameraReleaseImageBuffer(ctx->handle, raw_buf);

            fps_counter++;
            ctx->frame_count++;
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
            if (elapsed >= 1.0) {
                ctx->fps = fps_counter / elapsed;
                fps_counter = 0;
                last_fps_time = now;
            }
        }
    }

    free(rgb_buf);
}

void yoloInferenceThread(CameraThreadContext* ctx_l, CameraThreadContext* ctx_r, YoloThreadContext* yolo_ctx) {
    auto last_fps_time = std::chrono::steady_clock::now();
    int fps_counter = 0;

    while (yolo_ctx->running) {
        if (!yolo_ctx->enabled) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            continue;
        }

        cv::Mat frame_l, frame_r;
        {
            std::lock_guard<std::mutex> lock(ctx_l->mtx);
            if (!ctx_l->latest_frame.empty()) {
                frame_l = ctx_l->latest_frame.clone();
            }
        }
        {
            std::lock_guard<std::mutex> lock(ctx_r->mtx);
            if (!ctx_r->latest_frame.empty()) {
                frame_r = ctx_r->latest_frame.clone();
            }
        }

        if (frame_l.empty() && frame_r.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        auto t0 = std::chrono::steady_clock::now();
        std::vector<volleyball_vision::Detection> dets_l, dets_r;
        if (!frame_l.empty()) {
            dets_l = yolo_ctx->detector.detect(frame_l);
        }
        if (!frame_r.empty()) {
            dets_r = yolo_ctx->detector.detect(frame_r);
        }
        auto t1 = std::chrono::steady_clock::now();
        double lat = std::chrono::duration<double, std::milli>(t1 - t0).count();
        yolo_ctx->latency_ms = lat;

        {
            std::lock_guard<std::mutex> lock(yolo_ctx->mtx);
            yolo_ctx->latest_dets_l = dets_l;
            yolo_ctx->latest_dets_r = dets_r;
        }

        fps_counter++;
        double elapsed = std::chrono::duration<double>(t1 - last_fps_time).count();
        if (elapsed >= 1.0) {
            yolo_ctx->fps = fps_counter / elapsed;
            fps_counter = 0;
            last_fps_time = t1;
        }
    }
}

int main(int argc, char** argv) {
    bool mono_mode = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--mono" || arg == "-m" || arg == "mono") {
            mono_mode = true;
        }
    }

    CameraSdkInit(1);

    tSdkCameraDevInfo dev_list[16];
    int camera_count = 16;
    CameraSdkStatus status = CameraEnumerateDevice(dev_list, &camera_count);

    std::cout << "CameraEnumerateDevice found " << camera_count << " camera route entries." << std::endl;

    // Deduplicate physical cameras by SN, preferring 192.168.1.100 interface route
    std::vector<tSdkCameraDevInfo> unique_cameras;
    std::set<std::string> seen_sns;
    for (int i = 0; i < camera_count; ++i) {
        std::string port_str(dev_list[i].acPortType);
        std::string sn(dev_list[i].acSn);
        if (port_str.find("192.168.1.100") != std::string::npos && seen_sns.find(sn) == seen_sns.end()) {
            seen_sns.insert(sn);
            unique_cameras.push_back(dev_list[i]);
            std::cout << "  Selected Camera [" << unique_cameras.size() - 1 << "]: SN=" << sn
                      << ", Route=" << port_str << std::endl;
        }
    }
    if (unique_cameras.size() < (mono_mode ? 1 : 2)) {
        for (int i = 0; i < camera_count; ++i) {
            std::string sn(dev_list[i].acSn);
            if (seen_sns.find(sn) == seen_sns.end()) {
                seen_sns.insert(sn);
                unique_cameras.push_back(dev_list[i]);
            }
        }
    }

    if (unique_cameras.empty()) {
        std::cerr << "Error: No MindVision cameras found!" << std::endl;
        return -1;
    }
    if (!mono_mode && unique_cameras.size() < 2) {
        std::cout << "Only 1 camera found, automatically switching to monocular mode (--mono)." << std::endl;
        mono_mode = true;
    }

    CameraThreadContext ctx_l, ctx_r;
    ctx_l.dev_info = unique_cameras[0];
    ctx_l.name = "Left";
    if (!mono_mode) {
        ctx_r.dev_info = unique_cameras[1];
        ctx_r.name = "Right";
    }

    // 1. Init Left Camera
    if (CameraInit(&ctx_l.dev_info, -1, -1, &ctx_l.handle) != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Failed to init Left Camera SN " << ctx_l.dev_info.acSn << std::endl;
        return -1;
    }
    CameraSetIspOutFormat(ctx_l.handle, CAMERA_MEDIA_TYPE_BGR8);
    CameraSetTriggerMode(ctx_l.handle, 0); // 连续采集模式
    CameraSetExposureTime(ctx_l.handle, 2000.0); // 2ms 高速低拖影曝光
    CameraSetImageResolutionEx(ctx_l.handle, 0xFF, 0, 0, 0, 0, 1280, 720, 0, 0);

    if (mono_mode) {
        // 单目模式下网卡独占千兆带宽，直接飙至 Speed Level 2（最高速档位，~105 FPS）！
        CameraSetFrameSpeed(ctx_l.handle, 2);
        CameraSetFrameRate(ctx_l.handle, 0);
    } else {
        // 双目模式下控制在 Speed Level 1（均衡档位，~37 FPS）
        CameraSetFrameSpeed(ctx_l.handle, 1);
        CameraSetFrameRate(ctx_l.handle, 0);

        // 2. Init Right Camera
        if (CameraInit(&ctx_r.dev_info, -1, -1, &ctx_r.handle) != CAMERA_STATUS_SUCCESS) {
            std::cerr << "Failed to init Right Camera SN " << ctx_r.dev_info.acSn << std::endl;
            CameraUnInit(ctx_l.handle);
            return -1;
        }
        CameraSetIspOutFormat(ctx_r.handle, CAMERA_MEDIA_TYPE_BGR8);
        CameraSetTriggerMode(ctx_r.handle, 0);
        CameraSetExposureTime(ctx_r.handle, 2000.0);
        CameraSetImageResolutionEx(ctx_r.handle, 0xFF, 0, 0, 0, 0, 1280, 720, 0, 0);
        CameraSetFrameSpeed(ctx_r.handle, 1);
        CameraSetFrameRate(ctx_r.handle, 0);
        CameraPlay(ctx_r.handle);
    }

    CameraPlay(ctx_l.handle);

    std::cout << "\n=======================================================" << std::endl;
    if (mono_mode) {
        std::cout << "  [MONOCULAR SPEED TEST MODE] Started!" << std::endl;
        std::cout << "  Camera: SN " << ctx_l.dev_info.acSn << ", SpeedLevel=2 (MAX), ROI=1280x720" << std::endl;
    } else {
        std::cout << "  Multi-threaded Asynchronous Live Preview Started!" << std::endl;
        std::cout << "  Left Camera:  SN " << ctx_l.dev_info.acSn << ", SpeedLevel=1, ROI=1280x720" << std::endl;
        std::cout << "  Right Camera: SN " << ctx_r.dev_info.acSn << ", SpeedLevel=1, ROI=1280x720" << std::endl;
    }
    std::cout << "  Interactive Controls:" << std::endl;
    std::cout << "    [y] Toggle YOLO real-time detection ON / OFF" << std::endl;
    std::cout << "    [s] Save snapshot image to disk" << std::endl;
    std::cout << "    [q] or [ESC] Exit cleanly" << std::endl;
    std::cout << "=======================================================\n" << std::endl;

    // Initialize YOLO detector
    YoloThreadContext yolo_ctx;
    std::string model_path = "/home/aerialstewart/26_RC_Volleyball/src/volleyball_vision/models/yolov8n.onnx";
    if (yolo_ctx.detector.loadModel(model_path, 0.35f, 0.45f)) {
        std::cout << "[YOLO] Model loaded successfully: " << model_path << std::endl;
    } else {
        std::cerr << "[YOLO] Warning: Failed to load model. Running camera stream only." << std::endl;
        yolo_ctx.enabled = false;
    }

    // Launch threads
    std::thread th_l(cameraCaptureThread, &ctx_l);
    std::unique_ptr<std::thread> th_r;
    if (!mono_mode) {
        th_r = std::make_unique<std::thread>(cameraCaptureThread, &ctx_r);
    }
    std::thread th_yolo(yoloInferenceThread, &ctx_l, mono_mode ? nullptr : &ctx_r, &yolo_ctx);

    std::string win_name = mono_mode ? "High-Speed Monocular MindVision + YOLO Live Benchmark"
                                     : "High-Speed Dual MindVision + YOLO Live Preview";
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    if (mono_mode) {
        cv::resizeWindow(win_name, 960, 540);
    } else {
        cv::resizeWindow(win_name, 1280, 400);
    }

    cv::Mat last_disp_l, last_disp_r;
    int snapshot_idx = 0;

    while (true) {
        cv::Mat cur_l, cur_r;
        {
            std::lock_guard<std::mutex> lock(ctx_l.mtx);
            if (!ctx_l.latest_frame.empty()) {
                cur_l = ctx_l.latest_frame;
            }
        }
        if (!mono_mode) {
            std::lock_guard<std::mutex> lock(ctx_r.mtx);
            if (!ctx_r.latest_frame.empty()) {
                cur_r = ctx_r.latest_frame;
            }
        }

        const int disp_w = mono_mode ? 960 : 640;
        const int disp_h = mono_mode ? 540 : 360;
        const float sx = static_cast<float>(disp_w) / 1280.0f;
        const float sy = static_cast<float>(disp_h) / 720.0f;

        if (!cur_l.empty()) {
            cv::resize(cur_l, last_disp_l, cv::Size(disp_w, disp_h), 0, 0, cv::INTER_NEAREST);
        }
        if (!mono_mode && !cur_r.empty()) {
            cv::resize(cur_r, last_disp_r, cv::Size(disp_w, disp_h), 0, 0, cv::INTER_NEAREST);
        }

        bool can_display = mono_mode ? (!last_disp_l.empty()) : (!last_disp_l.empty() && !last_disp_r.empty());

        if (can_display) {
            cv::Mat show_l = last_disp_l.clone();
            cv::Mat show_r;
            if (!mono_mode) show_r = last_disp_r.clone();

            std::vector<volleyball_vision::Detection> dets_l, dets_r;
            if (yolo_ctx.enabled) {
                std::lock_guard<std::mutex> lock(yolo_ctx.mtx);
                dets_l = yolo_ctx.latest_dets_l;
                dets_r = yolo_ctx.latest_dets_r;
            }

            // Draw bounding boxes on Left display
            for (const auto& d : dets_l) {
                cv::Rect box(d.bbox.x * sx, d.bbox.y * sy, d.bbox.width * sx, d.bbox.height * sy);
                cv::rectangle(show_l, box, cv::Scalar(0, 255, 0), 2);
                cv::circle(show_l, cv::Point(d.center.x * sx, d.center.y * sy), 4, cv::Scalar(0, 0, 255), -1);

                std::string label = "Ball " + std::to_string(static_cast<int>(d.confidence * 100)) + "%";
                int base_line = 0;
                cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);
                cv::rectangle(show_l, cv::Point(box.x, std::max(0, box.y - text_size.height - 4)),
                              cv::Point(box.x + text_size.width, box.y), cv::Scalar(0, 255, 0), cv::FILLED);
                cv::putText(show_l, label, cv::Point(box.x, box.y - 2),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }

            // Draw bounding boxes on Right display (dual mode only)
            if (!mono_mode) {
                for (const auto& d : dets_r) {
                    cv::Rect box(d.bbox.x * sx, d.bbox.y * sy, d.bbox.width * sx, d.bbox.height * sy);
                    cv::rectangle(show_r, box, cv::Scalar(0, 255, 0), 2);
                    cv::circle(show_r, cv::Point(d.center.x * sx, d.center.y * sy), 4, cv::Scalar(0, 0, 255), -1);

                    std::string label = "Ball " + std::to_string(static_cast<int>(d.confidence * 100)) + "%";
                    int base_line = 0;
                    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);
                    cv::rectangle(show_r, cv::Point(box.x, std::max(0, box.y - text_size.height - 4)),
                                  cv::Point(box.x + text_size.width, box.y), cv::Scalar(0, 255, 0), cv::FILLED);
                    cv::putText(show_r, label, cv::Point(box.x, box.y - 2),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                }
            }

            // Overlays
            std::string yolo_status;
            cv::Scalar yolo_color;
            if (yolo_ctx.enabled) {
                yolo_status = "YOLO: [ON] " + std::to_string(static_cast<int>(yolo_ctx.fps.load())) +
                              " FPS (" + std::to_string(static_cast<int>(yolo_ctx.latency_ms.load())) + "ms)";
                yolo_color = cv::Scalar(0, 255, 0);
            } else {
                yolo_status = "YOLO: [OFF] (Press 'y' to toggle)";
                yolo_color = cv::Scalar(0, 165, 255);
            }

            if (mono_mode) {
                std::string text_cam = "Camera: SN " + std::string(ctx_l.dev_info.acSn) +
                                       " | Capture FPS: " + std::to_string(static_cast<int>(ctx_l.fps.load())) +
                                       " (Speed Level 2: Max)";
                std::string hint = "[y] Toggle YOLO  [s] Snapshot  [q] Quit";

                cv::putText(show_l, text_cam, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                cv::putText(show_l, yolo_status, cv::Point(15, 65), cv::FONT_HERSHEY_SIMPLEX, 0.7, yolo_color, 2);
                cv::putText(show_l, hint, cv::Point(15, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
                cv::imshow(win_name, show_l);
            } else {
                std::string text_l = "Left Cam: SN " + std::string(ctx_l.dev_info.acSn) +
                                     " | FPS: " + std::to_string(static_cast<int>(ctx_l.fps.load()));
                std::string text_r = "Right Cam: SN " + std::string(ctx_r.dev_info.acSn) +
                                     " | FPS: " + std::to_string(static_cast<int>(ctx_r.fps.load()));
                std::string hint = "[y] Toggle YOLO  [s] Snapshot  [q] Quit";

                cv::putText(show_l, text_l, cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                cv::putText(show_l, yolo_status, cv::Point(15, 55), cv::FONT_HERSHEY_SIMPLEX, 0.6, yolo_color, 2);

                cv::putText(show_r, text_r, cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                cv::putText(show_r, hint, cv::Point(15, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

                cv::Mat combined;
                cv::hconcat(show_l, show_r, combined);
                cv::imshow(win_name, combined);
            }
        }

        int key = cv::waitKey(15);
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        } else if (key == 'y' || key == 'Y') {
            yolo_ctx.enabled = !yolo_ctx.enabled;
            std::cout << "[YOLO] " << (yolo_ctx.enabled ? "ENABLED" : "DISABLED") << std::endl;
        } else if (key == 's' || key == 'S') {
            std::string fname_l = "snapshot_" + std::to_string(snapshot_idx) + (mono_mode ? ".jpg" : "_left.jpg");
            if (!cur_l.empty()) cv::imwrite(fname_l, cur_l);
            if (!mono_mode && !cur_r.empty()) {
                std::string fname_r = "snapshot_" + std::to_string(snapshot_idx) + "_right.jpg";
                cv::imwrite(fname_r, cur_r);
            }
            std::cout << "[Snapshot] Saved snapshot " << snapshot_idx << std::endl;
            snapshot_idx++;
        }
    }

    ctx_l.running = false;
    if (!mono_mode) ctx_r.running = false;
    yolo_ctx.running = false;

    if (th_l.joinable()) th_l.join();
    if (th_r && th_r->joinable()) th_r->join();
    if (th_yolo.joinable()) th_yolo.join();

    CameraStop(ctx_l.handle);
    CameraUnInit(ctx_l.handle);
    if (!mono_mode) {
        CameraStop(ctx_r.handle);
        CameraUnInit(ctx_r.handle);
    }
    cv::destroyAllWindows();

    std::cout << "Viewer exited cleanly." << std::endl;
    return 0;
}
