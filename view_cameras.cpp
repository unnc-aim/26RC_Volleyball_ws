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

int main() {
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
    if (unique_cameras.size() < 2) {
        for (int i = 0; i < camera_count; ++i) {
            std::string sn(dev_list[i].acSn);
            if (seen_sns.find(sn) == seen_sns.end()) {
                seen_sns.insert(sn);
                unique_cameras.push_back(dev_list[i]);
            }
        }
    }

    if (unique_cameras.size() < 2) {
        std::cerr << "Error: Need at least 2 distinct MindVision cameras!" << std::endl;
        return -1;
    }

    CameraThreadContext ctx_l, ctx_r;
    ctx_l.dev_info = unique_cameras[0];
    ctx_l.name = "Left";
    ctx_r.dev_info = unique_cameras[1];
    ctx_r.name = "Right";

    // 1. Init Left Camera
    if (CameraInit(&ctx_l.dev_info, -1, -1, &ctx_l.handle) != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Failed to init Left Camera SN " << ctx_l.dev_info.acSn << std::endl;
        return -1;
    }
    // 2. Init Right Camera
    if (CameraInit(&ctx_r.dev_info, -1, -1, &ctx_r.handle) != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Failed to init Right Camera SN " << ctx_r.dev_info.acSn << std::endl;
        CameraUnInit(ctx_l.handle);
        return -1;
    }

    tSdkCameraCapbility cap_l, cap_r;
    CameraGetCapability(ctx_l.handle, &cap_l);
    CameraGetCapability(ctx_r.handle, &cap_r);

    CameraSetIspOutFormat(ctx_l.handle, CAMERA_MEDIA_TYPE_BGR8);
    CameraSetIspOutFormat(ctx_r.handle, CAMERA_MEDIA_TYPE_BGR8);

    CameraSetTriggerMode(ctx_l.handle, 0); // 连续采集模式
    CameraSetTriggerMode(ctx_r.handle, 0);

    CameraSetExposureTime(ctx_l.handle, 2000.0); // 2ms 高速低拖影曝光
    CameraSetExposureTime(ctx_r.handle, 2000.0);

    // 方案 2: 垂直方向启用 ROI 裁剪 (1280x720, 截取上部 720 行，裁掉底部 304 行车体/地面死区，数据量骤降 30%)
    CameraSetImageResolutionEx(ctx_l.handle, 0xFF, 0, 0, 0, 0, 1280, 720, 0, 0);
    CameraSetImageResolutionEx(ctx_r.handle, 0xFF, 0, 0, 0, 0, 1280, 720, 0, 0);

    // 方案 1: 硬件降档至均衡档 (Speed Level 1)，将双目总带宽控制在约 700 Mbps，完全适配 1000M 交换机物理上限
    CameraSetFrameSpeed(ctx_l.handle, 1);
    CameraSetFrameRate(ctx_l.handle, 0);

    CameraSetFrameSpeed(ctx_r.handle, 1);
    CameraSetFrameRate(ctx_r.handle, 0);

    CameraPlay(ctx_l.handle);
    CameraPlay(ctx_r.handle);

    std::cout << "\n=======================================================" << std::endl;
    std::cout << "  Multi-threaded Asynchronous Live Preview Started!" << std::endl;
    std::cout << "  Left Camera:  SN " << ctx_l.dev_info.acSn << ", SpeedLevel=1, ROI=1280x720" << std::endl;
    std::cout << "  Right Camera: SN " << ctx_r.dev_info.acSn << ", SpeedLevel=1, ROI=1280x720" << std::endl;
    std::cout << "  Press 'q' or 'ESC' to exit." << std::endl;
    std::cout << "=======================================================\n" << std::endl;

    // Launch independent high-speed capture threads
    std::thread th_l(cameraCaptureThread, &ctx_l);
    std::thread th_r(cameraCaptureThread, &ctx_r);

    cv::namedWindow("High-Speed Dual MindVision Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("High-Speed Dual MindVision Preview", 1280, 512);

    cv::Mat last_disp_l, last_disp_r;

    while (true) {
        cv::Mat cur_l, cur_r;
        {
            std::lock_guard<std::mutex> lock(ctx_l.mtx);
            if (!ctx_l.latest_frame.empty()) {
                cur_l = ctx_l.latest_frame;
            }
        }
        {
            std::lock_guard<std::mutex> lock(ctx_r.mtx);
            if (!ctx_r.latest_frame.empty()) {
                cur_r = ctx_r.latest_frame;
            }
        }

        if (!cur_l.empty()) {
            cv::resize(cur_l, last_disp_l, cv::Size(640, 512), 0, 0, cv::INTER_NEAREST);
        }
        if (!cur_r.empty()) {
            cv::resize(cur_r, last_disp_r, cv::Size(640, 512), 0, 0, cv::INTER_NEAREST);
        }

        if (!last_disp_l.empty() && !last_disp_r.empty()) {
            cv::Mat show_l = last_disp_l.clone();
            cv::Mat show_r = last_disp_r.clone();

            std::string text_l = "Left: SN " + std::string(ctx_l.dev_info.acSn);
            std::string text_r = "Right: SN " + std::string(ctx_r.dev_info.acSn);
            std::string fps_l = "FPS: " + std::to_string(static_cast<int>(ctx_l.fps.load()));
            std::string fps_r = "FPS: " + std::to_string(static_cast<int>(ctx_r.fps.load()));

            cv::putText(show_l, text_l, cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(show_l, fps_l, cv::Point(20, 75), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);

            cv::putText(show_r, text_r, cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(show_r, fps_r, cv::Point(20, 75), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);

            cv::Mat combined;
            cv::hconcat(show_l, show_r, combined);
            cv::imshow("High-Speed Dual MindVision Preview", combined);
        }

        int key = cv::waitKey(15);
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }
    }

    ctx_l.running = false;
    ctx_r.running = false;

    if (th_l.joinable()) th_l.join();
    if (th_r.joinable()) th_r.join();

    CameraStop(ctx_l.handle);
    CameraStop(ctx_r.handle);
    CameraUnInit(ctx_l.handle);
    CameraUnInit(ctx_r.handle);
    cv::destroyAllWindows();

    std::cout << "Viewer exited cleanly." << std::endl;
    return 0;
}
