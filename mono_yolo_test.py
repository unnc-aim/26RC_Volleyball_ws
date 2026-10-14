#!/usr/bin/env python3
import ctypes
import time
import argparse
import cv2
import numpy as np
from ultralytics import YOLO

class tSdkCameraDevInfo(ctypes.Structure):
    _fields_ = [
        ('acProductSeries', ctypes.c_char * 32),
        ('acProductName', ctypes.c_char * 32),
        ('acFriendlyName', ctypes.c_char * 32),
        ('acLinkName', ctypes.c_char * 32),
        ('acDriverVersion', ctypes.c_char * 32),
        ('acSensorType', ctypes.c_char * 32),
        ('acPortType', ctypes.c_char * 32),
        ('acSn', ctypes.c_char * 32),
        ('uInstance', ctypes.c_uint),
    ]

class tSdkFrameHead(ctypes.Structure):
    _fields_ = [
        ('uiMediaType', ctypes.c_uint),
        ('uBytes', ctypes.c_uint),
        ('iWidth', ctypes.c_int),
        ('iHeight', ctypes.c_int),
        ('iWidthZoomSummary', ctypes.c_int),
        ('iHeightZoomSummary', ctypes.c_int),
        ('bIsBayer', ctypes.c_int),
        ('bIsColor', ctypes.c_int),
        ('uiTimeStamp', ctypes.c_uint),
        ('uiExpTime', ctypes.c_uint),
        ('uiAnalogGain', ctypes.c_uint),
        ('iGamma', ctypes.c_int),
        ('iContrast', ctypes.c_int),
        ('iSaturation', ctypes.c_int),
        ('uReserved', ctypes.c_uint * 4),
    ]

def main():
    parser = argparse.ArgumentParser(description="High-Speed Monocular MindVision + YOLO Live Benchmark")
    parser.add_argument("--imgsz", type=int, default=512, help="YOLO input size (640 for high accuracy, 512 for 100 FPS)")
    parser.add_argument("--conf", type=float, default=0.35, help="Confidence threshold")
    parser.add_argument("--model", type=str, default="/home/aerialstewart/26_RC_Volleyball/src/volleyball_vision/models/yolov8n.pt")
    args = parser.parse_args()

    print(f"Loading YOLO model from: {args.model} (imgsz={args.imgsz})...")
    model = YOLO(args.model)

    mv = ctypes.cdll.LoadLibrary('/home/aerialstewart/26_RC_Volleyball/src/volleyball_vision/3rdparty/mindvision/lib/libMVSDK.so')
    mv.CameraSdkInit(1)

    dev_list = (tSdkCameraDevInfo * 16)()
    num_dev = ctypes.c_int(16)
    mv.CameraEnumerateDevice(dev_list, ctypes.byref(num_dev))

    if num_dev.value < 1:
        print("Error: No MindVision camera found!")
        return

    # Select camera on 192.168.1.100 interface if available
    selected_idx = 0
    for i in range(num_dev.value):
        port = dev_list[i].acPortType.decode()
        if "192.168.1.100" in port:
            selected_idx = i
            break

    target_dev = dev_list[selected_idx]
    sn = target_dev.acSn.decode()
    print(f"Opening Camera [SN {sn}] on route: {target_dev.acPortType.decode()}")

    h_cam = ctypes.c_int(-1)
    st = mv.CameraInit(ctypes.byref(target_dev), -1, -1, ctypes.byref(h_cam))
    if st != 0:
        print(f"Failed to init camera, err={st}")
        return

    # ISP Out format: CAMERA_MEDIA_TYPE_BGR8 = 0x02180015
    CAMERA_MEDIA_TYPE_BGR8 = 0x02180015
    st_fmt = mv.CameraSetIspOutFormat(h_cam, CAMERA_MEDIA_TYPE_BGR8)
    if st_fmt != 0:
        print(f"Warning: CameraSetIspOutFormat returned {st_fmt}")
    mv.CameraSetTriggerMode(h_cam, 0) # continuous free-run
    mv.CameraSetExposureTime(h_cam, ctypes.c_double(2000.0)) # 2ms

    # Set ROI: 1280x720 (Offset X=0, Y=0)
    mv.CameraSetImageResolutionEx(h_cam, 0xFF, 0, 0, 0, 0, 1280, 720, 0, 0)

    # Set Speed Level 2 (High Speed Mode!)
    mv.CameraSetFrameSpeed(h_cam, 2)
    mv.CameraSetFrameRate(h_cam, 0) # 0 = uncap hardware limit

    mv.CameraPlay(h_cam)

    print("\n" + "=" * 60)
    print("  High-Speed Monocular YOLO Benchmark Started!")
    print(f"  Camera: SN {sn} | SpeedLevel=2 (Max Speed) | ROI=1280x720")
    print(f"  YOLO: PyTorch AVX-512 (imgsz={args.imgsz})")
    print("  Press 'q' or 'ESC' to exit | 'y' to toggle YOLO")
    print("=" * 60 + "\n")

    w, h = 1280, 720
    rgb_buf = (ctypes.c_ubyte * (w * h * 3))()
    head = tSdkFrameHead()
    raw_ptr = ctypes.c_void_p()

    cv2.namedWindow("Monocular High-Speed YOLO Benchmark", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Monocular High-Speed YOLO Benchmark", 960, 540)

    enable_yolo = True
    frame_count = 0
    t_start = time.time()
    cam_fps = 0.0
    yolo_fps = 0.0
    yolo_ms = 0.0

    try:
        while True:
            # Short timeout grab (30ms)
            st = mv.CameraGetImageBuffer(h_cam, ctypes.byref(head), ctypes.byref(raw_ptr), 30)
            if st == 0:
                mv.CameraImageProcess(h_cam, raw_ptr, rgb_buf, ctypes.byref(head))
                frame = np.ctypeslib.as_array(rgb_buf).reshape((head.iHeight, head.iWidth, 3)).copy()
                mv.CameraReleaseImageBuffer(h_cam, raw_ptr)

                frame_count += 1
                now = time.time()
                if now - t_start >= 1.0:
                    cam_fps = frame_count / (now - t_start)
                    frame_count = 0
                    t_start = now

                # YOLO Inference
                dets = []
                if enable_yolo:
                    t_inf0 = time.time()
                    res = model.predict(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
                    t_inf1 = time.time()
                    yolo_ms = (t_inf1 - t_inf0) * 1000.0
                    yolo_fps = 1000.0 / max(0.1, yolo_ms)

                    for box in res.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        score = float(box.conf[0])
                        dets.append((x1, y1, x2, y2, score))

                # Draw UI
                disp = cv2.resize(frame, (960, 540))
                sx = 960.0 / head.iWidth
                sy = 540.0 / head.iHeight

                for (x1, y1, x2, y2, score) in dets:
                    bx1, by1 = int(x1 * sx), int(y1 * sy)
                    bx2, by2 = int(x2 * sx), int(y2 * sy)
                    cx, cy = (bx1 + bx2) // 2, (by1 + by2) // 2

                    cv2.rectangle(disp, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                    cv2.circle(disp, (cx, cy), 4, (0, 0, 255), -1)
                    label = f"Ball {int(score * 100)}%"
                    cv2.putText(disp, label, (bx1, max(15, by1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Overlays
                cv2.putText(disp, f"Camera FPS: {cam_fps:.1f} (Speed Level 2)", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                y_status = f"YOLO: [ON] {yolo_fps:.1f} FPS ({yolo_ms:.1f}ms) | imgsz={args.imgsz}" if enable_yolo else "YOLO: [OFF] (Press 'y')"
                y_color = (0, 255, 0) if enable_yolo else (0, 165, 255)
                cv2.putText(disp, y_status, (15, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, y_color, 2)
                cv2.putText(disp, "[y] Toggle YOLO | [q] Exit", (15, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                cv2.imshow("Monocular High-Speed YOLO Benchmark", disp)

            key = cv2.waitKey(1)
            if key == ord('q') or key == ord('Q') or key == 27:
                break
            elif key == ord('y') or key == ord('Y'):
                enable_yolo = not enable_yolo

    finally:
        mv.CameraStop(h_cam)
        mv.CameraUnInit(h_cam)
        cv2.destroyAllWindows()
        print("Camera released cleanly.")

if __name__ == '__main__':
    main()
