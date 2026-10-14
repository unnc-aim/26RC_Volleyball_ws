#!/usr/bin/env python3
import ctypes
import time
import argparse
import threading
import cv2
import numpy as np
from ultralytics import YOLO

# MindVision Media Type definition
CAMERA_MEDIA_TYPE_BGR8 = 0x02180015

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

class CameraWorker:
    def __init__(self, mv, dev_info, name="Cam"):
        self.mv = mv
        self.dev_info = dev_info
        self.name = name
        self.sn = dev_info.acSn.decode()
        self.h_cam = ctypes.c_int(-1)
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.latest_frame = None
        self.fps = 0.0
        self.width = 1280
        self.height = 720

    def open(self):
        st = self.mv.CameraInit(ctypes.byref(self.dev_info), -1, -1, ctypes.byref(self.h_cam))
        if st != 0:
            print(f"[{self.name}] CameraInit failed, err={st}")
            return False

        # Correct BGR8 format (0x02180015)
        self.mv.CameraSetIspOutFormat(self.h_cam, CAMERA_MEDIA_TYPE_BGR8)
        self.mv.CameraSetTriggerMode(self.h_cam, 0) # continuous acquisition
        self.mv.CameraSetExposureTime(self.h_cam, ctypes.c_double(2000.0)) # 2ms exposure

        # Scheme 2: 1280x720 ROI cropping (Y=0, preserving aerial volleyball space)
        self.mv.CameraSetImageResolutionEx(self.h_cam, 0xFF, 0, 0, 0, 0, self.width, self.height, 0, 0)

        # Scheme 1: Speed Level 1 (Balanced mode, ~37 FPS each, ~700 Mbps total safe switch bandwidth)
        self.mv.CameraSetFrameSpeed(self.h_cam, 1)
        self.mv.CameraSetFrameRate(self.h_cam, 0)

        self.mv.CameraPlay(self.h_cam)
        return True

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        rgb_buf = (ctypes.c_ubyte * (self.width * self.height * 3))()
        head = tSdkFrameHead()
        raw_ptr = ctypes.c_void_p()

        fps_counter = 0
        last_t = time.time()

        while self.running:
            st = self.mv.CameraGetImageBuffer(self.h_cam, ctypes.byref(head), ctypes.byref(raw_ptr), 30)
            if st == 0:
                self.mv.CameraImageProcess(self.h_cam, raw_ptr, rgb_buf, ctypes.byref(head))
                img = np.ctypeslib.as_array(rgb_buf).reshape((head.iHeight, head.iWidth, 3)).copy()
                self.mv.CameraReleaseImageBuffer(self.h_cam, raw_ptr)

                with self.lock:
                    self.latest_frame = img

                fps_counter += 1
                now = time.time()
                if now - last_t >= 1.0:
                    self.fps = fps_counter / (now - last_t)
                    fps_counter = 0
                    last_t = now

    def get_frame(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None

    def close(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.h_cam.value > 0:
            self.mv.CameraStop(self.h_cam)
            self.mv.CameraUnInit(self.h_cam)
            self.h_cam.value = -1

def main():
    parser = argparse.ArgumentParser(description="High-Speed Stereo MindVision + PyTorch YOLO Live Benchmark")
    parser.add_argument("--imgsz", type=int, default=512, help="YOLO input size (640 for high accuracy, 512 for 60+ FPS)")
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

    seen_sns = set()
    unique_devs = []
    for i in range(num_dev.value):
        dev = dev_list[i]
        port = dev.acPortType.decode()
        sn = dev.acSn.decode()
        if "192.168.1.100" in port and sn not in seen_sns:
            seen_sns.add(sn)
            unique_devs.append(dev)

    if len(unique_devs) < 2:
        for i in range(num_dev.value):
            dev = dev_list[i]
            sn = dev.acSn.decode()
            if sn not in seen_sns:
                seen_sns.add(sn)
                unique_devs.append(dev)

    if len(unique_devs) < 2:
        print("Error: Need at least 2 distinct MindVision cameras for stereo mode!")
        return

    cam_l = CameraWorker(mv, unique_devs[0], name="Left")
    cam_r = CameraWorker(mv, unique_devs[1], name="Right")

    if not cam_l.open():
        return
    if not cam_r.open():
        cam_l.close()
        return

    cam_l.start()
    cam_r.start()

    print("\n" + "=" * 65)
    print("  High-Speed Stereo MindVision + PyTorch YOLO Live Benchmark Started!")
    print(f"  Left Camera:  SN {cam_l.sn} | SpeedLevel=1 | ROI=1280x720")
    print(f"  Right Camera: SN {cam_r.sn} | SpeedLevel=1 | ROI=1280x720")
    print(f"  Inference:    PyTorch AVX-512 Batched (Batch=2, imgsz={args.imgsz})")
    print("  Controls:     [y] Toggle YOLO | [s] Save Snapshot | [q] Exit")
    print("=" * 65 + "\n")

    disp_w, disp_h = 640, 360
    sx = disp_w / 1280.0
    sy = disp_h / 720.0

    cv2.namedWindow("High-Speed Stereo MindVision + PyTorch YOLO Benchmark", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("High-Speed Stereo MindVision + PyTorch YOLO Benchmark", 1280, 400)

    enable_yolo = True
    yolo_fps = 0.0
    yolo_ms = 0.0
    snapshot_idx = 0

    try:
        while True:
            frame_l = cam_l.get_frame()
            frame_r = cam_r.get_frame()

            if frame_l is None or frame_r is None:
                time.sleep(0.005)
                continue

            dets_l = []
            dets_r = []

            if enable_yolo:
                t0 = time.time()
                # Run high-performance Batched Inference on both Left and Right frames concurrently
                batch_results = model.predict([frame_l, frame_r], imgsz=args.imgsz, conf=args.conf, verbose=False)
                t1 = time.time()
                yolo_ms = (t1 - t0) * 1000.0
                yolo_fps = 1000.0 / max(0.1, yolo_ms)

                res_l, res_r = batch_results[0], batch_results[1]
                for box in res_l.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    dets_l.append((x1, y1, x2, y2, float(box.conf[0])))
                for box in res_r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    dets_r.append((x1, y1, x2, y2, float(box.conf[0])))

            disp_l = cv2.resize(frame_l, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)
            disp_r = cv2.resize(frame_r, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

            # Draw Left detections
            for (x1, y1, x2, y2, score) in dets_l:
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cx, cy = (bx1 + bx2) // 2, (by1 + by2) // 2
                cv2.rectangle(disp_l, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                cv2.circle(disp_l, (cx, cy), 4, (0, 0, 255), -1)
                label = f"Ball {int(score * 100)}%"
                cv2.putText(disp_l, label, (bx1, max(15, by1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw Right detections
            for (x1, y1, x2, y2, score) in dets_r:
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cx, cy = (bx1 + bx2) // 2, (by1 + by2) // 2
                cv2.rectangle(disp_r, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                cv2.circle(disp_r, (cx, cy), 4, (0, 0, 255), -1)
                label = f"Ball {int(score * 100)}%"
                cv2.putText(disp_r, label, (bx1, max(15, by1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Overlays Left
            cv2.putText(disp_l, f"Left Cam: SN {cam_l.sn} | FPS: {cam_l.fps:.1f}", (15, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_status = f"YOLO: [ON] {yolo_fps:.1f} FPS ({yolo_ms:.1f}ms/batch2)" if enable_yolo else "YOLO: [OFF] (Press 'y')"
            y_color = (0, 255, 0) if enable_yolo else (0, 165, 255)
            cv2.putText(disp_l, y_status, (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, y_color, 2)

            # Overlays Right
            cv2.putText(disp_r, f"Right Cam: SN {cam_r.sn} | FPS: {cam_r.fps:.1f}", (15, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(disp_r, "[y] Toggle YOLO  [s] Snapshot  [q] Quit", (15, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            combined = cv2.hconcat([disp_l, disp_r])
            cv2.imshow("High-Speed Stereo MindVision + PyTorch YOLO Benchmark", combined)

            key = cv2.waitKey(1)
            if key == ord('q') or key == ord('Q') or key == 27:
                break
            elif key == ord('y') or key == ord('Y'):
                enable_yolo = not enable_yolo
            elif key == ord('s') or key == ord('S'):
                fname_l = f"stereo_snap_{snapshot_idx}_left.jpg"
                fname_r = f"stereo_snap_{snapshot_idx}_right.jpg"
                cv2.imwrite(fname_l, frame_l)
                cv2.imwrite(fname_r, frame_r)
                print(f"[Snapshot] Saved {fname_l} and {fname_r}")
                snapshot_idx += 1

    finally:
        print("Stopping camera streams...")
        cam_l.close()
        cam_r.close()
        cv2.destroyAllWindows()
        print("Stereo benchmark exited cleanly.")

if __name__ == '__main__':
    main()
