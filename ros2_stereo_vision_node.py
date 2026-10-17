#!/usr/bin/env python3
import ctypes
import time
import argparse
import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from ultralytics import YOLO

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

        self.mv.CameraSetIspOutFormat(self.h_cam, CAMERA_MEDIA_TYPE_BGR8)
        self.mv.CameraSetTriggerMode(self.h_cam, 0)
        self.mv.CameraSetExposureTime(self.h_cam, ctypes.c_double(2000.0))
        self.mv.CameraSetImageResolutionEx(self.h_cam, 0xFF, 0, 0, 0, 0, self.width, self.height, 0, 0)
        self.mv.CameraSetFrameSpeed(self.h_cam, 1) # Speed Level 1
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

class Ros2StereoVisionNode(Node):
    def __init__(self):
        super().__init__('volleyball_vision_node')

        self.declare_parameter('imgsz', 512)
        self.declare_parameter('conf_thresh', 0.35)
        self.declare_parameter('model_path', '/home/aerialstewart/26_RC_Volleyball/src/volleyball_vision/models/yolov8n.pt')
        self.declare_parameter('show_window', True)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('baseline', 0.8)

        self.imgsz = int(self.get_parameter('imgsz').value)
        self.conf_thresh = float(self.get_parameter('conf_thresh').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.baseline = float(self.get_parameter('baseline').value)

        self.point_pub = self.create_publisher(PointStamped, '/volleyball/position', 10)
        self.debug_img_pub = self.create_publisher(Image, '/volleyball/debug_image', 10)

        self.get_logger().info(f"Loading PyTorch YOLO model: {self.model_path} (imgsz={self.imgsz})...")
        self.model = YOLO(self.model_path)

        # Init MindVision Cameras
        self.mv = ctypes.cdll.LoadLibrary('/home/aerialstewart/26_RC_Volleyball/src/volleyball_vision/3rdparty/mindvision/lib/libMVSDK.so')
        self.mv.CameraSdkInit(1)

        dev_list = (tSdkCameraDevInfo * 16)()
        num_dev = ctypes.c_int(16)
        self.mv.CameraEnumerateDevice(dev_list, ctypes.byref(num_dev))

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
            self.get_logger().error("Need at least 2 MindVision cameras for stereo mode!")
            raise RuntimeError("Camera count < 2")

        self.cam_l = CameraWorker(self.mv, unique_devs[0], name="Left")
        self.cam_r = CameraWorker(self.mv, unique_devs[1], name="Right")

        if not self.cam_l.open() or not self.cam_r.open():
            raise RuntimeError("Failed to open cameras")

        self.cam_l.start()
        self.cam_r.start()

        if self.show_window:
            cv2.namedWindow("ROS 2 Stereo Volleyball Vision (Direct OpenCV GUI)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ROS 2 Stereo Volleyball Vision (Direct OpenCV GUI)", 1280, 400)

        # 60 Hz timer loop
        self.timer = self.create_wall_timer(0.015, self.process_loop)
        self.get_logger().info("Ros2StereoVisionNode successfully started! Direct OpenCV GUI active.")

    def process_loop(self):
        frame_l = self.cam_l.get_frame()
        frame_r = self.cam_r.get_frame()

        if frame_l is None or frame_r is None:
            return

        t0 = time.time()
        # High-Speed Batch=2 inference
        results = self.model.predict([frame_l, frame_r], imgsz=self.imgsz, conf=self.conf_thresh, verbose=False)
        inf_ms = (time.time() - t0) * 1000.0

        res_l, res_r = results[0], results[1]

        dets_l = []
        dets_r = []
        for box in res_l.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            dets_l.append((x1, y1, x2, y2, float(box.conf[0])))
        for box in res_r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            dets_r.append((x1, y1, x2, y2, float(box.conf[0])))

        # Triangulation (if volleyball detected in both frames)
        if len(dets_l) > 0 and len(dets_r) > 0:
            bl = dets_l[0]
            br = dets_r[0]
            cx_l = (bl[0] + bl[2]) * 0.5
            cy_l = (bl[1] + bl[3]) * 0.5
            cx_r = (br[0] + br[2]) * 0.5
            cy_r = (br[1] + br[3]) * 0.5

            disparity = cx_l - cx_r
            if disparity > 1.0:
                # Approximate geometry (baseline=0.8m, focal length ~1400px at 1280x720)
                f_px = 1400.0
                z = (f_px * self.baseline) / disparity
                x = (cx_l - 640.0) * z / f_px
                y = (cy_l - 360.0) * z / f_px

                pt_msg = PointStamped()
                pt_msg.header.stamp = self.get_clock().now().to_msg()
                pt_msg.header.frame_id = self.frame_id
                pt_msg.point.x = float(x)
                pt_msg.point.y = float(y)
                pt_msg.point.z = float(z)
                self.point_pub.publish(pt_msg)

        # Prepare visual display / ROS image topic if needed
        need_display = self.show_window or (self.debug_img_pub.get_subscription_count() > 0)
        if need_display:
            disp_w, disp_h = 640, 360
            sx, sy = disp_w / 1280.0, disp_h / 720.0

            disp_l = cv2.resize(frame_l, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)
            disp_r = cv2.resize(frame_r, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

            for (x1, y1, x2, y2, score) in dets_l:
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cv2.rectangle(disp_l, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                cv2.circle(disp_l, ((bx1 + bx2)//2, (by1 + by2)//2), 4, (0, 0, 255), -1)
                cv2.putText(disp_l, f"Ball {int(score*100)}%", (bx1, max(15, by1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            for (x1, y1, x2, y2, score) in dets_r:
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cv2.rectangle(disp_r, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                cv2.circle(disp_r, ((bx1 + bx2)//2, (by1 + by2)//2), 4, (0, 0, 255), -1)
                cv2.putText(disp_r, f"Ball {int(score*100)}%", (bx1, max(15, by1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # HUD
            cv2.putText(disp_l, f"Left Cam: {self.cam_l.fps:.1f} FPS", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(disp_l, f"PyTorch Batch(2): {1000.0/max(0.1, inf_ms):.1f} FPS ({inf_ms:.1f}ms)", (15, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(disp_r, f"Right Cam: {self.cam_r.fps:.1f} FPS", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(disp_r, "ROS 2 Topics: /volleyball/position", (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            combined = cv2.hconcat([disp_l, disp_r])

            # Publish to ROS 2 debug image topic (viewable by rqt_image_view / rviz2)
            if self.debug_img_pub.get_subscription_count() > 0:
                img_msg = Image()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = self.frame_id
                img_msg.height, img_msg.width = combined.shape[:2]
                img_msg.encoding = "bgr8"
                img_msg.is_bigendian = 0
                img_msg.step = combined.shape[1] * 3
                img_msg.data = combined.tobytes()
                self.debug_img_pub.publish(img_msg)

            # Direct OpenCV GUI window
            if self.show_window:
                cv2.imshow("ROS 2 Stereo Volleyball Vision (Direct OpenCV GUI)", combined)
                key = cv2.waitKey(1)
                if key == ord('q') or key == ord('Q') or key == 27:
                    rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, 'cam_l'): self.cam_l.close()
        if hasattr(self, 'cam_r'): self.cam_r.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = Ros2StereoVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
