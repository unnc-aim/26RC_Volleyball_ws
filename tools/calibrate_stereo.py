#!/usr/bin/env python3
import os
import glob
import argparse
import cv2
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(description="Automated Stereo Camera Calibration for Volleyball Robot")
    parser.add_argument("--img_dir", type=str, default=".", help="Directory containing snapshot images (snapshot_*_left.jpg and snapshot_*_right.jpg)")
    parser.add_argument("--corners", type=str, default="9x6", help="Inner chessboard corners (width x height), e.g. 9x6 or 11x8")
    parser.add_argument("--square_size", type=float, default=35.0, help="Size of a chessboard square in millimeters (default: 35.0 mm)")
    parser.add_argument("--output", type=str, default="/home/aerialstewart/26_RC_Volleyball/src/volleyball_vision/config/stereo_calibration.yaml", help="Output YAML calibration file path")
    return parser.parse_args()

def main():
    args = parse_args()

    try:
        cw, ch = map(int, args.corners.split("x"))
    except ValueError:
        print(f"Error: Invalid corners format '{args.corners}', expected format like '9x6' or '11x8'")
        return

    pattern_size = (cw, ch)
    square_size_m = args.square_size / 1000.0 # Convert mm to meters

    # Prepare 3D object points in chessboard coordinate space (Z=0)
    objp = np.zeros((cw * ch, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cw, 0:ch].T.reshape(-1, 2) * square_size_m

    objpoints = [] # 3d points in real world space
    imgpoints_l = [] # 2d points in left image plane
    imgpoints_r = [] # 2d points in right image plane

    left_images = sorted(glob.glob(os.path.join(args.img_dir, "*left*.jpg")) + glob.glob(os.path.join(args.img_dir, "*left*.png")))
    right_images = sorted(glob.glob(os.path.join(args.img_dir, "*right*.jpg")) + glob.glob(os.path.join(args.img_dir, "*right*.png")))

    if not left_images or not right_images:
        print(f"Error: No image pairs found in '{args.img_dir}' matching '*left*.jpg' and '*right*.jpg'")
        print("Tip: Use 'tools/view_cameras' and press 's' while pointing at the chessboard to capture image pairs.")
        return

    print(f"============================================================")
    print(f"  Stereo Camera Calibration Tool")
    print(f"  Chessboard corners : {cw} x {ch}")
    print(f"  Square size        : {args.square_size} mm ({square_size_m:.4f} m)")
    print(f"  Image directory    : {os.path.abspath(args.img_dir)}")
    print(f"  Pairs found        : {min(len(left_images), len(right_images))}")
    print(f"============================================================")

    img_size = None
    valid_pairs = 0

    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for img_l_path in left_images:
        base_name = os.path.basename(img_l_path)
        # Find matching right image
        img_r_path = img_l_path.replace("left", "right")
        if not os.path.exists(img_r_path):
            continue

        img_l = cv2.imread(img_l_path)
        img_r = cv2.imread(img_r_path)

        if img_l is None or img_r is None:
            continue

        if img_size is None:
            img_size = (img_l.shape[1], img_l.shape[0])

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ret_l, corners_l = cv2.findChessboardCorners(gray_l, pattern_size, flags)
        ret_r, corners_r = cv2.findChessboardCorners(gray_r, pattern_size, flags)

        if ret_l and ret_r:
            corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), subpix_criteria)
            corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), subpix_criteria)

            objpoints.append(objp)
            imgpoints_l.append(corners_l)
            imgpoints_r.append(corners_r)
            valid_pairs += 1
            print(f"  [OK] Pair matched: {base_name}")
        else:
            reason = []
            if not ret_l: reason.append("Left failed")
            if not ret_r: reason.append("Right failed")
            print(f"  [SKIP] {base_name}: {', '.join(reason)}")

    print(f"\nSuccessfully extracted corners from {valid_pairs} valid stereo pairs.")
    if valid_pairs < 10:
        print("Warning: Recommended at least 15~25 valid pairs for accurate long-baseline stereo calibration.")
        if valid_pairs < 5:
            print("Error: Too few valid pairs (< 5) to calibrate.")
            return

    print("\nRunning single-camera calibrations...")
    ret_l, K1, D1, rvecs_l, tvecs_l = cv2.calibrateCamera(objpoints, imgpoints_l, img_size, None, None)
    ret_r, K2, D2, rvecs_r, tvecs_r = cv2.calibrateCamera(objpoints, imgpoints_r, img_size, None, None)
    print(f"  Left Camera  Reprojection RMS: {ret_l:.4f} px")
    print(f"  Right Camera Reprojection RMS: {ret_r:.4f} px")

    print("\nRunning stereo calibration (solving R and T)...")
    stereo_flags = cv2.CALIB_FIX_INTRINSIC
    stereo_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)

    ret_s, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_l, imgpoints_r,
        K1, D1, K2, D2, img_size,
        criteria=stereo_criteria, flags=stereo_flags
    )

    baseline_calc = float(np.linalg.norm(T))
    print(f"\n============================================================")
    print(f"  Stereo Calibration Finished!")
    print(f"  Stereo Reprojection RMS Error: {ret_s:.4f} px (Ideal: < 0.5 px)")
    print(f"  Calculated Baseline Distance : {baseline_calc:.4f} m (Target: ~0.80 m)")
    print(f"  Translation Vector T         : [{T[0, 0]:.4f}, {T[1, 0]:.4f}, {T[2, 0]:.4f}]")
    print(f"============================================================")

    # Save to standard OpenCV YAML format
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    fs = cv2.FileStorage(args.output, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", img_size[0])
    fs.write("image_height", img_size[1])
    fs.write("K1", K1)
    fs.write("D1", D1)
    fs.write("K2", K2)
    fs.write("D2", D2)
    fs.write("R", R)
    fs.write("T", T)
    fs.release()

    print(f"\nCalibration saved successfully to:")
    print(f"  -> {os.path.abspath(args.output)}")
    print("\nNext step: Run 'colcon build --packages-select volleyball_vision' to refresh install directory.")

if __name__ == "__main__":
    main()
