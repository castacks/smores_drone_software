import cv2
import yaml
import numpy as np
from typing import Tuple, List
from matplotlib import pyplot as plt
from camera_transformations import *

def uncalibrated_stereo_rectification(img1: np.ndarray, img2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Perform uncalibrated stereo rectification on a pair of images.

    Args:
        img1 (np.ndarray): First input image.
        img2 (np.ndarray): Second input image.

    Returns:
        Tuple[np.ndarray, np.ndarray]: Rectified images.
    """
    # Load camera parameters
    K1, D1, K2, D2 = load_camera_parameters('configs/sensor_parameters.yaml')

    # Undistort the images
    h, w = img1.shape[:2]
    img1_undistorted = cv2.undistort(img1, K1, D1, None, K1)
    img2_undistorted = cv2.undistort(img2, K2, D2, None, K2)

    # Detect and match features
    keypoints1, keypoints2, descriptors1, descriptors2 = detect_features(img1_undistorted, img2_undistorted)
    good_matches, pts1, pts2 = match_features(keypoints1, keypoints2, descriptors1, descriptors2)

    # Optionally, display keypoints and matches
    show_keypoints_and_matches(img1_undistorted, img2_undistorted, keypoints1, keypoints2, good_matches)

    # Compute the fundamental matrix
    F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC)

    # Select only inlier points
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]

    # Compute the rectification homographies
    _, h1, h2 = cv2.stereoRectifyUncalibrated(pts1, pts2, F, (w, h))

    # Warp the images
    img1_rectified = cv2.warpPerspective(img1_undistorted, h1, (w, h))
    img2_rectified = cv2.warpPerspective(img2_undistorted, h2, (w, h))

    # Display the rectified images
    cv2.imshow('Left Image Rectified', img1_rectified)
    cv2.imshow('Right Image Rectified', img2_rectified)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return img1_rectified, img2_rectified

def load_camera_parameters(config_file: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Load camera parameters from a YAML configuration file.

    Args:
        config_file (str): Path to the YAML configuration file.

    Returns:
        Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: Intrinsic parameters (K1, K2) and distortion coefficients (D1, D2) for two cameras.
    """
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    
    cam1 = config['thermalCamera1']
    cam2 = config['thermalCamera2']
    
    K1 = np.array([[cam1['fx'], 0, cam1['cx']], [0, cam1['fy'], cam1['cy']], [0, 0, 1]])
    D1 = np.array([cam1['k1'], cam1['k2'], cam1['p1'], cam1['p2'], 0])
    
    K2 = np.array([[cam2['fx'], 0, cam2['cx']], [0, cam2['fy'], cam2['cy']], [0, 0, 1]])
    D2 = np.array([cam2['k1'], cam2['k2'], cam2['p1'], cam2['p2'], 0])
    
    return K1, D1, K2, D2

def detect_features(img1: np.ndarray, img2: np.ndarray, feature_detector='orb') -> Tuple[list, list, np.ndarray, np.ndarray]:
    """
    Detect ORB features and compute descriptors for two images.

    Args:
        img1 (np.ndarray): First input image.
        img2 (np.ndarray): Second input image.

    Returns:
        Tuple[list, list, np.ndarray, np.ndarray]: Keypoints and descriptors for both images.
    """
    if feature_detector == 'orb':
        orb = cv2.ORB_create()
        keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
        keypoints2, descriptors2 = orb.detectAndCompute(img2, None)
    elif feature_detector == 'sift':
        sift = cv2.SIFT_create()
        keypoints1, descriptors1 = sift.detectAndCompute(img1,None)
        keypoints2, descriptors2 = sift.detectAndCompute(img2,None)

    return keypoints1, keypoints2, descriptors1, descriptors2

def match_features(keypoints1: list, keypoints2: list, descriptors1: np.ndarray, descriptors2: np.ndarray) -> Tuple[List[cv2.DMatch], np.ndarray, np.ndarray]:
    """
    Match features between two sets of keypoints and descriptors using BFMatcher and Lowe's ratio test.

    Args:
        keypoints1 (list): Keypoints from the first image.
        keypoints2 (list): Keypoints from the second image.
        descriptors1 (np.ndarray): Descriptors from the first image.
        descriptors2 (np.ndarray): Descriptors from the second image.

    Returns:
        Tuple[List[DMatch], np.ndarray, np.ndarray]: Cv2 matches object and matched points from both images.
    """
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(descriptors1, descriptors2, k=2)
    good_matches = []; pts1 = []; pts2 = []
    # Apply Lowe's ratio test
    for m, n in matches:
        if m.distance < 0.9 * n.distance:
            good_matches.append(m)
            pts1.append((keypoints1[m.queryIdx].pt))
            pts2.append((keypoints2[m.trainIdx].pt))

    return good_matches, np.int32(pts1), np.int32(pts2)

def find_fundamental_matrix(pts1, pts2):
    F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC)
    # Select only inlier points
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]

    return F, pts1, pts2

def estimate_R_and_T(F, pts1: np.ndarray, pts2: np.ndarray, K1: np.ndarray, K2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Estimate the rotation (R) and translation (T) between two cameras using matched points.

    Args:
        pts1 (np.ndarray): Matched points from the first image.
        pts2 (np.ndarray): Matched points from the second image.
        K1 (np.ndarray): Intrinsic parameters of the first camera.
        K2 (np.ndarray): Intrinsic parameters of the second camera.

    Returns:
        Tuple[np.ndarray, np.ndarray]: Rotation matrix (R) and translation vector (T) between the two cameras.
    """
    # F, mask = cv2.findFundamentalMat(pts1, pts2)
    # E = K2.T @ F @ K1
    # _, R, T, mask = cv2.recoverPose(E, pts1, pts2, K1)
    # Compute the essential matrix
    E, mask = cv2.findEssentialMat(pts1, pts2, K1, method=cv2.RANSAC)

    # E = K1.T @ F @ K2
    # E /= E[2,2]

    # Decompose the essential matrix to obtain R and T and resolve 4 possible configurations
    _, R, T, mask = cv2.recoverPose(E, pts1, pts2, K1)

    return R, T

def show_keypoints_and_matches(img1, img2, keypoints1, keypoints2, matches):

    # Draw keypoints
    img1_keypoints = cv2.drawKeypoints(img1, keypoints1, None, color=(0, 255, 0))
    img2_keypoints = cv2.drawKeypoints(img2, keypoints2, None, color=(0, 255, 0))

    # Draw matches
    img_matches = cv2.drawMatches(img1, keypoints1, img2, keypoints2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Display the keypoints
    cv2.imshow('Left Image Keypoints', img1_keypoints)
    cv2.imshow('Right Image Keypoints', img2_keypoints)

    # Display the matches
    cv2.imshow('Matches', img_matches)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def plot_epi_lines(pts1, pts2, F, img1, img2):
    lines1 = cv2.computeCorrespondEpilines(pts2.reshape(-1,1,2), 2, F)
    lines1 = lines1.reshape(-1,3)
    lines2 = cv2.computeCorrespondEpilines(pts1.reshape(-1,1,2), 1, F)
    lines2 = lines2.reshape(-1,3)
    img5,img6 = draw_lines(img1,img2,lines1,pts1,pts2)
    img3,img4 = draw_lines(img2,img1,lines2,pts2,pts1)

    cv2.imshow('right', img3)
    cv2.imshow('left', img5)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def draw_lines(img1,img2,lines,pts1,pts2):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r,c = img1.shape
    img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv2.line(img1, (x0,y0), (x1,y1), color,1)
        img1 = cv2.circle(img1,tuple(pt1),5,color,-1)
        img2 = cv2.circle(img2,tuple(pt2),5,color,-1)
    return img1,img2

def calibrated_stereo_rectification(img1, img2):
    # Load camera parameters
    K1, D1, K2, D2 = load_camera_parameters('configs/sensor_parameters.yaml')
    
    # Undistort the images
    h, w = img1.shape[:2]
    img1_undistorted = cv2.undistort(img1, K1, D1, None, K1)
    img2_undistorted = cv2.undistort(img2, K2, D2, None, K2)

    # Display the undistorted images
    cv2.imshow('left', img1)
    cv2.imshow('right', img2)
    cv2.imshow('Left Image Undistorted', img1_undistorted)
    cv2.imshow('Right Image Undistorted', img2_undistorted)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Detect features
    keypoints1, keypoints2, descriptors1, descriptors2 = detect_features(img1_undistorted, img2_undistorted)

    # Match features
    matches, pts1, pts2 = match_features(keypoints1, keypoints2, descriptors1, descriptors2)

    # Optionally, display keypoints and matches
    show_keypoints_and_matches(img1_undistorted, img2_undistorted, keypoints1, keypoints2, matches)
    
    F, pts1_inliers, pts2_inliers = find_fundamental_matrix(pts1, pts2)

    plot_epi_lines(pts1_inliers, pts2_inliers, F, img1_undistorted, img2_undistorted)

    # Estimate rotation and translation
    R, T = estimate_R_and_T(F, pts1_inliers, pts2_inliers, K1, K2)
    print(R)
    print(T)

    T = np.array([[-0.264],
                  [0],
                  [0]])

    # Stereo rectification
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, (w, h), R, T)

    # Compute rectification maps
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)

    # Apply rectification
    img1_rectified = cv2.remap(img1, map1x, map1y, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(img2, map2x, map2y, cv2.INTER_LINEAR)

    # Display the rectified images
    cv2.imshow('Left Image Rectified', img1_rectified)
    cv2.imshow('Right Image Rectified', img2_rectified)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return img1_rectified, img2_rectified

def check_correctness_rectified(left_rectified, right_rectified):
    # Detect features
    keypoints1, keypoints2, descriptors1, descriptors2 = detect_features(left_rectified, right_rectified)

    # Match features
    matches, pts1, pts2 = match_features(keypoints1, keypoints2, descriptors1, descriptors2)

    # Optionally, display keypoints and matches
    show_keypoints_and_matches(left_rectified, right_rectified, keypoints1, keypoints2, matches)
    
    F, pts1_inliers, pts2_inliers = find_fundamental_matrix(pts1, pts2)

    plot_epi_lines(pts1_inliers, pts2_inliers, F, left_rectified, right_rectified)

def rectification_with_calibrated_data(img1, img2):
    
    # Load camera parameters
    K1, D1, K2, D2 = load_camera_parameters('configs/sensor_parameters.yaml')
    h, w = img1.shape[:2]

    T_left = load_transformation_matrix("configs/ordv1_thermal_left.yaml")
    T_right = load_transformation_matrix("configs/ordv1_thermal_right.yaml")

    R, T = compute_transformation_between_cameras(T_left, T_right)

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K1, D1, K2, D2, (w,h), R, T)

    # Compute rectification maps
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)

    # Apply rectification
    img1_rectified = cv2.remap(img1, map1x, map1y, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(img2, map2x, map2y, cv2.INTER_LINEAR)

    # Crop the rectified images using the ROIs
    x1, y1, w1, h1 = roi1
    x2, y2, w2, h2 = roi2
    img1_rectified_cropped = img1_rectified[y1:y1+h1, x1:x1+w1]
    img2_rectified_cropped = img2_rectified[y2:y2+h2, x2:x2+w2]

    return img1_rectified_cropped, img2_rectified_cropped

if __name__ == '__main__':
    calibrated_stereo_rectification()