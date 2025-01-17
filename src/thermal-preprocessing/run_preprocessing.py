import os
import glob
import cv2
from histogram import *
from stereo_rectification import *

def run_preprocessing():
    """
        - Loads images from the /raw/ folder with raw outputs from thermal cameras
        - Iterates over all valid paths and loads images, applying histogram filtering followed by CLAHE
        - Saves output maintaining the same path but to the /preprocessed/ folder in place of /raw/
    """
    raw_base_path = 'data/Airlab_Depth_Estimation/raw'
    preprocessed_base_path = 'data/Airlab_Depth_Estimation/preprocessed'

    # Find all PNG images in the raw directory and its subdirectories
    image_paths = glob.glob(os.path.join(raw_base_path, '**', '*.png'), recursive=True)

    for path in image_paths:

        # Load the image
        img = load_img(path)
        hist, bins = img_to_hist(img)
        # print_hist(hist, bins)

        # Apply histogram filtering, CLAHE, Bilateral Filtering
        img = filter_outliers(img)
        img = apply_clahe(img)
        img = bilateral_filtering(img, d=50, sigmaColor=0, sigmaSpace=50)

        # Check data type and bit depth
        # print(f"Data type after CLAHE: {clahe_img.dtype}")
        # print(f"Bit depth after CLAHE: {clahe_img.itemsize * 8} bits")

        # Determine the save path
        relative_path = os.path.relpath(path, raw_base_path)
        save_path = os.path.join(preprocessed_base_path, relative_path)

        # Create the necessary subdirectories
        os.makedirs(os.path.dirname(save_path), exist_ok=True)

        # Save the image with uint8 data type
        cv2.imwrite(save_path, img)
        # print(f"Image saved at: {save_path}")

        # Optionally display the image
        # cv2.imshow("CLAHE Image", clahe_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

def rectify_images():
    preprocessed_base_path = 'data/Airlab_Depth_Estimation/preprocessed'

    # Find all PNG images in the raw directory and its subdirectories
    left_image_paths = glob.glob(os.path.join(preprocessed_base_path, '**', 'left.png'), recursive=True)
    right_image_paths = glob.glob(os.path.join(preprocessed_base_path, '**', 'right.png'), recursive=True)

    for left, right in list(zip(left_image_paths, right_image_paths)):

        left_img = load_img(left)
        right_img = load_img(right)

        left_rectified, right_rectified = rectification_with_calibrated_data(left_img, right_img)
        check_correctness_rectified(left_rectified, right_rectified)

        # Modify the file paths to include '_rectified'
        left_rectified_path = left.replace('left.png', 'left_rectified.png')
        right_rectified_path = right.replace('right.png', 'right_rectified.png')

        cv2.imwrite(left_rectified_path, left_rectified)
        cv2.imwrite(right_rectified_path, right_rectified)

if __name__ == '__main__':
    # run_preprocessing()
    rectify_images()