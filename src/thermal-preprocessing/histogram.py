import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

def load_img(path):
    # Check if path exists, load image as 16 bit and convert to grayscale
    if os.path.exists(path):
        img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise ValueError(f"Failed to load image: {path}")
        if len(img.shape) == 3 and img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        raise FileNotFoundError(f"Invalid path: {path}")
    return img

def img_to_hist(img):
    # Convert to a numpy histogram
    hist, bins = np.histogram(img, bins=256, range=[0,256])
    return hist, bins

def plot_hist(hist, bins):
    # Plot the histogram to see visually the intensity distribution
    plt.plot(bins[:-1], hist)  # bins[:-1] to match the length of hist
    plt.xlim([0,256])
    plt.title("Histogram of pixel intensity of thermal image")
    plt.xlabel("Pixel Intensity")
    plt.ylabel("Frequency")
    plt.show()

def print_hist(hist, bins):
    # Print counts for non zero bins, alternate to viewing plots
    for bin_edge, count in zip(bins[:-1], hist):
        if count != 0:
            print(f"Bin {bin_edge}: {count} pixels")

def filter_outliers(img, lower_percentile=1, upper_percentile=99):
    # Filtering outliers above and below 2 certain thresholds. Used to remove extremely bright spots like the sun
    lower_bound = np.percentile(img, lower_percentile)
    upper_bound = np.percentile(img, upper_percentile)
    filtered_img = np.clip(img, lower_bound, upper_bound)
    filtered_img = ((filtered_img - lower_bound) / (upper_bound - lower_bound) * 255).astype(np.uint8)
    return filtered_img

def calc_intensity_binding(hist):
    # Calculates normal histogram equalization and returns a transform
    cdf = hist.cumsum()
    cdf_normalized = cdf * float(hist.max()) / cdf.max()
    plt.plot(cdf, color = 'b')
    plt.xlim([0,256])
    plt.legend(('cdf','histogram'), loc = 'upper left')
    plt.show()
    cdf_m = np.ma.masked_equal(cdf,0)
    cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())
    cdf = np.ma.filled(cdf_m,0).astype('uint8')
    return cdf

def apply_intensity_binding(img, cdf):
    # Apllies returned transform
    img2 = cdf[img]
    return img2

def apply_clahe(img):
    # Applies CLAHE (a specific type of histogram equalization)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(img)

def bilateral_filtering(img, d=9, sigmaColor=75, sigmaSpace=75):
    # Applies edge aware smoothing, smoothing type is gaussian blur under the hood
    return cv2.bilateralFilter(img, d=d, sigmaColor=sigmaColor, sigmaSpace=sigmaSpace)