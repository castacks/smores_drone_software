import yaml
import numpy as np

def load_transformation_matrix(yaml_file: str) -> np.ndarray:
    """
    Load the transformation matrix from a YAML file.

    Args:
        yaml_file (str): Path to the YAML file.

    Returns:
        np.ndarray: The transformation matrix.
    """
    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)
    
    T_imu_cam = np.array(config['cam0']['T_imu_cam'])
    return T_imu_cam

def compute_transformation_between_cameras(T_left: np.ndarray, T_right: np.ndarray) -> np.ndarray:
    """
    Compute the transformation matrix between the left and right cameras.

    Args:
        T_left (np.ndarray): Transformation matrix for the left camera.
        T_right (np.ndarray): Transformation matrix for the right camera.

    Returns:
        np.ndarray: The transformation matrix from the left camera to the right camera.
    """
    T_left_inv = np.linalg.inv(T_left)
    print(f"Inverse of thermal left (source) to imu (target):\n {T_left_inv}")
    print(f"Thermal right (source) to imu (target):\n {T_right}")

    T_left_to_right = T_left_inv @ T_right
    R = T_left_to_right[:3, :3]
    T = T_left_to_right[:3, 3:]
    return R, T

def get_transformation():
    # Load transformation matrices from YAML files
    T_imu_thermal_left = load_transformation_matrix('configs/ordv1_thermal_left.yaml')
    T_imu_thermal_right = load_transformation_matrix('configs/ordv1_thermal_right.yaml')

    # Compute the transformation matrix from the left camera to the right camera
    R, T = compute_transformation_between_cameras(T_imu_thermal_left, T_imu_thermal_right)

    print("Transformation matrix from left camera to right camera:\n", R, T)

if __name__ == '__main__':
    get_transformation()