import open3d as o3d
import numpy as np
import random
import os

def load_point_clouds_from_folder(path_to_folder: str):
    """
    Loads a list of PointCloud objects from given path_to_folder with supported formats.
    """
    point_clouds = [o3d.io.read_point_cloud(os.path.join(path_to_folder, file)) for file in os.listdir(path_to_folder)]
    return point_clouds

def visualize_random_point_clouds_with_gradients(folder_path: str, num_pcs: int = 10):
    """
    Loads `num_pcs` random point clouds from the given folder, applies different color gradients,
    and visualizes them.
    """
    random.seed(2)

    point_clouds = load_point_clouds_from_folder(folder_path)
    
    sampled_indices = random.sample(range(550, 621), num_pcs)
    # sampled_indices = random.sample(range(len(point_clouds)), num_pcs)
    random_pcs = [point_clouds[i] for i in sampled_indices]
    print(f"Sampled indices: {sampled_indices}")
    # random_pcs = random.sample(point_clouds, num_pcs)
    for i, pcd in enumerate(random_pcs):
        points = np.asarray(pcd.points)
        
        z_min = np.min(points[:, 2])
        z_max = np.max(points[:, 2])
        z_range = z_max - z_min
        normalized_z = (points[:, 2] - z_min) / z_range
        
        # Apply a unique gradient for each point cloud
        colors = np.zeros((points.shape[0], 3))
        # Generate a unique color by blending based on the index
        base_color = np.array([i % 3 == 0, i % 3 == 1, i % 3 == 2], dtype=float)
        blend_factor = (i % 10) / 10.0
        unique_color = base_color * (1 - blend_factor) + np.array([1, 1, 1]) * blend_factor
        colors[:] = unique_color * normalized_z[:, None]
        # colors[:, i % 3] = normalized_z  # Cycle through RGB channels
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        o3d.visualization.draw_geometries([pcd])
    
    print("Done")

if __name__ == '__main__':
    point_clouds_list = load_point_clouds_from_folder('/external/mapping_pcl/foundation_pcl/clouds/')

    # source = o3d.io.read_point_cloud("/external/mapping_pcl/madpose_airlab1/7_point_cloud_0.ply")
    # target = o3d.io.read_point_cloud("/external/mapping_pcl/madpose_airlab1/89_point_cloud_0.ply")
    # visualize_point_cloud(target)
    # pair_registration(source, target)

    # visualize_random_point_clouds_with_gradients(folder_path='/external/mapping_pcl/foundation_pcl/clouds/')