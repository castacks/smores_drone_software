import os
import copy
import numpy as np
import open3d as o3d
import random

from typing import list as list_t, tuple as tuple_t, float as float_t, int as int_t
from numpy import typing as npt

voxel_size = 0.02
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5

def load_point_clouds_from_folder(path_to_folder: str):
    """
    Loads a list of PointCloud objects from given path_to_folder with supported formats.
    """
    point_clouds = [o3d.io.read_point_cloud(os.path.join(path_to_folder, file)) for file in os.listdir(path_to_folder)]
    return point_clouds


def color_point_cloud_based_on_position(list_of_pcs: list):
    """
    Color point clouds with a gradient according to coordinates to differentiate depth better
    """

    for pcd in list_of_pcs:
        points = np.asarray(pcd.points)
        # NOTE: Can convert to float32 here
        
        # Normalize the Z-coordinate to the range [0, 1] for coloring
        z_min = np.min(points[:, 2])
        z_max = np.max(points[:, 2])
        z_range = z_max - z_min
        normalized_z = (points[:, 2] - z_min) / z_range
        
        # Create a color map (for example, a gradient from blue to red)
        colors = np.zeros((points.shape[0], 3))
        colors[:, 0] = normalized_z  # Red channel
        colors[:, 2] = 1 - normalized_z  # Blue channel
        
        pcd.colors = o3d.utility.Vector3dVector(colors)

def visualize_point_cloud(point_cloud):
    point_cloud.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([point_cloud],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])
    
def draw_registration_result(source, target, transformation):
    """
    Draw source and target point clouds with different colours and visualized transformed source from registration result
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])


def preprocess_point_cloud(pcd, every_k_points = 5, voxel_size = 0.1):
    """
    Preprocess point cloud by down sampling, estimating normals, computing FPFH features (feature descriptors)
    NOTE: Using uniform down sampling to avoid errors thrown by voxel down_sampling (voxel_size too small, deemed numerically unstable)
    """
    pcd_down = pcd.uniform_down_sample(every_k_points)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)) # max neighbours used to estimate normals

    # Use larger search radius for feature descriptions capturing unique features
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    return pcd_down, pcd_fpfh


def pairwise_registration(source: o3d.geometry.PointCloud, 
                          target: o3d.geometry.PointCloud):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp

def posegraph_registeration(pcds:list_t, 
                            max_correspondence_distance_coarse: float_t,
                            max_correspondence_distance_fine: float_t) -> o3d.registration.PoseGraph:
    """
    Params:
        pcds: list of point clouds to be registered
        max_correspondence_distance_coarse: max distance for coarse registration
        max_correspondence_distance_fine: max distance for fine registration
    """
    pose_graph = o3d.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=True))
    return pose_graph

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size=0.1):
    """
    Executes global (ransac based) registration between source and target pointclouds.
    NOTE: Not desired for more complicated problems
    """

    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    """
    Executes fast global registration based on feature matching 
    """
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

# def global_registration(source_down, source_fpfh, target_down, target_fpfh, voxel_size):
#     result_ransac = execute_global_registration(source_down, target_down,
#                                                 source_fpfh, target_fpfh,
#                                                 voxel_size)
#     print(result_ransac)
    # draw_registration_result(source_down, target_down, result_ransac.transformation)


# def multi_way_registration(list_of_pcs: list = None, voxel_size=0.5):
#     # print(len(list_of_pcs))
#     preprocessed_pcd = []
#     for pcd in list_of_pcs[:-1]:
#         # NOTE: Using uniform down_sampling to avoid voxel_size too small issue with voxel_grid down_sampling
#         # pcd_down.append(pcd.uniform_down_sample(every_k_points=5))
#         preprocessed_pcd.append(preprocess_point_cloud(pcd))
#     color_point_cloud_based_on_position(pcd_down)
#     o3d.visualization.draw_geometries(pcd_down,
#                                     zoom=0.3412,
#                                     front=[0.4257, -0.2125, -0.8795],
#                                     lookat=[2.6172, 2.0475, 1.532],
#                                     up=[-0.0694, -0.9768, 0.2024])

def pair_registration(source, target, registration_type="fast"):
    """
    Performs registration using desired method between source and target point cloudssss
    """

    VOXEL_SIZE = 0.05 # means at every 5 cm

    # Downsample point clouds and generate feature descriptors for matching
    source_down, source_feature_descriptor = preprocess_point_cloud(source, every_k_points=2)
    target_down, target_feature_descriptor = preprocess_point_cloud(target, every_k_points=2)
    
    if registration_type == "global":
        result = execute_global_registration(source_down, target_down,
                                                source_feature_descriptor, target_feature_descriptor,
                                                VOXEL_SIZE)
    elif registration_type == "fast": 
        result = execute_fast_global_registration(source_down, target_down,
                                                   source_feature_descriptor, target_feature_descriptor,
                                                   VOXEL_SIZE)
    else:
        raise ValueError ("Invalid value for argument registration_type, use one of [global, fast]")
    
    print(result)
    # visualize_point_cloud(source_down)
    visualize_point_cloud(target_down)
    draw_registration_result(source_down, target_down, result.transformation)

def visualize_random_point_clouds_with_gradients(folder_path: str, num_pcs: int = 10):
    """
    Loads `num_pcs` random point clouds from the given folder, applies different color gradients,
    and visualizes them.
    """
    random.seed(2)

    point_clouds = load_point_clouds_from_folder(folder_path)
    
    sampled_indices = random.sample(range(1, 40), num_pcs)
    # sampled_indices = random.sample(range(len(point_clouds)), num_pcs)
    random_pcs = [point_clouds[i] for i in sampled_indices]
    print(f"Sampled indices: {sampled_indices}")
    # random_pcs = random.sample(point_clouds, num_pcs)
    pcd_down = []
    for i, pcd in enumerate(random_pcs):
        points = np.asarray(pcd.points)
        # Calculate x, y, z ranges and normalize points
        x_min, y_min, z_min = np.min(points, axis=0)
        x_max, y_max, z_max = np.max(points, axis=0)
        x_range, y_range, z_range = x_max - x_min, y_max - y_min, z_max - z_min

        # Normalize all points to the range [0, 1]
        points[:, 0] = (points[:, 0] - x_min) / x_range + 2*(i+1)
        points[:, 1] = (points[:, 1] - y_min) / y_range
        points[:, 2] = (points[:, 2] - z_min) / z_range
        pcd.points = o3d.utility.Vector3dVector(points)
        
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

        pcd_down.append(pcd.uniform_down_sample(every_k_points=10))
    
    [print(f"Index: {sampled_indices[i]}, Min: {np.min(np.asarray(random_pcs[i].points), 0)}, Max: {np.max(np.asarray(random_pcs[i].points), 0)}") for i in range(len(random_pcs))]
    
    o3d.visualization.draw_geometries(pcd_down,
                                        zoom=0.4559,
                                        front=[0.6452, -0.3036, -0.7011],
                                        lookat=[1.9892, 2.0208, 1.8945],
                                        up=[-0.2779, -0.9482, 0.1556])
    
    print("Done")


if __name__ == '__main__':
    # point_clouds_list = load_point_clouds_from_folder('/external/mapping_pcl/foundation_pcl/clouds/')

    source = o3d.io.read_point_cloud("/storage/mapping_pcl/foundation_pcl/clouds/cloud_1.ply")
    target = o3d.io.read_point_cloud("/storage/mapping_pcl/foundation_pcl/clouds/cloud_2.ply")
    visualize_point_cloud(source)
    visualize_point_cloud(target)
    pair_registration(source, target)

    # visualize_random_point_clouds_with_gradients(folder_path='/storage/mapping_pcl/foundation_pcl/clouds')