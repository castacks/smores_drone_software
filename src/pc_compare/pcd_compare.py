import numpy as np
import open3d as o3d
# from numba import njit
# import torch
import argparse
import pytest

def get_pred2gt_schamfer_dist(gt_pcd, pred_pcd, voxel_downsample=0.05):
    """
    Calculates one way schamfer distance between two point clouds.
    It computes the distance from each point in the prediction to the closest point in the ground truth.
    The distance is computed as the Euclidean distance between the points.

    Args:
        gt_pcd (o3d.geometry.PointCloud): Ground truth point cloud.
        target_pcd (o3d.geometry.PointCloud): Target point cloud.

    Returns:
        float: The distance between the two point clouds.

    Example usage:
    >>>  get_pred2gt_schamfer_dist(generate_sphere_given_radius(100, 1), generate_sphere_given_radius(90, 0.9))
    1.0

    """
    
    downsampled_gt_pcd = gt_pcd.voxel_down_sample(voxel_size=voxel_downsample)
    downsampled_pred_pcd = pred_pcd.voxel_down_sample(voxel_size=voxel_downsample)
    
    num_pred_points = len(downsampled_pred_pcd.points)
    num_gt_points = len(downsampled_gt_pcd.points)
    
    gt_points = np.asarray(downsampled_gt_pcd.points).astype(np.float32)      # (G, 3)
    gt_points = np.expand_dims(np.asarray(gt_points), axis=1) # (G, 1, 3)
    
    pred_points = np.asarray(downsampled_pred_pcd.points).astype(np.float32)                          # (P, 3)
    pred_points = np.expand_dims(np.asarray(pred_points), axis=1).transpose(1, 0, 2)  # (1, P, 3)

    # Along a column, we have the distances from every point in prediction to 1 in gt
    # Along a row have the distances from every point in gt to 1 in prediction
    # gt_points = np.repeat(gt_points, num_pred_points, axis=1)      # (G, P, 3)
    # pred_points = np.repeat(pred_points, num_gt_points, axis=0)    # (G, P, 3)
    gt_points = np.repeat(gt_points, num_pred_points, axis=1)      # (G, P, 3) repeats G, 1, 3 for P cols
    pred_points = np.repeat(pred_points, num_gt_points, axis=0)    # (G, P, 3) repeats 1, P, 3 for G rows
            
    dists = np.linalg.norm(gt_points - pred_points, axis=2) # (G, P)
        
    # So given a pred (col), which gt point (row) is the closest? min down the row
    min_dists = np.min(dists, axis=0)  # (P,)

    one_way_schamfer_dist = np.mean(min_dists) # (1,)

    return one_way_schamfer_dist

def generate_sphere_given_radius(num_coords=50, radius=1.0):
    coord = np.linspace(-radius, radius, num_coords)
    x, y = np.meshgrid(coord, coord)
    x_flat = x.flatten()
    y_flat = y.flatten()
    x_sq = np.square(x_flat)
    y_sq = np.square(y_flat)

    # Only keep the points where x^2 + y^2 <= r^2 bc otherwise, bad things happen
    inside_circle = x_sq + y_sq <= radius**2
    x_flat = x_flat[inside_circle]
    y_flat = y_flat[inside_circle]
    z_sq = radius**2 - x_flat**2 - y_flat**2
    z_flat = np.sqrt(z_sq)

    # THIS ONLY SOLVES FOR ONE HALF
    coords = np.vstack((np.concatenate([x_flat, x_flat]),
                        np.concatenate([y_flat, y_flat]),
                        np.concatenate([z_flat, -z_flat])))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(coords.T)
    return pcd


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Calculate Chamfer distance between two point clouds.")
    parser.add_argument('gt_pcd', type=str, help="Path to the ground truth point cloud file.")
    parser.add_argument('pred_pcd', type=str, help="Path to the predicted point cloud file.")
    args = parser.parse_args()

    gt_pcd = o3d.io.read_point_cloud(args.gt_pcd)
    pred_pcd = o3d.io.read_point_cloud(args.pred_pcd)

    #### TEST ####
    # pc1 = generate_sphere_given_radius(100, 1)
    # pc2 = generate_sphere_given_radius(90, 0.9)
    # o3d.visualization.draw_geometries([pc1, pc2], window_name="Ground Truth Point Cloud")

    one_way_schamfer_dist = get_pred2gt_schamfer_dist(gt_pcd, pred_pcd)
    
    print(f"One way schamfer distance from pred to gt: {one_way_schamfer_dist}")
