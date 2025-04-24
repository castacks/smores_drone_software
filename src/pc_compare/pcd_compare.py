import numpy as np
import open3d as o3d
# from numba import njit
# import torch
import argparse

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
    """
    
    downsampled_gt_pcd = gt_pcd.voxel_down_sample(voxel_size=voxel_downsample)
    downsampled_pred_pcd = pred_pcd.voxel_down_sample(voxel_size=voxel_downsample)
    
    num_pred_points = len(downsampled_pred_pcd.points)
    num_gt_points = len(downsampled_gt_pcd.points)
    
    gt_points = np.asarray(downsampled_gt_pcd.points).astype(np.float32)      # (G, 3)
    gt_points = np.expand_dims(np.asarray(downsampled_gt_pcd.points), axis=1) # (G, 1, 3)
    
    pred_points = np.asarray(downsampled_pred_pcd.points).astype(np.float32)                          # (P, 3)
    pred_points = np.expand_dims(np.asarray(downsampled_pred_pcd.points), axis=1).transpose(1, 0, 2)  # (1, P, 3)

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

# def chamfer_loss(point_cloud_src: torch.Tensor, point_cloud_tgt: torch.Tensor):
# 	# point_cloud_src, point_cloud_src: b x n_points x 3  
# 	# loss_chamfer = 
# 	# implement chamfer loss from scratch
# 	#NOTE: For each point in set 1, we want to find the closest point in set 2, take L2 Loss, and vice versa. Should we use the whole pc1 and pc2 as set1 and set2
# 	B, N, _ = point_cloud_src.shape
# 	_, M, _ = point_cloud_tgt.shape
# 	pc_src_expanded = point_cloud_src.unsqueeze(2).expand(-1, -1, M, -1) # B x N x M x 3 (where M x 3 is x,y,z repeated M times)
# 	pc_tgt_expanded = point_cloud_tgt.unsqueeze(1).expand(-1, N, -1, -1) # B x N x M x 3 (where N x M x 3 is M x 3 repeated N times)

# 	distance = torch.sqrt(torch.sum((pc_tgt_expanded - pc_src_expanded)**2, dim=-1)) # Sum squared differences alng last dimension (x,y,z) -> B x N x M
# 	# Loss part 1 -> Take min over M to get the point from target with minimmum distance to the repeated point in src
# 	# Then sum over all the points in src, hence along dimension N
# 	# loss_pt1 = torch.sum(torch.min(distance, dim=2).values, dim=1) # After min -> (B x N), after sum across N, (B,)
# 	loss_pt2 = torch.sum(torch.min(distance, dim=1).values, dim=1) # After min -> (B x M), after sum across M, (B,)
	
# 	loss_chamfer = loss_pt1 + loss_pt2
# 	loss_chamfer = loss_chamfer.mean() # Across batch size B
# 	return loss_chamfer
    
    
def visualize_pointcloud_offscreen(pcd_path, output_image="output.png"):
    """
    Visualizes a point cloud in headless mode and saves the result as an image.

    Args:
        pcd_path (str): Path to the point cloud file.
        output_image (str): Path to save the rendered image.
    """
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)

    # Create a visualizer for offscreen rendering
    vis = o3d.visualization.O3DVisualizer("Offscreen Point Cloud Visualization", 800, 600)
    vis.show_settings = True
    vis.add_geometry("PointCloud", pcd)

    # Render and save the image
    vis.capture_screen_image(output_image)
    print(f"Point cloud visualization saved to {output_image}")
    


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Calculate Chamfer distance between two point clouds.")
    parser.add_argument('gt_pcd', type=str, help="Path to the ground truth point cloud file.")
    parser.add_argument('pred_pcd', type=str, help="Path to the predicted point cloud file.")
    args = parser.parse_args()

    gt_pcd = o3d.io.read_point_cloud(args.gt_pcd)
    pred_pcd = o3d.io.read_point_cloud(args.pred_pcd)
    
    # o3d.visualization.draw_geometries([gt_pcd, pred_pcd], window_name="Ground Truth Point Cloud")
    # o3d.visualization.draw_geometries([], window_name="Ground Truth Point Cloud")
    
    # gt_tensor = torch.tensor(gt_pcd.points).unsqueeze(0)
    # pred_tensor = torch.tensor(pred_pcd.points).unsqueeze(0)
    
    # chamfer_loss(gt_tensor, pred_tensor)
    one_way_schamfer_dist = get_pred2gt_schamfer_dist(gt_pcd, pred_pcd, voxel_downsample=0.05)
    
    print(f"One way schamfer distance from pred to gt: {one_way_schamfer_dist}")
