import numpy as np
import cupy as cp  # Import CuPy
from scipy.spatial import cKDTree
import open3d as o3d
import os
def load_xyz(filename, use_gpu=False):
    """Loads a point cloud from an XYZ file.

    Args:
        filename (str): Path to the XYZ file.
        use_gpu (bool): If True, loads the data onto the GPU.

    Returns:
        cupy.ndarray or numpy.ndarray: Nx3 array representing the point cloud, or None if an error occurs.
    """
    try:
        data = np.loadtxt(filename)
        if use_gpu:
            return cp.asarray(data)  # Move data to GPU
        else:
            return data
    except FileNotFoundError:
        print(f"Error: File not found: {filename}")
        return None
    except Exception as e:
        print(f"An error occurred while loading the file: {e}")
        return None

def hausdorff_distance_gpu(point_cloud1, point_cloud2):
    """Calculates the mean and RMS Hausdorff distance between two point clouds using GPU.

    Args:
        point_cloud1 (cupy.ndarray): Nx3 array representing the first point cloud (on GPU).
        point_cloud2 (cupy.ndarray): Mx3 array representing the second point cloud (on GPU).

    Returns:
        tuple: (mean_hausdorff, rms_hausdorff)
    """
    if point_cloud1 is None or point_cloud2 is None:
        return None, None

    # Calculate distances from point_cloud1 to point_cloud2
    distances1_to_2 = cp.linalg.norm(point_cloud1[:, None, :] - point_cloud2[None, :, :], axis=2).min(axis=1)

    # Calculate distances from point_cloud2 to point_cloud1
    distances2_to_1 = cp.linalg.norm(point_cloud2[:, None, :] - point_cloud1[None, :, :], axis=2).min(axis=1)

    mean_hausdorff = (cp.mean(distances1_to_2) + cp.mean(distances2_to_1)) / 2.0
    rms_hausdorff = cp.sqrt((cp.mean(distances1_to_2**2) + cp.mean(distances2_to_1**2)) / 2.0)

    return cp.asnumpy(mean_hausdorff), cp.asnumpy(rms_hausdorff) #bring the results back to the cpu.

def hausdorff_distance_cpu(point_cloud1, point_cloud2):
    """Calculates the mean and RMS Hausdorff distance between two point clouds using CPU.

    Args:
        point_cloud1 (numpy.ndarray): Nx3 array representing the first point cloud (on CPU).
        point_cloud2 (numpy.ndarray): Mx3 array representing the second point cloud (on CPU).

    Returns:
        tuple: (mean_hausdorff, rms_hausdorff)
    """
    if point_cloud1 is None or point_cloud2 is None:
        return None, None

    tree1 = cKDTree(point_cloud1)
    tree2 = cKDTree(point_cloud2)

    distances1_to_2, _ = tree2.query(point_cloud1)
    distances2_to_1, _ = tree1.query(point_cloud2)

    mean_hausdorff = (np.mean(distances1_to_2) + np.mean(distances2_to_1)) / 2.0
    rms_hausdorff = np.sqrt((np.mean(distances1_to_2**2) + np.mean(distances2_to_1**2)) / 2.0)

    return mean_hausdorff, rms_hausdorff

def hausdorff_distance_gpu_chunked(point_cloud1, point_cloud2, chunk_size=1000):
    distances1_to_2_all = []
    for i in range(0, point_cloud1.shape[0], chunk_size):
        chunk1 = point_cloud1[i:i + chunk_size]
        distances1_to_2_chunk = cp.linalg.norm(chunk1[:, None, :] - point_cloud2[None, :, :], axis=2).min(axis=1)
        distances1_to_2_all.append(distances1_to_2_chunk)
    distances1_to_2 = cp.concatenate(distances1_to_2_all)

    distances2_to_1_all = []
    for i in range(0, point_cloud2.shape[0], chunk_size):
        chunk2 = point_cloud2[i:i + chunk_size]
        distances2_to_1_chunk = cp.linalg.norm(chunk2[:, None, :] - point_cloud1[None, :, :], axis=2).min(axis=1)
        distances2_to_1_all.append(distances2_to_1_chunk)
    distances2_to_1 = cp.concatenate(distances2_to_1_all)

    mean_hausdorff = (cp.mean(distances1_to_2) + cp.mean(distances2_to_1)) / 2.0
    rms_hausdorff = cp.sqrt((cp.mean(distances1_to_2**2) + cp.mean(distances2_to_1**2)) / 2.0)

    return cp.asnumpy(mean_hausdorff), cp.asnumpy(rms_hausdorff)

def filter_and_save(pcd_path, nb_points=100, radius=0.05):
    pcd = o3d.io.read_point_cloud(pcd_path)
    pcd_filtered, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    new_path = os.path.splitext(pcd_path)[0] + "_filtered.xyz"
    o3d.io.write_point_cloud(new_path, pcd_filtered)
    return new_path

if __name__ == "__main__":
    # Replace with your actual XYZ file paths
    # missions=[1]
    # objs=[12]

    # for obj in objs:
    #     for  m in missions:
    m = 1
    obj = 0
    file1 = (f"/home/lh/Desktop/Coverage_algorithm/reconstructions/GT/GTpcd-1-obj0.xyz")
    file2 = (f"/home/lh/Desktop/Coverage_algorithm/reconstructions/classic/classic-1-auv-0.xyz")
    #file2 = "/home/lh/Desktop/Coverage_algorithm/1-auv-obj0-Cloud.ply"
    #file2 = filter_and_save(file2)
    point_cloud1 = load_xyz(file1,use_gpu=False)
    point_cloud2 = load_xyz(file2,use_gpu=False)

    if point_cloud1 is not None and point_cloud2 is not None:
        mean_h, rms_h = hausdorff_distance_cpu(point_cloud1, point_cloud2)
        print(f"Mean {m} {obj}: {mean_h}")
        print(f"RMS {m} {obj}: {rms_h}")