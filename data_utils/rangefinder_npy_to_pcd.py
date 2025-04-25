import numpy as np
import open3d as o3d
import json
import os

def rotation(roll, pitch, yaw):
        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0], 
                            [-np.sin(pitch), 0, np.cos(pitch)]])

        R_roll = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R = (np.dot(R_yaw, np.dot(R_pitch, R_roll))).T
        return R

#files = os.listdir("/home/lh/Desktop/Coverage_algorithm/Sonar-Dataset-mission-1-obj1/1-auv-0-data/Meta-data")
files = os.listdir("/home/lh/Desktop/Coverage_algorithm/states/states_npy")
#all_coords = []
for i,file in enumerate(files):
    # with open(f"/home/lh/Desktop/Coverage_algorithm/Sonar-Dataset-mission-1-obj1/1-auv-0-data/Meta-data/{i}.json",'r') as arquivo:
    #     dados_json = json.load(arquivo)

    # azi = dados_json["sonar_azimuth"]
    # minR = dados_json["sonar_range_min"]
    # maxR = dados_json["sonar_range_max"]
    # sonar_pos_x = dados_json["x"]
    # sonar_pos_y = dados_json["y"]
    # sonar_pos_z = dados_json["z"]
    # pitch = dados_json["pitch"]
    # yaw = dados_json["yaw"]
    # roll = dados_json["roll"]

    r = np.load(f"/home/lh/Desktop/Coverage_algorithm/states/states_npy/{i}.npy")

    elevacoes = np.radians(np.linspace(-10, 10, 20))
    azimutes = np.radians(np.linspace(-30, 30, 192))

    ele, azi = np.meshgrid(elevacoes, azimutes, indexing="ij")

    x = r * np.cos(ele) * np.cos(azi) #+ sonar_pos_x
    y = r * np.cos(ele) * np.sin(azi) #+ sonar_pos_y
    z = r * np.sin(ele) #+ sonar_pos_z

    mask = r < 6

    coords = np.column_stack((x[mask], 
                              y[mask],
                              z[mask]))
                    
    #pointcloud = np.dot(coords, rotation(roll,pitch,yaw))
    #all_coords.append(pointcloud)
                
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(coords)
    o3d.io.write_point_cloud(f"/home/lh/Desktop/Coverage_algorithm/states/states_pcd/{i}.xyz", pcd)
    pcd0_load = o3d.io.read_point_cloud(f"/home/lh/Desktop/Coverage_algorithm/states/states_pcd/{i}.xyz")
    #o3d.visualization.draw_geometries([pcd0_load])

# all_coords = np.vstack(all_coords)
# cloud = all_coords @ rotation(roll, pitch, yaw)
            
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(cloud)
# o3d.io.write_point_cloud(f"/home/lh/Desktop/Coverage_algorithm/states_pcd/full.xyz", pcd)
# pcd_load = o3d.io.read_point_cloud(f"/home/lh/Desktop/Coverage_algorithm/states_pcd/full.xyz")

# o3d.visualization.draw_geometries([pcd_load])