import numpy as np
from PIL import Image
import json
import tqdm
import csv
import open3d as o3d
#
def coord_translate(raio, azimuth, elev): 
        x = raio * np.cos(azimuth) * np.sin(elev)
        y = raio * np.sin(azimuth) * np.sin(elev)
        z = raio * np.cos(elev)
        return x,y,z

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

def matrix2xyz(matrix_path='/home/lh/Desktop/Coverage_algorithm/Sonar-Dataset-mission-1-obj0/1-sphere-0-data/Raw-data',meta_data="/home/lh/Desktop/Coverage_algorithm/Sonar-Dataset-mission-1-obj0/1-sphere-0-data/Meta-data"):
        all_coords = []
        i =12
        #for i in range(101):
        data = np.load(matrix_path + f"/{i}.npy")
        with open(meta_data + f'/{i}.json','r') as arquivo:
            data_json = json.load(arquivo)
        
        distances = np.linspace(data_json["sonar_range_min"], data_json["sonar_range_max"], len(data))
        angulos = np.radians(np.linspace(-data_json["sonar_azimuth"]/2 + data_json["yaw"], data_json["sonar_azimuth"]/2 + data_json["yaw"], len(data[:][0])))
        dist_vals,angul_vals = np.meshgrid(distances,angulos,indexing="ij")
        x,y,z = coord_translate(dist_vals, angul_vals, np.radians(90 - data_json["pitch"]))
                            
        mask = data > np.max(data) * 0.5 

        # if self.distance_filter:
        #     mask_distances = np.sqrt(x**2 + y**2 + z**2) < 3.5
        #     mask = mask_points & mask_distances
        # else:
        #     mask = mask_points

        coords = np.column_stack((x[mask] + data_json["x"], 
                                            y[mask] + data_json["y"],
                                            z[mask] + data_json["z"]))
        all_coords.append(coords)

        all_coords = np.vstack(all_coords)
        cloud = all_coords @ rotation(data_json["roll"], data_json["pitch"], data_json["yaw"])
                
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        o3d.io.write_point_cloud(f"/home/lh/Desktop/Coverage_algorithm/reconstructions/classic/classic-0-auv-000.xyz", pcd)


matrix2xyz()


# def matrix2xyz(matrix,output_xyz_filepath,index,config,auv_metadata_dir="/home/lh/Desktop/Coverage_algorithm/coverage-mission-1-data/Sonar-Dataset-mission-1-obj0/1-sphere-0-data/Meta-data/"):
#         # sonar_configuration = json.load(open('sonar-configuration.json'))
#         sonar_model=config
#         auv_metadata=json.load(open(auv_metadata_dir+f"{index}.json"))
#         radius, theta = matrix.shape
#         max_value = np.max(matrix)
#         #try:
#         with open(output_xyz_filepath, 'a') as outfile:
#                 cont=0
#                 for t in range(theta):
#                     for r in range(radius):
#                             #if matrix[r][t]>=max_value*0.01:
                        
#                             rad = (r*sonar_model["RangeMax"])/sonar_model["RangeBins"] + sonar_model["RangeMin"]
#                             phi_= -auv_metadata["pitch"]
#                             theta_= ((t*(sonar_model["Azimuth"])/theta)-sonar_model["Azimuth"]/2) + auv_metadata["yaw"]
                            
#                             x=(rad*np.cos(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["x"])
#                             y=(rad*np.sin(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["y"])
#                             z=(rad*np.sin(np.deg2rad(phi_))) + auv_metadata["z"]
#                             outfile.write(f"{x} {y} {z}\n")
#                             cont += 1
#                             print(cont)
#                             break
#                     break
#         #except:
#             #pass


# # with open(f"mission{m}.csv", newline='') as f:
# #     reader = csv.reader(f)
# #     mission_metadata = list(reader)
# #     mission_metadata.pop(0)

# # mission=mission_metadata[auv]
# output_xyz_filepath=(f"/home/lh/Desktop/Coverage_algorithm/reconstructions/classic/classic-0-auv-0.xyz")
# config={
#                     "RangeBins": 256,
#                     "AzimuthBins": 192,
#                     "RangeMin": 0,
#                     "RangeMax": 5,
#                     "InitOctreeRange": 10,
#                     "Elevation": 20,
#                     "Azimuth": 60,
#                     "AzimuthStreaks": -1,
#                     "ScaleNoise": True,
#                     "AddSigma": 0.15,
#                     "MultSigma": 0.2,
#                     "RangeSigma": 0.0,
#                     "MultiPath": True,
#                     "ViewOctree": -1,
#                     "ViewRegion": True,
#                     "ViewOctree": True}

# for i in tqdm.tqdm(range(101)):
#     #image_file_path = (f"/home/guilherme/Documents/Holoocean-imaging-sonar/experiments-elevatenet/{m}-{auv}/{i}.png")
#     matrix=np.load((f"coverage-mission-1-data/Sonar-Dataset-mission-1-obj0/1-sphere-0-data/Raw-data/{i}.npy"))
#     matrix2xyz(matrix=matrix,output_xyz_filepath=output_xyz_filepath,index=i,config=config)
# # Example usage:
# #image_file_path = "your_image.png"  # Replace with your image file path
# #grayscale_matrix = png_to_grayscale_numpy(image_file_path)

