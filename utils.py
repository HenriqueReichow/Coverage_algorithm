import open3d as o3d #mgmatheus
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import os, csv

class MeshProcessor:
    def __init__(self, obj_id, csv_path="/home/lh/Desktop/Coverage_algorithm/obj_locations/mission1.csv", ground_truth_path="/home/lh/Documents/Ground-Truth/stl", save_pcd:bool=False, number_of_points:int=100):
        self.csv_path = csv_path
        self.obj_id = obj_id
        self.ground_truth_path = ground_truth_path
        self.save_pcd = save_pcd
        self.number_of_points = number_of_points

    def locate_mesh(self)->None:
        with open(self.csv_path,"r") as arquivo:
            arquivo_csv = csv.DictReader(arquivo, delimiter=",")
            for line in arquivo_csv:
                if line["ID"] == str(self.obj_id):
                    name = line["Name"]
                    break
            self.mesh_path = self.ground_truth_path+"/"+name+".stl"      
    
    def load_mesh_and_sample_points(self):
        #mesh para pointclouds 
        self.locate_mesh()
        mesh = o3d.io.read_triangle_mesh(self.mesh_path)
        pcd = mesh.sample_points_uniformly(number_of_points=self.number_of_points)
        if self.save_pcd:
            o3d.io.write_point_cloud(os.path.splitext(self.mesh_path)[0] + ".xyz",pcd)
        return np.asarray(pcd.points)
    
    def config_obj_in_world(self):
        structure_points = self.load_mesh_and_sample_points()
        with open(self.csv_path, "r") as arquivo:
            arquivo_csv = csv.DictReader(arquivo, delimiter=",")
            for linha in arquivo_csv:
                #print("linha=",linha)
                if linha["ID"] == str(self.obj_id):
                    #print(f"linha={linha["ID"]} e obj_id={str(obj_id)}")
                    x = float(linha["X"])
                    y = -float(linha["Y"])
                    z = float(linha["Z"])
                    h = float(linha["H"])
                    break

            angle = np.radians(90)
            rotation_matrix = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle),  np.cos(angle), 0],
                [0,              0,             1]
            ])
            structure_points = np.dot(structure_points, rotation_matrix)

            structure_points[:, 0] += x
            structure_points[:, 1] += y
            structure_points[:, 2] += z

            return structure_points