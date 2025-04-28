import open3d as o3d #mgmatheus
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import os, csv, pickle

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

def spawn_rangefinders(config_of_sonar:dict, auv)->None:

    for i in range(config_of_sonar["Elevation"]): #spawn rangefinder
        angle_elev = np.linspace(-config_of_sonar["Elevation"]/2,config_of_sonar["Elevation"]/2, config_of_sonar["Elevation"])
        angle_azi = np.linspace(-config_of_sonar["Azimuth"]/2,config_of_sonar["Azimuth"]/2, config_of_sonar["AzimuthBins"])
        for j in range(config_of_sonar["AzimuthBins"]):
            auv.agent['sensors'].append({   "sensor_name":f"RangeFinderSensor_{i}_{j}",
                                            "sensor_type":"RangeFinderSensor",
                                            "socket": "SonarSocket",
                                            "rotation":[0,0,angle_azi[j]],
                                            "Hz": 20,
                                            "configuration":{
                                                "LaserMaxDistance" : 10,
                                                "LaserCount": 1,
                                                "LaserAngle": angle_elev[i],
                                                "LaserDebug": True}})
            
def update_waypoints(env, waypoints, visited, next_idx, structure_points)->None:

    for i, wp in enumerate(waypoints):
        if i in visited:
            env.draw_point(list(wp),color= [0, 255, 0], lifetime=2)  # Verde para visitados
        elif i == next_idx:
            env.draw_point(list(wp),color= [0, 0, 255], lifetime=2)  # Azul para o próximo destino
        else:
            env.draw_point(list(wp), color=[255, 0, 0], lifetime=2)  # vermelho para waypoints normais

    for insp in structure_points:
        env.draw_point(list(insp), [0,50,255])

def pose_sensor_update(state, auv):  # Atualiza a rotação e posição do ROV
    pose = state[auv.name]['PoseSensor']
    rotation_matrix = pose[:3, :3]
    rotation = Rot.from_matrix(rotation_matrix)
    actual_rotation = rotation.as_euler('xyz', degrees=True)
    actual_location = pose[:3, 3]
    return actual_location, actual_rotation, rotation_matrix

def save_data_rangefinder(pickle_dir, config):
    for i,pickle_file in enumerate(pickle_dir):
        if pickle_file.endswith(".pkl"):
            if not os.path.exists("/home/lh/Desktop/Coverage_algorithm/states_npy"):
                os.mkdir("/home/lh/Desktop/Coverage_algorithm/states_npy")
            convert_pickle_to_npy(pickle_file, f"/home/lh/Desktop/Coverage_algorithm/states_npy/{i}.npy", config=config)

def translate_rangefinder_data(state, auv, config, structure_points, env, lifetime=10):
    loc, rot, _ = pose_sensor_update(state, auv)

    range_matrix = np.zeros((config["Elevation"], config["AzimuthBins"]))

    for i in range(config["Elevation"]):
        for j in range(config["AzimuthBins"]):
            key = f"RangeFinderSensor_{i}_{j}"
            if key in state[auv.name]:
                range_matrix[i, j] = state[auv.name][key][0]

    elevacoes = np.radians(np.linspace(-config["Elevation"]/2, config["Elevation"]/2, config["Elevation"]))
    azimutes = np.radians(np.linspace((-config["Azimuth"]/2)+rot[2], (config["Azimuth"]/2)+rot[2], config["AzimuthBins"]))

    ele, azi = np.meshgrid(elevacoes, azimutes, indexing="ij")

    x = range_matrix * np.cos(ele) * np.cos(azi) + loc[0]
    y = range_matrix * np.cos(ele) * np.sin(azi) + loc[1]
    z = range_matrix * np.sin(ele) + loc[2]

    mask = range_matrix < 9
    coords = np.column_stack((x[mask], y[mask], z[mask]))

    for coord in coords:
        env.draw_point(list(coord),color=[100,100,100],lifetime=lifetime)
    index = []
    for i, point in enumerate(structure_points):
        for coord in coords:
            if np.linalg.norm(point - coord) <= 0.2:
                index.append(i)

    structure_points = np.delete(structure_points, sorted(index, reverse=True), axis=0)

    if len(structure_points) == 0:
        print("O objeto foi totalmente coberto")
    else:
        print(f"Faltaram {len(structure_points)} pontos para cobertura total!")

    return structure_points

def convert_pickle_to_npy(pickle_file, save_path, config):
    with open(pickle_file, "rb") as file:
        state = pickle.load(file)

    range_matrix = np.zeros((config["Elevation"], config["AzimuthBins"]))

    for i in range(config["Elevation"]):
        for j in range(config["AzimuthBins"]):
            if f"RangeFinderSensor_{i}_{j}" in state:
                print("state ",state[f"RangeFinderSensor_{i}_{j}"][0])
                range_matrix[i, j] = state[f"RangeFinderSensor_{i}_{j}"][0]

    np.save(save_path, range_matrix)
    print(f"Arquivo salvo em: {save_path}")