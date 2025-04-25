import holoocean.agents
import holoocean.command
import open3d as o3d #mgmatheus
import numpy as np
import networkx as nx
import holoocean
from utils import MeshProcessor
import route
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
import os, sys, json, csv, pickle
sys.path.append('/home/lh/Desktop/HoloOceanUtils')
import HoloOceanVehicles
import HoloOceanScenarios
import HoloOceanSensors
#import rangefinder_pkl_to_npy as pkl_to_npy
#https://github.com/byu-holoocean/HoloOcean/blob/UE5.3_Prerelease/client/src/holoocean/sensors.py
#markov decision process

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
  
def main():
        #for k in range(40):
        obj_id = 0
        structure_points = MeshProcessor(obj_id).config_obj_in_world()

        traj = route.TrajectoryPlanner(structure_points) #planejar a trajetoria
        waypoints, trajectory = traj.plan_waypoints(plot_tour=True,obj_id=obj_id)
        centro = traj.center

        #iniciar world holoocean .json
        scenario = HoloOceanScenarios.Scenario("__" ,"64-tank-Map-1", "Dataset", 200)

        #auv = HoloOceanVehicles.AUV(id='0',control_scheme=1,location=list(trajectory[0]),rotation=[0,0,0],mission=1,waypoints=waypoints,sonar_model='obj'+str(obj_id))
        auv = HoloOceanVehicles.SphereAgent(id='1',control_scheme=0,location=list(trajectory[0]),rotation=list(waypoints[0,3:]),mission=1,waypoints=waypoints,sonar_model='obj'+str(obj_id))
        config={
                    "RangeBins": 256,
                    "AzimuthBins": 192,
                    "RangeMin": 0,
                    "RangeMax": 5,
                    "InitOctreeRange": 50,
                    "Elevation": 20,
                    "Azimuth": 60,
                    "AzimuthStreaks": -1,
                    "ScaleNoise": True,
                    "AddSigma": 0.15,
                    "MultSigma": 0.2,
                    "RangeSigma": 0.0,
                    "MultiPath": True,
                    "ViewOctree": -1,
                    "ViewRegion": True}
        
        #spawn_rangefinders(config, auv)
        auv.addSonarImaging(configuration = config)
        auv.imageViwer()
        auv.addSensor("PoseSensor","SonarSocket")
        auv.addSensor('LocationSensor',"SonarSocket")
        auv.addSensor('RotationSensor',"SonarSocket")
        auv.addSensor('CollisionSensor','Origin')
        scenario.addAgent(auv.agent)

        #iniciar a simulacao 
        env = holoocean.make(scenario_cfg=scenario.cfg,verbose=False)
        state = env.tick()
        env.draw_point(list(centro),color=[0,255,255],lifetime=0) #centro
        #env.move_viewport(waypoints[0,:3],waypoints[0,3:])

        visited= [] 
        next_idx = 0 
        coord_temp = structure_points

        while True:
            while next_idx < len(waypoints):
                state = env.tick()
                if "ImagingSonar" in state[auv.name]: #and 'RangeFinderSensor_0_0' in state[auv.name]: 
                    # if state[auv.name]["CollisionSensor"]: #testa a colisao e caso afirmativo encerra a simulacao
                    #     print("Colisao detectada, cancelando missão...")
                    #     os.system("killall -e Holodeck")
                    #     break

                    update_waypoints(env,trajectory,visited,next_idx,coord_temp)
                    auv.updateState(state)

                    coord_temp = translate_rangefinder_data(state,auv,config,coord_temp,env) 
                    traj.saveState(auv, state, obj_id) #deixar sempre ativo - salva em pkl
                    print(next_idx, waypoints[next_idx])

                    env.agents[auv.name].teleport(waypoints[next_idx,:3], waypoints[next_idx,3:])
                    visited.append(next_idx)
                    next_idx += 1
            break
        
        # while True: #simulação
        #     update_waypoints(env, trajectory, visited, next_idx, coord_temp)
        #     env.act(auv.name,waypoints[next_idx])
        #     state = env.tick()
        #     if state[auv.name]["CollisionSensor"]: #testa a colisao e caso afirmativo encerra a simulacao
        #         print("Colisao detectada, cancelando missão...")
        #         os.system("killall -e Holodeck")
        #         break
        #     actual_location, actual_rotation, _ = pose_sensor_update(state,auv)
            

        #     if np.linalg.norm(actual_location-trajectory[next_idx]) < 0.5 and np.linalg.norm(actual_rotation[2] - waypoints[next_idx][5]) < 7 and waypoints[next_idx][3] < 3 and "RangeFinderSensor_0_0" in state[auv.name]: #and 'ImagingSonar' in state[auv.name]:
        #         coord_temp = translate_rangefinder_data(state,auv,config,coord_temp,env)
        #         traj.saveState(auv, state, obj_id) #deixar sempre ativo - salva em pkl
        #         print(next_idx, waypoints[next_idx])
        #         #auv.updateState(state)
        #         visited.append(next_idx)
        #         next_idx += 1

        #     if next_idx + 1 >= len(trajectory): #fim da missao
        #         print("Fim da missão!")
        #         os.system("killall -e Holodeck")
        #         break

if __name__ == "__main__":
    main()


# potential fields
# a estrela e dijkstra
# quaternions para pitch

##garantir o hov sempre olhando o objeto
##gerar imagens dos dados de sonar
##treinar uma rede que me ajude a tratar as imagens e gerar as pointclouds ##= prioridade

# salvar os states
# salvar no formato do dataset do bomba

# ler todas as 3000 casas e criar csv com os pontos junto com as meshs 
# diretorio e ponto

#unet e aprender spbre ia e treinamento (aulas do guerra)
#mexer com o pitch e coisas relacionadas
# spawnar o rov em cada ponto dos waypoints e pegar os dados