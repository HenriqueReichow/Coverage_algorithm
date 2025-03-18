import holoocean.command
import open3d as o3d #mgmatheus
import numpy as np
import networkx as nx
import holoocean
import sys
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
import os
import json
sys.path.append('/home/lh/Desktop/HoloOceanUtils')
import HoloOceanVehicles
import HoloOceanScenarios
import HoloOceanSensors
#https://github.com/byu-holoocean/HoloOcean/blob/UE5.3_Prerelease/client/src/holoocean/sensors.py
#markov decision process

class Utils:
    def __init__(self):
        self.azi = 40 #sonar_config['Azimuth']
        self.minR = 0 #sonar_config['RangeMin']
        self.maxR = 3 #sonar_config['RangeMax']
        self.binsR = 512 #sonar_config['RangeBins']
        self.binsA = 120 #sonar_config['AzimuthBins']

    #def visualize_image_sonar(self): #Cria uma imagem que se atualiza durante a simulação mostrando a visão do sonar
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8, 5))
        self.ax.set_theta_zero_location("N")

        self.ax.set_thetamin(-self.azi / 2)
        self.ax.set_thetamax(self.azi / 2)

        theta = np.linspace(-self.azi / 2, self.azi / 2, self.binsA) * np.pi / 180
        r = np.linspace(self.minR, self.maxR, self.binsR)
        T, R = np.meshgrid(theta, r)
        z = np.zeros_like(T)
            
        plt.grid(False)
        self.plot = self.ax.pcolormesh(T, R, z, cmap='gray', shading='auto', vmin=0, vmax=1)
        plt.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def update_sonar_image(self, state): #Atualiza a imagem do campo de visão do sonar a cada step
            s = state['auv0']['ImagingSonar']
            self.plot.set_array(s.ravel())
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

def load_mesh_and_sample_points(mesh_path, voxel_size=0.4):
    #mesh para pointclouds 
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pcd = mesh.sample_points_uniformly(number_of_points=50)
    #pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(pcd.points)

############## metodos de inspecao
def scaled_inspection_points(points, scale_factor=3.6):
    center = np.mean(points, axis=0) 
    expanded_points = center + (points - center) * scale_factor  # Expande para fora
    return expanded_points

def fix_inspection_points(inspection_points): #tanque
    inspection_points = np.asarray(inspection_points)
    x_min,x_max = -2.5, 2.5
    y_min,y_max = -2.5, 2.5
    z_min,z_max = -4.5, -0.2
    mask = (
        (inspection_points[:, 0] >= x_min) & (inspection_points[:, 0] <= x_max) &
        (inspection_points[:, 1] >= y_min) & (inspection_points[:, 1] <= y_max) &
        (inspection_points[:, 2] >= z_min) & (inspection_points[:, 2] <= z_max))
    return inspection_points[mask]
############## fim metodos de inspecao

def pose_sensor_update(state):  # Atualiza a rotação e posição do ROV
    pose = state['auv0']['PoseSensor']
    rotation_matrix = pose[:3, :3]
    rotation = Rot.from_matrix(rotation_matrix)
    actual_rotation = rotation.as_euler('xyz', degrees=True)
    actual_location = pose[:3, 3]
    return actual_location, actual_rotation, rotation_matrix

def calculate_orientation(trajectory, center):
    list_of_orientations = []
    yaw_inicial = 0

    for i,point in enumerate(trajectory):
        x,y,_ = point
        yaw_desejado = np.degrees(np.arctan2(center[1] - y, center[0] - x))

        delta_z = trajectory[i][2] - center[2]
        pitch_desejado = np.degrees(np.arcsin(delta_z / np.linalg.norm(trajectory[i] - center)))

        if pitch_desejado > 180:
            pitch_desejado -= 360
        elif pitch_desejado < -180:
            pitch_desejado += 360

        heading = yaw_desejado - yaw_inicial
        if heading > 180:
                heading -= 360
        elif heading < -180:
                heading += 360
        list_of_orientations.append([0,pitch_desejado,heading])
    
    return list_of_orientations

def is_visible(q, s, sensor_range=10.0):
    return np.linalg.norm(q - s) <= sensor_range

def plot_tour(structure_points,inspection_points,trajectory):
    figure = plt.figure()
    ax = figure.add_subplot(111,projection="3d")
    ax.scatter(structure_points[:, 0], structure_points[:, 1], structure_points[:, 2], c='blue', marker='o', s=2, label='Objeto')

    ax.scatter(inspection_points[:, 0], inspection_points[:, 1], inspection_points[:, 2], c='red', marker='.', label='Inspeção')
    trajectory = np.array(trajectory)
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], c='green', label='Trajetória')
   
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()

def point_in_obj(point, structure_points, error=2):
    xmax = np.max(structure_points[:,0]) + error
    xmin = np.min(structure_points[:,0]) - error
    ymax = np.max(structure_points[:,1]) + error
    ymin = np.min(structure_points[:,1]) - error
    zmax = np.max(structure_points[:,2]) + error
    zmin = np.min(structure_points[:,2]) - error
    if (point[0] <= xmax and point[0] >= xmin) and (point[1] <= ymax and point[1] >= ymin) and (point[2] <= zmax and point[2] >= zmin):
        return True
    else:
        return False

def segment_intersects_obj(p1, p2, structure_points, error=2):
    p1 = np.array(p1)
    p2 = np.array(p2)
    for i in np.linspace(0, 1, num=1000):
        interpolated_point = (1 - i) * np.array(p1) + i * np.array(p2)
        if point_in_obj(interpolated_point, structure_points, error):
            return True
    return False

def check_collision(trajectory , structure_points, error=2):
    #verifica se os pontos da trajectory estao dentro do objeto
    #verifica se o deslocamento entre p1 e p2 intercepta o objeto
    #retorna uma nova trajetoria sem colisoes
    new_trajectory = []
    for i,point in enumerate(trajectory):
        if i < len(trajectory)-1:
            if not point_in_obj(point, structure_points, error): 
                if not segment_intersects_obj(point, trajectory[i+1], structure_points, error=2):
                    new_trajectory.append(point)

    return np.asarray(new_trajectory)        

def build_inspection_graph(inspection_points, object_points, sensor_range=40.0):
    graph = nx.Graph()
    for i, ponto in enumerate(inspection_points):
        graph.add_node(i, config=ponto)
        for j, obj in enumerate(object_points):
            if is_visible(ponto, obj, sensor_range) and segment_intersects_obj(ponto, obj, object_points):
                dist = np.linalg.norm(np.array(ponto) - np.array(obj))
                graph.add_edge(i, j, weight=dist)  #Adiciona um peso com base na distância
    return graph

def complete_graph(graph, structure_points):
    nodes = list(graph.nodes)
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if not graph.has_edge(nodes[i], nodes[j]) and not segment_intersects_obj(nodes[i], nodes[j], structure_points):
                p1 = np.array(graph.nodes[nodes[i]]["config"])
                p2 = np.array(graph.nodes[nodes[j]]["config"])

                # Verificar se p1 e p2 têm 3 dimensões
                if p1.shape != (3,) or p2.shape != (3,):
                    raise ValueError(f"Os pontos nos nós {nodes[i]} e {nodes[j]} não têm 3 dimensões.")
                
                dist = np.linalg.norm(p1 - p2)
                graph.add_edge(nodes[i], nodes[j], weight=dist)
    return graph


def plan_trajectory(graph, structure_points):
    graph = complete_graph(graph, structure_points)
    path = list(nx.approximation.threshold_accepting_tsp(graph, "greedy", weight="weight"))
    return [graph.nodes[n]["config"] for n in path]

def main():
    mesh = "/home/lh/Documents/Ground-Truth/stl/Wedge_A.stl"
    structure_points = load_mesh_and_sample_points(mesh) 
    
    ############################# - ajustar o objeto no world
    angle = np.radians(90)
    structure_points = np.dot(structure_points,[[np.cos(angle),-np.sin(angle),0],[np.sin(angle), np.cos(angle), 0], [0,0,1]])
    for points in structure_points: #temporario define a posicao da mesh no world
         points[2] = points[2] - 2.5
    centro = [0, 0,-2]
    ############################# - ajustar o objeto no world

    #calculo da trajetoria do Rov
    inspection_points = scaled_inspection_points(structure_points)
    inspection_points = fix_inspection_points(inspection_points) #manter os pontos dentro do tanque
    graph = build_inspection_graph(inspection_points, structure_points)
    trajectory = plan_trajectory(graph, structure_points)
    orientations = calculate_orientation(trajectory, centro)

    #iniciar world holoocean
    waypoints=np.concatenate((trajectory,orientations),axis=1)
    scenario = HoloOceanScenarios.Scenario("test_rgb_camera" ,"64-tank-Map-1", "Dataset",200)
    auv = HoloOceanVehicles.AUV(id='0',control_scheme=1,location=list(trajectory[0]),rotation=[0,0,0],mission=0,waypoints=waypoints,sonar_model='test4')
    auv.addSonarImaging(configuration= {
                        "RangeBins": 256,
                        "AzimuthBins": 96*2,
                        "RangeMin": 0,
                        "RangeMax": 4,
                        "InitOctreeRange": 50,
                        "Elevation": 10,
                        "Azimuth": 28.8*2,
                        "AzimuthStreaks": -1,
                        "ScaleNoise": True,
                        "AddSigma": 0.15,
                        "MultSigma": 0.2,
                        "RangeSigma": 0.0,
                        "MultiPath": True,
						"ViewOctree": -1,
                        "ViewRegion": True
                    })
                        
                        # {
                        #         "RangeBins": 256,
                        #         "AzimuthBins": 256,
                        #         "RangeMin": 0,
                        #         "RangeMax": 5,
                        #         "InitOctreeRange": 50,
                        #         "Elevation": 10,
                        #         "Azimuth": 60,
                        #         "AzimuthStreaks": -1,
                        #         "ScaleNoise": True,
                        #         "AddSigma": 0.15,
                        #         "MultSigma": 0.2,
                        #         "RangeSigma": 0.1,
                        #         "MultiPath": True,
                                
                        #     })


    auv.addSensor("PoseSensor","SonarSocket")
    auv.addSensor('LocationSensor',"Origin")
    auv.addSensor('RotationSensor',"Origin")
    auv.addSensor('CollisionSensor','Origin')
    auv.imageViwer()
    scenario.addAgent(auv.agent)

    env = holoocean.make(scenario_cfg=scenario.cfg,verbose=False)
    state = env.tick()

    for loc in trajectory: #desenhar os pontos no world
        env.draw_point(list(loc),lifetime=0)

    env.draw_point(centro,color=[0,255,0],lifetime=0) #centro
    #plot_tour(structure_points,inspection_points,trajectory) #plotar o grafo com matplotlib

    i = 0
    #auv.updateState(state)

    #auv.saveState(state)
    while True: #simulação

        env.act('auv0',np.concatenate((trajectory[i],orientations[i]),axis=None))
        state = env.tick()
        if 'ImagingSonar' in state[auv.name]:
            auv.updateState(state)
            
        #auv.updateState(state)
        if state[auv.name]["CollisionSensor"]: 
            print('colisao') #funciona
        
        actual_location, actual_rotation, rotation_matrix = pose_sensor_update(state)

        if np.linalg.norm(actual_location-trajectory[i]) < 0.5 and np.linalg.norm(actual_rotation[2] - orientations[i][2]) < 5 and orientations[i][0] < 0.3 and 'ImagingSonar' in state[auv.name]:
            #auv.updateState(state)
            print(i, np.concatenate((trajectory[i],orientations[i]),axis=None))
            i += 1

        if i + 1 > len(trajectory):
            print("Fim da missão!")
            os.system("killall -e Holodeck")
            break

if __name__ == "__main__":
    main()

#testar tudo e deixar funcional 100%
#mudar de cor quando ele passar por um waypoint

# potential fields
# a estrela e dijkstra
# quaternions para pitch

##garantir o hov sempre olhando o objeto
##gerar imagens dos dados de sonar
##treinar uma rede que me ajude a tratar as imagens e gerar as pointclouds ##= prioridade

# salvar os states
# salvar no formato do dataset do bomba