import numpy as np
import networkx as nx
import os, pickle
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    def __init__(self, mesh_points, scale_factor=3.4, sensor_range=4):
        self.mesh_points = np.asarray(mesh_points)
        self.graph = nx.Graph()
        self.sensor_range = sensor_range
        self.scale_factor = scale_factor
        self.center = np.mean(self.mesh_points, axis=0)
        self.inspection_points = self.center + (self.mesh_points - self.center) * self.scale_factor
        self.inspection_points = self.fix_inspection_points()
        self.counter = 0

    def point_in_obj(self, point, error=2.0):
        xmax = np.max(self.mesh_points[:,0]) + 2.5
        xmin = np.min(self.mesh_points[:,0]) - 2.5
        ymax = np.max(self.mesh_points[:,1]) + 2.5
        ymin = np.min(self.mesh_points[:,1]) - 2.5
        zmax = np.max(self.mesh_points[:,2]) + 2.5
        zmin = np.min(self.mesh_points[:,2]) - 2.5
        #print(point,"here")
        if (point[0] <= xmax and point[0] >= xmin) and (point[1] <= ymax and point[1] >= ymin) and (point[2] <= zmax and point[2] >= zmin):
            return True
        else:
            return False
    
    def segment_intersects_obj(self, p1, p2, error=2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        for i in np.linspace(0, 1, num=10000):
            interpolated_point = (1 - i) * np.array(p1) + i * np.array(p2)
            if self.point_in_obj(interpolated_point, error):
                return True
        return False
    
    def fix_inspection_points(self): #tanque #mission1 apenas
        self.inspection_points = np.asarray(self.inspection_points)
        x_min,x_max = self.center[0] -2.5, self.center[0] + 2.5
        y_min,y_max = self.center[1] -2.5, self.center[1] + 2.5
        z_min,z_max = -4.5, -0.2
        mask = (
            (self.inspection_points[:, 0] >= x_min) & (self.inspection_points[:, 0] <= x_max) &
            (self.inspection_points[:, 1] >= y_min) & (self.inspection_points[:, 1] <= y_max) &
            (self.inspection_points[:, 2] >= z_min) & (self.inspection_points[:, 2] <= z_max))
        return self.inspection_points[mask]

    def is_visible(self, q, s):
        return np.linalg.norm(q - s) <= self.sensor_range

    def build_inspection_graph(self):
        for i, ponto in enumerate(self.inspection_points):
            self.graph.add_node(i, config=ponto)
            for j, obj in enumerate(self.mesh_points):
                print(ponto[0])
                if self.is_visible(ponto, obj): #and self.segment_intersects_obj(ponto, obj, self.mesh_points):
                    dist = np.linalg.norm(np.array(ponto) - np.array(obj))
                    self.graph.add_edge(i, j, weight=dist)

    def complete_graph(self):
        nodes = list(self.graph.nodes)
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if not self.graph.has_edge(nodes[i], nodes[j]) and not self.segment_intersects_obj(nodes[i], nodes[j]):
                    p1 = np.array(self.graph.nodes[nodes[i]]["config"])
                    p2 = np.array(self.graph.nodes[nodes[j]]["config"])
                    dist = np.linalg.norm(p1 - p2)
                    self.graph.add_edge(nodes[i], nodes[j], weight=dist)

    def plan_trajectory(self):
        #usa o algoritmo tsp para planejar uma trajetoria entre os nodos do grafo
        path = list(nx.approximation.threshold_accepting_tsp(self.graph, "greedy", weight="weight"))
        self.trajectory = [self.graph.nodes[n]["config"] for n in path]
        return self.trajectory

    def calculate_orientation(self):
        #calcula o roll,pitch,yaw para cada waypoint
        list_of_orientations = []
        yaw_inicial = 0

        for i,point in enumerate(self.trajectory):
            x,y,_ = point
            yaw_desejado = np.degrees(np.arctan2(self.center[1] - y, self.center[0] - x))

            delta_z = self.trajectory[i][2] - self.center[2]
            pitch_desejado = np.degrees(np.arcsin(delta_z / np.linalg.norm(self.trajectory[i] - self.center)))

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
    
    def saveState(self, auv, state, obj_id)->None:
        if not os.path.exists("/home/lh/Desktop/Coverage_algorithm/states/states_pkl_"+str(obj_id)):
            os.mkdir("/home/lh/Desktop/Coverage_algorithm/states/states_pkl_"+str(obj_id))
    
        with open(f"/home/lh/Desktop/Coverage_algorithm/states/states_pkl_{str(obj_id)}/"+str(self.counter)+'.pkl', 'wb') as file:  
            pickle.dump(state[auv.name], file)
            self.counter+=1

    def plan_waypoints(self, plot_tour=False, obj_id=str):
        self.build_inspection_graph()
        self.complete_graph()
        self.plan_trajectory()
        orientations = self.calculate_orientation()
        way = np.concatenate((self.trajectory, orientations), axis=1)

        if not os.path.exists(os.getcwd() + f"/trajectories_data/obj{obj_id}" ):
            os.makedirs(os.getcwd() + f"/trajectories_data/obj{obj_id}")

        file_path = os.getcwd() + f"/trajectories_data/obj{obj_id}/" + str(obj_id) + "_waypoints.txt"

        if not os.path.exists(file_path):
            with open(file_path, "w") as arquivo:
                for waypoint in way:
                    arquivo.write(" ".join(map(str, waypoint)) + "\n")
            if plot_tour:
                self.plot_tour(self.mesh_points, self.inspection_points, self.trajectory )
            return np.asarray(way) , self.trajectory
        
        else:
            wayp = []
            with open(file_path, "r") as f:
                linhas = f.readlines()
                for line in linhas:
                    wayp.append([float(num) for num in line.strip().split()])

                if plot_tour:
                    wayp = np.asarray(wayp)
                    self.plot_tour(self.mesh_points, wayp[:, :3], wayp[:, :3])

            return np.asarray(wayp), np.asarray(wayp[:, :3])

    def plot_tour(self, structure_points, inspection_points, trajectory):
        #plota os inspection points e a trajetoria
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