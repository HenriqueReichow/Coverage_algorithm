# import holoocean.agents
# import holoocean.command
# import open3d as o3d #mgmatheus
# import numpy as np
import holoocean
import route, utils
import sys, os
sys.path.append('/home/lh/Desktop/HoloOceanUtils')
import HoloOceanVehicles
import HoloOceanScenarios
import HoloOceanSensors
#import rangefinder_pkl_to_npy as pkl_to_npy
#https://github.com/byu-holoocean/HoloOcean/blob/UE5.3_Prerelease/client/src/holoocean/sensors.py
#markov decision process

# def simulation_all_objects():
     
def main():
    for k in range(22,25):
        obj_id = k
        structure_points = utils.MeshProcessor(obj_id).config_obj_in_world()

        traj = route.TrajectoryPlanner(structure_points) #planejar a trajetoria
        waypoints, trajectory = traj.plan_waypoints(plot_tour=True,obj_id=obj_id)
        centro = traj.center

        #iniciar world holoocean .json
        scenario = HoloOceanScenarios.Scenario("__" ,"64-tank-Map-1", "DatasetSonar", 200)

        #auv = HoloOceanVehicles.AUV(id='0',control_scheme=1,location=list(trajectory[0]),rotation=[0,0,0],mission=1,waypoints=waypoints,sonar_model='obj'+str(obj_id))
        auv = HoloOceanVehicles.SphereAgent(id='0',control_scheme=0,location=list(trajectory[0]),rotation=list(waypoints[0,3:]),mission=1,waypoints=waypoints,sonar_model='obj'+str(obj_id))
        config={
                    "RangeBins": 256,
                    "AzimuthBins": 192,
                    "RangeMin": 0,
                    "RangeMax": 5,
                    "InitOctreeRange": 10,
                    "Elevation": 20,
                    "Azimuth": 60,
                    "AzimuthStreaks": -1,
                    "ScaleNoise": True,
                    "AddSigma": 0.15,
                    "MultSigma": 0.2,
                    "RangeSigma": 0.0,
                    "MultiPath": True,
                    "ViewOctree": -1,
                    "ViewRegion": True,
                    "ViewOctree": True}
        
        # utils.spawn_rangefinders(config, auv)
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
        env.move_viewport(waypoints[0,:3],waypoints[0,3:])

        visited= [] 
        next_idx = 0 
        coord_temp = structure_points

        while True:
            while next_idx < len(waypoints):
                state = env.tick()
                if "ImagingSonar" in state[auv.name]: #and 'RangeFinderSensor_0_0' in state[auv.name]: 
                    if state[auv.name]["CollisionSensor"]: #testa a colisao e caso afirmativo encerra a simulacao
                        print("Colisao detectada, cancelando missão...")
                        os.system("killall -e Holodeck")
                        break

                    utils.update_waypoints(env,trajectory,visited,next_idx,coord_temp)
                    auv.updateState(state)

                    coord_temp = utils.translate_rangefinder_data(state,auv,config,coord_temp,env) 
                    traj.saveState(auv, state, obj_id) #deixar sempre ativo - salva em pkl
                    print(next_idx, waypoints[next_idx])

                    env.agents[auv.name].teleport(waypoints[next_idx,:3], waypoints[next_idx,3:])
                    visited.append(next_idx)
                    next_idx += 1
            
            break

if __name__ == "__main__":
    main()




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