import sys
import gc
import os
import holoocean
import numpy as np
import route, utils
import matplotlib.pyplot as plt
sys.path.append('/home/lh/Desktop/HoloOceanUtils')
import HoloOceanVehicles
import HoloOceanScenarios
import json

def run_simulation(k):
    obj_id = k
    mission = 1
    structure_points = utils.MeshProcessor(obj_id=obj_id, mission=mission).config_obj_in_world()
    traj = route.TrajectoryPlanner(mesh_points=structure_points, mission=mission, obj_id=obj_id)
    waypoints, trajectory = traj.plan_waypoints(plot_tour=False, obj_id=obj_id)
    centro = traj.center

    scenario = HoloOceanScenarios.Scenario("__", f"64-tank-Map-{mission}", "DatasetSonar", 200)
    auv = HoloOceanVehicles.SphereAgent(
        id='0',
        root=f"coverage-mission-{mission}-data-mv",
        control_scheme=0,
        location=list(trajectory[0]),
        rotation=list(waypoints[0, 3:]),
        mission=mission,
        waypoints=waypoints,
        sonar_model='obj' + str(obj_id)
    )

    sonar_configuration = json.load(open('sonar-configuration.json'))
    sonar_model = sonar_configuration["P900"]
    auv.addSonarImaging(configuration=sonar_model)
    auv.addSonarGT([0, 0, 0])
    auv.imageViwer()
    auv.addSensor("LocationSensor", "Origin")
    auv.addSensor("RotationSensor", "Origin")
    auv.addSensor("CollisionSensor", "Origin")
    auv.addSensor("RGBCamera", "CameraSocket")
    scenario.addAgent(auv.agent)

    env = holoocean.make(scenario_cfg=scenario.cfg, verbose=False)
    env.move_viewport([centro[0], centro[1], centro[2] + 7], [0, 270, 0])
    state = env.tick()

    # No início do seu script, ou onde as variáveis globais/constantes são definidas
    pitch_angles_mv = [-10.0, 0.0, 10.0]
    next_idx = 0
    auv.counter = 0
    #utils.update_waypoints(env, trajectory)

    while next_idx < len(waypoints):
        current_waypoint_pos = waypoints[next_idx, :3]
        current_waypoint_ori = waypoints[next_idx, 3:]

        for i,pitch in enumerate(pitch_angles_mv):
            new_ori = list([current_waypoint_ori[0],current_waypoint_ori[1]+pitch,current_waypoint_ori[2]])
            env.agents[auv.name].teleport(current_waypoint_pos, new_ori)
            state = env.tick()

            while True:
                if "ImagingSonar" in state[auv.name] and "0 0" in state[auv.name]:
                    print(f"[Simulação {k}] Waypoint {next_idx} - Pitch {i}")
                    auv.updateState(state)
                    auv.counter += 1
                    del state
                    gc.collect()
                    break
                else:
                    state = env.tick()

        next_idx += 1

    try:
        env.close()
    except:
        pass

    del structure_points, waypoints, trajectory
    del env, auv
    plt.close("all")
    gc.collect()
    os.system("killall -e Holodeck")
    gc.collect()


if __name__ == "__main__":
    k = int(sys.argv[1])
    run_simulation(k)
