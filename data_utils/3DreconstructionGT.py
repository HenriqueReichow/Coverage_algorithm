import numpy as np
from PIL import Image
import json
import tqdm
import csv
import os

def GT2xyz(output_xyz_filepath,auv_metadata_dir, matrix_path,mission_metadata):
        sonar_configuration = json.load(open('/home/lh/Desktop/Coverage_algorithm/sonar-configuration.json'))
        sonar_model=sonar_configuration["P900"]
        try:
            with open(output_xyz_filepath, 'w') as outfile:
                    for i in tqdm.tqdm(range(299)):

                        filename=(f"{i}.xyz")
                        matrix=np.load(matrix_path+f"/{i}.npy")
                        theta,phi=matrix.shape
                        #if filename.endswith(".xyz"):
                        #filepath = os.path.join(GT_folder, filename)
                        #try:
                        #with open(filepath, 'r') as infile:
                        for t in range(theta):
                            for p in range(phi): 
                                #parts = line.strip().split() #split each line by space.
                                auv_metadata=json.load(open(auv_metadata_dir+f"{i+1}.json"))
                                #print("here")
                                r=matrix[t][p]  
                                if matrix[t][p] < 6:
                                    theta_=((t*(sonar_model["Azimuth"])/theta)-sonar_model["Azimuth"]/2) + auv_metadata["yaw"]
                                    phi_=((p*(sonar_model["Elevation"])/phi)-sonar_model["Elevation"]/2) + (auv_metadata['pitch']*-1)
                
                
                                    x=r*np.cos(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_))+(auv_metadata["x"]-float(mission_metadata[2]))
                                    y=r*np.sin(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_))+(auv_metadata["y"]+float(mission_metadata[3]))
                                    z=r*np.sin(np.deg2rad(phi_)) + auv_metadata["z"]
                                
                                    outfile.write(f"{x} {y} {z}\n")
                                

                        #print(f"Merged and offset {filename}")
                        # except FileNotFoundError:
                        #     print(f"Error: File not found: {output_xyz_filepath}")
                        #except Exception as e:
                            # print(f"Error processing {filename}: {e}")

                    print(f"Successfully merged all .xyz files into {output_xyz_filepath}")

        except Exception as e:
            print(f"An error occurred: {e}")

for m in range(1,5):
    for i in range(1):
        if i != 36:
            obj = i
            #for i in range(1):#range(len(os.listdir("/home/lh/Desktop/Coverage_algorithm/coverage-mission-1-data"))):
                
            #GT_folder=(f"/home/lh/Desktop/Coverage_algorithm/experiments-unet/pcd_GT")
            #output_xyz_filepath=(f"/home/lh/Desktop/Coverage_algorithm/reconstructions/GT/GTpcd-{m}-obj{obj}.xyz")
            output_xyz_filepath=(f"/home/lh/Desktop/Coverage_algorithm/reconstructions/GT/mv/GTpcd-{m}-obj{obj}.xyz")

            

            with open(f"/home/lh/Desktop/Coverage_algorithm/obj_locations/mission{m}.csv", newline='') as f:
                    reader = csv.reader(f)
                    mission_metadata = list(reader)
                    mission_metadata.pop(0)
            #print(mission_metadata)
            mission_met=mission_metadata[0]
            #print(mission_met)

            #auv_metadata_dir = f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{m}-data/Sonar-Dataset-mission-{m}-obj{obj}/{m}-sphere-0-data/Meta-data/"
            #matrix_path = f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{m}-data/Sonar-Dataset-mission-{m}-obj{obj}/{m}-sphere-0-data/GT-folder"
            auv_metadata_dir = f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{m}-data-mv/Sonar-Dataset-mission-{m}-obj{obj}/{m}-sphere-0-data/Meta-data/"
            matrix_path = f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{m}-data-mv/Sonar-Dataset-mission-{m}-obj0/{m}-sphere-0-data/GT-folder"
            GT2xyz(output_xyz_filepath,auv_metadata_dir,matrix_path,mission_met)