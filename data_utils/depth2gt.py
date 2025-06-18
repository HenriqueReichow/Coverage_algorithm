import numpy as np
import tqdm
import csv
import os
import json
from PIL import Image
import math


def cp_bin(gt_filepath,out_filepath):
    try:
        os.system(f"cp {gt_filepath} {out_filepath}")
    except:
        pass

def bin_to_gt(gt_filepath, output_png_filepath,output_xyz_filepath,sonar_model):
    #prime_num=np.array([2,3,5,7,11,13,17,19,23,29])
    sonar_configuration = json.load(open('sonar-configuration.json'))
    sonar_model=sonar_configuration[sonar_model]
    gt_data=np.load(gt_filepath)
    theta, phi = gt_data.shape
    gt_image=np.zeros(shape=(sonar_model["RangeBins"],sonar_model["AzimuthBins"],3))
    gt_matrix=np.zeros(shape=(sonar_model["RangeBins"],sonar_model["AzimuthBins"]))
    
    #try:
        #with open(output_xyz_filepath, 'w') as outfile:
    for t in range(theta):
            for p in range(phi):
                    #r=gt_data[t][p]
                    #theta_=(t*(sonar_model["Azimuth"])/theta)-sonar_model["Azimuth"]/2
                    #phi_=((p*(sonar_model["Elevation"])/phi)-sonar_model["Elevation"]/2)-45
                
                
                    #x=r*np.cos(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_))
                    #y=r*np.sin(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_))
                    #z=r*np.sin(np.deg2rad(phi_))

                    r_index = int(math.floor(((gt_data[t][p]-(sonar_model["RangeMin"]))*sonar_model["RangeBins"])/sonar_model["RangeMax"]))
                    #if p>=10:
                    #    point=prime_num[p-10]
                    #    gt_image[r_index][t]+=[point,0,0]
                    #else:
                    #    point=prime_num[p]
                    #    gt_image[r_index][t]+=[0,point,0]
                    
                    gt_matrix[r_index][t]=p
                       
                    #outfile.write(f"{x} {y} {z}\n")
        #np.save(output_png_filepath,gt_matrix)
    image=(gt_matrix).astype(np.uint8)
    cartesian_image=Image.fromarray(image, mode='L').rotate(180)
        #print(cartesian_image)
    cartesian_image.save(output_png_filepath,format='PNG')
    #except:
        ##print("erro")
        #pass

"""
mission_id=1
sonar="P900"
i=0
auv=0
sonar_filepath=(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{auv}/Raw-data/{i}.npy")
gt_filepath=(f"Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{auv}/GT-folder/{i}.npy")
#pkl_file = (f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{auv}/RGBD-images/{i}.pkl")  # Replace with your .pkl file path
xyz_file="teste.xyz"
gt_file="teste.png"
bin_to_gt(gt_filepath,gt_file, xyz_file,sonar)
#create_matrix_from_file(xyz_file,gt_file)
"""

#missions=[1,2,3,4]
missions=[1]
sonar="P900"

for obj_id in range(8,40):

    if obj_id != 36:
        mission_id=1

        with open('/home/lh/Desktop/Coverage_algorithm/obj_locations/mission'+str(mission_id)+'.csv', newline='') as f:
            reader = csv.reader(f)
            mission_metadata = list(reader)
            mission_metadata.pop(0)

        mission_met=mission_metadata
        #for mission in tqdm.tqdm(mission_met):
        for i in tqdm.tqdm(range(303)):
            #npy_file = (f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/GT-bin/{i}.npy")  # Replace with your .npy file path
            npy_file = (f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{mission_id}-data-mv/Sonar-Dataset-mission-{mission_id}-obj{obj_id}/1-sphere-0-data/GT-folder/{i}.npy")  # Replace with your .npy file path

            #try:
            #    os.mkdir(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/Point-cloud")
            #except FileExistsError:
            #    pass
            #try:
            #    os.mkdir(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/GT-images")
            #except FileExistsError:
            #    pass
            
            xyz_file=(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-Point-cloud/{i}.xyz")  # Replace with desired output path
            #gt_file=(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/GT-images/{i}.png")
            if not os.path.exists(f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{mission_id}-data-mv/Sonar-Dataset-mission-{mission_id}-obj{obj_id}/1-sphere-0-data/GT-images"):
                os.makedirs(f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{mission_id}-data-mv/Sonar-Dataset-mission-{mission_id}-obj{obj_id}/1-sphere-0-data/GT-images")

            gt_file=(f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-1-data-mv/Sonar-Dataset-mission-1-obj{obj_id}/1-sphere-0-data/GT-images/{i}.png")
            
            #gt_file=(f"/home/guilherme/Documents/Pytorch-UNet/data/masks/{mission_id}-{sonar}-auv-{mission[0]}-{i}.png")
            try:
                bin_to_gt(npy_file,gt_file, xyz_file,sonar)
                #cp_bin(npy_file,(f"/home/guilherme/Documents/SEE-Dataset/Sonar-Dataset-mission-{mission_id}-{sonar}/auv-{mission[0]}/GT-bin/{i}.npy"))
            except:
                pass