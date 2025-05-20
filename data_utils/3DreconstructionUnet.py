import numpy as np
from PIL import Image
import json
import tqdm
import csv

def png2matrix(image_path):
    """
    Opens a PNG image, converts it to grayscale, and returns its pixel data as a NumPy array.

    Args:
        image_path (str): The path to the PNG image file.

    Returns:
        numpy.ndarray: A 2D NumPy array representing the grayscale pixel data, or None if an error occurs.
    """
    try:
        # Open the image using Pillow (PIL)
        img = Image.open(image_path)
        img=img.rotate(180)

        # Convert the image to grayscale
        img_gray = img.convert("L")  # "L" mode for grayscale

        # Convert the grayscale image to a NumPy array
        numpy_array = np.array(img_gray)
        #print(numpy_array[0])
        return numpy_array

    except FileNotFoundError:
        print(f"Error: Image file not found at {image_path}")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def matrix2xyz(matrix,output_xyz_filepath,index,config,auv_metadata_dir,mission_metadata):
        # sonar_configuration = json.load(open('sonar-configuration.json'))
        # sonar_model=sonar_configuration["P900"]
        sonar_model = config
        auv_metadata=json.load(open(auv_metadata_dir+f"{index}.json"))
        radius, theta = matrix.shape
        #print(radius, theta)
        #print(matrix.shape)
        # try:
        # with open(output_xyz_filepath, 'a') as outfile:
        #         for t in range(theta):
        #             for r in range(radius):
        #                 if matrix[r][t]>0:
        #                     #print("aqio")
                            
        #                     rad = (r*(sonar_model["RangeMax"])/sonar_model["RangeBins"]) + sonar_model["RangeMin"]
        #                     #print("rad",rad)
        #                     phi_= (matrix[r][t] - (sonar_model["Elevation"]/2)) - auv_metadata["pitch"]
        #                     #print("phi",phi_)
        #                     theta_= ((t*(sonar_model["Azimuth"])/theta)-sonar_model["Azimuth"]/2) +auv_metadata["yaw"]
        #                     #print("theta",theta_)
                            
        #                     x=(rad*np.cos(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["x"])
        #                     y=(rad*np.sin(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["y"])
        #                     z=(rad*np.sin(np.deg2rad(phi_))) + auv_metadata["z"]
        #                     outfile.write(f"{x} {y} {z}\n")

        with open(output_xyz_filepath, 'a') as outfile:
                for t in range(theta):
                    for r in range(radius):
                        if matrix[r][t]>0:
                            #print("aqio")
                            
                            rad = (r*sonar_model["RangeMax"])/sonar_model["RangeBins"] + sonar_model["RangeMin"]
                            phi_= (matrix[r][t]-(sonar_model["Elevation"]/2))-auv_metadata["pitch"]
                            theta_= ((t*(sonar_model["Azimuth"])/theta)-sonar_model["Azimuth"]/2) +auv_metadata["yaw"]
                            
                            
                            x=(rad*np.cos(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["x"]-float(mission_metadata[2]))
                            y=(rad*np.sin(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["y"]+float(mission_metadata[3]))
                            z=(rad*np.sin(np.deg2rad(phi_))) + auv_metadata["z"]+float(mission_metadata[4])
                            outfile.write(f"{x} {y} {z}\n")
        # except:
            # pass

output_xyz_filepath=(f"experiments-unet/obj_0_pointcloud/full0.xyz")
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

with open(f"obj_locations/mission1.csv", newline='') as f:
    reader = csv.reader(f)
    mission_metadata = list(reader)
    mission_metadata.pop(0)
    print(mission_metadata)
mission_metadata=mission_metadata[0]

for i in tqdm.tqdm(range(10)):
    image_file_path = (f"/home/lh/Desktop/Coverage_algorithm/experiments-unet/obj-0/{i}.png")
    auv_metadata_dir = "/home/lh/Desktop/Coverage_algorithm/coverage-mission-1-data/Sonar-Dataset-mission-1-obj0/1-sphere-0-data/Meta-data/"
    matrix2xyz(matrix=png2matrix(image_file_path),output_xyz_filepath=output_xyz_filepath,index=i,config=config, auv_metadata_dir=auv_metadata_dir,mission_metadata=mission_metadata)


# image_file_path = (f"/home/lh/Desktop/Coverage_algorithm/experiments-unet/obj-0/12.png")
# auv_metadata_dir = "/home/lh/Desktop/Coverage_algorithm/coverage-mission-1-data/Sonar-Dataset-mission-1-obj0/1-sphere-0-data/Meta-data/"
# matrix2xyz(matrix=png2matrix(image_file_path),output_xyz_filepath=output_xyz_filepath,index=12,config=config, auv_metadata_dir=auv_metadata_dir)


