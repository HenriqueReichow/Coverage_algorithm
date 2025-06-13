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
        return numpy_array

    except FileNotFoundError:
        print(f"Error: Image file not found at {image_path}")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def matrix2xyz(matrix,output_xyz_filepath,index,mission_metadata,auv=0,mission=2,obj=0):
        sonar_configuration = json.load(open('sonar-configuration.json'))
        sonar_model=sonar_configuration["P900"]
        auv_metadata=json.load(open(f"/home/lh/Desktop/Coverage_algorithm/coverage-mission-{mission}-data/Sonar-Dataset-mission-{mission}-obj{obj}/{mission}-sphere-{auv}-data/Meta-data/{index+1}.json"))

        radius, theta = matrix.shape
        #print(matrix.shape)
        try:
            with open(output_xyz_filepath, 'a') as outfile:
                for t in range(theta):
                    for r in range(radius):
                        if matrix[r][t]>0:
                            #print("aqio")
                            
                            rad = (r*sonar_model["RangeMax"])/sonar_model["RangeBins"] + sonar_model["RangeMin"]
                            phi_= (matrix[r][t]-(sonar_model["Elevation"]/2))+ (auv_metadata["pitch"]*-1)
                            theta_= ((t*(sonar_model["Azimuth"])/theta)-sonar_model["Azimuth"]/2) +auv_metadata["yaw"]
                            
                            
                            x=(rad*np.cos(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["x"]-float(mission_metadata[2]))
                            y=(rad*np.sin(np.deg2rad(theta_))*np.cos(np.deg2rad(phi_)))+(auv_metadata["y"]+float(mission_metadata[3]))
                            z=(rad*np.sin(np.deg2rad(phi_))) + auv_metadata["z"]
                            #if (z <2 and z> -7) and (x < 3.6 and x> -3.6) and (y<3.6 and y>-3.6):
                            outfile.write(f"{x} {y} {z}\n")
        except:
            pass

for m in range(1,5):
    for i in range(40):
        if i != 36:
            obj=i

            with open(f"obj_locations/mission{m}.csv", newline='') as f:
                reader = csv.reader(f)
                mission_metadata = list(reader)
                mission_metadata.pop(0)

            mission=mission_metadata[0]
            output_xyz_filepath=(f"/home/lh/Desktop/Coverage_algorithm/reconstructions/unet/Unetpcd-{m}-obj{obj}.xyz")

            for i in tqdm.tqdm(range(0,99)):
                image_file_path = (f"/home/lh/Desktop/Coverage_algorithm/experiments-unet/mission{m}-inference/obj-{obj}/{i}.png")
                matrix2xyz(matrix=png2matrix(image_file_path),output_xyz_filepath=output_xyz_filepath,index=i,mission=m,auv=0,mission_metadata=mission,obj=obj)

