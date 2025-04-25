import os
import pickle
import numpy as np

def save_data_rangefinder(pickle_dir:str, config:dict, save_dir:str="/home/lh/Desktop/Coverage_algorithm/states/states_npy"):
    if not os.path.exists(save_dir):
            os.mkdir(save_dir)

    files = os.listdir(pickle_dir)
    for i,pickle_file in enumerate(files):
        convert_pickle_to_npy(pickle_dir+"/"+pickle_file, os.path.join(save_dir, f"{i}.npy"), config)

def convert_pickle_to_npy(pickle_file, save_path:str, config:dict):
    if type(pickle_file) == str:
        with open(pickle_file, "rb") as file:
            state = pickle.load(file)
    else:
        state = pickle_file
        
    range_matrix = np.zeros((config["Elevation"], config["AzimuthBins"]))

    for i in range(config["Elevation"]):
        for j in range(config["AzimuthBins"]):
            if f"RangeFinderSensor_{i}_{j}" in state:
                range_matrix[i, j] = state[f"RangeFinderSensor_{i}_{j}"][0]

    np.save(save_path, range_matrix)
    return range_matrix

# Configuração dos sensores
config = {
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
    "ViewRegion": False
}
# Chamada da função
save_data_rangefinder("/home/lh/Desktop/Coverage_algorithm/states/states_pkl", config)