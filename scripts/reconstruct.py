#!/usr/bin/python
#! -*- encoding: utf-8 -*-

# Python script to launch OpenMVG SfM tools on an image dataset
#
# usage : python tutorial_demo.py
#

import commands
import os,pwd
import subprocess
import sys
from PIL import Image

os_name = os.name
username = pwd.getpwuid(os.getuid())[0]

OPENMVG_SFM_BIN = None
# binary location for openMVG binaries
OPENMVG_SFM_BIN = "/home/" + username + "/Modelling/openMVG_Original/build/Linux-x86_64-Release/"

# Indicate the openMVG camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/home/" + username + "/Modelling/openMVG_Original/src/software/SfM" + "/../../openMVG/exif/sensor_width_database"

def get_parent_dir(directory):
    import os
    return os.path.dirname(directory)

input_eval_dir = "images"
# Checkout an OpenMVG image dataset with Git

cur_dir = os.getcwd()
img_filepath = cur_dir+os.sep+input_eval_dir+os.sep
image_filepath = img_filepath+os.listdir(img_filepath)[0]
im = Image.open(image_filepath)

focal_length = 1.2*max(im.size)
focal_length_str = str(focal_length)

print ("Focal:",focal_length_str)

output_eval_dir = "reconstructions"
try:
    os.removedir(output_eval_dir)
except:
    pass
if not os.path.exists(output_eval_dir):
  os.mkdir(output_eval_dir)
if not os.path.exists(output_images):
  os.mkdir(output_images)

input_dir = input_eval_dir
output_dir = output_eval_dir
print ("Using input dir  : ", input_dir)
print ("      output_dir : ", output_dir)

matches_dir = os.path.join(output_dir, "matches")
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

# Create the ouput/matches folder if not present
if not os.path.exists(matches_dir): os.mkdir(matches_dir)

print ("1. Intrinsics analysis - GPS Enabled")
pIntrisics = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"),
    "-P",
    "-d", camera_file_params,
    "-i", input_dir,
    "-o", output_eval_dir,
    "-f", focal_length_str,
    "-c","1",
    "-m","1", # Use UTM Output
    # Camera calibration, will allow in future to be used from the user
    #    "-k","1039.70055049;0.;647.9527915;0;1039.70055049;495.41426277;0;0;1."
    ] )
pIntrisics.wait()

print ("2. Compute features")
pFeatures = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"),
    "-i", output_eval_dir+"/sfm_data.json",
    "-o", matches_dir,
    "-m", "SIFT",
    "-p", "HIGH",
    "-n", "4"] )# n limit threads to 4
pFeatures.wait()

print ("3. Limiting Matching pairs")
pLimitMatching = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ListMatchingPairs"),
    "-G",
    "-i", output_eval_dir+"/sfm_data.json",
    "-o", matches_dir+"/pair_list.txt",
    "-n", "8"] )#-n limit matching pairs to 8 neighbours , "-n", "8"
pLimitMatching.wait()

# Reconstruction for the global SfM pipeline
# - global SfM pipeline use matches filtered by the essential matrices
# - here we reuse photometric matches and perform only the essential matrix filering
print ("4. Compute matches (for the global SfM Pipeline)")
pMatches = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeMatches"),
    "-i", output_eval_dir+"/sfm_data.json",
#    "-l", matches_dir+"/pair_list.txt" ,
    "-g", "e",
    "-r","0.8",
    "-o", matches_dir] )
pMatches.wait()

reconstruction_dir = os.path.join(output_dir,"reconstruction_global")
print ("5. Do Global reconstruction")
pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_GlobalSfM"),
    "-P",
    "-i", output_dir+"/sfm_data.json",
    "-m", matches_dir,
    "-o", reconstruction_dir] )
pRecons.wait()

print ("6. Colorize Structure")
pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeSfM_DataColor"),
    "-i", reconstruction_dir+"/sfm_data.bin",
    "-o", os.path.join(reconstruction_dir,"colorized.ply")] )
pRecons.wait()

print ("7. Geodesy rectify")
f = open(reconstruction_dir+"/geodectic_correction.txt","w")
pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_geodesy_registration_to_gps_position"),
    "-i", reconstruction_dir+"/sfm_data.bin",
    "-o", reconstruction_dir+"/sfm_data_adjusted.bin"] ,stdout=f )
pRecons.wait()
f.close()

# optional, compute final valid structure from the known camera poses
#print ("7. Structure from Known Poses (robust triangulation)")
#pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeStructureFromKnownPoses"),
#    "-i", reconstruction_dir+"/sfm_data.bin",
#    "-m", matches_dir,
#    "-f", os.path.join(matches_dir, "matches.e.bin"),
#    "-o", os.path.join(reconstruction_dir,"sfm_data_robust.bin")] )
#pRecons.wait()

#print ("8. Robust Colorization")
#pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeSfM_DataColor"),
#    "-i", reconstruction_dir+"/sfm_data_robust.bin",
#    "-o", os.path.join(reconstruction_dir,"robust_colorized.ply")] )
#pRecons.wait()

print ("9. Creating sfm_data.json file (Extract External Orientation information)")
input_format = "/sfm_data.bin"
output_format = "/sfm_data.json"
pConvert = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),
    "-i", reconstruction_dir+input_format,
    "-o", reconstruction_dir+output_format, ] )
pConvert.wait()

#input_format = "/sfm_data_adjusted.bin"
#output_format = "/sfm_data_adjusted.json"
#pConvert = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ConvertSfM_DataFormat"),
#    "-i", reconstruction_dir+input_format,
#    "-o", reconstruction_dir+output_format, ])
#pConvert.wait()

#pTransform = subprocess.Popen( ["python","transform.py"])
#pTransform.wait()
