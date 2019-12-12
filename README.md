# How to use this code

## Steps
1. Run OpenMVG pipeline without gps priors
2. Run Rigid GPS to XYZ Transformation, pipe output to a file
3. Run transform.py in the scripts folder on the output file from above
4. This gets the geodetic transform and creates a .json file
5. Run apply_transform in the build directory, giving the location of the openMVG/MVE PLY file and location of the .json file
6. Output is a PLY file which is in Earth Centered, Earth Fixed (ECEF) format 


 Python code for geodesy transform file reconstruction script</h2>
```python
print ("7. Geodesy rectify")
f = open(reconstruction_dir+"/geodectic_correction.txt","w")
pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_geodesy_registration_to_gps_position"),
    "-i", reconstruction_dir+"/sfm_data.bin",
    "-o", reconstruction_dir+"/sfm_data_adjusted.bin"] ,stdout=f )
pRecons.wait()
f.close()
```


### Maintainer: Evan O'Keeffe, evanokeeffe@gmail.com
