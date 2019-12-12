<h1> How to use this code </h1>

<h2> Steps </h2>
<ol>
	<li> Run OpenMVG pipeline without gps priors </li>
	<li> Run Rigid GPS to XYZ Transformation, pipe output to a file </li>
	<li> Run transform.py in the scripts folder on the output file from above</li>
	<li> This gets the geodetic transform and creates a .json file</li>
	<li> Run apply_transform in the build directory, giving the location of the openMVG/MVE PLY file and location of the .json file</li>
	<li> Output is a PLY file which is in Earth Centered, Earth Fixed (ECEF) format</li>
</ol> 


<h2> Python code for geodesy transform file reconstruction script</h2>
```python
print ("7. Geodesy rectify")
f = open(reconstruction_dir+"/geodectic_correction.txt","w")
pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_geodesy_registration_to_gps_position"),
    "-i", reconstruction_dir+"/sfm_data.bin",
    "-o", reconstruction_dir+"/sfm_data_adjusted.bin"] ,stdout=f )
pRecons.wait()
f.close()
```


<h3> Maintainer: Evan O'Keeffe, evanokeeffe@gmail.com </h3>
