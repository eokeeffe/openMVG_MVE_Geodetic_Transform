<h1> How to use this code </h1>

<h2> Steps </h2>
<ol>
	<li> Run OpenMVG pipeline without gps priori </li>
	<li> Run Rigid GPS to XYZ Transformation, pipe output to a file </li>
	<li> Run transform.py in the scripts folder on the output file from above</li>
	<li> This gets the geodetic transform and creates a .json file</li>
	<li> Run apply_transform in the build directory, giving the location of the openMVG/MVE PLY file and location of the .json file</li>
	<li> Output is a PLY file which is in Earth Centered, Earth Fixed (ECEF) format</li>
</ol> 

<h3> Maintainer: Evan O'Keeffe, evanokeeffe@gmail.com </h3>
