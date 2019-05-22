from __future__ import print_function
import numpy as np
import json

def getTransform(input_filename, output_filename):
    f = open(input_filename, "r")
    text = f.read()
    f.close()
    
    lines = text.lower().split("\n")
    
    scale = ""
    translation = ""
    rotation = ""
    rotation_count = 0
    
    for i in xrange(0,len(lines)):
        line = lines[i]
        if("scale" in line):
            scale = float(line.split(":")[1])
        if("translation" in line):
            pieces = line.split(" ")
            translation = np.zeros((1,3))
            count = 0
            for piece in pieces[2:]:
                translation[0][count] = float(piece)
                count+=1
        if("rotation" in line):
            rotations = lines[i+1:i+4]
            rotation = np.zeros((3,3))
            index = 0
            for rot in rotations:
                rots = [float(a) for a in rot.split(' ') if a]
                rotation[index] = rots
                index += 1
    
    #print(text)
    #print("-------")
    #print(scale)
    #print(translation)
    #print(rotation)
    
    values = {"scale":scale, "translation": translation.tolist(), "rotation":rotation.tolist()}
    #print(values)
    
    with open(output_filename, 'w') as fp:
        json.dump(values, fp)

if __name__ == '__main__':
    reconstruction_folder = "reconstructions/"
    sfm_folder = reconstruction_folder+ "reconstruction_global/"
    infilename = sfm_folder+"geodectic_correction.txt"
    outfilename = reconstruction_folder+"geodetic_transform.json"
    getTransform(infilename, outfilename)
